//! Layout-independent little-endian canonical codec. No API serializes Zig memory.
const std = @import("std");
const fp = @import("../math/fp.zig");
const geometry = @import("../math/geometry.zig");
const ids = @import("../core/ids.zig");
const config = @import("../core/config.zig");

pub const Error = error{ EndOfInput, OutOfSpace, InvalidBool, InvalidEnum, InvalidSectionOrder, DuplicateSection, LengthOverflow, SectionTooLarge, UnknownRequiredSection, TrailingBytes, InvalidVersion, InvalidConfig };
pub const required_section_bit: u16 = 0x8000;
pub const max_section_payload: u32 = 16 * 1024 * 1024;

pub const Writer = struct {
    bytes: ?[]u8,
    at: usize = 0,
    last_section: ?u16 = null,

    pub fn sizing() Writer {
        return .{ .bytes = null };
    }
    pub fn init(bytes: []u8) Writer {
        return .{ .bytes = bytes };
    }
    pub fn written(self: Writer) usize {
        return self.at;
    }
    fn put(self: *Writer, data: []const u8) Error!void {
        const end = std.math.add(usize, self.at, data.len) catch return error.LengthOverflow;
        if (self.bytes) |output| {
            if (end > output.len) return error.OutOfSpace;
            @memcpy(output[self.at..end], data);
        }
        self.at = end;
    }
    pub fn byte(self: *Writer, value: u8) Error!void {
        try self.put(&.{value});
    }
    pub fn boolean(self: *Writer, value: bool) Error!void {
        try self.byte(@intFromBool(value));
    }
    pub fn unsigned(self: *Writer, comptime T: type, value: T) Error!void {
        var data: [@sizeOf(T)]u8 = undefined;
        std.mem.writeInt(T, &data, value, .little);
        try self.put(&data);
    }
    pub fn signed(self: *Writer, comptime T: type, value: T) Error!void {
        try self.unsigned(std.meta.Int(.unsigned, @bitSizeOf(T)), @bitCast(value));
    }
    pub fn fpValue(self: *Writer, value: fp.Fp) Error!void {
        try self.signed(i64, value.raw);
    }
    pub fn vec3(self: *Writer, value: geometry.Vec3) Error!void {
        try self.fpValue(value.x);
        try self.fpValue(value.y);
        try self.fpValue(value.z);
    }
    pub fn quat(self: *Writer, value: geometry.Quat) Error!void {
        try self.fpValue(value.x);
        try self.fpValue(value.y);
        try self.fpValue(value.z);
        try self.fpValue(value.w);
    }
    pub fn id(self: *Writer, value: ids.Id) Error!void {
        try self.unsigned(u64, value.value);
    }
};

pub const Reader = struct {
    bytes: []const u8,
    at: usize = 0,
    pub fn init(bytes: []const u8) Reader {
        return .{ .bytes = bytes };
    }
    fn take(self: *Reader, len: usize) Error![]const u8 {
        const end = std.math.add(usize, self.at, len) catch return error.LengthOverflow;
        if (end > self.bytes.len) return error.EndOfInput;
        defer self.at = end;
        return self.bytes[self.at..end];
    }
    pub fn byte(self: *Reader) Error!u8 {
        return (try self.take(1))[0];
    }
    /// Advances over unstructured payload bytes after the caller has checked
    /// their canonical length. This never exposes an unchecked slice.
    pub fn skip(self: *Reader, len: usize) Error!void {
        _ = try self.take(len);
    }
    pub fn boolean(self: *Reader) Error!bool {
        return switch (try self.byte()) {
            0 => false,
            1 => true,
            else => error.InvalidBool,
        };
    }
    pub fn unsigned(self: *Reader, comptime T: type) Error!T {
        const data = try self.take(@sizeOf(T));
        return std.mem.readInt(T, @ptrCast(data.ptr), .little);
    }
    pub fn signed(self: *Reader, comptime T: type) Error!T {
        return @bitCast(try self.unsigned(std.meta.Int(.unsigned, @bitSizeOf(T))));
    }
    pub fn fpValue(self: *Reader) Error!fp.Fp {
        return .{ .raw = try self.signed(i64) };
    }
    pub fn vec3(self: *Reader) Error!geometry.Vec3 {
        return .{ .x = try self.fpValue(), .y = try self.fpValue(), .z = try self.fpValue() };
    }
    pub fn quat(self: *Reader) Error!geometry.Quat {
        return .{ .x = try self.fpValue(), .y = try self.fpValue(), .z = try self.fpValue(), .w = try self.fpValue() };
    }
    pub fn id(self: *Reader) Error!ids.Id {
        return .{ .value = try self.unsigned(u64) };
    }
    pub fn finish(self: Reader) Error!void {
        if (self.at != self.bytes.len) return error.TrailingBytes;
    }
};

pub const Section = struct { id: u16, payload: []const u8 };
/// Validates a complete versioned TLV stream and invokes `visit` only after each
/// section header is verified. Unknown optional IDs are passed to the visitor.
pub fn readSections(reader: *Reader, expected_version: u16, comptime Context: type, context: *Context, comptime visit: fn (*Context, Section) Error!void) Error!void {
    if (try reader.unsigned(u16) != expected_version) return error.InvalidVersion;
    const count = try reader.unsigned(u16);
    var prior: ?u16 = null;
    var n: u16 = 0;
    while (n < count) : (n += 1) {
        const id = try reader.unsigned(u16);
        const length = try reader.unsigned(u32);
        if (prior) |previous| {
            if (id == previous) return error.DuplicateSection;
            if (id < previous) return error.InvalidSectionOrder;
        }
        prior = id;
        if (length > max_section_payload) return error.SectionTooLarge;
        const payload = try reader.take(@intCast(length));
        try visit(context, .{ .id = id, .payload = payload });
    }
    try reader.finish();
}

/// As `readSections`, but rejects an unrecognized required section before its
/// payload is observed. Optional future sections remain forward-compatible.
pub fn readKnownSections(reader: *Reader, expected_version: u16, known_ids: []const u16, comptime Context: type, context: *Context, comptime visit: fn (*Context, Section) Error!void) Error!void {
    if (try reader.unsigned(u16) != expected_version) return error.InvalidVersion;
    const count = try reader.unsigned(u16);
    var prior: ?u16 = null;
    var n: u16 = 0;
    while (n < count) : (n += 1) {
        const id = try reader.unsigned(u16);
        const length = try reader.unsigned(u32);
        if (prior) |previous| {
            if (id == previous) return error.DuplicateSection;
            if (id < previous) return error.InvalidSectionOrder;
        }
        prior = id;
        if (length > max_section_payload) return error.SectionTooLarge;
        if ((id & required_section_bit) != 0 and std.mem.indexOfScalar(u16, known_ids, id) == null) return error.UnknownRequiredSection;
        const payload = try reader.take(@intCast(length));
        try visit(context, .{ .id = id, .payload = payload });
    }
    try reader.finish();
}

pub fn writeHeader(writer: *Writer, version: u16, count: u16) Error!void {
    writer.last_section = null;
    try writer.unsigned(u16, version);
    try writer.unsigned(u16, count);
}
pub fn writeSection(writer: *Writer, id: u16, payload: []const u8) Error!void {
    if (payload.len > max_section_payload) return error.SectionTooLarge;
    if (writer.last_section) |previous| {
        if (id == previous) return error.DuplicateSection;
        if (id < previous) return error.InvalidSectionOrder;
    }
    writer.last_section = id;
    try writer.unsigned(u16, id);
    try writer.unsigned(u32, @intCast(payload.len));
    try writer.put(payload);
}

const ConfigWriter = struct {
    writer: *Writer,
    pub fn field(self: *ConfigWriter, _: []const u8, value: anytype) void {
        const T = @TypeOf(value);
        if (T == u32) self.writer.unsigned(u32, value) catch unreachable else if (T == i64) self.writer.signed(i64, value) catch unreachable else @compileError("unsupported config field");
    }
};
pub const config_encoded_size: usize = 29 * 4 + 15 * 8;
pub fn encodeConfig(writer: *Writer, value: config.SimulationConfig) Error!void {
    var sink = ConfigWriter{ .writer = writer };
    value.visitCanonical(&sink);
}
pub fn decodeConfig(reader: *Reader) Error!config.SimulationConfig {
    var value = config.SimulationConfig.default;
    inline for (.{ &value.capacities.body, &value.capacities.collider, &value.capacities.joint, &value.capacities.command_per_tick, &value.capacities.broad_pair, &value.capacities.contact_patch, &value.capacities.contact_point, &value.capacities.sensor_overlap, &value.capacities.event_per_tick, &value.capacities.rollback_window, &value.capacities.convex_hull_vertices, &value.capacities.convex_hull_faces, &value.capacities.compound_children, &value.capacities.compound_depth, &value.capacities.mesh_vertices, &value.capacities.mesh_triangles, &value.capacities.heightfield_axis_samples }) |field| field.* = try reader.unsigned(u32);
    inline for (.{ &value.tolerances.linear_slop.raw, &value.tolerances.angular_slop.raw, &value.tolerances.convex_skin.raw, &value.tolerances.aabb_margin.raw, &value.tolerances.max_position_correction.raw, &value.tolerances.max_angular_correction.raw, &value.tolerances.restitution_threshold.raw, &value.tolerances.warmstart_normal_cos_min.raw, &value.tolerances.sleep_linear_threshold.raw, &value.tolerances.sleep_angular_threshold.raw }) |field| field.* = try reader.signed(i64);
    inline for (.{ &value.iterations.tick_hz, &value.iterations.substeps, &value.iterations.velocity, &value.iterations.position, &value.iterations.sleep_ticks, &value.iterations.gjk, &value.iterations.epa, &value.iterations.epa_max_faces, &value.iterations.shape_cast, &value.iterations.ccd_toi_per_substep, &value.iterations.mesh_bvh_leaf_triangles }) |field| field.* = try reader.unsigned(u32);
    inline for (.{ &value.envelope.max_position.raw, &value.envelope.max_linear_velocity.raw, &value.envelope.max_angular_velocity.raw, &value.envelope.max_dynamic_size.raw, &value.envelope.max_mass.raw }) |field| field.* = try reader.signed(i64);
    value.features = @bitCast(try reader.unsigned(u32));
    value.validate() catch return error.InvalidConfig;
    return value;
}
