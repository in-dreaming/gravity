//! Canonical GRAVSNAP envelope shared by save/load and replay.
const std = @import("std");
const codec = @import("codec.zig");
const config = @import("../core/config.zig");
const hash = @import("hash.zig");
const version = @import("../version.zig");
const pipeline = @import("../dynamics/pipeline.zig");
const contact_cache = @import("../collision/contact_cache.zig");

pub const magic = "GRAVSNAP";
pub const Error = codec.Error || error{ InvalidMagic, InvalidProtocol };
pub const Header = struct { configuration: config.SimulationConfig, asset_set: hash.Hash256 };
pub const pipeline_section: u16 = 0x8001;
pub const contacts_section: u16 = 0x8004;

/// Writes the fixed, layout-independent envelope before ascending snapshot
/// sections. The body is intentionally supplied by the higher-level two-pass
/// encoder, so sizing and output passes use exactly the same header path.
pub fn writeHeader(writer: *codec.Writer, value: Header) Error!void {
    for (magic) |byte| try writer.byte(byte);
    try writer.unsigned(u32, version.snapshot_format_version);
    try writer.unsigned(u32, version.protocol_version);
    try codec.encodeConfig(writer, value.configuration);
    for (value.asset_set) |byte| try writer.byte(byte);
}

pub fn readHeader(reader: *codec.Reader) Error!Header {
    var found: [magic.len]u8 = undefined;
    for (&found) |*byte| byte.* = try reader.byte();
    if (!std.mem.eql(u8, &found, magic)) return error.InvalidMagic;
    if (try reader.unsigned(u32) != version.snapshot_format_version) return error.InvalidVersion;
    if (try reader.unsigned(u32) != version.protocol_version) return error.InvalidProtocol;
    const configuration = try codec.decodeConfig(reader);
    var asset_set: hash.Hash256 = undefined;
    for (&asset_set) |*byte| byte.* = try reader.byte();
    return .{ .configuration = configuration, .asset_set = asset_set };
}

/// Canonical payload for required section 0x8001. It is separate from the
/// envelope so the outer two-pass loader can validate every section before
/// committing its destination State.
pub fn writePipeline(writer: *codec.Writer, value: pipeline.State) Error!void {
    try writer.unsigned(u64, value.tick);
    if (value.fault) |fault| {
        try writer.boolean(true);
        try writer.unsigned(u64, fault.tick);
        try writer.byte(@intFromEnum(fault.phase));
        try writer.boolean(fault.object != null);
        if (fault.object) |object| try writer.unsigned(u64, object);
        try writer.byte(@intFromEnum(fault.code));
        try writer.byte(@intFromEnum(fault.detail));
        try writer.byte(@intFromEnum(fault.math_fault));
    } else try writer.boolean(false);
}

pub fn encodePipeline(value: pipeline.State, output: []u8) Error![]const u8 {
    var sizing = codec.Writer.sizing();
    try writePipeline(&sizing, value);
    if (sizing.written() > output.len) return error.OutOfSpace;
    var writer = codec.Writer.init(output);
    try writePipeline(&writer, value);
    return output[0..writer.written()];
}

/// Required section 0x8004 uses the Task 12 codec verbatim: its decode path
/// validates canonical order and commits only after filling caller scratch.
pub fn encodeContacts(value: *const contact_cache.Cache, output: []u8) contact_cache.CodecError![]const u8 {
    return contact_cache.encode(value, output);
}
pub fn decodeContacts(input: []const u8, value: *contact_cache.Cache, scratch: []contact_cache.Patch) contact_cache.CodecError!void {
    return contact_cache.decode(input, value, scratch);
}

pub fn readPipeline(reader: *codec.Reader) codec.Error!pipeline.State {
    var result = pipeline.State{ .tick = try reader.unsigned(u64) };
    if (!try reader.boolean()) return result;
    const tick = try reader.unsigned(u64);
    const phase = std.enums.fromInt(pipeline.Phase, try reader.byte()) orelse return error.InvalidEnum;
    const has_object = try reader.boolean();
    const object = if (has_object) try reader.unsigned(u64) else null;
    const code = std.enums.fromInt(pipeline.FaultCode, try reader.byte()) orelse return error.InvalidEnum;
    const detail = std.enums.fromInt(pipeline.FaultDetail, try reader.byte()) orelse return error.InvalidEnum;
    const math_fault = std.enums.fromInt(@import("../math/fp.zig").MathFault, try reader.byte()) orelse return error.InvalidEnum;
    result.fault = .{ .tick = tick, .phase = phase, .object = object, .code = code, .detail = detail, .math_fault = math_fault };
    return result;
}

/// First complete GRAVSNAP route. Later required sections reuse this exact
/// envelope/TLV contract; `payload` is caller scratch, never an allocation.
pub fn encodePipelineSnapshot(header: Header, state: pipeline.State, output: []u8, payload: []u8) Error![]const u8 {
    const encoded = try encodePipeline(state, payload);
    var sizing = codec.Writer.sizing();
    try writeHeader(&sizing, header);
    try codec.writeHeader(&sizing, 1, 1);
    try codec.writeSection(&sizing, pipeline_section, encoded);
    if (sizing.written() > output.len) return error.OutOfSpace;
    var writer = codec.Writer.init(output);
    try writeHeader(&writer, header);
    try codec.writeHeader(&writer, 1, 1);
    try codec.writeSection(&writer, pipeline_section, encoded);
    return output[0..writer.written()];
}

pub const PipelineSnapshot = struct { header: Header, state: pipeline.State };
pub fn decodePipelineSnapshot(input: []const u8) Error!PipelineSnapshot {
    var reader = codec.Reader.init(input);
    const header = try readHeader(&reader);
    var context = PipelineContext{};
    try codec.readKnownSections(&reader, 1, &.{pipeline_section}, PipelineContext, &context, PipelineContext.visit);
    if (!context.seen) return error.UnknownRequiredSection;
    return .{ .header = header, .state = context.state };
}
const PipelineContext = struct {
    seen: bool = false,
    state: pipeline.State = .{},
    fn visit(self: *PipelineContext, section: codec.Section) codec.Error!void {
        if (section.id != pipeline_section) return;
        var reader = codec.Reader.init(section.payload);
        self.state = try readPipeline(&reader);
        try reader.finish();
        self.seen = true;
    }
};

pub const PipelineContactsSnapshot = struct { header: Header, state: pipeline.State };
/// Two required sections with an atomic contact-cache destination commit.
pub fn encodePipelineContactsSnapshot(header: Header, state: pipeline.State, contacts: *const contact_cache.Cache, output: []u8, pipeline_payload: []u8, contacts_payload: []u8) (Error || contact_cache.Error)![]const u8 {
    const pipeline_bytes = try encodePipeline(state, pipeline_payload);
    const contact_bytes = try encodeContacts(contacts, contacts_payload);
    var sizing = codec.Writer.sizing();
    try writeHeader(&sizing, header);
    try codec.writeHeader(&sizing, 1, 2);
    try codec.writeSection(&sizing, pipeline_section, pipeline_bytes);
    try codec.writeSection(&sizing, contacts_section, contact_bytes);
    if (sizing.written() > output.len) return error.OutOfSpace;
    var writer = codec.Writer.init(output);
    try writeHeader(&writer, header);
    try codec.writeHeader(&writer, 1, 2);
    try codec.writeSection(&writer, pipeline_section, pipeline_bytes);
    try codec.writeSection(&writer, contacts_section, contact_bytes);
    return output[0..writer.written()];
}
pub fn decodePipelineContactsSnapshot(input: []const u8, contacts: *contact_cache.Cache, stage: []contact_cache.Patch, scratch: []contact_cache.Patch) (Error || contact_cache.Error)!PipelineContactsSnapshot {
    var reader = codec.Reader.init(input);
    const header = try readHeader(&reader);
    var context = PipelineContactsContext{ .stage = .{ .patches = stage } , .scratch = scratch };
    try codec.readKnownSections(&reader, 1, &.{ pipeline_section, contacts_section }, PipelineContactsContext, &context, PipelineContactsContext.visit);
    if (!context.pipeline_seen or !context.contacts_seen) return error.UnknownRequiredSection;
    if (contacts.patches.len < context.stage.len) return error.CapacityExceeded;
    @memcpy(contacts.patches[0..context.stage.len], context.stage.active());
    contacts.len = context.stage.len;
    return .{ .header = header, .state = context.state };
}
const PipelineContactsContext = struct {
    pipeline_seen: bool = false, contacts_seen: bool = false, state: pipeline.State = .{}, stage: contact_cache.Cache, scratch: []contact_cache.Patch,
    fn visit(self: *PipelineContactsContext, section: codec.Section) codec.Error!void {
        var reader = codec.Reader.init(section.payload);
        switch (section.id) {
            pipeline_section => { self.state = try readPipeline(&reader); self.pipeline_seen = true; },
            contacts_section => { decodeContacts(section.payload, &self.stage, self.scratch) catch return error.InvalidConfig; self.contacts_seen = true; return; },
            else => return,
        }
        try reader.finish();
    }
};
