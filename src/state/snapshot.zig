//! Canonical GRAVSNAP envelope shared by save/load and replay.
const std = @import("std");
const codec = @import("codec.zig");
const config = @import("../core/config.zig");
const hash = @import("hash.zig");
const version = @import("../version.zig");
const pipeline = @import("../dynamics/pipeline.zig");
const contact_cache = @import("../collision/contact_cache.zig");
const world_mod = @import("../dynamics/world.zig");
const geometry = @import("../math/geometry.zig");
const shapes = @import("../collision/shapes.zig");
const ids = @import("../core/ids.zig");
const joints = @import("../dynamics/joints.zig");
const sleeping = @import("../dynamics/sleeping.zig");

pub const magic = "GRAVSNAP";
pub const Error = codec.Error || error{ InvalidMagic, InvalidProtocol, CapacityExceeded, HeaderMismatch };
pub const Header = struct { configuration: config.SimulationConfig, asset_set: hash.Hash256 };
pub const pipeline_section: u16 = 0x8001;
pub const contacts_section: u16 = 0x8004;
pub const bodies_section: u16 = 0x8002;
pub const colliders_section: u16 = 0x8003;
pub const joints_section: u16 = 0x8005;
pub const sleep_section: u16 = 0x8006;
pub const ccd_section: u16 = 0x8007;

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

/// Binds a snapshot to the exact simulation configuration and immutable asset
/// set selected by the receiving World. Call this immediately after
/// `readHeader`, before staging or committing any state.
pub fn validateHeader(found: Header, expected: Header) Error!void {
    if (!std.meta.eql(found.configuration, expected.configuration) or !std.mem.eql(u8, &found.asset_set, &expected.asset_set)) return error.HeaderMismatch;
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
    var context = PipelineContactsContext{ .stage = .{ .patches = stage }, .scratch = scratch };
    try codec.readKnownSections(&reader, 1, &.{ pipeline_section, contacts_section }, PipelineContactsContext, &context, PipelineContactsContext.visit);
    if (!context.pipeline_seen or !context.contacts_seen) return error.UnknownRequiredSection;
    if (contacts.patches.len < context.stage.len) return error.CapacityExceeded;
    @memcpy(contacts.patches[0..context.stage.len], context.stage.active());
    contacts.len = context.stage.len;
    return .{ .header = header, .state = context.state };
}
const PipelineContactsContext = struct {
    pipeline_seen: bool = false,
    contacts_seen: bool = false,
    state: pipeline.State = .{},
    stage: contact_cache.Cache,
    scratch: []contact_cache.Patch,
    fn visit(self: *PipelineContactsContext, section: codec.Section) codec.Error!void {
        var reader = codec.Reader.init(section.payload);
        switch (section.id) {
            pipeline_section => {
                self.state = try readPipeline(&reader);
                self.pipeline_seen = true;
            },
            contacts_section => {
                decodeContacts(section.payload, &self.stage, self.scratch) catch return error.InvalidConfig;
                self.contacts_seen = true;
                return;
            },
            else => return,
        }
        try reader.finish();
    }
};

/// Encodes every World-owned body slot, including dead-slot generation and
/// retirement metadata. Derived integration scratch is not represented here.
pub fn encodeBodies(world: *const world_mod.World, output: []u8) Error![]const u8 {
    var sizing = codec.Writer.sizing();
    try writeBodies(&sizing, world);
    if (sizing.written() > output.len) return error.OutOfSpace;
    var writer = codec.Writer.init(output);
    try writeBodies(&writer, world);
    return output[0..writer.written()];
}

pub fn writeBodies(writer: *codec.Writer, world: *const world_mod.World) Error!void {
    const storage = world.storage;
    try writer.unsigned(u32, @intCast(storage.alive.len));
    try writer.vec3(world.settings.gravity);
    try writer.fpValue(world.settings.linear_damping);
    try writer.fpValue(world.settings.angular_damping);
    try writer.fpValue(world.settings.max_linear_speed);
    try writer.fpValue(world.settings.max_angular_speed);
    for (storage.alive, 0..) |alive, index| {
        try writer.boolean(alive);
        try writer.boolean(storage.retired[index]);
        try writer.unsigned(u32, storage.generation[index]);
        if (!alive) continue;
        try writer.byte(@intFromEnum(storage.body_type[index]));
        try writer.vec3(storage.position[index]);
        try writer.quat(storage.orientation[index]);
        try writer.vec3(storage.linear_velocity[index]);
        try writer.vec3(storage.angular_velocity[index]);
        try writer.fpValue(storage.inverse_mass[index]);
        try writeSymmetric(writer, storage.inverse_inertia_local[index]);
        try writer.vec3(storage.force[index]);
        try writer.vec3(storage.torque[index]);
        try writer.byte(@bitCast(storage.locks[index]));
        try writer.boolean(storage.has_target[index]);
        try writer.vec3(storage.target_position[index]);
        try writer.quat(storage.target_orientation[index]);
    }
}

/// Validates all bytes into `stage`; `target` is only touched after the
/// complete section has been accepted. Both Worlds must have equal capacity.
pub fn decodeBodies(input: []const u8, target: *world_mod.World, stage: *world_mod.World) Error!void {
    try ensureBodyCapacity(target.storage, stage.storage);
    try parseBodies(input, stage);
    copyBodies(target, stage);
}
fn parseBodies(input: []const u8, stage: *world_mod.World) Error!void {
    var reader = codec.Reader.init(input);
    const count = try reader.unsigned(u32);
    if (count != stage.storage.alive.len) return error.CapacityExceeded;
    stage.settings = .{ .gravity = try reader.vec3(), .linear_damping = try reader.fpValue(), .angular_damping = try reader.fpValue(), .max_linear_speed = try reader.fpValue(), .max_angular_speed = try reader.fpValue() };
    for (stage.storage.alive, 0..) |_, index| {
        const alive = try reader.boolean();
        const retired = try reader.boolean();
        const generation = try reader.unsigned(u32);
        if (alive and retired) return error.InvalidConfig;
        stage.storage.alive[index] = alive;
        stage.storage.retired[index] = retired;
        stage.storage.generation[index] = generation;
        if (!alive) continue;
        const body_type = std.enums.fromInt(shapes.BodyType, try reader.byte()) orelse return error.InvalidEnum;
        const position = try reader.vec3();
        const orientation = try readCanonicalQuat(&reader);
        const linear_velocity = try reader.vec3();
        const angular_velocity = try reader.vec3();
        const inverse_mass = try reader.fpValue();
        const inverse_inertia_local = try readSymmetric(&reader);
        const force = try reader.vec3();
        const torque = try reader.vec3();
        const lock_bits = try reader.byte();
        if ((lock_bits & 0xc0) != 0) return error.InvalidConfig;
        const has_target = try reader.boolean();
        const target_position = try reader.vec3();
        const target_orientation = try readCanonicalQuat(&reader);
        if ((body_type == .dynamic and inverse_mass.raw <= 0) or (body_type != .dynamic and inverse_mass.raw != 0)) return error.InvalidConfig;
        stage.storage.body_type[index] = body_type;
        stage.storage.position[index] = position;
        stage.storage.orientation[index] = orientation;
        stage.storage.linear_velocity[index] = linear_velocity;
        stage.storage.angular_velocity[index] = angular_velocity;
        stage.storage.inverse_mass[index] = inverse_mass;
        stage.storage.inverse_inertia_local[index] = inverse_inertia_local;
        stage.storage.force[index] = force;
        stage.storage.torque[index] = torque;
        stage.storage.locks[index] = @bitCast(lock_bits);
        stage.storage.has_target[index] = has_target;
        stage.storage.target_position[index] = target_position;
        stage.storage.target_orientation[index] = target_orientation;
    }
    try reader.finish();
}

fn writeSymmetric(writer: *codec.Writer, value: geometry.SymmetricMat3) Error!void {
    inline for (.{ value.xx, value.yy, value.zz, value.xy, value.xz, value.yz }) |entry| try writer.fpValue(entry);
}
fn readSymmetric(reader: *codec.Reader) codec.Error!geometry.SymmetricMat3 {
    return .{ .xx = try reader.fpValue(), .yy = try reader.fpValue(), .zz = try reader.fpValue(), .xy = try reader.fpValue(), .xz = try reader.fpValue(), .yz = try reader.fpValue() };
}
fn readCanonicalQuat(reader: *codec.Reader) codec.Error!geometry.Quat {
    const value = try reader.quat();
    // The sign rule is the representation-level canonical invariant. Unit
    // length is produced by the fixed-point integrator; malformed zero and
    // noncanonical sign encodings are rejected before any target mutation.
    if ((value.x.raw == 0 and value.y.raw == 0 and value.z.raw == 0 and value.w.raw == 0) or value.w.raw < 0 or (value.w.raw == 0 and (value.x.raw < 0 or (value.x.raw == 0 and (value.y.raw < 0 or (value.y.raw == 0 and value.z.raw < 0)))))) return error.InvalidConfig;
    return value;
}
fn ensureBodyCapacity(target: world_mod.Storage, stage: world_mod.Storage) Error!void {
    const count = target.alive.len;
    inline for (.{ target.body_type.len, target.position.len, target.orientation.len, target.linear_velocity.len, target.angular_velocity.len, target.inverse_mass.len, target.inverse_inertia_local.len, target.force.len, target.torque.len, target.locks.len, target.generation.len, target.retired.len, target.has_target.len, target.target_position.len, target.target_orientation.len, stage.body_type.len, stage.position.len, stage.orientation.len, stage.linear_velocity.len, stage.angular_velocity.len, stage.inverse_mass.len, stage.inverse_inertia_local.len, stage.force.len, stage.torque.len, stage.locks.len, stage.generation.len, stage.alive.len, stage.retired.len, stage.has_target.len, stage.target_position.len, stage.target_orientation.len }) |len| if (len != count) return error.CapacityExceeded;
}
fn copyBodies(target: *world_mod.World, stage: *const world_mod.World) void {
    target.settings = stage.settings;
    inline for (.{ "body_type", "position", "orientation", "linear_velocity", "angular_velocity", "inverse_mass", "inverse_inertia_local", "force", "torque", "locks", "generation", "alive", "retired", "has_target", "target_position", "target_orientation" }) |name| @memcpy(@field(target.storage, name), @field(stage.storage, name));
}

pub fn encodeColliders(world: *const world_mod.World, output: []u8) Error![]const u8 {
    var sizing = codec.Writer.sizing();
    try writeColliders(&sizing, world);
    if (sizing.written() > output.len) return error.OutOfSpace;
    var writer = codec.Writer.init(output);
    try writeColliders(&writer, world);
    return output[0..writer.written()];
}
fn writeColliders(writer: *codec.Writer, world: *const world_mod.World) Error!void {
    const storage = world.colliders orelse {
        try writer.boolean(false);
        return;
    };
    try writer.boolean(true);
    try writer.unsigned(u32, @intCast(storage.alive.len));
    for (storage.alive, 0..) |alive, index| {
        try writer.boolean(alive);
        try writer.boolean(storage.retired[index]);
        try writer.unsigned(u32, storage.generation[index]);
        if (!alive) continue;
        try writer.unsigned(u64, storage.body[index].value);
        try writer.vec3(storage.local[index].position);
        try writer.quat(storage.local[index].orientation);
        try writeShape(writer, storage.shape[index]);
        try writer.fpValue(storage.material[index].friction);
        try writer.fpValue(storage.material[index].restitution);
        try writer.unsigned(u32, storage.category[index]);
        try writer.unsigned(u32, storage.mask[index]);
        try writer.signed(i32, storage.group[index]);
        try writer.boolean(storage.sensor[index]);
        try writer.boolean(storage.enabled[index]);
        try writer.unsigned(u32, storage.revision[index]);
    }
}
pub fn decodeColliders(input: []const u8, target: *world_mod.World, stage: *world_mod.World) Error!void {
    try ensureColliderCapacity(target, stage);
    try parseColliders(input, stage);
    copyColliders(target, stage);
}
fn parseColliders(input: []const u8, stage: *world_mod.World) Error!void {
    var reader = codec.Reader.init(input);
    const present = try reader.boolean();
    const storage = stage.colliders orelse {
        if (present) return error.CapacityExceeded;
        try reader.finish();
        return;
    };
    if (!present) return error.InvalidConfig;
    if (try reader.unsigned(u32) != storage.alive.len) return error.CapacityExceeded;
    for (storage.alive, 0..) |_, index| {
        const alive = try reader.boolean();
        const retired = try reader.boolean();
        const generation = try reader.unsigned(u32);
        if (alive and retired) return error.InvalidConfig;
        storage.alive[index] = alive;
        storage.retired[index] = retired;
        storage.generation[index] = generation;
        if (!alive) continue;
        const body: ids.BodyId = .{ .value = try reader.unsigned(u64) };
        if (body.index() >= stage.storage.alive.len or !stage.storage.alive[body.index()] or stage.storage.generation[body.index()] != body.generation()) return error.InvalidConfig;
        const local = geometry.Transform3{ .position = try reader.vec3(), .orientation = try readCanonicalQuat(&reader) };
        const shape = try readShape(&reader);
        shapes.validateBodyShape(shape, stage.storage.body_type[body.index()]) catch return error.InvalidConfig;
        const material = shapes.Material{ .friction = try reader.fpValue(), .restitution = try reader.fpValue() };
        const category = try reader.unsigned(u32);
        const mask = try reader.unsigned(u32);
        const group = try reader.signed(i32);
        const sensor = try reader.boolean();
        const enabled = try reader.boolean();
        const revision = try reader.unsigned(u32);
        storage.body[index] = body;
        storage.local[index] = local;
        storage.shape[index] = shape;
        storage.material[index] = material;
        storage.category[index] = category;
        storage.mask[index] = mask;
        storage.group[index] = group;
        storage.sensor[index] = sensor;
        storage.enabled[index] = enabled;
        storage.revision[index] = revision;
    }
    try reader.finish();
}
fn writeShape(writer: *codec.Writer, value: shapes.Shape) Error!void {
    try writer.byte(@intFromEnum(value));
    switch (value) {
        .sphere => |entry| try writer.fpValue(entry.radius),
        .box => |entry| try writer.vec3(entry.half_extents),
        .capsule => |entry| {
            try writer.fpValue(entry.radius);
            try writer.fpValue(entry.half_height);
        },
        inline else => |entry| {
            try writer.unsigned(u64, entry.source_id);
            try writer.unsigned(u64, entry.asset.value);
            try writer.unsigned(u32, entry.revision);
        },
    }
}
fn readShape(reader: *codec.Reader) codec.Error!shapes.Shape {
    const kind = std.enums.fromInt(shapes.ShapeKind, try reader.byte()) orelse return error.InvalidEnum;
    return switch (kind) {
        .sphere => .{ .sphere = .{ .radius = try reader.fpValue() } },
        .box => .{ .box = .{ .half_extents = try reader.vec3() } },
        .capsule => .{ .capsule = .{ .radius = try reader.fpValue(), .half_height = try reader.fpValue() } },
        inline else => |tag| @unionInit(shapes.Shape, @tagName(tag), .{ .source_id = try reader.unsigned(u64), .asset = .{ .value = try reader.unsigned(u64) }, .revision = try reader.unsigned(u32) }),
    };
}
fn ensureColliderCapacity(target: *const world_mod.World, stage: *const world_mod.World) Error!void {
    const source = target.colliders;
    const scratch = stage.colliders;
    if ((source == null) != (scratch == null)) return error.CapacityExceeded;
    if (source) |a| {
        const b = scratch.?;
        const count = a.alive.len;
        inline for (.{ a.body.len, a.local.len, a.shape.len, a.material.len, a.category.len, a.mask.len, a.group.len, a.sensor.len, a.enabled.len, a.revision.len, a.generation.len, a.retired.len, b.body.len, b.local.len, b.shape.len, b.material.len, b.category.len, b.mask.len, b.group.len, b.sensor.len, b.enabled.len, b.revision.len, b.generation.len, b.alive.len, b.retired.len }) |len| if (len != count) return error.CapacityExceeded;
    }
}
fn copyColliders(target: *world_mod.World, stage: *const world_mod.World) void {
    if (target.colliders) |*destination| {
        const source = stage.colliders.?;
        inline for (.{ "body", "local", "shape", "material", "category", "mask", "group", "sensor", "enabled", "revision", "generation", "alive", "retired" }) |name| @memcpy(@field(destination.*, name), @field(source, name));
    }
}

pub fn encodeJoints(pool: *const joints.Pool, output: []u8) Error![]const u8 {
    var sizing = codec.Writer.sizing();
    try writeJoints(&sizing, pool);
    if (sizing.written() > output.len) return error.OutOfSpace;
    var writer = codec.Writer.init(output);
    try writeJoints(&writer, pool);
    return output[0..writer.written()];
}
fn writeJoints(writer: *codec.Writer, pool: *const joints.Pool) Error!void {
    const storage = pool.storage;
    try writer.unsigned(u32, @intCast(storage.values.len));
    for (storage.values, 0..) |value, index| {
        try writer.boolean(storage.alive[index]);
        try writer.boolean(storage.retired[index]);
        try writer.unsigned(u32, storage.generation[index]);
        if (!storage.alive[index]) continue;
        try writer.byte(@intFromEnum(value.kind));
        try writer.unsigned(u64, value.body_a.value);
        try writer.unsigned(u64, value.body_b.value);
        inline for (.{ value.frame_a, value.frame_b }) |frame| {
            try writer.vec3(frame.anchor);
            try writer.vec3(frame.axis);
            try writer.vec3(frame.secondary);
        }
        try writer.fpValue(value.reference);
        try writer.fpValue(value.swing_reference);
        try writer.quat(value.reference_orientation);
        inline for (.{ value.limit.min, value.limit.max, value.motor.target_velocity, value.motor.max_force, value.spring.frequency, value.spring.damping_ratio, value.cone_twist.swing_max, value.cone_twist.twist_min, value.cone_twist.twist_max }) |entry| try writer.fpValue(entry);
        try writer.boolean(value.limit.enabled);
        try writer.boolean(value.motor.enabled);
        try writer.boolean(value.spring.enabled);
        try writer.boolean(value.cone_twist.enabled);
        try writer.byte(@intFromEnum(value.limit_state));
        for (value.cone_states) |state| try writer.byte(@intFromEnum(state));
        for (value.impulses) |impulse| try writer.fpValue(impulse);
    }
}
pub fn decodeJoints(input: []const u8, world: *const world_mod.World, target: *joints.Pool, stage: *joints.Pool) Error!void {
    try parseJoints(input, world, target, stage);
    copyJoints(target, stage);
}
fn parseJoints(input: []const u8, world: *const world_mod.World, target: *const joints.Pool, stage: *joints.Pool) Error!void {
    const target_storage = target.storage;
    const stage_storage = stage.storage;
    if (target_storage.values.len != stage_storage.values.len or target_storage.generation.len != target_storage.values.len or target_storage.alive.len != target_storage.values.len or target_storage.retired.len != target_storage.values.len or stage_storage.generation.len != target_storage.values.len or stage_storage.alive.len != target_storage.values.len or stage_storage.retired.len != target_storage.values.len) return error.CapacityExceeded;
    var reader = codec.Reader.init(input);
    if (try reader.unsigned(u32) != target_storage.values.len) return error.CapacityExceeded;
    for (stage.storage.values, 0..) |_, index| {
        const alive = try reader.boolean();
        const retired = try reader.boolean();
        const generation = try reader.unsigned(u32);
        if (alive and retired) return error.InvalidConfig;
        stage.storage.alive[index] = alive;
        stage.storage.retired[index] = retired;
        stage.storage.generation[index] = generation;
        if (!alive) continue;
        const kind = std.enums.fromInt(joints.Kind, try reader.byte()) orelse return error.InvalidEnum;
        const body_a: ids.BodyId = .{ .value = try reader.unsigned(u64) };
        const body_b: ids.BodyId = .{ .value = try reader.unsigned(u64) };
        if (world.bodyIndex(body_a) == null or world.bodyIndex(body_b) == null or body_a.value == body_b.value) return error.InvalidConfig;
        const frame_a = joints.Frame{ .anchor = try reader.vec3(), .axis = try reader.vec3(), .secondary = try reader.vec3() };
        const frame_b = joints.Frame{ .anchor = try reader.vec3(), .axis = try reader.vec3(), .secondary = try reader.vec3() };
        const reference = try reader.fpValue();
        const swing_reference = try reader.fpValue();
        const reference_orientation = try readCanonicalQuat(&reader);
        var value = joints.Joint{ .kind = kind, .body_a = body_a, .body_b = body_b, .frame_a = frame_a, .frame_b = frame_b, .reference = reference, .swing_reference = swing_reference, .reference_orientation = reference_orientation, .limit = .{ .min = try reader.fpValue(), .max = try reader.fpValue() }, .motor = .{ .target_velocity = try reader.fpValue(), .max_force = try reader.fpValue() }, .spring = .{ .frequency = try reader.fpValue(), .damping_ratio = try reader.fpValue() }, .cone_twist = .{ .swing_max = try reader.fpValue(), .twist_min = try reader.fpValue(), .twist_max = try reader.fpValue() } };
        value.limit.enabled = try reader.boolean();
        value.motor.enabled = try reader.boolean();
        value.spring.enabled = try reader.boolean();
        value.cone_twist.enabled = try reader.boolean();
        if ((value.limit.enabled and value.limit.min.raw > value.limit.max.raw) or value.motor.max_force.raw < 0) return error.InvalidConfig;
        value.limit_state = std.enums.fromInt(joints.LimitState, try reader.byte()) orelse return error.InvalidEnum;
        for (&value.cone_states) |*state| state.* = std.enums.fromInt(joints.LimitState, try reader.byte()) orelse return error.InvalidEnum;
        for (&value.impulses) |*impulse| impulse.* = try reader.fpValue();
        stage.storage.values[index] = value;
    }
    try reader.finish();
}
fn copyJoints(target: *joints.Pool, stage: *const joints.Pool) void {
    @memcpy(target.storage.values, stage.storage.values);
    @memcpy(target.storage.generation, stage.storage.generation);
    @memcpy(target.storage.alive, stage.storage.alive);
    @memcpy(target.storage.retired, stage.storage.retired);
}

pub fn encodeSleep(value: sleeping.Storage, output: []u8) Error![]const u8 {
    var sizing = codec.Writer.sizing();
    try writeSleep(&sizing, value);
    if (sizing.written() > output.len) return error.OutOfSpace;
    var writer = codec.Writer.init(output);
    try writeSleep(&writer, value);
    return output[0..writer.written()];
}
fn writeSleep(writer: *codec.Writer, value: sleeping.Storage) Error!void {
    if (value.awake.len != value.counter.len or value.awake.len != value.reason.len) return error.CapacityExceeded;
    try writer.unsigned(u32, @intCast(value.awake.len));
    for (value.awake, value.counter, value.reason) |awake, counter, reason| {
        try writer.boolean(awake);
        try writer.unsigned(u32, counter);
        try writer.byte(@intFromEnum(reason));
    }
}
pub fn decodeSleep(input: []const u8, target: sleeping.Storage, stage: sleeping.Storage) Error!void {
    if (target.awake.len != target.counter.len or target.awake.len != target.reason.len or stage.awake.len != target.awake.len or stage.counter.len != target.awake.len or stage.reason.len != target.awake.len) return error.CapacityExceeded;
    try parseSleep(input, stage);
    @memcpy(target.awake, stage.awake);
    @memcpy(target.counter, stage.counter);
    @memcpy(target.reason, stage.reason);
}
fn parseSleep(input: []const u8, stage: sleeping.Storage) Error!void {
    if (stage.awake.len != stage.counter.len or stage.awake.len != stage.reason.len) return error.CapacityExceeded;
    var reader = codec.Reader.init(input);
    if (try reader.unsigned(u32) != stage.awake.len) return error.CapacityExceeded;
    for (stage.awake, stage.counter, stage.reason) |*awake, *counter, *reason| {
        awake.* = try reader.boolean();
        counter.* = try reader.unsigned(u32);
        reason.* = std.enums.fromInt(sleeping.WakeReason, try reader.byte()) orelse return error.InvalidEnum;
    }
    try reader.finish();
}

/// CCD cursors are substep-derived. The persistent CCD policy is the
/// collider-indexed enable column, which must survive rollback verbatim.
pub fn encodeCcd(enabled: []const bool, output: []u8) Error![]const u8 {
    var sizing = codec.Writer.sizing();
    try sizing.unsigned(u32, @intCast(enabled.len));
    for (enabled) |entry| try sizing.boolean(entry);
    if (sizing.written() > output.len) return error.OutOfSpace;
    var writer = codec.Writer.init(output);
    try writer.unsigned(u32, @intCast(enabled.len));
    for (enabled) |entry| try writer.boolean(entry);
    return output[0..writer.written()];
}
pub fn decodeCcd(input: []const u8, target: []bool, stage: []bool) Error!void {
    if (target.len != stage.len) return error.CapacityExceeded;
    try parseCcd(input, stage);
    @memcpy(target, stage);
}
fn parseCcd(input: []const u8, stage: []bool) Error!void {
    var reader = codec.Reader.init(input);
    if (try reader.unsigned(u32) != stage.len) return error.CapacityExceeded;
    for (stage) |*entry| entry.* = try reader.boolean();
    try reader.finish();
}

/// All future-relevant logical state, staged by the caller for allocation-free
/// atomic load. Derived SAP/BVH/island/row data is deliberately absent.
pub const FullSnapshot = struct { header: Header, state: pipeline.State };
/// Recreates caller-owned transient state (SAP endpoints, traversal stacks,
/// islands, and constraint rows) after a validated logical-state commit. The
/// callback must be infallible: validation has completed and the logical
/// snapshot is already committed when it runs.
pub const DerivedRebuildFn = *const fn (?*anyopaque) void;
pub fn encodeFullSnapshot(header: Header, state: pipeline.State, world: *const world_mod.World, contacts: *const contact_cache.Cache, joint_pool: *const joints.Pool, sleep: sleeping.Storage, ccd_enabled: []const bool, output: []u8, pipeline_payload: []u8, bodies_payload: []u8, colliders_payload: []u8, contacts_payload: []u8, joints_payload: []u8, sleep_payload: []u8, ccd_payload: []u8) (Error || contact_cache.Error)![]const u8 {
    const p = try encodePipeline(state, pipeline_payload);
    const b = try encodeBodies(world, bodies_payload);
    const c = try encodeColliders(world, colliders_payload);
    const contacts_bytes = try encodeContacts(contacts, contacts_payload);
    const j = try encodeJoints(joint_pool, joints_payload);
    const s = try encodeSleep(sleep, sleep_payload);
    const ccd = try encodeCcd(ccd_enabled, ccd_payload);
    var sizing = codec.Writer.sizing();
    try writeHeader(&sizing, header);
    try codec.writeHeader(&sizing, 1, 7);
    inline for (.{ .{ pipeline_section, p }, .{ bodies_section, b }, .{ colliders_section, c }, .{ contacts_section, contacts_bytes }, .{ joints_section, j }, .{ sleep_section, s }, .{ ccd_section, ccd } }) |section| try codec.writeSection(&sizing, section[0], section[1]);
    if (sizing.written() > output.len) return error.OutOfSpace;
    var writer = codec.Writer.init(output);
    try writeHeader(&writer, header);
    try codec.writeHeader(&writer, 1, 7);
    inline for (.{ .{ pipeline_section, p }, .{ bodies_section, b }, .{ colliders_section, c }, .{ contacts_section, contacts_bytes }, .{ joints_section, j }, .{ sleep_section, s }, .{ ccd_section, ccd } }) |section| try codec.writeSection(&writer, section[0], section[1]);
    return output[0..writer.written()];
}
pub fn decodeFullSnapshot(input: []const u8, expected: Header, state: *pipeline.State, world: *world_mod.World, stage_world: *world_mod.World, contacts: *contact_cache.Cache, stage_contacts: []contact_cache.Patch, contact_scratch: []contact_cache.Patch, joint_pool: *joints.Pool, stage_joint_pool: *joints.Pool, sleep: sleeping.Storage, stage_sleep: sleeping.Storage, ccd_enabled: []bool, stage_ccd: []bool) (Error || contact_cache.Error)!FullSnapshot {
    var reader = codec.Reader.init(input);
    const header = try readHeader(&reader);
    try validateHeader(header, expected);
    var context = FullContext{ .target_world = world, .stage_world = stage_world, .stage_contacts = .{ .patches = stage_contacts }, .contact_scratch = contact_scratch, .joint_pool = joint_pool, .stage_joint_pool = stage_joint_pool, .sleep = sleep, .stage_sleep = stage_sleep, .ccd_enabled = ccd_enabled, .stage_ccd = stage_ccd };
    try codec.readKnownSections(&reader, 1, &.{ pipeline_section, bodies_section, colliders_section, contacts_section, joints_section, sleep_section, ccd_section }, FullContext, &context, FullContext.visit);
    if (!context.complete()) return error.UnknownRequiredSection;
    if (contacts.patches.len < context.stage_contacts.len) return error.CapacityExceeded;
    copyBodies(world, stage_world);
    copyColliders(world, stage_world);
    @memcpy(contacts.patches[0..context.stage_contacts.len], context.stage_contacts.active());
    contacts.len = context.stage_contacts.len;
    copyJoints(joint_pool, stage_joint_pool);
    @memcpy(sleep.awake, stage_sleep.awake);
    @memcpy(sleep.counter, stage_sleep.counter);
    @memcpy(sleep.reason, stage_sleep.reason);
    @memcpy(ccd_enabled, stage_ccd);
    state.* = context.state;
    return .{ .header = header, .state = context.state };
}

/// Atomically loads all future-relevant state and then rebuilds the derived
/// data deliberately omitted from GRAVSNAP. World hosts should use this entry
/// point rather than calling the low-level decoder directly.
pub fn decodeFullSnapshotAndRebuild(input: []const u8, expected: Header, state: *pipeline.State, world: *world_mod.World, stage_world: *world_mod.World, contacts: *contact_cache.Cache, stage_contacts: []contact_cache.Patch, contact_scratch: []contact_cache.Patch, joint_pool: *joints.Pool, stage_joint_pool: *joints.Pool, sleep: sleeping.Storage, stage_sleep: sleeping.Storage, ccd_enabled: []bool, stage_ccd: []bool, rebuild_context: ?*anyopaque, rebuild: DerivedRebuildFn) (Error || contact_cache.Error)!FullSnapshot {
    const decoded = try decodeFullSnapshot(input, expected, state, world, stage_world, contacts, stage_contacts, contact_scratch, joint_pool, stage_joint_pool, sleep, stage_sleep, ccd_enabled, stage_ccd);
    rebuild(rebuild_context);
    return decoded;
}
const FullContext = struct {
    target_world: *world_mod.World,
    stage_world: *world_mod.World,
    stage_contacts: contact_cache.Cache,
    contact_scratch: []contact_cache.Patch,
    joint_pool: *joints.Pool,
    stage_joint_pool: *joints.Pool,
    sleep: sleeping.Storage,
    stage_sleep: sleeping.Storage,
    ccd_enabled: []bool,
    stage_ccd: []bool,
    state: pipeline.State = .{},
    seen: u8 = 0,
    fn complete(self: *const FullContext) bool {
        return self.seen == 0x7f;
    }
    fn visit(self: *FullContext, section: codec.Section) codec.Error!void {
        switch (section.id) {
            pipeline_section => {
                var r = codec.Reader.init(section.payload);
                self.state = readPipeline(&r) catch return error.InvalidConfig;
                r.finish() catch return error.InvalidConfig;
                self.seen |= 1;
            },
            bodies_section => {
                ensureBodyCapacity(self.target_world.storage, self.stage_world.storage) catch return error.InvalidConfig;
                parseBodies(section.payload, self.stage_world) catch return error.InvalidConfig;
                self.seen |= 2;
            },
            colliders_section => {
                ensureColliderCapacity(self.target_world, self.stage_world) catch return error.InvalidConfig;
                parseColliders(section.payload, self.stage_world) catch return error.InvalidConfig;
                self.seen |= 4;
            },
            contacts_section => {
                decodeContacts(section.payload, &self.stage_contacts, self.contact_scratch) catch return error.InvalidConfig;
                self.seen |= 8;
            },
            joints_section => {
                parseJoints(section.payload, self.stage_world, self.joint_pool, self.stage_joint_pool) catch return error.InvalidConfig;
                self.seen |= 16;
            },
            sleep_section => {
                parseSleep(section.payload, self.stage_sleep) catch return error.InvalidConfig;
                self.seen |= 32;
            },
            ccd_section => {
                parseCcd(section.payload, self.stage_ccd) catch return error.InvalidConfig;
                self.seen |= 64;
            },
            else => {},
        }
    }
};

pub const PipelineBodiesContactsSnapshot = struct { header: Header, state: pipeline.State };

/// Canonical three-section snapshot for the state currently owned by the
/// World pipeline. The payload buffers are caller scratch and may be reused
/// immediately after this call returns.
pub fn encodePipelineBodiesContactsSnapshot(header: Header, state: pipeline.State, world: *const world_mod.World, contacts: *const contact_cache.Cache, output: []u8, pipeline_payload: []u8, bodies_payload: []u8, colliders_payload: []u8, contacts_payload: []u8) (Error || contact_cache.Error)![]const u8 {
    const pipeline_bytes = try encodePipeline(state, pipeline_payload);
    const body_bytes = try encodeBodies(world, bodies_payload);
    const collider_bytes = try encodeColliders(world, colliders_payload);
    const contact_bytes = try encodeContacts(contacts, contacts_payload);
    var sizing = codec.Writer.sizing();
    try writeHeader(&sizing, header);
    try codec.writeHeader(&sizing, 1, 4);
    try codec.writeSection(&sizing, pipeline_section, pipeline_bytes);
    try codec.writeSection(&sizing, bodies_section, body_bytes);
    try codec.writeSection(&sizing, colliders_section, collider_bytes);
    try codec.writeSection(&sizing, contacts_section, contact_bytes);
    if (sizing.written() > output.len) return error.OutOfSpace;
    var writer = codec.Writer.init(output);
    try writeHeader(&writer, header);
    try codec.writeHeader(&writer, 1, 4);
    try codec.writeSection(&writer, pipeline_section, pipeline_bytes);
    try codec.writeSection(&writer, bodies_section, body_bytes);
    try codec.writeSection(&writer, colliders_section, collider_bytes);
    try codec.writeSection(&writer, contacts_section, contact_bytes);
    return output[0..writer.written()];
}

/// Full two-pass decode for the sections currently implemented. `stage_world`
/// and `stage_contacts` receive all parsing writes; target world/cache remain
/// unchanged until every known required section has been accepted.
pub fn decodePipelineBodiesContactsSnapshot(input: []const u8, world: *world_mod.World, stage_world: *world_mod.World, contacts: *contact_cache.Cache, stage_contacts: []contact_cache.Patch, contact_scratch: []contact_cache.Patch) (Error || contact_cache.Error)!PipelineBodiesContactsSnapshot {
    var reader = codec.Reader.init(input);
    const header = try readHeader(&reader);
    var context = PipelineBodiesContactsContext{ .target_world = world, .stage_world = stage_world, .stage_contacts = .{ .patches = stage_contacts }, .contact_scratch = contact_scratch };
    try codec.readKnownSections(&reader, 1, &.{ pipeline_section, bodies_section, colliders_section, contacts_section }, PipelineBodiesContactsContext, &context, PipelineBodiesContactsContext.visit);
    if (!context.pipeline_seen or !context.bodies_seen or !context.colliders_seen or !context.contacts_seen) return error.UnknownRequiredSection;
    if (contacts.patches.len < context.stage_contacts.len) return error.CapacityExceeded;
    // All parsing has succeeded. These are the only writes to destination
    // state, fulfilling the snapshot two-pass transaction contract.
    copyBodies(world, stage_world);
    copyColliders(world, stage_world);
    @memcpy(contacts.patches[0..context.stage_contacts.len], context.stage_contacts.active());
    contacts.len = context.stage_contacts.len;
    return .{ .header = header, .state = context.state };
}

/// Checked load entry point for World receivers. Header binding is completed
/// before the regular two-pass parser receives the destination pointers.
pub fn decodePipelineBodiesContactsSnapshotChecked(input: []const u8, expected: Header, world: *world_mod.World, stage_world: *world_mod.World, contacts: *contact_cache.Cache, stage_contacts: []contact_cache.Patch, contact_scratch: []contact_cache.Patch) (Error || contact_cache.Error)!PipelineBodiesContactsSnapshot {
    var reader = codec.Reader.init(input);
    try validateHeader(try readHeader(&reader), expected);
    return decodePipelineBodiesContactsSnapshot(input, world, stage_world, contacts, stage_contacts, contact_scratch);
}
const PipelineBodiesContactsContext = struct {
    target_world: *world_mod.World,
    stage_world: *world_mod.World,
    stage_contacts: contact_cache.Cache,
    contact_scratch: []contact_cache.Patch,
    pipeline_seen: bool = false,
    bodies_seen: bool = false,
    colliders_seen: bool = false,
    contacts_seen: bool = false,
    state: pipeline.State = .{},
    fn visit(self: *PipelineBodiesContactsContext, section: codec.Section) codec.Error!void {
        switch (section.id) {
            pipeline_section => {
                var reader = codec.Reader.init(section.payload);
                self.state = try readPipeline(&reader);
                try reader.finish();
                self.pipeline_seen = true;
            },
            bodies_section => {
                ensureBodyCapacity(self.target_world.storage, self.stage_world.storage) catch return error.InvalidConfig;
                parseBodies(section.payload, self.stage_world) catch return error.InvalidConfig;
                self.bodies_seen = true;
            },
            colliders_section => {
                ensureColliderCapacity(self.target_world, self.stage_world) catch return error.InvalidConfig;
                parseColliders(section.payload, self.stage_world) catch return error.InvalidConfig;
                self.colliders_seen = true;
            },
            contacts_section => {
                decodeContacts(section.payload, &self.stage_contacts, self.contact_scratch) catch return error.InvalidConfig;
                self.contacts_seen = true;
            },
            else => {},
        }
    }
};
