//! Canonical GRAVREPL recordings: an initial GRAVSNAP plus per-tick inputs.
const std = @import("std");
const codec = @import("codec.zig");
const hash = @import("hash.zig");
const version = @import("../version.zig");
const world = @import("../dynamics/world.zig");
const pipeline = @import("../dynamics/pipeline.zig");
const snapshot = @import("snapshot.zig");
const contact_cache = @import("../collision/contact_cache.zig");
const joints = @import("../dynamics/joints.zig");
const sleeping = @import("../dynamics/sleeping.zig");
const fp = @import("../math/fp.zig");
const geometry = @import("../math/geometry.zig");
const ids = @import("../core/ids.zig");

pub const magic = "GRAVREPL";
pub const Error = codec.Error || error{ InvalidMagic, InvalidProtocol, InvalidTickOrder, InvalidCommandOrder, CapacityExceeded, HashMismatch };

pub const Entry = struct { tick: u64, input: []const u8, expected_hash: hash.Hash128 };
pub const Recording = struct { initial_snapshot: []const u8, entries: []const Entry };

/// Writes the stable replay container. Inputs are already canonical command
/// streams; this module deliberately does not reinterpret them.
pub fn encode(value: Recording, output: []u8) Error![]const u8 {
    var sizing = codec.Writer.sizing();
    try write(&sizing, value);
    if (sizing.written() > output.len) return error.OutOfSpace;
    var writer = codec.Writer.init(output);
    try write(&writer, value);
    return output[0..writer.written()];
}
pub fn write(writer: *codec.Writer, value: Recording) Error!void {
    for (magic) |byte| try writer.byte(byte);
    try writer.unsigned(u32, version.snapshot_format_version);
    try writer.unsigned(u32, version.protocol_version);
    try writer.unsigned(u32, @intCast(value.initial_snapshot.len));
    for (value.initial_snapshot) |byte| try writer.byte(byte);
    try writer.unsigned(u32, @intCast(value.entries.len));
    var previous: ?u64 = null;
    for (value.entries) |entry| {
        if (previous) |tick| if (entry.tick <= tick) return error.InvalidTickOrder;
        previous = entry.tick;
        try writer.unsigned(u64, entry.tick);
        try writer.unsigned(u32, @intCast(entry.input.len));
        for (entry.expected_hash) |byte| try writer.byte(byte);
        for (entry.input) |byte| try writer.byte(byte);
    }
}

/// Decodes only into caller-owned slices. The input arena holds the initial
/// snapshot followed by command bytes; no allocation or target mutation occurs.
pub fn decode(input: []const u8, entries: []Entry, arena: []u8) Error!Recording {
    var reader = codec.Reader.init(input);
    var found: [magic.len]u8 = undefined;
    for (&found) |*byte| byte.* = try reader.byte();
    if (!std.mem.eql(u8, &found, magic)) return error.InvalidMagic;
    if (try reader.unsigned(u32) != version.snapshot_format_version) return error.InvalidVersion;
    if (try reader.unsigned(u32) != version.protocol_version) return error.InvalidProtocol;
    const snapshot_len = try reader.unsigned(u32);
    if (snapshot_len > arena.len) return error.CapacityExceeded;
    for (arena[0..snapshot_len]) |*byte| byte.* = try reader.byte();
    const count = try reader.unsigned(u32);
    if (count > entries.len) return error.CapacityExceeded;
    var used: usize = snapshot_len;
    var previous: ?u64 = null;
    for (entries[0..count]) |*entry| {
        const tick = try reader.unsigned(u64);
        if (previous) |prior| if (tick <= prior) return error.InvalidTickOrder;
        previous = tick;
        const length = try reader.unsigned(u32);
        const end = std.math.add(usize, used, length) catch return error.LengthOverflow;
        if (end > arena.len) return error.CapacityExceeded;
        var expected_hash: hash.Hash128 = undefined;
        for (&expected_hash) |*byte| byte.* = try reader.byte();
        for (arena[used..end]) |*byte| byte.* = try reader.byte();
        entry.* = .{ .tick = tick, .input = arena[used..end], .expected_hash = expected_hash };
        used = end;
    }
    try reader.finish();
    return .{ .initial_snapshot = arena[0..snapshot_len], .entries = entries[0..count] };
}

pub const RunFn = *const fn (?*anyopaque, Entry) anyerror!hash.Hash128;
pub const LoadFn = *const fn (?*anyopaque, []const u8) anyerror!void;
pub const Result = struct { first_mismatch: ?usize };

/// Caller-buffered bridge from a GRAVREPL recording to the released World
/// state.  The host owns no allocation and does not weaken snapshot loading:
/// every replay starts with the same complete two-pass load used by rollback,
/// then decodes each canonical command batch before stepping the fixed World
/// pipeline and hashing all future-relevant state.
///
/// `rebuild` recreates the caller's SAP/BVH/island/row scratch after a load.
/// It is deliberately supplied by the owning World integration because that
/// scratch is derived and is not part of GRAVSNAP.
/// Compatibility bridge for low-level World integration tests.  It is not a
/// GRAVREPL production host because it intentionally omits collision, solve,
/// sleeping, CCD and event phases.
pub const IntegrationOnlyHost = struct {
    header: snapshot.Header,
    state: *pipeline.State,
    value: *world.World,
    stage_world: *world.World,
    contacts: *contact_cache.Cache,
    stage_contacts: []contact_cache.Patch,
    contact_scratch: []contact_cache.Patch,
    joint_pool: *joints.Pool,
    stage_joint_pool: *joints.Pool,
    sleep: sleeping.Storage,
    stage_sleep: sleeping.Storage,
    ccd_enabled: []bool,
    stage_ccd: []bool,
    decoded_commands: []world.Command,
    workspace: *pipeline.Workspace,
    math_status: *fp.MathStatus,
    rebuild_context: ?*anyopaque,
    rebuild: snapshot.DerivedRebuildFn,

    /// Callback suitable for `run`; all output pointers refer to caller-owned
    /// state and scratch supplied when this host was created.
    pub fn load(context: ?*anyopaque, bytes: []const u8) anyerror!void {
        const self: *IntegrationOnlyHost = @ptrCast(@alignCast(context orelse return error.InvalidHost));
        _ = try snapshot.decodeFullSnapshotAndRebuild(bytes, self.header, self.state, self.value, self.stage_world, self.contacts, self.stage_contacts, self.contact_scratch, self.joint_pool, self.stage_joint_pool, self.sleep, self.stage_sleep, self.ccd_enabled, self.stage_ccd, self.rebuild_context, self.rebuild);
    }

    /// Callback suitable for `run`; it accepts only the canonical command
    /// representation stored by GRAVREPL and returns the complete state hash.
    pub fn step(context: ?*anyopaque, entry: Entry) anyerror!hash.Hash128 {
        const self: *IntegrationOnlyHost = @ptrCast(@alignCast(context orelse return error.InvalidHost));
        const commands = try decodeCommands(entry.input, self.decoded_commands);
        _ = try pipeline.stepBodies(self.value, self.state, self.header.configuration, commands, self.workspace, self.math_status);
        return pipeline.canonicalStateHash(self.value, self.state, self.header.configuration, .{ .cache = self.contacts, .joint = self.joint_pool, .sleep = self.sleep, .ccd_enabled = self.ccd_enabled });
    }
};

/// The only production replay bridge.  Its workspace is the caller-owned
/// canonical serial Task 20 profile, so every replayed tick executes the
/// complete analytic-solver pipeline; there is deliberately no fallback to
/// `stepBodies`.
pub const FullWorldHost = struct {
    base: *IntegrationOnlyHost,
    solver_workspace: *pipeline.AnalyticSolverPipelineWorkspace,

    pub fn load(context: ?*anyopaque, bytes: []const u8) anyerror!void {
        const self: *FullWorldHost = @ptrCast(@alignCast(context orelse return error.InvalidHost));
        try IntegrationOnlyHost.load(self.base, bytes);
    }

    pub fn step(context: ?*anyopaque, entry: Entry) anyerror!hash.Hash128 {
        const self: *FullWorldHost = @ptrCast(@alignCast(context orelse return error.InvalidHost));
        if (entry.tick != self.base.state.tick + 1) return error.InvalidTickOrder;
        const commands = try decodeCommands(entry.input, self.base.decoded_commands);
        const result = try pipeline.stepWithAnalyticSolver(self.base.value, self.base.state, self.base.header.configuration, commands, self.base.workspace, self.solver_workspace, self.base.math_status);
        if (result.step.tick != entry.tick) return error.InvalidTickOrder;
        return pipeline.canonicalStateHash(self.base.value, self.base.state, self.base.header.configuration, .{ .cache = self.base.contacts, .joint = self.base.joint_pool, .sleep = self.base.sleep, .ccd_enabled = self.base.ccd_enabled });
    }
};
/// Executes a recording through the caller's real snapshot loader and tick
/// stepper. The callbacks keep state ownership with the World host while this
/// format layer guarantees that the initial load happens before tick zero.
pub fn run(context: ?*anyopaque, recording: Recording, load: LoadFn, step: RunFn) anyerror!Result {
    try load(context, recording.initial_snapshot);
    return .{ .first_mismatch = try firstMismatch(context, recording.entries, step) };
}
/// Runs in canonical tick order and returns the first mismatch. A binary
/// search is only valid for a monotonic predicate; simulation divergence is
/// not monotonic, so this exact scan is the sound mismatch locator.
pub fn firstMismatch(context: ?*anyopaque, entries: []const Entry, step: RunFn) anyerror!?usize {
    for (entries, 0..) |entry, index| if (!std.mem.eql(u8, &entry.expected_hash, &(try step(context, entry)))) return index;
    return null;
}

/// Encodes a canonical World command batch for an Entry. The input is checked
/// in the same total order used by the World before any bytes are published.
pub fn encodeCommands(commands: []const world.Command, output: []u8) Error![]const u8 {
    var sizing = codec.Writer.sizing();
    try writeCommands(&sizing, commands);
    if (sizing.written() > output.len) return error.OutOfSpace;
    var writer = codec.Writer.init(output);
    try writeCommands(&writer, commands);
    return output[0..writer.written()];
}
pub fn decodeCommands(input: []const u8, output: []world.Command) Error![]const world.Command {
    var reader = codec.Reader.init(input);
    const count = try reader.unsigned(u32);
    if (count > output.len) return error.CapacityExceeded;
    var prior: ?world.CommandKey = null;
    for (output[0..count]) |*command| {
        const key = world.CommandKey{ .phase_priority = try reader.byte(), .issuer = try reader.unsigned(u32), .sequence = try reader.unsigned(u32), .discriminant = try reader.byte() };
        const body: ids.BodyId = .{ .value = try reader.unsigned(u64) };
        command.* = .{ .key = key, .op = switch (key.discriminant) {
            0 => .{ .force = .{ .body = body, .value = try reader.vec3() } },
            1 => .{ .torque = .{ .body = body, .value = try reader.vec3() } },
            2 => .{ .impulse_at_point = .{ .body = body, .impulse = try reader.vec3(), .point = try reader.vec3() } },
            3 => .{ .velocity = .{ .body = body, .linear = try reader.vec3(), .angular = try reader.vec3() } },
            4 => .{ .kinematic_target = .{ .body = body, .target = .{ .position = try reader.vec3(), .orientation = try reader.quat() } } },
            5 => .{ .locks = .{ .body = body, .value = @bitCast(try reader.byte()) } },
            else => return error.InvalidEnum,
        } };
        if (prior) |previous| if (!world.CommandKey.lessThan(previous, key)) return error.InvalidCommandOrder;
        prior = key;
    }
    try reader.finish();
    return output[0..count];
}
fn writeCommands(writer: *codec.Writer, commands: []const world.Command) Error!void {
    try writer.unsigned(u32, @intCast(commands.len));
    var prior: ?world.CommandKey = null;
    for (commands) |command| {
        const key = command.canonicalKey();
        if (prior) |previous| if (!world.CommandKey.lessThan(previous, key)) return error.InvalidCommandOrder;
        prior = key;
        try writer.byte(key.phase_priority);
        try writer.unsigned(u32, key.issuer);
        try writer.unsigned(u32, key.sequence);
        try writer.byte(key.discriminant);
        switch (command.op) {
            .force => |v| {
                try writer.unsigned(u64, v.body.value);
                try writer.vec3(v.value);
            },
            .torque => |v| {
                try writer.unsigned(u64, v.body.value);
                try writer.vec3(v.value);
            },
            .impulse_at_point => |v| {
                try writer.unsigned(u64, v.body.value);
                try writer.vec3(v.impulse);
                try writer.vec3(v.point);
            },
            .velocity => |v| {
                try writer.unsigned(u64, v.body.value);
                try writer.vec3(v.linear);
                try writer.vec3(v.angular);
            },
            .kinematic_target => |v| {
                try writer.unsigned(u64, v.body.value);
                try writer.vec3(v.target.position);
                try writer.quat(v.target.orientation);
            },
            .locks => |v| {
                try writer.unsigned(u64, v.body.value);
                try writer.byte(@bitCast(v.value));
            },
        }
    }
}
