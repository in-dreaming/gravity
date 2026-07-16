const std = @import("std");
const gravity = @import("gravity");
const codec = gravity.state.codec;
const hash = gravity.state.hash;
const snapshot = gravity.state.snapshot;
const rollback = gravity.state.rollback;
const replay = gravity.state.replay;
const state_diff = gravity.state.diff;
const fp = gravity.math.fp;
const geometry = gravity.math.geometry;
const shapes = gravity.collision.shapes;
const world_mod = gravity.dynamics.world;
const joints = gravity.dynamics.joints;
const sleeping = gravity.dynamics.sleeping;
const pipeline = gravity.dynamics.pipeline;

const BodyFixture = struct {
    types: [2]shapes.BodyType = undefined,
    position: [2]geometry.Vec3 = undefined,
    orientation: [2]geometry.Quat = undefined,
    linear: [2]geometry.Vec3 = undefined,
    angular: [2]geometry.Vec3 = undefined,
    mass: [2]fp.Fp = undefined,
    inertia: [2]geometry.SymmetricMat3 = undefined,
    force: [2]geometry.Vec3 = undefined,
    torque: [2]geometry.Vec3 = undefined,
    locks: [2]world_mod.DofLock = undefined,
    generation: [2]u32 = undefined,
    alive: [2]bool = undefined,
    retired: [2]bool = undefined,
    targets: [2]bool = undefined,
    target_position: [2]geometry.Vec3 = undefined,
    target_orientation: [2]geometry.Quat = undefined,
    fn init(self: *BodyFixture) !world_mod.World {
        return world_mod.World.init(.{ .body_type = &self.types, .position = &self.position, .orientation = &self.orientation, .linear_velocity = &self.linear, .angular_velocity = &self.angular, .inverse_mass = &self.mass, .inverse_inertia_local = &self.inertia, .force = &self.force, .torque = &self.torque, .locks = &self.locks, .generation = &self.generation, .alive = &self.alive, .retired = &self.retired, .has_target = &self.targets, .target_position = &self.target_position, .target_orientation = &self.target_orientation });
    }
};
fn unitInertia() geometry.SymmetricMat3 {
    return .{ .xx = .one, .yy = .one, .zz = .one, .xy = .zero, .xz = .zero, .yz = .zero };
}

/// Minimal complete logical World used by the integration-only rollback
/// corpus. It deliberately keeps contacts/joints empty; production replay
/// instead requires `FullWorldHost` with Task 20 workspaces.
const ReplayFixture = struct {
    body: BodyFixture = .{},
    stage_body: BodyFixture = .{},
    value: world_mod.World = undefined,
    stage_value: world_mod.World = undefined,
    state: pipeline.State = .{},
    status: fp.MathStatus = .{},
    joint_values: [0]joints.Joint = .{},
    joint_generation: [0]u32 = .{},
    joint_alive: [0]bool = .{},
    joint_retired: [0]bool = .{},
    joints_value: joints.Pool = undefined,
    stage_joint_values: [0]joints.Joint = .{},
    stage_joint_generation: [0]u32 = .{},
    stage_joint_alive: [0]bool = .{},
    stage_joint_retired: [0]bool = .{},
    stage_joints_value: joints.Pool = undefined,
    awake: [2]bool = [_]bool{true} ** 2,
    counter: [2]u32 = [_]u32{0} ** 2,
    reason: [2]sleeping.WakeReason = [_]sleeping.WakeReason{.none} ** 2,
    stage_awake: [2]bool = undefined,
    stage_counter: [2]u32 = undefined,
    stage_reason: [2]sleeping.WakeReason = undefined,
    ccd: [2]bool = [_]bool{false} ** 2,
    stage_ccd: [2]bool = undefined,
    patches: [0]gravity.collision.contact_cache.Patch = .{},
    contacts: gravity.collision.contact_cache.Cache = undefined,
    decoded_commands: [1]world_mod.Command = undefined,
    command_scratch: [1]world_mod.Command = undefined,
    trace: [4]pipeline.Phase = undefined,
    workspace: pipeline.Workspace = undefined,

    fn init(self: *ReplayFixture) !void {
        self.value = try self.body.init();
        self.stage_value = try self.stage_body.init();
        self.joints_value = try joints.Pool.init(.{ .values = &self.joint_values, .generation = &self.joint_generation, .alive = &self.joint_alive, .retired = &self.joint_retired });
        self.stage_joints_value = try joints.Pool.init(.{ .values = &self.stage_joint_values, .generation = &self.stage_joint_generation, .alive = &self.stage_joint_alive, .retired = &self.stage_joint_retired });
        self.contacts = .{ .patches = &self.patches };
        self.workspace = .{ .commands = &self.command_scratch, .trace = &self.trace };
    }

    fn stateHash(self: *ReplayFixture, configuration: gravity.core.config.SimulationConfig) hash.Hash128 {
        return pipeline.canonicalStateHash(&self.value, &self.state, configuration, .{ .cache = &self.contacts, .joint = &self.joints_value, .sleep = .{ .awake = &self.awake, .counter = &self.counter, .reason = &self.reason }, .ccd_enabled = &self.ccd });
    }

    fn host(self: *ReplayFixture, header: snapshot.Header) replay.IntegrationOnlyHost {
        const Rebuild = struct {
            fn run(_: ?*anyopaque) void {}
        };
        return .{ .header = header, .state = &self.state, .value = &self.value, .stage_world = &self.stage_value, .contacts = &self.contacts, .stage_contacts = &self.patches, .contact_scratch = &self.patches, .joint_pool = &self.joints_value, .stage_joint_pool = &self.stage_joints_value, .sleep = .{ .awake = &self.awake, .counter = &self.counter, .reason = &self.reason }, .stage_sleep = .{ .awake = &self.stage_awake, .counter = &self.stage_counter, .reason = &self.stage_reason }, .ccd_enabled = &self.ccd, .stage_ccd = &self.stage_ccd, .decoded_commands = &self.decoded_commands, .workspace = &self.workspace, .math_status = &self.status, .rebuild_context = null, .rebuild = Rebuild.run };
    }
};

test "canonical primitives and config size pass round trip" {
    const config = gravity.core.config.SimulationConfig.default;
    var size = codec.Writer.sizing();
    try codec.encodeConfig(&size, config);
    try std.testing.expectEqual(codec.config_encoded_size, size.written());
    var bytes: [codec.config_encoded_size]u8 = undefined;
    var writer = codec.Writer.init(&bytes);
    try codec.encodeConfig(&writer, config);
    try std.testing.expectEqual(size.written(), writer.written());
    var reader = codec.Reader.init(&bytes);
    const decoded = try codec.decodeConfig(&reader);
    try reader.finish();
    var again: [codec.config_encoded_size]u8 = undefined;
    var again_writer = codec.Writer.init(&again);
    try codec.encodeConfig(&again_writer, decoded);
    try std.testing.expectEqualSlices(u8, &bytes, &again);
    const expected = [_]u8{ 0xe5, 0x8a, 0xb8, 0xe8, 0x3e, 0xc6, 0x7a, 0x11, 0xac, 0xba, 0xbf, 0x35, 0xda, 0x59, 0x92, 0x2e, 0xc5, 0x45, 0x74, 0x17, 0x73, 0x7d, 0xcb, 0xf9, 0x85, 0x90, 0x54, 0x7d, 0x1c, 0x9a, 0x4b, 0x49 };
    try std.testing.expectEqualSlices(u8, &expected, &hash.oneShot256(.config, &bytes));
}

test "TLV rejects truncation ordering duplication and length bombs" {
    var payload = [_]u8{ 1, 2, 3 };
    var bytes: [32]u8 = undefined;
    var writer = codec.Writer.init(&bytes);
    try codec.writeHeader(&writer, 1, 2);
    try codec.writeSection(&writer, 2, &payload);
    try codec.writeSection(&writer, 3, &payload);
    try std.testing.expectError(error.DuplicateSection, codec.writeSection(&writer, 3, &payload));
    const Context = struct { calls: usize = 0 };
    const Visit = struct {
        fn visit(ctx: *Context, section: codec.Section) codec.Error!void {
            _ = section;
            ctx.calls += 1;
        }
    }.visit;
    var ctx = Context{};
    var reader = codec.Reader.init(bytes[0..writer.written()]);
    try codec.readSections(&reader, 1, Context, &ctx, Visit);
    try std.testing.expectEqual(@as(usize, 2), ctx.calls);
    var truncated = codec.Reader.init(bytes[0 .. writer.written() - 1]);
    try std.testing.expectError(error.EndOfInput, codec.readSections(&truncated, 1, Context, &ctx, Visit));
    var duplicate = bytes;
    duplicate[13] = 2;
    var duplicate_reader = codec.Reader.init(duplicate[0..writer.written()]);
    try std.testing.expectError(error.DuplicateSection, codec.readSections(&duplicate_reader, 1, Context, &ctx, Visit));
    var bomb = [_]u8{ 1, 0, 1, 0, 1, 0, 0, 0, 0xff, 0xff, 0xff, 0xff };
    var bomb_reader = codec.Reader.init(&bomb);
    try std.testing.expectError(error.SectionTooLarge, codec.readSections(&bomb_reader, 1, Context, &ctx, Visit));
    var required = [_]u8{ 1, 0, 1, 0, 1, 0x80, 0, 0, 0, 0 };
    var required_reader = codec.Reader.init(&required);
    try std.testing.expectError(error.UnknownRequiredSection, codec.readKnownSections(&required_reader, 1, &.{}, Context, &ctx, Visit));
}

test "bool enum-like bits and malformed config are rejected" {
    var bool_reader = codec.Reader.init(&[_]u8{2});
    try std.testing.expectError(error.InvalidBool, bool_reader.boolean());
    var data: [codec.config_encoded_size]u8 = undefined;
    var writer = codec.Writer.init(&data);
    try codec.encodeConfig(&writer, .default);
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    var reader = codec.Reader.init(&data);
    try std.testing.expectError(error.InvalidConfig, codec.decodeConfig(&reader));
}

test "domain hash streaming equals one shot and domains differ" {
    const input = "canonical payload";
    var sink = hash.Sink.init(.config);
    sink.update(input[0..4]);
    sink.update(input[4..]);
    const chunked = sink.final256();
    try std.testing.expectEqualSlices(u8, &hash.oneShot256(.config, input), &chunked);
    try std.testing.expect(!std.mem.eql(u8, &chunked, &hash.oneShot256(.state, input)));
    const short = hash.oneShot128(.config, input);
    try std.testing.expectEqualSlices(u8, short[0..], chunked[0..16]);
}

test "GRAVSNAP envelope round trips and rejects corrupt metadata" {
    const header = snapshot.Header{ .configuration = gravity.core.config.SimulationConfig.default, .asset_set = [_]u8{0xa5} ** 32 };
    var sizing = codec.Writer.sizing();
    try snapshot.writeHeader(&sizing, header);
    var bytes: [512]u8 = undefined;
    var writer = codec.Writer.init(&bytes);
    try snapshot.writeHeader(&writer, header);
    var reader = codec.Reader.init(bytes[0..writer.written()]);
    const decoded = try snapshot.readHeader(&reader);
    try reader.finish();
    try std.testing.expectEqualDeep(header, decoded);
    bytes[0] ^= 1;
    reader = codec.Reader.init(bytes[0..writer.written()]);
    try std.testing.expectError(error.InvalidMagic, snapshot.readHeader(&reader));
    try std.testing.expectError(error.HeaderMismatch, snapshot.validateHeader(header, .{ .configuration = .default, .asset_set = [_]u8{0} ** 32 }));
}

test "GRAVSNAP pipeline payload preserves fault and rejects invalid enum" {
    var state = gravity.dynamics.pipeline.State{ .tick = 9 };
    state.fault = .{ .tick = 8, .phase = .narrowphase, .object = 42, .code = .contact, .detail = .contact };
    var bytes: [64]u8 = undefined;
    var writer = codec.Writer.init(&bytes);
    try snapshot.writePipeline(&writer, state);
    var reader = codec.Reader.init(bytes[0..writer.written()]);
    const decoded = try snapshot.readPipeline(&reader);
    try reader.finish();
    try std.testing.expectEqualDeep(state.fault, decoded.fault);
    bytes[17] = 255;
    reader = codec.Reader.init(bytes[0..writer.written()]);
    try std.testing.expectError(error.InvalidEnum, snapshot.readPipeline(&reader));
}

test "GRAVSNAP pipeline snapshot uses required canonical section" {
    const header = snapshot.Header{ .configuration = gravity.core.config.SimulationConfig.default, .asset_set = [_]u8{7} ** 32 };
    const state = gravity.dynamics.pipeline.State{ .tick = 123 };
    var output: [512]u8 = undefined;
    var payload: [64]u8 = undefined;
    const bytes = try snapshot.encodePipelineSnapshot(header, state, &output, &payload);
    const decoded = try snapshot.decodePipelineSnapshot(bytes);
    try std.testing.expectEqualDeep(header, decoded.header);
    try std.testing.expectEqual(@as(u64, 123), decoded.state.tick);
}

test "GRAVSNAP pipeline and contacts sections restore atomically" {
    const header = snapshot.Header{ .configuration = gravity.core.config.SimulationConfig.default, .asset_set = [_]u8{9} ** 32 };
    var source_patches: [1]gravity.collision.contact_cache.Patch = undefined;
    const source = gravity.collision.contact_cache.Cache{ .patches = &source_patches };
    var output: [1024]u8 = undefined;
    var pipeline_payload: [64]u8 = undefined;
    var contacts_payload: [64]u8 = undefined;
    const bytes = try snapshot.encodePipelineContactsSnapshot(header, .{ .tick = 17 }, &source, &output, &pipeline_payload, &contacts_payload);
    var target_patches: [1]gravity.collision.contact_cache.Patch = undefined;
    var target = gravity.collision.contact_cache.Cache{ .patches = &target_patches };
    var stage: [1]gravity.collision.contact_cache.Patch = undefined;
    var scratch: [1]gravity.collision.contact_cache.Patch = undefined;
    const decoded = try snapshot.decodePipelineContactsSnapshot(bytes, &target, &stage, &scratch);
    try std.testing.expectEqual(@as(u64, 17), decoded.state.tick);
    try std.testing.expectEqual(@as(usize, 0), target.len);
}

test "full rollback ring preserves canonical snapshots and rejects stale slots" {
    var ticks: [2]u64 = undefined;
    var snapshot_lengths: [2]usize = undefined;
    var input_lengths: [2]usize = undefined;
    var hashes: [2]hash.Hash128 = undefined;
    var snapshots: [16]u8 = undefined;
    var inputs: [8]u8 = undefined;
    var valid: [2]bool = undefined;
    var ring = try rollback.Ring.init(&ticks, &snapshot_lengths, &input_lengths, &hashes, &snapshots, &inputs, &valid, 8, 4);
    const first_hash = [_]u8{1} ** 16;
    try ring.save(5, "snapshot", "cmd", first_hash);
    const first = try ring.get(5);
    try std.testing.expectEqualStrings("snapshot", first.snapshot);
    try std.testing.expectEqualStrings("cmd", first.input);
    try std.testing.expectEqualSlices(u8, &first_hash, &first.state_hash);
    try ring.save(7, "new", "", [_]u8{2} ** 16);
    try std.testing.expectError(error.MissingTick, ring.get(5));
    try std.testing.expectError(error.SnapshotTooLarge, ring.save(8, "too-large", "", first_hash));
}

test "120-tick rollback ring survives 100k deterministic save and restore probes" {
    var ticks: [120]u64 = undefined;
    var snapshot_lengths: [120]usize = undefined;
    var input_lengths: [120]usize = undefined;
    var hashes: [120]hash.Hash128 = undefined;
    var snapshots: [120 * 8]u8 = undefined;
    var inputs: [120 * 4]u8 = undefined;
    var valid: [120]bool = undefined;
    var ring = try rollback.Ring.init(&ticks, &snapshot_lengths, &input_lengths, &hashes, &snapshots, &inputs, &valid, 8, 4);
    var seed: u32 = 0x42c0ffee;
    for (0..100_000) |step| {
        seed = seed *% 1_664_525 +% 1_013_904_223;
        const tick: u64 = @intCast(step);
        var snapshot_bytes: [8]u8 = undefined;
        std.mem.writeInt(u64, &snapshot_bytes, tick ^ seed, .little);
        var input: [4]u8 = undefined;
        std.mem.writeInt(u32, &input, seed, .little);
        const state_hash = hash.oneShot128(.state, &snapshot_bytes);
        try ring.save(tick, &snapshot_bytes, &input, state_hash);
        const rollback_tick = tick - @min(tick, @as(u64, seed % 120));
        const record = try ring.get(rollback_tick);
        try std.testing.expectEqual(rollback_tick, record.tick);
        try std.testing.expectEqualSlices(u8, &hash.oneShot128(.state, record.snapshot), &record.state_hash);
    }
}

test "GRAVREPL canonical recording rejects reordered ticks and locates hash mismatch" {
    const entries = [_]replay.Entry{
        .{ .tick = 4, .input = "a", .expected_hash = [_]u8{4} ** 16 },
        .{ .tick = 5, .input = "bc", .expected_hash = [_]u8{5} ** 16 },
    };
    var bytes: [256]u8 = undefined;
    const encoded = try replay.encode(.{ .initial_snapshot = "snap", .entries = &entries }, &bytes);
    var decoded_entries: [2]replay.Entry = undefined;
    var arena: [32]u8 = undefined;
    const decoded = try replay.decode(encoded, &decoded_entries, &arena);
    try std.testing.expectEqualStrings("snap", decoded.initial_snapshot);
    try std.testing.expectEqualStrings("bc", decoded.entries[1].input);
    const Runner = struct {
        fn run(_: ?*anyopaque, entry: replay.Entry) !hash.Hash128 {
            return if (entry.tick == 5) [_]u8{9} ** 16 else entry.expected_hash;
        }
    };
    try std.testing.expectEqual(@as(?usize, 1), try replay.firstMismatch(null, decoded.entries, Runner.run));
    var reordered = entries;
    reordered[1].tick = 4;
    try std.testing.expectError(error.InvalidTickOrder, replay.encode(.{ .initial_snapshot = "snap", .entries = &reordered }, &bytes));
}

test "GRAVREPL runner loads the initial snapshot before stepping canonical ticks" {
    const entries = [_]replay.Entry{.{ .tick = 1, .input = "x", .expected_hash = [_]u8{1} ** 16 }};
    const Host = struct {
        loaded: bool = false,
        fn load(context: ?*anyopaque, initial: []const u8) !void {
            const host: *@This() = @ptrCast(@alignCast(context.?));
            try std.testing.expectEqualStrings("initial", initial);
            host.loaded = true;
        }
        fn step(context: ?*anyopaque, entry: replay.Entry) !hash.Hash128 {
            const host: *@This() = @ptrCast(@alignCast(context.?));
            try std.testing.expect(host.loaded);
            return entry.expected_hash;
        }
    };
    var host = Host{};
    const result = try replay.run(&host, .{ .initial_snapshot = "initial", .entries = &entries }, Host.load, Host.step);
    try std.testing.expect(result.first_mismatch == null);
}

test "GRAVREPL command batches are canonical and reject reordered commands" {
    const body: gravity.core.ids.BodyId = .{ .value = 7 };
    const commands = [_]world_mod.Command{
        .{ .key = .{ .phase_priority = 1, .issuer = 2, .sequence = 3, .discriminant = 0 }, .op = .{ .force = .{ .body = body, .value = .{ .x = .one } } } },
        .{ .key = .{ .phase_priority = 1, .issuer = 2, .sequence = 4, .discriminant = 0 }, .op = .{ .velocity = .{ .body = body, .linear = .{}, .angular = .{} } } },
    };
    var bytes: [256]u8 = undefined;
    const encoded = try replay.encodeCommands(&commands, &bytes);
    var decoded: [2]world_mod.Command = undefined;
    const round_trip = try replay.decodeCommands(encoded, &decoded);
    try std.testing.expectEqual(@as(usize, 2), round_trip.len);
    try std.testing.expectEqual(body.value, round_trip[0].op.force.body.value);
    var unordered = commands;
    unordered[1].key.sequence = 2;
    try std.testing.expectError(error.InvalidCommandOrder, replay.encodeCommands(&unordered, &bytes));
}

test "GRAVSNAP diff reports the exact canonical section payload byte" {
    const header = snapshot.Header{ .configuration = .default, .asset_set = [_]u8{1} ** 32 };
    var left: [512]u8 = undefined;
    var right: [512]u8 = undefined;
    var payload: [64]u8 = undefined;
    const a = try snapshot.encodePipelineSnapshot(header, .{ .tick = 2 }, &left, &payload);
    const b = try snapshot.encodePipelineSnapshot(header, .{ .tick = 3 }, &right, &payload);
    const difference = (try state_diff.first(a, b)).?;
    try std.testing.expectEqual(snapshot.pipeline_section, difference.section.?);
    try std.testing.expectEqual(state_diff.Field.pipeline_tick, difference.field);
    try std.testing.expectEqual(@as(usize, 0), difference.offset);
}

test "GRAVSNAP diff identifies the owning body slot" {
    var left_fixture: BodyFixture = .{};
    var left_world = try left_fixture.init();
    var right_fixture: BodyFixture = .{};
    var right_world = try right_fixture.init();
    var status = fp.MathStatus{};
    const body = try left_world.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia() }, &status);
    _ = try right_world.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia() }, &status);
    right_world.storage.linear_velocity[0].x = .one;
    var patches: [0]gravity.collision.contact_cache.Patch = .{};
    const contacts = gravity.collision.contact_cache.Cache{ .patches = &patches };
    var left: [2048]u8 = undefined;
    var right: [2048]u8 = undefined;
    var pipeline_payload: [64]u8 = undefined;
    var bodies_payload: [1024]u8 = undefined;
    var colliders_payload: [64]u8 = undefined;
    var contacts_payload: [64]u8 = undefined;
    const header = snapshot.Header{ .configuration = .default, .asset_set = [_]u8{0x12} ** 32 };
    const a = try snapshot.encodePipelineBodiesContactsSnapshot(header, .{}, &left_world, &contacts, &left, &pipeline_payload, &bodies_payload, &colliders_payload, &contacts_payload);
    const b = try snapshot.encodePipelineBodiesContactsSnapshot(header, .{}, &right_world, &contacts, &right, &pipeline_payload, &bodies_payload, &colliders_payload, &contacts_payload);
    const difference = (try state_diff.first(a, b)).?;
    try std.testing.expectEqual(snapshot.bodies_section, difference.section.?);
    try std.testing.expectEqual(@as(?u64, body.value), difference.id);
    try std.testing.expectEqual(state_diff.Field.body_linear_velocity_x, difference.field);
}

test "GRAVSNAP bodies section restores future state atomically" {
    var source_fixture: BodyFixture = .{};
    var source = try source_fixture.init();
    var status = fp.MathStatus{};
    const id = try source.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia(), .locks = .{ .linear_z = true } }, &status);
    source_fixture.force[id.index()] = .{ .x = .one };
    source_fixture.targets[id.index()] = true;
    source_fixture.target_position[id.index()] = .{ .y = .one };
    var bytes: [512]u8 = undefined;
    const encoded = try snapshot.encodeBodies(&source, &bytes);
    var target_fixture: BodyFixture = .{};
    var target = try target_fixture.init();
    var stage_fixture: BodyFixture = .{};
    var stage = try stage_fixture.init();
    try snapshot.decodeBodies(encoded, &target, &stage);
    try std.testing.expectEqualDeep(source.settings, target.settings);
    try std.testing.expectEqualDeep(source_fixture.force[id.index()], target_fixture.force[id.index()]);
    try std.testing.expectEqualDeep(source_fixture.target_position[id.index()], target_fixture.target_position[id.index()]);
    const before = target_fixture.position;
    var corrupt = bytes;
    // count (4), settings (56), slot metadata (6), type (1), position (24),
    // then x/y/z of the first quaternion (24): zero its stored w component.
    @memset(corrupt[115..123], 0);
    try std.testing.expectError(error.InvalidConfig, snapshot.decodeBodies(corrupt[0..encoded.len], &target, &stage));
    try std.testing.expectEqualDeep(before, target_fixture.position);
}

test "GRAVSNAP preserves deterministic body ID reuse generations" {
    var source_fixture: BodyFixture = .{};
    var source = try source_fixture.init();
    var status = fp.MathStatus{};
    const first = try source.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia() }, &status);
    _ = try source.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia() }, &status);
    try source.destroy(first);
    const reused = try source.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia() }, &status);
    try std.testing.expectEqual(first.index(), reused.index());
    try std.testing.expectEqual(first.generation() + 1, reused.generation());

    var bytes: [1024]u8 = undefined;
    const encoded = try snapshot.encodeBodies(&source, &bytes);
    var restored_fixture: BodyFixture = .{};
    var restored = try restored_fixture.init();
    var stage_fixture: BodyFixture = .{};
    var stage = try stage_fixture.init();
    try snapshot.decodeBodies(encoded, &restored, &stage);
    try std.testing.expectEqualSlices(u32, &source_fixture.generation, &restored_fixture.generation);

    try source.destroy(reused);
    try restored.destroy(reused);
    const source_next = try source.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia() }, &status);
    const restored_next = try restored.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia() }, &status);
    try std.testing.expectEqual(source_next.value, restored_next.value);
    try std.testing.expectEqual(reused.generation() + 1, restored_next.generation());
}

test "combined GRAVSNAP leaves body and contact targets intact on later-section failure" {
    var source_fixture: BodyFixture = .{};
    var source_world = try source_fixture.init();
    var status = fp.MathStatus{};
    _ = try source_world.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia() }, &status);
    var source_patches: [1]gravity.collision.contact_cache.Patch = undefined;
    const source_contacts = gravity.collision.contact_cache.Cache{ .patches = &source_patches };
    var bytes: [2048]u8 = undefined;
    var pipeline_payload: [64]u8 = undefined;
    var bodies_payload: [512]u8 = undefined;
    var colliders_payload: [128]u8 = undefined;
    var contacts_payload: [64]u8 = undefined;
    const encoded = try snapshot.encodePipelineBodiesContactsSnapshot(.{ .configuration = .default, .asset_set = [_]u8{0x3c} ** 32 }, .{ .tick = 21 }, &source_world, &source_contacts, &bytes, &pipeline_payload, &bodies_payload, &colliders_payload, &contacts_payload);
    var target_fixture: BodyFixture = .{};
    var target_world = try target_fixture.init();
    var stage_fixture: BodyFixture = .{};
    var stage_world = try stage_fixture.init();
    var target_patches: [1]gravity.collision.contact_cache.Patch = undefined;
    var target_contacts = gravity.collision.contact_cache.Cache{ .patches = &target_patches };
    var stage_patches: [1]gravity.collision.contact_cache.Patch = undefined;
    var scratch_patches: [1]gravity.collision.contact_cache.Patch = undefined;
    var corrupt = bytes;
    corrupt[encoded.len - 1] = 2;
    try std.testing.expectError(error.InvalidConfig, snapshot.decodePipelineBodiesContactsSnapshot(corrupt[0..encoded.len], &target_world, &stage_world, &target_contacts, &stage_patches, &scratch_patches));
    try std.testing.expect(!target_fixture.alive[0]);
    try std.testing.expectEqual(@as(usize, 0), target_contacts.len);
}

test "checked GRAVSNAP rejects wrong asset before world mutation" {
    var source_fixture: BodyFixture = .{};
    var source = try source_fixture.init();
    var status = fp.MathStatus{};
    _ = try source.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia() }, &status);
    var source_patches: [1]gravity.collision.contact_cache.Patch = undefined;
    const source_contacts = gravity.collision.contact_cache.Cache{ .patches = &source_patches };
    var bytes: [2048]u8 = undefined;
    var pipeline_payload: [64]u8 = undefined;
    var bodies_payload: [512]u8 = undefined;
    var colliders_payload: [128]u8 = undefined;
    var contacts_payload: [64]u8 = undefined;
    const encoded = try snapshot.encodePipelineBodiesContactsSnapshot(.{ .configuration = .default, .asset_set = [_]u8{0x42} ** 32 }, .{ .tick = 1 }, &source, &source_contacts, &bytes, &pipeline_payload, &bodies_payload, &colliders_payload, &contacts_payload);
    var target_fixture: BodyFixture = .{};
    var target = try target_fixture.init();
    var stage_fixture: BodyFixture = .{};
    var stage = try stage_fixture.init();
    var target_patches: [1]gravity.collision.contact_cache.Patch = undefined;
    var target_contacts = gravity.collision.contact_cache.Cache{ .patches = &target_patches };
    var stage_patches: [1]gravity.collision.contact_cache.Patch = undefined;
    var scratch: [1]gravity.collision.contact_cache.Patch = undefined;
    try std.testing.expectError(error.HeaderMismatch, snapshot.decodePipelineBodiesContactsSnapshotChecked(encoded, .{ .configuration = .default, .asset_set = [_]u8{0} ** 32 }, &target, &stage, &target_contacts, &stage_patches, &scratch));
    try std.testing.expect(!target_fixture.alive[0]);
}

test "full GRAVSNAP commits joint sleep CCD and pipeline only after every section validates" {
    var source_fixture: BodyFixture = .{};
    var source = try source_fixture.init();
    var math_status = fp.MathStatus{};
    const a = try source.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia() }, &math_status);
    const b = try source.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia() }, &math_status);
    source_fixture.position[b.index()] = .{ .x = .one };
    var source_joint_values: [1]joints.Joint = undefined;
    var source_joint_generation: [1]u32 = undefined;
    var source_joint_alive: [1]bool = undefined;
    var source_joint_retired: [1]bool = undefined;
    var source_joints = try joints.Pool.init(.{ .values = &source_joint_values, .generation = &source_joint_generation, .alive = &source_joint_alive, .retired = &source_joint_retired });
    _ = try source_joints.create(&source, .{ .kind = .distance, .body_a = a, .body_b = b }, &math_status);
    var source_awake = [_]bool{ false, true };
    var source_counter = [_]u32{ 9, 3 };
    var source_reason = [_]sleeping.WakeReason{ .joint, .contact };
    const source_sleep = sleeping.Storage{ .awake = &source_awake, .counter = &source_counter, .reason = &source_reason };
    const source_ccd = [_]bool{ true, false };
    var source_patches: [1]gravity.collision.contact_cache.Patch = undefined;
    const source_contacts = gravity.collision.contact_cache.Cache{ .patches = &source_patches };
    var output: [8192]u8 = undefined;
    var p: [64]u8 = undefined;
    var bodies: [1024]u8 = undefined;
    var colliders: [128]u8 = undefined;
    var contacts: [64]u8 = undefined;
    var joints_bytes: [512]u8 = undefined;
    var sleep_bytes: [64]u8 = undefined;
    var ccd_bytes: [64]u8 = undefined;
    const encoded = try snapshot.encodeFullSnapshot(.{ .configuration = .default, .asset_set = [_]u8{0xa1} ** 32 }, .{ .tick = 77 }, &source, &source_contacts, &source_joints, source_sleep, &source_ccd, &output, &p, &bodies, &colliders, &contacts, &joints_bytes, &sleep_bytes, &ccd_bytes);
    var target_fixture: BodyFixture = .{};
    var target = try target_fixture.init();
    var stage_fixture: BodyFixture = .{};
    var stage = try stage_fixture.init();
    var target_joint_values: [1]joints.Joint = undefined;
    var target_joint_generation: [1]u32 = undefined;
    var target_joint_alive: [1]bool = undefined;
    var target_joint_retired: [1]bool = undefined;
    var target_joints = try joints.Pool.init(.{ .values = &target_joint_values, .generation = &target_joint_generation, .alive = &target_joint_alive, .retired = &target_joint_retired });
    var stage_joint_values: [1]joints.Joint = undefined;
    var stage_joint_generation: [1]u32 = undefined;
    var stage_joint_alive: [1]bool = undefined;
    var stage_joint_retired: [1]bool = undefined;
    var stage_joints = try joints.Pool.init(.{ .values = &stage_joint_values, .generation = &stage_joint_generation, .alive = &stage_joint_alive, .retired = &stage_joint_retired });
    var target_awake = [_]bool{true} ** 2;
    var target_counter = [_]u32{0} ** 2;
    var target_reason = [_]sleeping.WakeReason{.none} ** 2;
    var stage_awake: [2]bool = undefined;
    var stage_counter: [2]u32 = undefined;
    var stage_reason: [2]sleeping.WakeReason = undefined;
    var target_ccd = [_]bool{false} ** 2;
    var stage_ccd: [2]bool = undefined;
    var target_patches: [1]gravity.collision.contact_cache.Patch = undefined;
    var target_contacts = gravity.collision.contact_cache.Cache{ .patches = &target_patches };
    var stage_patches: [1]gravity.collision.contact_cache.Patch = undefined;
    var contact_scratch: [1]gravity.collision.contact_cache.Patch = undefined;
    var state = gravity.dynamics.pipeline.State{};
    const decoded = try snapshot.decodeFullSnapshot(encoded, .{ .configuration = .default, .asset_set = [_]u8{0xa1} ** 32 }, &state, &target, &stage, &target_contacts, &stage_patches, &contact_scratch, &target_joints, &stage_joints, .{ .awake = &target_awake, .counter = &target_counter, .reason = &target_reason }, .{ .awake = &stage_awake, .counter = &stage_counter, .reason = &stage_reason }, &target_ccd, &stage_ccd);
    try std.testing.expectEqual(@as(u64, 77), decoded.state.tick);
    try std.testing.expectEqual(@as(u64, 77), state.tick);
    try std.testing.expect(target_joint_alive[0]);
    try std.testing.expectEqual(@as(u32, 9), target_counter[0]);
    try std.testing.expect(target_ccd[0]);
    var reencoded_bytes: [8192]u8 = undefined;
    const reencoded = try snapshot.encodeFullSnapshot(.{ .configuration = .default, .asset_set = [_]u8{0xa1} ** 32 }, state, &target, &target_contacts, &target_joints, .{ .awake = &target_awake, .counter = &target_counter, .reason = &target_reason }, &target_ccd, &reencoded_bytes, &p, &bodies, &colliders, &contacts, &joints_bytes, &sleep_bytes, &ccd_bytes);
    try std.testing.expectEqualSlices(u8, encoded, reencoded);
    state.tick = 999;
    target_joint_alive[0] = false;
    target_counter[0] = 100;
    target_ccd[0] = false;
    var corrupt = output;
    corrupt[encoded.len - 1] = 2;
    try std.testing.expectError(error.InvalidConfig, snapshot.decodeFullSnapshot(corrupt[0..encoded.len], .{ .configuration = .default, .asset_set = [_]u8{0xa1} ** 32 }, &state, &target, &stage, &target_contacts, &stage_patches, &contact_scratch, &target_joints, &stage_joints, .{ .awake = &target_awake, .counter = &target_counter, .reason = &target_reason }, .{ .awake = &stage_awake, .counter = &stage_counter, .reason = &stage_reason }, &target_ccd, &stage_ccd));
    try std.testing.expectEqual(@as(u64, 999), state.tick);
    try std.testing.expect(!target_joint_alive[0]);
    try std.testing.expectEqual(@as(u32, 100), target_counter[0]);
    try std.testing.expect(!target_ccd[0]);
}

test "GRAVSNAP load rebuilds then matches continuous World steps per tick" {
    var continuous_fixture: BodyFixture = .{};
    var continuous = try continuous_fixture.init();
    var status = fp.MathStatus{};
    const body = try continuous.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia() }, &status);
    continuous.storage.linear_velocity[body.index()] = .{ .x = .one };
    var continuous_state = pipeline.State{};
    var commands: [0]world_mod.Command = .{};
    var trace: [4]pipeline.Phase = undefined;
    var workspace = pipeline.Workspace{ .commands = &commands, .trace = &trace };
    for (0..3) |_| _ = try pipeline.stepBodies(&continuous, &continuous_state, .default, &.{}, &workspace, &status);

    var empty_joint_values: [0]joints.Joint = .{};
    var empty_joint_generation: [0]u32 = .{};
    var empty_joint_alive: [0]bool = .{};
    var empty_joint_retired: [0]bool = .{};
    var continuous_joints = try joints.Pool.init(.{ .values = &empty_joint_values, .generation = &empty_joint_generation, .alive = &empty_joint_alive, .retired = &empty_joint_retired });
    var continuous_awake = [_]bool{true} ** 2;
    var continuous_counter = [_]u32{0} ** 2;
    var continuous_reason = [_]sleeping.WakeReason{.none} ** 2;
    var no_ccd: [0]bool = .{};
    var no_patches: [0]gravity.collision.contact_cache.Patch = .{};
    var continuous_contacts = gravity.collision.contact_cache.Cache{ .patches = &no_patches };
    var bytes: [4096]u8 = undefined;
    var pipeline_bytes: [64]u8 = undefined;
    var body_bytes: [1024]u8 = undefined;
    var collider_bytes: [64]u8 = undefined;
    var contact_bytes: [64]u8 = undefined;
    var joint_bytes: [64]u8 = undefined;
    var sleep_bytes: [64]u8 = undefined;
    var ccd_bytes: [64]u8 = undefined;
    const header = snapshot.Header{ .configuration = .default, .asset_set = [_]u8{0x33} ** 32 };
    const encoded = try snapshot.encodeFullSnapshot(header, continuous_state, &continuous, &continuous_contacts, &continuous_joints, .{ .awake = &continuous_awake, .counter = &continuous_counter, .reason = &continuous_reason }, &no_ccd, &bytes, &pipeline_bytes, &body_bytes, &collider_bytes, &contact_bytes, &joint_bytes, &sleep_bytes, &ccd_bytes);

    var resumed_fixture: BodyFixture = .{};
    var resumed = try resumed_fixture.init();
    var stage_fixture: BodyFixture = .{};
    var stage = try stage_fixture.init();
    var resumed_joint_values: [0]joints.Joint = .{};
    var resumed_joint_generation: [0]u32 = .{};
    var resumed_joint_alive: [0]bool = .{};
    var resumed_joint_retired: [0]bool = .{};
    var resumed_joints = try joints.Pool.init(.{ .values = &resumed_joint_values, .generation = &resumed_joint_generation, .alive = &resumed_joint_alive, .retired = &resumed_joint_retired });
    var stage_joint_values: [0]joints.Joint = .{};
    var stage_joint_generation: [0]u32 = .{};
    var stage_joint_alive: [0]bool = .{};
    var stage_joint_retired: [0]bool = .{};
    var stage_joints = try joints.Pool.init(.{ .values = &stage_joint_values, .generation = &stage_joint_generation, .alive = &stage_joint_alive, .retired = &stage_joint_retired });
    var resumed_awake: [2]bool = undefined;
    var resumed_counter: [2]u32 = undefined;
    var resumed_reason: [2]sleeping.WakeReason = undefined;
    var stage_awake: [2]bool = undefined;
    var stage_counter: [2]u32 = undefined;
    var stage_reason: [2]sleeping.WakeReason = undefined;
    var resumed_contacts = gravity.collision.contact_cache.Cache{ .patches = &no_patches };
    var resumed_state = pipeline.State{};
    var rebuilt = false;
    const Rebuild = struct {
        fn run(context: ?*anyopaque) void {
            @as(*bool, @ptrCast(@alignCast(context.?))).* = true;
        }
    };
    _ = try snapshot.decodeFullSnapshotAndRebuild(encoded, header, &resumed_state, &resumed, &stage, &resumed_contacts, &no_patches, &no_patches, &resumed_joints, &stage_joints, .{ .awake = &resumed_awake, .counter = &resumed_counter, .reason = &resumed_reason }, .{ .awake = &stage_awake, .counter = &stage_counter, .reason = &stage_reason }, &no_ccd, &no_ccd, &rebuilt, Rebuild.run);
    try std.testing.expect(rebuilt);
    for (0..5) |_| {
        _ = try pipeline.stepBodies(&continuous, &continuous_state, .default, &.{}, &workspace, &status);
        _ = try pipeline.stepBodies(&resumed, &resumed_state, .default, &.{}, &workspace, &status);
        const continuous_hash = pipeline.canonicalStateHash(&continuous, &continuous_state, .default, .{ .cache = &continuous_contacts });
        const resumed_hash = pipeline.canonicalStateHash(&resumed, &resumed_state, .default, .{ .cache = &resumed_contacts });
        try std.testing.expectEqualSlices(u8, &continuous_hash, &resumed_hash);
    }
}

test "GRAVREPL integration-only bridge loads a complete snapshot and hashes World steps" {
    var source_fixture: BodyFixture = .{};
    var source = try source_fixture.init();
    var status = fp.MathStatus{};
    const body = try source.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia() }, &status);
    source.storage.linear_velocity[body.index()] = .{ .x = .one };
    var source_state = pipeline.State{};
    var source_joint_values: [0]joints.Joint = .{};
    var source_joint_generation: [0]u32 = .{};
    var source_joint_alive: [0]bool = .{};
    var source_joint_retired: [0]bool = .{};
    var source_joints = try joints.Pool.init(.{ .values = &source_joint_values, .generation = &source_joint_generation, .alive = &source_joint_alive, .retired = &source_joint_retired });
    var source_awake = [_]bool{true} ** 2;
    var source_counter = [_]u32{0} ** 2;
    var source_reason = [_]sleeping.WakeReason{.none} ** 2;
    var source_ccd = [_]bool{false} ** 2;
    var no_patches: [0]gravity.collision.contact_cache.Patch = .{};
    var source_contacts = gravity.collision.contact_cache.Cache{ .patches = &no_patches };
    const header = snapshot.Header{ .configuration = .default, .asset_set = [_]u8{0x68} ** 32 };
    var snapshot_bytes: [4096]u8 = undefined;
    var pipeline_bytes: [64]u8 = undefined;
    var bodies_bytes: [1024]u8 = undefined;
    var colliders_bytes: [64]u8 = undefined;
    var contacts_bytes: [64]u8 = undefined;
    var joints_bytes: [64]u8 = undefined;
    var sleep_bytes: [64]u8 = undefined;
    var ccd_bytes: [64]u8 = undefined;
    const initial = try snapshot.encodeFullSnapshot(header, source_state, &source, &source_contacts, &source_joints, .{ .awake = &source_awake, .counter = &source_counter, .reason = &source_reason }, &source_ccd, &snapshot_bytes, &pipeline_bytes, &bodies_bytes, &colliders_bytes, &contacts_bytes, &joints_bytes, &sleep_bytes, &ccd_bytes);
    var no_commands: [0]world_mod.Command = .{};
    var source_scratch: [1]world_mod.Command = undefined;
    var source_trace: [4]pipeline.Phase = undefined;
    var source_workspace = pipeline.Workspace{ .commands = &source_scratch, .trace = &source_trace };
    _ = try pipeline.stepBodies(&source, &source_state, .default, &no_commands, &source_workspace, &status);
    const expected = pipeline.canonicalStateHash(&source, &source_state, .default, .{ .cache = &source_contacts, .joint = &source_joints, .sleep = .{ .awake = &source_awake, .counter = &source_counter, .reason = &source_reason }, .ccd_enabled = &source_ccd });
    var command_bytes: [4]u8 = undefined;
    const input = try replay.encodeCommands(&no_commands, &command_bytes);
    const entries = [_]replay.Entry{.{ .tick = 1, .input = input, .expected_hash = expected }};

    var target_fixture: BodyFixture = .{};
    var target = try target_fixture.init();
    var stage_fixture: BodyFixture = .{};
    var stage = try stage_fixture.init();
    var target_joint_values: [0]joints.Joint = .{};
    var target_joint_generation: [0]u32 = .{};
    var target_joint_alive: [0]bool = .{};
    var target_joint_retired: [0]bool = .{};
    var target_joints = try joints.Pool.init(.{ .values = &target_joint_values, .generation = &target_joint_generation, .alive = &target_joint_alive, .retired = &target_joint_retired });
    var stage_joint_values: [0]joints.Joint = .{};
    var stage_joint_generation: [0]u32 = .{};
    var stage_joint_alive: [0]bool = .{};
    var stage_joint_retired: [0]bool = .{};
    var stage_joints = try joints.Pool.init(.{ .values = &stage_joint_values, .generation = &stage_joint_generation, .alive = &stage_joint_alive, .retired = &stage_joint_retired });
    var target_awake: [2]bool = undefined;
    var target_counter: [2]u32 = undefined;
    var target_reason: [2]sleeping.WakeReason = undefined;
    var stage_awake: [2]bool = undefined;
    var stage_counter: [2]u32 = undefined;
    var stage_reason: [2]sleeping.WakeReason = undefined;
    var target_ccd: [2]bool = undefined;
    var stage_ccd: [2]bool = undefined;
    var target_contacts = gravity.collision.contact_cache.Cache{ .patches = &no_patches };
    var target_state = pipeline.State{};
    var decoded_commands: [1]world_mod.Command = undefined;
    var target_scratch: [1]world_mod.Command = undefined;
    var target_trace: [4]pipeline.Phase = undefined;
    var target_workspace = pipeline.Workspace{ .commands = &target_scratch, .trace = &target_trace };
    const Rebuild = struct {
        fn run(_: ?*anyopaque) void {}
    };
    var host = replay.IntegrationOnlyHost{ .header = header, .state = &target_state, .value = &target, .stage_world = &stage, .contacts = &target_contacts, .stage_contacts = &no_patches, .contact_scratch = &no_patches, .joint_pool = &target_joints, .stage_joint_pool = &stage_joints, .sleep = .{ .awake = &target_awake, .counter = &target_counter, .reason = &target_reason }, .stage_sleep = .{ .awake = &stage_awake, .counter = &stage_counter, .reason = &stage_reason }, .ccd_enabled = &target_ccd, .stage_ccd = &stage_ccd, .decoded_commands = &decoded_commands, .workspace = &target_workspace, .math_status = &status, .rebuild_context = null, .rebuild = Rebuild.run };
    const result = try replay.run(&host, .{ .initial_snapshot = initial, .entries = &entries }, replay.IntegrationOnlyHost.load, replay.IntegrationOnlyHost.step);
    try std.testing.expectEqual(@as(?usize, null), result.first_mismatch);
}

test "100k random full World rollback restores and resimulates the next tick" {
    const header = snapshot.Header{ .configuration = .default, .asset_set = [_]u8{0x91} ** 32 };
    var continuous: ReplayFixture = .{};
    try continuous.init();
    const body = try continuous.value.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia() }, &continuous.status);
    continuous.value.storage.linear_velocity[body.index()] = .{ .x = .one };
    var restored: ReplayFixture = .{};
    try restored.init();
    var ticks: [120]u64 = undefined;
    var snapshot_lengths: [120]usize = undefined;
    var input_lengths: [120]usize = undefined;
    var hashes: [120]hash.Hash128 = undefined;
    var snapshots: [120 * 4096]u8 = undefined;
    var inputs: [120 * 4]u8 = undefined;
    var valid: [120]bool = undefined;
    var ring = try rollback.Ring.init(&ticks, &snapshot_lengths, &input_lengths, &hashes, &snapshots, &inputs, &valid, 4096, 4);
    var command_bytes: [4]u8 = undefined;
    const no_commands: [0]world_mod.Command = .{};
    const input = try replay.encodeCommands(&no_commands, &command_bytes);
    var output: [4096]u8 = undefined;
    var pipeline_bytes: [64]u8 = undefined;
    var body_bytes: [1024]u8 = undefined;
    var collider_bytes: [64]u8 = undefined;
    var contact_bytes: [64]u8 = undefined;
    var joint_bytes: [64]u8 = undefined;
    var sleep_bytes: [64]u8 = undefined;
    var ccd_bytes: [64]u8 = undefined;
    var seed: u32 = 0x91c0_ffee;
    for (1..100_001) |tick_index| {
        _ = try pipeline.stepBodies(&continuous.value, &continuous.state, header.configuration, &no_commands, &continuous.workspace, &continuous.status);
        const encoded = try snapshot.encodeFullSnapshot(header, continuous.state, &continuous.value, &continuous.contacts, &continuous.joints_value, .{ .awake = &continuous.awake, .counter = &continuous.counter, .reason = &continuous.reason }, &continuous.ccd, &output, &pipeline_bytes, &body_bytes, &collider_bytes, &contact_bytes, &joint_bytes, &sleep_bytes, &ccd_bytes);
        const tick: u64 = @intCast(tick_index);
        try ring.save(tick, encoded, input, continuous.stateHash(header.configuration));
        if (tick < 2) continue;
        seed = seed *% 1_664_525 +% 1_013_904_223;
        // Both the restored tick and its one-tick successor must remain in
        // the 120-slot ring, so the furthest valid restore is tick - 119.
        const span = @min(tick - 2, @as(u64, 118));
        const rollback_tick = tick - 1 - @as(u64, seed % @as(u32, @intCast(span + 1)));
        const before = try ring.get(rollback_tick);
        const expected = try ring.get(rollback_tick + 1);
        restored.status = .{};
        var host = restored.host(header);
        try replay.IntegrationOnlyHost.load(&host, before.snapshot);
        const actual = try replay.IntegrationOnlyHost.step(&host, .{ .tick = rollback_tick + 1, .input = expected.input, .expected_hash = expected.state_hash });
        try std.testing.expectEqualSlices(u8, &expected.state_hash, &actual);
    }
}
