const std = @import("std");
const gravity = @import("gravity");
const abi = gravity.abi;

fn expectOk(result: u32) !void {
    if (result != abi.ok) {
        std.debug.print("ABI error {d}: {s}\n", .{ result, abi.gravity_v1_result_string(result) });
        return error.AbiFailure;
    }
}

fn defaultTransform() abi.Transform {
    return .{ .position = .{ .x = 0, .y = 0, .z = 0 }, .orientation = .{ .x = 0, .y = 0, .z = 0, .w = 1 << 32 } };
}

fn badDispatch(_: ?*anyopaque, _: u32, _: abi.RunJobFn, _: ?*anyopaque) callconv(.c) u32 {
    return abi.ok;
}
fn duplicateDispatch(_: ?*anyopaque, _: u32, run: abi.RunJobFn, context: ?*anyopaque) callconv(.c) u32 {
    _ = run(context, 0);
    return run(context, 0);
}
fn reentrantDispatch(user: ?*anyopaque, _: u32, run: abi.RunJobFn, context: ?*anyopaque) callconv(.c) u32 {
    const world: *abi.World = @ptrCast(@alignCast(user.?));
    var tick: u64 = 0;
    if (abi.gravity_v1_world_tick(world, &tick) != abi.reentrant) return abi.internal;
    return run(context, 0);
}
fn countingDispatch(user: ?*anyopaque, job_count: u32, run: abi.RunJobFn, context: ?*anyopaque) callconv(.c) u32 {
    const batches: *u32 = @ptrCast(@alignCast(user.?));
    batches.* += 1;
    var index: u32 = 0;
    while (index < job_count) : (index += 1) {
        const result = run(context, index);
        if (result != abi.ok) return result;
    }
    return abi.ok;
}
const FailingDispatcher = struct { calls: u32 = 0, fail_at: u32 };
fn failingDispatch(user: ?*anyopaque, job_count: u32, run: abi.RunJobFn, context: ?*anyopaque) callconv(.c) u32 {
    const state: *FailingDispatcher = @ptrCast(@alignCast(user.?));
    state.calls += 1;
    if (state.calls == state.fail_at) return abi.callback_error;
    var index: u32 = 0;
    while (index < job_count) : (index += 1) {
        const result = run(context, index);
        if (result != abi.ok) return result;
    }
    return abi.ok;
}

test "caller-memory ABI drives World query hash and snapshot transaction" {
    const allocator = std.testing.allocator;
    var asset_desc = abi.AssetStoreDesc{ .struct_size = @sizeOf(abi.AssetStoreDesc), .reserved = 0, .assets = null, .asset_count = 0, .reserved1 = 0 };
    var asset_size: u64 = 0;
    var asset_alignment: u32 = 0;
    try expectOk(abi.gravity_v1_asset_store_memory_required(&asset_desc, &asset_size, &asset_alignment));
    try std.testing.expectEqual(@as(u32, @alignOf(abi.AssetStore)), asset_alignment);
    const asset_memory = try allocator.alignedAlloc(u8, .fromByteUnits(@alignOf(abi.AssetStore)), @intCast(asset_size));
    defer allocator.free(asset_memory);
    var assets: *abi.AssetStore = undefined;
    try expectOk(abi.gravity_v1_asset_store_init(asset_memory.ptr, asset_memory.len, &asset_desc, &assets));
    defer expectOk(abi.gravity_v1_asset_store_deinit(assets)) catch unreachable;

    var world_desc = abi.WorldDesc{
        .struct_size = @sizeOf(abi.WorldDesc),
        .reserved = 0,
        .body_capacity = 4,
        .collider_capacity = 4,
        .command_capacity = 4,
        .contact_capacity = 4,
        .gravity = .{ .x = 0, .y = 0, .z = 0 },
        .linear_damping = 0,
        .angular_damping = 0,
        .max_linear_speed = std.math.maxInt(i64),
        .max_angular_speed = std.math.maxInt(i64),
        .substeps = 2,
        .tick_hz = 60,
        .assets = assets,
    };
    var world_size: u64 = 0;
    var world_alignment: u32 = 0;
    try expectOk(abi.gravity_v1_world_memory_required(&world_desc, &world_size, &world_alignment));
    try std.testing.expectEqual(@as(u32, @alignOf(abi.World)), world_alignment);
    const world_memory = try allocator.alignedAlloc(u8, .fromByteUnits(@alignOf(abi.World)), @intCast(world_size));
    defer allocator.free(world_memory);
    var world: *abi.World = undefined;
    try expectOk(abi.gravity_v1_world_init(world_memory.ptr, world_memory.len, &world_desc, &world));
    defer expectOk(abi.gravity_v1_world_deinit(world)) catch unreachable;

    var body_desc = abi.BodyDesc{
        .struct_size = @sizeOf(abi.BodyDesc),
        .reserved = 0,
        .body_type = 1,
        .dof_locks = 0,
        .transform = defaultTransform(),
        .inverse_mass = 1 << 32,
        .inverse_inertia_xx = 1 << 32,
        .inverse_inertia_yy = 1 << 32,
        .inverse_inertia_zz = 1 << 32,
        .inverse_inertia_xy = 0,
        .inverse_inertia_xz = 0,
        .inverse_inertia_yz = 0,
    };
    var body: u64 = 0;
    try expectOk(abi.gravity_v1_world_create_body(world, &body_desc, &body));
    var reference_hash: abi.Hash128 = undefined;
    try expectOk(abi.gravity_v1_world_hash(world, &reference_hash));
    try std.testing.expectEqualStrings("4336297d3f06a9c557e75aea2a839853", &std.fmt.bytesToHex(reference_hash.bytes, .lower));
    var parity_snapshot_size: u64 = 0;
    try expectOk(abi.gravity_v1_world_snapshot_size(world, &parity_snapshot_size));
    const parity_snapshot = try allocator.alloc(u8, @intCast(parity_snapshot_size));
    defer allocator.free(parity_snapshot);
    var parity_snapshot_required: u64 = 0;
    try expectOk(abi.gravity_v1_world_snapshot_save(world, parity_snapshot.ptr, parity_snapshot.len, &parity_snapshot_required));
    try expectOk(abi.gravity_v1_world_step(world, null, 0));
    var replay_hash: abi.Hash128 = undefined;
    try expectOk(abi.gravity_v1_world_hash(world, &replay_hash));
    try std.testing.expectEqualStrings("3abdf5be432885c4b137c5367272516f", &std.fmt.bytesToHex(replay_hash.bytes, .lower));
    try expectOk(abi.gravity_v1_world_snapshot_load(world, parity_snapshot.ptr, parity_snapshot.len));
    var restored_reference_hash: abi.Hash128 = undefined;
    try expectOk(abi.gravity_v1_world_hash(world, &restored_reference_hash));
    try std.testing.expectEqualSlices(u8, &reference_hash.bytes, &restored_reference_hash.bytes);
    var collider_desc = abi.ColliderDesc{
        .struct_size = @sizeOf(abi.ColliderDesc),
        .reserved = 0,
        .body = body,
        .shape_kind = 0,
        .flags = 0,
        .local = defaultTransform(),
        .dimensions = .{ .x = 1 << 32, .y = 0, .z = 0 },
        .asset_source_id = 0,
        .friction = 1 << 32,
        .restitution = 0,
        .category = 1,
        .mask = std.math.maxInt(u32),
        .group = 0,
        .revision = 1,
    };
    var collider: u64 = 0;
    try expectOk(abi.gravity_v1_world_create_collider(world, &collider_desc, &collider));
    var static_desc = body_desc;
    static_desc.body_type = 0;
    static_desc.inverse_mass = 0;
    static_desc.inverse_inertia_xx = 0;
    static_desc.inverse_inertia_yy = 0;
    static_desc.inverse_inertia_zz = 0;
    static_desc.transform.position.x = (3 * (@as(i64, 1) << 32)) / 2;
    var static_body: u64 = 0;
    try expectOk(abi.gravity_v1_world_create_body(world, &static_desc, &static_body));
    collider_desc.body = static_body;
    var static_collider: u64 = 0;
    try expectOk(abi.gravity_v1_world_create_collider(world, &collider_desc, &static_collider));

    var point = abi.PointQuery{ .struct_size = @sizeOf(abi.PointQuery), .reserved = 0, .point = .{ .x = 0, .y = 0, .z = 0 }, .filter = .{ .category = 1, .mask = std.math.maxInt(u32), .group = 0, .reserved = 0 }, .mode = 2, .reserved1 = 0 };
    var hits: [2]abi.QueryHit = undefined;
    var required: u32 = 0;
    try expectOk(abi.gravity_v1_world_query_point(world, &point, &hits, hits.len, &required));
    try std.testing.expectEqual(@as(u32, 1), required);
    try std.testing.expectEqual(collider, hits[0].collider);

    var command = abi.Command{ .struct_size = @sizeOf(abi.Command), .reserved = 0, .type = 3, .phase_priority = 0, .issuer = 1, .sequence = 1, .body = body, .first = .{ .x = 1 << 32, .y = 0, .z = 0 }, .second = .{ .x = 0, .y = 0, .z = 0 }, .transform = defaultTransform(), .dof_locks = 0, .reserved1 = 0 };
    try expectOk(abi.gravity_v1_world_step(world, &command, 1));
    var tick: u64 = 0;
    try expectOk(abi.gravity_v1_world_tick(world, &tick));
    try std.testing.expectEqual(@as(u64, 1), tick);
    var event_count: u32 = 0;
    var events: [2]abi.Event = undefined;
    try expectOk(abi.gravity_v1_world_events(world, &events, events.len, &event_count));
    try std.testing.expectEqual(@as(u32, 1), event_count);
    try std.testing.expectEqual(collider, events[0].collider_a);
    try std.testing.expectEqual(static_collider, events[0].collider_b);
    var hash_before: abi.Hash128 = undefined;
    try expectOk(abi.gravity_v1_world_hash(world, &hash_before));

    var snapshot_size: u64 = 0;
    try expectOk(abi.gravity_v1_world_snapshot_size(world, &snapshot_size));
    const snapshot_memory = try allocator.alloc(u8, @intCast(snapshot_size));
    defer allocator.free(snapshot_memory);
    var snapshot_required: u64 = 0;
    try expectOk(abi.gravity_v1_world_snapshot_save(world, snapshot_memory.ptr, snapshot_memory.len, &snapshot_required));
    try std.testing.expectEqual(snapshot_size, snapshot_required);
    var too_small_required: u64 = 0;
    try std.testing.expectEqual(abi.buffer_too_small, abi.gravity_v1_world_snapshot_save(world, snapshot_memory.ptr, snapshot_memory.len - 1, &too_small_required));
    try std.testing.expectEqual(snapshot_size, too_small_required);
    command.sequence = 2;
    try expectOk(abi.gravity_v1_world_step(world, &command, 1));
    try expectOk(abi.gravity_v1_world_snapshot_load(world, snapshot_memory.ptr, snapshot_memory.len));
    var hash_after: abi.Hash128 = undefined;
    try expectOk(abi.gravity_v1_world_hash(world, &hash_after));
    try std.testing.expectEqualSlices(u8, &hash_before.bytes, &hash_after.bytes);
    snapshot_memory[0] ^= 0xff;
    try std.testing.expectEqual(abi.corrupt_input, abi.gravity_v1_world_snapshot_load(world, snapshot_memory.ptr, snapshot_memory.len));
    var hash_after_corrupt: abi.Hash128 = undefined;
    try expectOk(abi.gravity_v1_world_hash(world, &hash_after_corrupt));
    try std.testing.expectEqualSlices(u8, &hash_after.bytes, &hash_after_corrupt.bytes);
    snapshot_memory[0] ^= 0xff;

    var required_bodies: u32 = 0;
    var sentinel = abi.BodyState{ .struct_size = 0xdeadbeef, .reserved = 0, .id = 0, .body_type = 0, .dof_locks = 0, .transform = defaultTransform(), .linear_velocity = .{ .x = 0, .y = 0, .z = 0 }, .angular_velocity = .{ .x = 0, .y = 0, .z = 0 } };
    try std.testing.expectEqual(abi.buffer_too_small, abi.gravity_v1_world_body_states(world, &sentinel, 0, &required_bodies));
    try std.testing.expectEqual(@as(u32, 2), required_bodies);
    try std.testing.expectEqual(@as(u32, 0xdeadbeef), sentinel.struct_size);

    var bad_command = command;
    bad_command.reserved = 1;
    try std.testing.expectEqual(abi.bad_struct, abi.gravity_v1_world_step(world, &bad_command, 1));
    var tick_after: u64 = 0;
    try expectOk(abi.gravity_v1_world_tick(world, &tick_after));
    try std.testing.expectEqual(@as(u64, 1), tick_after);

    var dispatcher = abi.Dispatcher{ .struct_size = @sizeOf(abi.Dispatcher), .reserved = 0, .user = null, .dispatch_batch = badDispatch };
    var before_partial_failure: abi.Hash128 = undefined;
    try expectOk(abi.gravity_v1_world_hash(world, &before_partial_failure));
    var failing = FailingDispatcher{ .fail_at = 5 };
    dispatcher.user = &failing;
    dispatcher.dispatch_batch = failingDispatch;
    try expectOk(abi.gravity_v1_world_set_dispatcher(world, &dispatcher));
    try std.testing.expectEqual(abi.callback_error, abi.gravity_v1_world_step(world, &command, 1));
    var after_partial_failure: abi.Hash128 = undefined;
    try expectOk(abi.gravity_v1_world_hash(world, &after_partial_failure));
    try std.testing.expectEqualSlices(u8, &before_partial_failure.bytes, &after_partial_failure.bytes);
    try expectOk(abi.gravity_v1_world_tick(world, &tick_after));
    try std.testing.expectEqual(@as(u64, 1), tick_after);
    dispatcher.user = null;
    dispatcher.dispatch_batch = badDispatch;
    try expectOk(abi.gravity_v1_world_set_dispatcher(world, &dispatcher));
    const failure = abi.gravity_v1_world_step(world, &command, 1);
    try std.testing.expectEqual(abi.callback_error, failure);
    try expectOk(abi.gravity_v1_world_tick(world, &tick_after));
    try std.testing.expectEqual(@as(u64, 1), tick_after);
    dispatcher.dispatch_batch = duplicateDispatch;
    try expectOk(abi.gravity_v1_world_set_dispatcher(world, &dispatcher));
    try std.testing.expectEqual(abi.callback_error, abi.gravity_v1_world_step(world, &command, 1));
    dispatcher.user = world;
    dispatcher.dispatch_batch = reentrantDispatch;
    try expectOk(abi.gravity_v1_world_set_dispatcher(world, &dispatcher));
    try std.testing.expectEqual(abi.callback_error, abi.gravity_v1_world_step(world, &command, 1));
    try expectOk(abi.gravity_v1_world_tick(world, &tick_after));
    try std.testing.expectEqual(@as(u64, 1), tick_after);
    var phase_batches: u32 = 0;
    dispatcher.user = &phase_batches;
    dispatcher.dispatch_batch = countingDispatch;
    try expectOk(abi.gravity_v1_world_set_dispatcher(world, &dispatcher));
    try expectOk(abi.gravity_v1_world_step(world, &command, 1));
    // Task 24 partitions contact and row preparation into four additional
    // stable batches; the host callback observes the complete fixed plan.
    try std.testing.expectEqual(@as(u32, 18), phase_batches);
    try expectOk(abi.gravity_v1_world_tick(world, &tick_after));
    try std.testing.expectEqual(@as(u64, 2), tick_after);

    // Task 25 bounded ABI mutation corpus: every descriptor remains inside
    // controlled storage, while sizes, discriminants, reserved fields and
    // scalar envelopes cover their hostile wire values.
    var seed: u64 = 0x25ab_1f22;
    for (0..1_000) |_| {
        seed = seed *% 6_364_136_223_846_793_005 +% 1;
        var mutated_command = command;
        switch (seed % 5) {
            0 => mutated_command.struct_size = @truncate(seed),
            1 => mutated_command.reserved = @truncate(seed >> 32),
            2 => mutated_command.type = @truncate(seed >> 8),
            3 => mutated_command.dof_locks = @truncate(seed >> 16),
            else => mutated_command.first.x = (@as(i64, @intCast(seed % 101)) - 50) * (@as(i64, 1) << 32),
        }
        try std.testing.expect(abi.gravity_v1_world_step(world, &mutated_command, 1) != abi.internal);

        var mutated_query = point;
        switch ((seed >> 7) % 6) {
            0 => mutated_query.struct_size = @truncate(seed >> 3),
            1 => mutated_query.reserved = @truncate(seed >> 35),
            2 => mutated_query.mode = @truncate(seed >> 11),
            3 => mutated_query.reserved1 = @truncate(seed >> 43),
            4 => mutated_query.filter.reserved = @truncate(seed >> 27),
            else => mutated_query.point.x = @bitCast(seed),
        }
        try std.testing.expect(abi.gravity_v1_world_query_point(world, &mutated_query, &hits, hits.len, &required) != abi.internal);
    }

    // Mutated snapshots either load completely or leave the canonical hash
    // unchanged. Restore the valid baseline after every probe.
    try expectOk(abi.gravity_v1_world_snapshot_save(world, snapshot_memory.ptr, snapshot_memory.len, &snapshot_required));
    var mutation_baseline: abi.Hash128 = undefined;
    try expectOk(abi.gravity_v1_world_hash(world, &mutation_baseline));
    const mutation_length: usize = @intCast(snapshot_required);
    for (0..@min(mutation_length, 1_024)) |iteration| {
        const index = (iteration * 2_654_435_761) % mutation_length;
        snapshot_memory[index] ^= @truncate((iteration % 255) + 1);
        const result = abi.gravity_v1_world_snapshot_load(world, snapshot_memory.ptr, mutation_length);
        if (result != abi.ok) {
            var after_rejection: abi.Hash128 = undefined;
            try expectOk(abi.gravity_v1_world_hash(world, &after_rejection));
            try std.testing.expectEqualSlices(u8, &mutation_baseline.bytes, &after_rejection.bytes);
        }
        snapshot_memory[index] ^= @truncate((iteration % 255) + 1);
        try expectOk(abi.gravity_v1_world_snapshot_load(world, snapshot_memory.ptr, mutation_length));
    }
}
