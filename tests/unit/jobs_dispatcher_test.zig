const std = @import("std");
const builtin = @import("builtin");
const contract = @import("gravity_jobs");
const spindle_adapter = @import("gravity_spindle_jobs");
const host_adapter = @import("gravity_host_jobs");
const spindle = @import("spindle_executor");
const abi = @import("gravity").abi;

const Context = struct {
    counts: []std.atomic.Value(u32),
    fail_index: ?u32 = null,
};
fn run(raw: *anyopaque, index: u32) !void {
    const context: *Context = @ptrCast(@alignCast(raw));
    if (context.fail_index == index) return error.Intentional;
    _ = context.counts[index].fetchAdd(1, .monotonic);
}

test "Spindle adapter preserves logical ownership across repeated batch reuses" {
    var executor = try spindle.executor.WorkStealingExecutor.init(std.testing.allocator, .{ .workers = 1, .local_capacity = 32, .injection_capacity = 32, .urgent_capacity = 16 });
    errdefer executor.deinit();
    var slots: [8]spindle_adapter.Dispatcher.Slot = undefined;
    var adapter = spindle_adapter.Dispatcher.init(&executor, &slots, 16);
    var counts: [8]std.atomic.Value(u32) = undefined;
    for (&counts) |*value| value.* = std.atomic.Value(u32).init(0);
    var context = Context{ .counts = &counts };
    const custom = adapter.custom();
    const dispatcher = contract.Dispatcher{ .custom = &custom };
    try dispatcher.dispatch(.{ .context = &context, .job_count = 8, .run = run });
    const batches: u32 = 32;
    for (0..batches) |_| try dispatcher.dispatch(.{ .context = &context, .job_count = 1, .run = run });
    try std.testing.expectEqual(batches + 1, counts[0].load(.monotonic));
    for (counts[1..]) |value| try std.testing.expectEqual(@as(u32, 1), value.load(.monotonic));
    for (slots) |slot| try std.testing.expect(slot.task.status() == .completed);
    executor.shutdown(.drain);
    executor.shutdown(.drain);
    executor.deinit();
}

test "Spindle Task slab survives one million strict batch lifecycle reuses" {
    const Probe = struct {
        value: *u64,
        fn run(task: *spindle.executor.Task) void {
            const self: *@This() = @ptrCast(@alignCast(task.context.?));
            self.value.* += 1;
        }
    };
    var value: u64 = 0;
    var probe = Probe{ .value = &value };
    var task = spindle.executor.Task.init(Probe.run, &probe);
    var previous_generation = task.generation.load(.acquire);
    const iterations: u64 = if (builtin.mode == .ReleaseFast) 1_000_000 else 10_000;
    for (0..iterations) |_| {
        try std.testing.expect(task.tryQueue());
        task.retainQueueReference();
        task.execute();
        task.releaseQueueReference();
        try task.wait();
        try task.waitQueueReleased();
        try task.reset();
        const generation = task.generation.load(.acquire);
        try std.testing.expect(generation != previous_generation);
        previous_generation = generation;
        try std.testing.expectEqual(@as(u32, 0), task.queue_references.load(.acquire));
    }
    try std.testing.expectEqual(iterations, value);
}

test "FixedPool baseline and DeterministicExecutor replay stay tooling-only" {
    const Probe = struct {
        value: *std.atomic.Value(u32),
        fn run(task: *spindle.executor.Task) void {
            const self: *@This() = @ptrCast(@alignCast(task.context.?));
            _ = self.value.fetchAdd(1, .monotonic);
        }
    };
    var value = std.atomic.Value(u32).init(0);
    var probe = Probe{ .value = &value };
    var pool = try spindle.executor.FixedPool.init(std.testing.allocator, 2, 8);
    var pool_tasks: [4]spindle.executor.Task = undefined;
    for (&pool_tasks) |*task| {
        task.* = spindle.executor.Task.init(Probe.run, &probe);
        try pool.submit(task, .{});
    }
    for (&pool_tasks) |*task| {
        try task.wait();
        try task.waitQueueReleased();
    }
    pool.shutdown(.drain);
    pool.deinit();
    try std.testing.expectEqual(@as(u32, 4), value.load(.monotonic));

    var recorded = spindle.executor.DeterministicExecutor.init(std.testing.allocator);
    defer recorded.deinit();
    var record_tasks = [_]spindle.executor.Task{ spindle.executor.Task.init(Probe.run, &probe), spindle.executor.Task.init(Probe.run, &probe) };
    try recorded.submitWithId(&record_tasks[0], 11);
    try recorded.submitWithId(&record_tasks[1], 29);
    try recorded.run();
    var log = try recorded.recordLog();
    defer log.deinit(std.testing.allocator);
    var replay = try spindle.executor.DeterministicExecutor.initReplay(std.testing.allocator, &log);
    defer replay.deinit();
    var replay_tasks = [_]spindle.executor.Task{ spindle.executor.Task.init(Probe.run, &probe), spindle.executor.Task.init(Probe.run, &probe) };
    try replay.submitWithId(&replay_tasks[0], 11);
    try replay.submitWithId(&replay_tasks[1], 29);
    try replay.run();
    try replay.finishReplay();
}

test "cancelled intrusive Task waits releases and resets without stale generation" {
    const Noop = struct {
        fn run(_: *spindle.executor.Task) void {}
    };
    var task = spindle.executor.Task.init(Noop.run, null);
    const handle = task.handle();
    try std.testing.expect(task.tryQueue());
    task.retainQueueReference();
    try std.testing.expect(task.cancel());
    task.releaseQueueReference();
    try task.wait();
    try task.waitQueueReleased();
    try task.reset();
    try std.testing.expect(!handle.isValid());
    try std.testing.expectEqual(spindle.executor.TaskState.created, task.status());
}

test "Spindle adapter rejects callback failure without a serial fallback" {
    var executor = try spindle.executor.WorkStealingExecutor.init(std.testing.allocator, .{ .workers = 2, .local_capacity = 8, .injection_capacity = 8, .urgent_capacity = 4 });
    errdefer executor.deinit();
    var slots: [2]spindle_adapter.Dispatcher.Slot = undefined;
    var adapter = spindle_adapter.Dispatcher.init(&executor, &slots, 4);
    var counts = [_]std.atomic.Value(u32){ std.atomic.Value(u32).init(0), std.atomic.Value(u32).init(0) };
    var context = Context{ .counts = &counts, .fail_index = 1 };
    const custom = adapter.custom();
    const dispatcher = contract.Dispatcher{ .custom = &custom };
    try std.testing.expectError(error.CallbackFailed, dispatcher.dispatch(.{ .context = &context, .job_count = 2, .run = run }));
    try std.testing.expectEqual(@as(u32, 1), counts[0].load(.monotonic));
    try std.testing.expectEqual(@as(u32, 0), counts[1].load(.monotonic));
    executor.shutdown(.drain);
    executor.deinit();
}

test "Spindle adapter preflights slab backpressure and reports shutdown" {
    var executor = try spindle.executor.WorkStealingExecutor.init(std.testing.allocator, .{ .workers = 1, .local_capacity = 2, .injection_capacity = 2, .urgent_capacity = 2 });
    errdefer executor.deinit();
    var slots: [2]spindle_adapter.Dispatcher.Slot = undefined;
    var adapter = spindle_adapter.Dispatcher.init(&executor, &slots, 1);
    var counts = [_]std.atomic.Value(u32){ std.atomic.Value(u32).init(0), std.atomic.Value(u32).init(0), std.atomic.Value(u32).init(0) };
    var context = Context{ .counts = &counts };
    var custom = adapter.custom();
    const dispatcher = contract.Dispatcher{ .custom = &custom };
    try std.testing.expectError(error.CapacityExceeded, dispatcher.dispatch(.{ .context = &context, .job_count = 3, .run = run }));
    try std.testing.expectError(error.Backpressure, dispatcher.dispatch(.{ .context = &context, .job_count = 2, .run = run }));
    for (counts) |value| try std.testing.expectEqual(@as(u32, 0), value.load(.monotonic));
    executor.shutdown(.drain);
    try std.testing.expectError(error.Shutdown, dispatcher.dispatch(.{ .context = &context, .job_count = 1, .run = run }));
    executor.deinit();
}

test "host adapter rejects duplicate and missing logical jobs" {
    const Host = struct {
        fn duplicate(_: ?*anyopaque, _: u32, run_job: host_adapter.RunFn, raw: ?*anyopaque) callconv(.c) u32 {
            _ = run_job(raw, 0);
            return run_job(raw, 0);
        }
        fn missing(_: ?*anyopaque, _: u32, run_job: host_adapter.RunFn, raw: ?*anyopaque) callconv(.c) u32 {
            return run_job(raw, 0);
        }
    };
    var counts = [_]std.atomic.Value(u32){ std.atomic.Value(u32).init(0), std.atomic.Value(u32).init(0) };
    var context = Context{ .counts = &counts };
    var seen: [2]u8 = undefined;
    var adapter = host_adapter.Dispatcher{ .user = null, .dispatch_batch = Host.duplicate, .seen = &seen };
    var custom = adapter.custom();
    var dispatcher = contract.Dispatcher{ .custom = &custom };
    try std.testing.expectError(error.CallbackFailed, dispatcher.dispatch(.{ .context = &context, .job_count = 2, .run = run }));
    adapter.dispatch_batch = Host.missing;
    try std.testing.expectError(error.CallbackFailed, dispatcher.dispatch(.{ .context = &context, .job_count = 2, .run = run }));
}

const SpindleHost = struct {
    adapter: *spindle_adapter.Dispatcher,
    delay_seed: u64 = 0,

    fn dispatch(user: ?*anyopaque, job_count: u32, run_job: abi.RunJobFn, batch_context: ?*anyopaque) callconv(.c) u32 {
        const self: *SpindleHost = @ptrCast(@alignCast(user orelse return abi.invalid_argument));
        var bridge = Bridge{ .run_job = run_job, .batch_context = batch_context, .delay_seed = self.delay_seed };
        self.adapter.dispatch(.{ .context = &bridge, .job_count = job_count, .run = Bridge.run }) catch return abi.callback_error;
        return abi.ok;
    }
    const Bridge = struct {
        run_job: abi.RunJobFn,
        batch_context: ?*anyopaque,
        delay_seed: u64 = 0,
        fn run(raw: *anyopaque, index: u32) !void {
            const self: *Bridge = @ptrCast(@alignCast(raw));
            var mixed = self.delay_seed ^ (@as(u64, index) *% 0x9e37_79b9_7f4a_7c15);
            mixed ^= mixed >> 30;
            for (0..@as(usize, @intCast(mixed & 0xff))) |_| std.atomic.spinLoopHint();
            if (self.run_job(self.batch_context, index) != abi.ok) return error.CallbackFailed;
        }
    };
};

const TestHost = struct {
    scheduler: contract.TestDispatcher,
    max_job_count: u32 = 0,

    fn dispatch(user: ?*anyopaque, job_count: u32, run_job: abi.RunJobFn, batch_context: ?*anyopaque) callconv(.c) u32 {
        const self: *TestHost = @ptrCast(@alignCast(user orelse return abi.invalid_argument));
        self.max_job_count = @max(self.max_job_count, job_count);
        var bridge = SpindleHost.Bridge{ .run_job = run_job, .batch_context = batch_context };
        var custom = self.scheduler.custom();
        const dispatcher = contract.Dispatcher{ .custom = &custom };
        dispatcher.dispatch(.{ .context = &bridge, .job_count = job_count, .run = SpindleHost.Bridge.run }) catch return abi.callback_error;
        return abi.ok;
    }
};

const FaultHost = struct {
    fn dispatch(user: ?*anyopaque, job_count: u32, run_job: abi.RunJobFn, batch_context: ?*anyopaque) callconv(.c) u32 {
        _ = @as(*FaultHost, @ptrCast(@alignCast(user orelse return abi.invalid_argument)));
        var index: u32 = 0;
        while (index < job_count) : (index += 1) {
            if (run_job(batch_context, index) != abi.ok) return abi.callback_error;
        }
        return if (job_count > 1) abi.callback_error else abi.ok;
    }
};

fn expectAbi(result: u32) !void {
    if (result != abi.ok) return error.AbiFailure;
}
fn identityTransform() abi.Transform {
    return .{ .position = .{ .x = 0, .y = 0, .z = 0 }, .orientation = .{ .x = 0, .y = 0, .z = 0, .w = 1 << 32 } };
}

test "production multi-range ABI pipeline hash matches serial across Spindle worker counts and backend switches" {
    const allocator = std.testing.allocator;
    var asset_desc = abi.AssetStoreDesc{ .struct_size = @sizeOf(abi.AssetStoreDesc), .reserved = 0, .assets = null, .asset_count = 0, .reserved1 = 0 };
    var asset_size: u64 = 0;
    var asset_alignment: u32 = 0;
    try expectAbi(abi.gravity_v1_asset_store_memory_required(&asset_desc, &asset_size, &asset_alignment));
    const asset_memory = try allocator.alignedAlloc(u8, .fromByteUnits(@alignOf(abi.AssetStore)), @intCast(asset_size));
    defer allocator.free(asset_memory);
    var assets: *abi.AssetStore = undefined;
    try expectAbi(abi.gravity_v1_asset_store_init(asset_memory.ptr, asset_memory.len, &asset_desc, &assets));
    defer expectAbi(abi.gravity_v1_asset_store_deinit(assets)) catch unreachable;

    var world_desc = abi.WorldDesc{
        .struct_size = @sizeOf(abi.WorldDesc),
        .reserved = 0,
        .body_capacity = 300,
        .collider_capacity = 65,
        .command_capacity = 2,
        .contact_capacity = 2,
        .gravity = .{ .x = 0, .y = -(1 << 32), .z = 0 },
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
    try expectAbi(abi.gravity_v1_world_memory_required(&world_desc, &world_size, &world_alignment));
    const world_memory = try allocator.alignedAlloc(u8, .fromByteUnits(@alignOf(abi.World)), @intCast(world_size));
    defer allocator.free(world_memory);
    var world: *abi.World = undefined;
    try expectAbi(abi.gravity_v1_world_init(world_memory.ptr, world_memory.len, &world_desc, &world));
    defer expectAbi(abi.gravity_v1_world_deinit(world)) catch unreachable;
    var body_desc = abi.BodyDesc{
        .struct_size = @sizeOf(abi.BodyDesc),
        .reserved = 0,
        .body_type = 1,
        .dof_locks = 0,
        .transform = identityTransform(),
        .inverse_mass = 1 << 32,
        .inverse_inertia_xx = 1 << 32,
        .inverse_inertia_yy = 1 << 32,
        .inverse_inertia_zz = 1 << 32,
        .inverse_inertia_xy = 0,
        .inverse_inertia_xz = 0,
        .inverse_inertia_yz = 0,
    };
    var body: u64 = 0;
    var collider_desc = abi.ColliderDesc{
        .struct_size = @sizeOf(abi.ColliderDesc),
        .reserved = 0,
        .body = 0,
        .shape_kind = 0,
        .flags = 0,
        .local = identityTransform(),
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
    for (0..8) |index| {
        body_desc.transform.position.x = (@as(i64, @intCast(index)) * 4) << 32;
        try expectAbi(abi.gravity_v1_world_create_body(world, &body_desc, &body));
    }
    var static_desc = body_desc;
    static_desc.body_type = 0;
    static_desc.transform = identityTransform();
    static_desc.inverse_mass = 0;
    static_desc.inverse_inertia_xx = 0;
    static_desc.inverse_inertia_yy = 0;
    static_desc.inverse_inertia_zz = 0;
    try expectAbi(abi.gravity_v1_world_create_body(world, &static_desc, &body));
    collider_desc.body = body;
    for (0..65) |index| {
        collider_desc.local.position.x = (@as(i64, @intCast(index)) * 4) << 32;
        try expectAbi(abi.gravity_v1_world_create_collider(world, &collider_desc, &collider));
    }

    var point_query = abi.PointQuery{ .struct_size = @sizeOf(abi.PointQuery), .reserved = 0, .point = .{ .x = 0, .y = 0, .z = 0 }, .filter = .{ .category = 1, .mask = std.math.maxInt(u32), .group = 0, .reserved = 0 }, .mode = 2, .reserved1 = 0 };
    var ray_query = abi.RayQuery{ .struct_size = @sizeOf(abi.RayQuery), .reserved = 0, .origin = .{ .x = -(2 << 32), .y = 0, .z = 0 }, .direction = .{ .x = 300 << 32, .y = 0, .z = 0 }, .max_fraction = 1 << 32, .filter = .{ .category = 1, .mask = std.math.maxInt(u32), .group = 0, .reserved = 0 }, .mode = 1, .reserved1 = 0 };
    var query_hits: [1]abi.QueryHit = undefined;
    var query_required: u32 = 0;
    try expectAbi(abi.gravity_v1_world_query_point(world, &point_query, &query_hits, query_hits.len, &query_required));
    try std.testing.expectEqual(@as(u32, 1), query_required);
    try expectAbi(abi.gravity_v1_world_query_ray(world, &ray_query, &query_hits, query_hits.len, &query_required));
    try std.testing.expectEqual(@as(u32, 1), query_required);

    var snapshot_size: u64 = 0;
    try expectAbi(abi.gravity_v1_world_snapshot_size(world, &snapshot_size));
    const snapshot = try allocator.alloc(u8, @intCast(snapshot_size));
    defer allocator.free(snapshot);
    var required: u64 = 0;
    try expectAbi(abi.gravity_v1_world_snapshot_save(world, snapshot.ptr, snapshot.len, &required));
    var golden: [4]abi.Hash128 = undefined;
    for (&golden) |*expected| {
        try expectAbi(abi.gravity_v1_world_step(world, null, 0));
        try expectAbi(abi.gravity_v1_world_hash(world, expected));
    }

    try expectAbi(abi.gravity_v1_world_snapshot_load(world, snapshot.ptr, snapshot.len));
    var before_failure: abi.Hash128 = undefined;
    try expectAbi(abi.gravity_v1_world_hash(world, &before_failure));
    var fault_host = FaultHost{};
    var fault_dispatcher = abi.Dispatcher{ .struct_size = @sizeOf(abi.Dispatcher), .reserved = 0, .user = &fault_host, .dispatch_batch = FaultHost.dispatch };
    try expectAbi(abi.gravity_v1_world_set_dispatcher(world, &fault_dispatcher));
    try std.testing.expectEqual(abi.callback_error, abi.gravity_v1_world_step(world, null, 0));
    var after_failure: abi.Hash128 = undefined;
    try expectAbi(abi.gravity_v1_world_hash(world, &after_failure));
    try std.testing.expectEqualSlices(u8, &before_failure.bytes, &after_failure.bytes);
    try expectAbi(abi.gravity_v1_world_set_dispatcher(world, null));

    inline for (.{ contract.TestDispatcher.Order.reverse, contract.TestDispatcher.Order.permuted }) |order| {
        try expectAbi(abi.gravity_v1_world_snapshot_load(world, snapshot.ptr, snapshot.len));
        var host = TestHost{ .scheduler = .{ .order = order } };
        var dispatcher = abi.Dispatcher{ .struct_size = @sizeOf(abi.Dispatcher), .reserved = 0, .user = &host, .dispatch_batch = TestHost.dispatch };
        try expectAbi(abi.gravity_v1_world_set_dispatcher(world, &dispatcher));
        host.max_job_count = 0;
        try expectAbi(abi.gravity_v1_world_query_point(world, &point_query, &query_hits, query_hits.len, &query_required));
        try std.testing.expectEqual(@as(u32, 1), query_required);
        try expectAbi(abi.gravity_v1_world_query_ray(world, &ray_query, &query_hits, query_hits.len, &query_required));
        try std.testing.expectEqual(@as(u32, 1), query_required);
        try std.testing.expect(host.max_job_count >= 3);
        for (golden) |expected| {
            try expectAbi(abi.gravity_v1_world_step(world, null, 0));
            var actual: abi.Hash128 = undefined;
            try expectAbi(abi.gravity_v1_world_hash(world, &actual));
            try std.testing.expectEqualSlices(u8, &expected.bytes, &actual.bytes);
        }
        try std.testing.expect(host.max_job_count >= 8);
        try expectAbi(abi.gravity_v1_world_set_dispatcher(world, null));
    }

    for ([_]usize{ 1, 2, 4, 8 }) |workers| {
        try expectAbi(abi.gravity_v1_world_snapshot_load(world, snapshot.ptr, snapshot.len));
        var executor = try spindle.executor.WorkStealingExecutor.init(allocator, .{ .workers = workers, .local_capacity = 64, .injection_capacity = 128, .urgent_capacity = 16 });
        errdefer executor.deinit();
        var slots: [64]spindle_adapter.Dispatcher.Slot = undefined;
        var adapter = spindle_adapter.Dispatcher.init(&executor, &slots, 64);
        var host = SpindleHost{ .adapter = &adapter, .delay_seed = 0xd1b5_4a32_d192_ed03 ^ workers };
        var dispatcher = abi.Dispatcher{ .struct_size = @sizeOf(abi.Dispatcher), .reserved = 0, .user = &host, .dispatch_batch = SpindleHost.dispatch };
        try expectAbi(abi.gravity_v1_world_set_dispatcher(world, &dispatcher));
        for (golden) |expected| {
            try expectAbi(abi.gravity_v1_world_step(world, null, 0));
            var actual: abi.Hash128 = undefined;
            try expectAbi(abi.gravity_v1_world_hash(world, &actual));
            try std.testing.expectEqualSlices(u8, &expected.bytes, &actual.bytes);
        }
        try expectAbi(abi.gravity_v1_world_set_dispatcher(world, null));
        executor.shutdown(.drain);
        executor.deinit();
    }

    // A rollback may resume on a different worker count every Tick. The
    // canonical replay hashes must remain identical across those backend
    // switches rather than only across one fixed executor lifetime.
    try expectAbi(abi.gravity_v1_world_snapshot_load(world, snapshot.ptr, snapshot.len));
    for ([_]usize{ 1, 8, 2, 4 }, golden) |workers, expected| {
        var executor = try spindle.executor.WorkStealingExecutor.init(allocator, .{ .workers = workers, .local_capacity = 64, .injection_capacity = 128, .urgent_capacity = 16 });
        errdefer executor.deinit();
        var slots: [64]spindle_adapter.Dispatcher.Slot = undefined;
        var adapter = spindle_adapter.Dispatcher.init(&executor, &slots, 64);
        var host = SpindleHost{ .adapter = &adapter, .delay_seed = 0x94d0_49bb_1331_11eb ^ workers };
        var dispatcher = abi.Dispatcher{ .struct_size = @sizeOf(abi.Dispatcher), .reserved = 0, .user = &host, .dispatch_batch = SpindleHost.dispatch };
        try expectAbi(abi.gravity_v1_world_set_dispatcher(world, &dispatcher));
        try expectAbi(abi.gravity_v1_world_step(world, null, 0));
        var actual: abi.Hash128 = undefined;
        try expectAbi(abi.gravity_v1_world_hash(world, &actual));
        try std.testing.expectEqualSlices(u8, &expected.bytes, &actual.bytes);
        try expectAbi(abi.gravity_v1_world_set_dispatcher(world, null));
        executor.shutdown(.drain);
        executor.deinit();
    }
}
