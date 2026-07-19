const std = @import("std");
const abi = @import("gravity").abi;
const contract = @import("gravity_jobs");
const spindle_adapter = @import("gravity_spindle_jobs");
const spindle = @import("spindle_executor");

const ticks = 120;
const warmup_ticks = 12;

const SpindleHost = struct {
    adapter: *spindle_adapter.Dispatcher,

    fn dispatch(user: ?*anyopaque, job_count: u32, run_job: abi.RunJobFn, batch_context: ?*anyopaque) callconv(.c) u32 {
        const self: *SpindleHost = @ptrCast(@alignCast(user orelse return abi.invalid_argument));
        var bridge = Bridge{ .run_job = run_job, .batch_context = batch_context };
        self.adapter.dispatch(.{ .context = &bridge, .job_count = job_count, .run = Bridge.run }) catch return abi.callback_error;
        return abi.ok;
    }

    const Bridge = struct {
        run_job: abi.RunJobFn,
        batch_context: ?*anyopaque,
        fn run(raw: *anyopaque, index: u32) !void {
            const self: *Bridge = @ptrCast(@alignCast(raw));
            if (self.run_job(self.batch_context, index) != abi.ok) return error.CallbackFailed;
        }
    };
};

fn expectAbi(result: u32) !void {
    if (result != abi.ok) {
        std.debug.print("unexpected ABI result {d}\n", .{result});
        return error.AbiFailure;
    }
}

fn identityTransform() abi.Transform {
    return .{ .position = .{ .x = 0, .y = 0, .z = 0 }, .orientation = .{ .x = 0, .y = 0, .z = 0, .w = 1 << 32 } };
}

fn nowNs() i96 {
    return std.Io.Clock.Timestamp.now(std.Options.debug_io, .awake).raw.nanoseconds;
}

fn runTicks(world: *abi.World, count: usize) !u64 {
    const started = nowNs();
    for (0..count) |_| try expectAbi(abi.gravity_v1_world_step(world, null, 0));
    return @intCast(nowNs() - started);
}

fn printRow(label: []const u8, body_count: u32, workers: u32, elapsed_ns: u64, serial_ns: u64) void {
    const ns_per_tick = elapsed_ns / ticks;
    const speedup = @as(f64, @floatFromInt(serial_ns)) / @as(f64, @floatFromInt(elapsed_ns));
    std.debug.print("{s},{d},{d},{d},{d},{d:.3}\n", .{ label, body_count, workers, ticks, ns_per_tick, speedup });
}

fn runScene(allocator: std.mem.Allocator, label: []const u8, body_count: u32) !void {
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
        .body_capacity = body_count,
        .collider_capacity = 1,
        .command_capacity = 1,
        .contact_capacity = 1,
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
    for (0..body_count) |index| {
        body_desc.transform.position.x = @as(i64, @intCast(index)) << 32;
        try expectAbi(abi.gravity_v1_world_create_body(world, &body_desc, &body));
    }

    var snapshot_size: u64 = 0;
    try expectAbi(abi.gravity_v1_world_snapshot_size(world, &snapshot_size));
    const snapshot = try allocator.alloc(u8, @intCast(snapshot_size));
    defer allocator.free(snapshot);
    var required: u64 = 0;
    try expectAbi(abi.gravity_v1_world_snapshot_save(world, snapshot.ptr, snapshot.len, &required));

    _ = try runTicks(world, warmup_ticks);
    try expectAbi(abi.gravity_v1_world_snapshot_load(world, snapshot.ptr, snapshot.len));
    const serial_ns = try runTicks(world, ticks);
    var expected: abi.Hash128 = undefined;
    try expectAbi(abi.gravity_v1_world_hash(world, &expected));
    printRow(label, body_count, 0, serial_ns, serial_ns);

    for ([_]usize{ 1, 2, 4, 8 }) |workers| {
        try expectAbi(abi.gravity_v1_world_snapshot_load(world, snapshot.ptr, snapshot.len));
        var executor = try spindle.executor.WorkStealingExecutor.init(allocator, .{ .workers = workers, .local_capacity = 64, .injection_capacity = 128, .urgent_capacity = 16 });
        defer executor.deinit();
        var slots: [contract.maximum_batch_jobs]spindle_adapter.Dispatcher.Slot = undefined;
        var adapter = spindle_adapter.Dispatcher.init(&executor, &slots, contract.maximum_batch_jobs);
        var host = SpindleHost{ .adapter = &adapter };
        var dispatcher = abi.Dispatcher{ .struct_size = @sizeOf(abi.Dispatcher), .reserved = 0, .user = &host, .dispatch_batch = SpindleHost.dispatch };
        try expectAbi(abi.gravity_v1_world_set_dispatcher(world, &dispatcher));
        _ = try runTicks(world, warmup_ticks);
        try expectAbi(abi.gravity_v1_world_snapshot_load(world, snapshot.ptr, snapshot.len));
        const elapsed_ns = try runTicks(world, ticks);
        var actual: abi.Hash128 = undefined;
        try expectAbi(abi.gravity_v1_world_hash(world, &actual));
        if (!std.mem.eql(u8, &expected.bytes, &actual.bytes)) return error.HashMismatch;
        printRow(label, body_count, @intCast(workers), elapsed_ns, serial_ns);
        try expectAbi(abi.gravity_v1_world_set_dispatcher(world, null));
        executor.shutdown(.drain);
    }
}

pub fn main() !void {
    std.debug.print("scene,bodies,workers,ticks,ns_per_tick,speedup_vs_serial\n", .{});
    try runScene(std.heap.page_allocator, "Medium", 2_048);
    try runScene(std.heap.page_allocator, "Stress", 16_384);
}
