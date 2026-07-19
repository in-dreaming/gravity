const std = @import("std");
const spindle = @import("spindle_executor").executor;

const task_count = 4_096;

const Batch = struct {
    remaining: std.atomic.Value(usize),
    checksum: std.atomic.Value(u64) = .init(0),
};

fn runTask(task: *spindle.Task) void {
    const batch: *Batch = @ptrCast(@alignCast(task.context.?));
    var value: u64 = @intFromPtr(task);
    for (0..64) |_| value = value *% 6_364_136_223_846_793_005 +% 1_442_695_040_888_963_407;
    _ = batch.checksum.fetchXor(value, .monotonic);
    _ = batch.remaining.fetchSub(1, .release);
}

fn complete(context: *anyopaque) bool {
    const batch: *Batch = @ptrCast(@alignCast(context));
    return batch.remaining.load(.acquire) == 0;
}

fn initTasks(tasks: []spindle.Task, batch: *Batch) void {
    batch.* = .{ .remaining = .init(tasks.len) };
    for (tasks) |*task| task.* = spindle.Task.init(runTask, batch);
}

fn resetTasks(tasks: []spindle.Task, batch: *Batch) !void {
    batch.remaining.store(tasks.len, .release);
    for (tasks) |*task| try task.reset();
}

fn waitAll(tasks: []spindle.Task) !void {
    for (tasks) |*task| try task.wait();
}

fn report(backend: []const u8, workers: usize, submit_ns: u64, barrier_ns: u64, help_until_ns: u64, shutdown_ns: u64, active_workers: usize, min_tasks: u64, max_tasks: u64) void {
    std.debug.print("{{\"schema\":\"gravity.executor-overhead.v1\",\"backend\":\"{s}\",\"workers\":{d},\"tasks\":{d},\"submit_total_ns\":{d},\"submit_per_task_ns\":{d},\"barrier_ns\":{d},\"help_until_ns\":{d},\"shutdown_ns\":{d},\"active_workers\":{d},\"min_tasks_per_worker\":{d},\"max_tasks_per_worker\":{d}}}\n", .{ backend, workers, task_count, submit_ns, submit_ns / task_count, barrier_ns, help_until_ns, shutdown_ns, active_workers, min_tasks, max_tasks });
}

fn benchSerial(tasks: []spindle.Task, batch: *Batch) !void {
    initTasks(tasks, batch);
    var timer = try std.time.Timer.start();
    for (tasks) |*task| {
        if (!task.tryQueue()) return error.DuplicateSubmission;
        task.execute();
    }
    const submit_ns = timer.read();
    timer.reset();
    try waitAll(tasks);
    const barrier_ns = timer.read();
    try resetTasks(tasks, batch);
    timer.reset();
    for (tasks) |*task| {
        if (!task.tryQueue()) return error.DuplicateSubmission;
        task.execute();
    }
    while (!complete(batch)) std.atomic.spinLoopHint();
    const help_ns = timer.read();
    report("serial-inline", 1, submit_ns, barrier_ns, help_ns, 0, 1, task_count, task_count);
}

fn benchFixed(allocator: std.mem.Allocator, tasks: []spindle.Task, batch: *Batch, workers: usize) !void {
    var pool = try spindle.FixedPool.init(allocator, workers, task_count * 2);
    defer pool.deinit();
    initTasks(tasks, batch);
    var timer = try std.time.Timer.start();
    for (tasks) |*task| try pool.submit(task, .{});
    const submit_ns = timer.read();
    timer.reset();
    try waitAll(tasks);
    const barrier_ns = timer.read();
    try resetTasks(tasks, batch);
    for (tasks) |*task| try pool.submit(task, .{});
    timer.reset();
    pool.helpUntil(batch, complete);
    const help_ns = timer.read();
    timer.reset();
    pool.shutdown(.drain);
    report("fixed-pool-diagnostic", workers, submit_ns, barrier_ns, help_ns, timer.read(), workers, 0, 0);
}

fn benchWorkStealing(allocator: std.mem.Allocator, tasks: []spindle.Task, batch: *Batch, workers: usize) !void {
    var pool = try spindle.WorkStealingExecutor.init(allocator, .{ .workers = workers, .local_capacity = task_count, .injection_capacity = task_count * 2, .urgent_capacity = 64 });
    defer pool.deinit();
    initTasks(tasks, batch);
    var timer = try std.time.Timer.start();
    for (tasks) |*task| try pool.submit(task, .{});
    const submit_ns = timer.read();
    timer.reset();
    try waitAll(tasks);
    const barrier_ns = timer.read();
    try resetTasks(tasks, batch);
    for (tasks) |*task| try pool.submit(task, .{});
    timer.reset();
    pool.helpUntil(batch, complete);
    const help_ns = timer.read();
    var active: usize = 0;
    var min_tasks: u64 = std.math.maxInt(u64);
    var max_tasks: u64 = 0;
    for (0..workers) |id| {
        const stats = pool.workerStats(id);
        if (stats.executed != 0) active += 1;
        min_tasks = @min(min_tasks, stats.executed);
        max_tasks = @max(max_tasks, stats.executed);
    }
    timer.reset();
    pool.shutdown(.drain);
    report("spindle-work-stealing", workers, submit_ns, barrier_ns, help_ns, timer.read(), active, min_tasks, max_tasks);
}

pub fn main() !void {
    var debug = std.heap.DebugAllocator(.{}){};
    defer _ = debug.deinit();
    const allocator = debug.allocator();
    const tasks = try allocator.alloc(spindle.Task, task_count);
    defer allocator.free(tasks);
    var batch: Batch = undefined;
    try benchSerial(tasks, &batch);
    try benchFixed(allocator, tasks, &batch, 8);
    try benchWorkStealing(allocator, tasks, &batch, 8);
}
