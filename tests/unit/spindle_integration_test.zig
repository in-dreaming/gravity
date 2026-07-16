const std = @import("std");
const spindle = @import("spindle");

test "pinned Spindle executor-only surface excludes upper modules" {
    try std.testing.expect(@hasDecl(spindle, "executor"));
    try std.testing.expect(!@hasDecl(spindle, "runtime"));
    try std.testing.expect(!@hasDecl(spindle, "parallel"));
    try std.testing.expect(!@hasDecl(spindle, "task_graph"));
    try std.testing.expect(!@hasDecl(spindle, "ecs"));
    try std.testing.expect(!@hasDecl(spindle, "resource_graph"));
    try std.testing.expect(!@hasDecl(spindle, "workflow"));
}

test "pinned Spindle surface supports schedule recording and work stealing" {
    var deterministic = spindle.executor.DeterministicExecutor.init(std.testing.allocator);
    defer deterministic.deinit();
    var deterministic_value: u32 = 0;
    const Increment = struct {
        fn run(task: *spindle.executor.Task) void {
            const value: *u32 = @ptrCast(@alignCast(task.context.?));
            value.* += 1;
        }
    };
    var deterministic_task = spindle.executor.Task.init(Increment.run, &deterministic_value);
    try deterministic.submitWithId(&deterministic_task, 7);
    try deterministic.run();
    try std.testing.expectEqual(@as(u32, 1), deterministic_value);

    var pool = try spindle.executor.WorkStealingExecutor.init(std.testing.allocator, .{
        .workers = 2,
        .local_capacity = 8,
        .injection_capacity = 8,
        .urgent_capacity = 4,
    });
    defer pool.deinit();
    var native_value = std.atomic.Value(u32).init(0);
    const AtomicIncrement = struct {
        fn run(task: *spindle.executor.Task) void {
            const value: *std.atomic.Value(u32) = @ptrCast(@alignCast(task.context.?));
            _ = value.fetchAdd(1, .monotonic);
        }
    };
    var native_tasks: [4]spindle.executor.Task = undefined;
    for (&native_tasks) |*task| {
        task.* = spindle.executor.Task.init(AtomicIncrement.run, &native_value);
        try pool.submit(task, .{});
    }
    for (&native_tasks) |*task| {
        try task.wait();
        try task.waitQueueReleased();
        try task.reset();
    }
    try std.testing.expectEqual(@as(u32, 4), native_value.load(.monotonic));
}
