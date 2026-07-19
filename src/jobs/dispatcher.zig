//! Gravity-owned synchronous batch contract.  Jobs are identified solely by
//! their caller-assigned logical index; executors never choose output slots.
const std = @import("std");

/// Frozen upper bound for one synchronous Gravity-owned batch. Host adapters
/// reserve duplicate/missing-job tracking for this many logical jobs.
pub const maximum_batch_jobs: u32 = 64;

pub const Error = error{ CapacityExceeded, Backpressure, Cancelled, WorkerFault, CallbackFailed, Reentrant, Shutdown };

/// A borrowed batch callback.  It may write only storage owned by `index`.
/// Returning from `dispatch` is the batch barrier: no callback may still use
/// this context after that point.
pub const RunFn = *const fn (context: *anyopaque, index: u32) anyerror!void;

pub const Batch = struct {
    context: *anyopaque,
    job_count: u32,
    run: RunFn,
};

/// Canonical half-open input range owned by one logical job.  Ranges depend
/// only on input cardinality and protocol-frozen grain, never worker count.
pub const Range = struct {
    begin: u32,
    end: u32,
};

pub const RangePlan = struct {
    item_count: u32,
    grain: u32,
    job_count: u32,

    pub fn init(item_count: usize, grain: u32, maximum_jobs: u32) Error!RangePlan {
        if (grain == 0 or maximum_jobs == 0 or item_count > std.math.maxInt(u32)) return error.CapacityExceeded;
        const count: u32 = @intCast(item_count);
        if (count == 0) return .{ .item_count = 0, .grain = grain, .job_count = 0 };
        const required = (count + grain - 1) / grain;
        if (required > maximum_jobs) return error.CapacityExceeded;
        return .{ .item_count = count, .grain = grain, .job_count = required };
    }

    /// Uses the preferred grain when possible and deterministically widens it
    /// for larger caller-configured capacities while respecting the slab cap.
    pub fn initBounded(item_count: usize, preferred_grain: u32, maximum_jobs: u32) Error!RangePlan {
        if (preferred_grain == 0 or maximum_jobs == 0 or item_count > std.math.maxInt(u32)) return error.CapacityExceeded;
        const count: u32 = @intCast(item_count);
        if (count == 0) return .{ .item_count = 0, .grain = preferred_grain, .job_count = 0 };
        const minimum_grain = (count + maximum_jobs - 1) / maximum_jobs;
        const actual_grain = @max(preferred_grain, minimum_grain);
        return init(item_count, actual_grain, maximum_jobs);
    }

    pub fn range(self: RangePlan, logical_job: u32) Error!Range {
        if (logical_job >= self.job_count) return error.CapacityExceeded;
        const begin = logical_job * self.grain;
        return .{ .begin = begin, .end = @min(begin + self.grain, self.item_count) };
    }
};

/// The sole execution seam exposed to Gravity pipeline phases.  A backend
/// validates capacity before it starts work and never performs a serial
/// fallback after a submission failure.
pub const Dispatcher = union(enum) {
    serial: void,
    custom: *const Custom,

    pub fn dispatch(self: Dispatcher, batch: Batch) Error!void {
        return switch (self) {
            .serial => serialDispatch(batch),
            .custom => |value| value.dispatch(batch),
        };
    }
};

pub const Custom = struct {
    context: *anyopaque,
    dispatch_fn: *const fn (*anyopaque, Batch) Error!void,

    pub fn dispatch(self: *const Custom, batch: Batch) Error!void {
        return self.dispatch_fn(self.context, batch);
    }
};

pub fn serialDispatch(batch: Batch) Error!void {
    var index: u32 = 0;
    while (index < batch.job_count) : (index += 1) {
        batch.run(batch.context, index) catch return error.CallbackFailed;
    }
}

/// Allocation-free scheduler perturbation used by determinism qualification.
/// It changes execution order only; logical indices and ownership are intact.
pub const TestDispatcher = struct {
    order: Order,
    pub const Order = enum { forward, reverse, permuted };

    pub fn custom(self: *TestDispatcher) Custom {
        return .{ .context = self, .dispatch_fn = dispatchErased };
    }
    fn dispatchErased(raw: *anyopaque, batch: Batch) Error!void {
        const self: *TestDispatcher = @ptrCast(@alignCast(raw));
        return self.dispatch(batch);
    }
    pub fn dispatch(self: *TestDispatcher, batch: Batch) Error!void {
        switch (self.order) {
            .forward => return serialDispatch(batch),
            .reverse => {
                var remaining = batch.job_count;
                while (remaining > 0) {
                    remaining -= 1;
                    batch.run(batch.context, remaining) catch return error.CallbackFailed;
                }
            },
            .permuted => {
                if (batch.job_count == 0) return;
                var step: u32 = 5;
                while (gcd(step, batch.job_count) != 1) step += 2;
                var ordinal: u32 = 0;
                while (ordinal < batch.job_count) : (ordinal += 1) {
                    const index = (3 + ordinal * step) % batch.job_count;
                    batch.run(batch.context, index) catch return error.CallbackFailed;
                }
            },
        }
    }
    fn gcd(a_value: u32, b_value: u32) u32 {
        var a = a_value;
        var b = b_value;
        while (b != 0) {
            const next = a % b;
            a = b;
            b = next;
        }
        return a;
    }
};

test "serial dispatcher invokes every logical index exactly once" {
    const Context = struct { seen: [4]u8 = [_]u8{0} ** 4 };
    const Run = struct {
        fn call(raw: *anyopaque, index: u32) !void {
            const context: *Context = @ptrCast(@alignCast(raw));
            context.seen[index] += 1;
        }
    };
    var context = Context{};
    const dispatcher = Dispatcher{ .serial = {} };
    try dispatcher.dispatch(.{ .context = &context, .job_count = 4, .run = Run.call });
    try std.testing.expectEqualSlices(u8, &[_]u8{ 1, 1, 1, 1 }, &context.seen);
}

test "range plan is worker-independent and covers every input exactly once" {
    const plan = try RangePlan.init(10, 3, 4);
    try std.testing.expectEqual(@as(u32, 4), plan.job_count);
    try std.testing.expectEqual(Range{ .begin = 0, .end = 3 }, try plan.range(0));
    try std.testing.expectEqual(Range{ .begin = 3, .end = 6 }, try plan.range(1));
    try std.testing.expectEqual(Range{ .begin = 6, .end = 9 }, try plan.range(2));
    try std.testing.expectEqual(Range{ .begin = 9, .end = 10 }, try plan.range(3));
    try std.testing.expectError(error.CapacityExceeded, RangePlan.init(10, 3, 3));
    const bounded = try RangePlan.initBounded(10, 3, 3);
    try std.testing.expectEqual(@as(u32, 3), bounded.job_count);
    try std.testing.expectEqual(@as(u32, 4), bounded.grain);
}

test "test dispatcher reverse and permutation execute every logical job once" {
    const Context = struct { seen: [11]u8 = [_]u8{0} ** 11 };
    const Run = struct {
        fn call(raw: *anyopaque, index: u32) !void {
            const context: *Context = @ptrCast(@alignCast(raw));
            context.seen[index] += 1;
        }
    };
    inline for (.{ TestDispatcher.Order.reverse, TestDispatcher.Order.permuted }) |order| {
        var context = Context{};
        var scheduler = TestDispatcher{ .order = order };
        const custom = scheduler.custom();
        const dispatcher = Dispatcher{ .custom = &custom };
        try dispatcher.dispatch(.{ .context = &context, .job_count = context.seen.len, .run = Run.call });
        try std.testing.expectEqualSlices(u8, &([_]u8{1} ** 11), &context.seen);
    }
}
