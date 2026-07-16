//! Gravity-owned synchronous batch contract.  Jobs are identified solely by
//! their caller-assigned logical index; executors never choose output slots.
const std = @import("std");

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
