//! Native Spindle adapter.  This module is deliberately separate from core
//! and WASM builds: only a native host imports the executor-only entry point.
const std = @import("std");
const spindle = @import("spindle_executor");
const contract = @import("gravity_jobs");

pub const Dispatcher = struct {
    executor: *spindle.executor.WorkStealingExecutor,
    slots: []Slot,
    submission_capacity: u32,
    active: std.atomic.Value(bool) = std.atomic.Value(bool).init(false),

    pub const Slot = struct {
        task: spindle.executor.Task,
        context: Context = undefined,
        /// Monotonic reuse evidence owned by Gravity; never simulation state.
        generation: u64 = 0,
    };
    const Context = struct {
        batch: contract.Batch,
        index: u32,
        result: std.atomic.Value(u8) = std.atomic.Value(u8).init(0),
    };

    /// Initializes caller-owned slab records.  Executor allocation happens
    /// before this adapter is created; dispatch itself allocates nothing.
    pub fn init(executor: *spindle.executor.WorkStealingExecutor, slots: []Slot, submission_capacity: u32) Dispatcher {
        for (slots) |*slot| {
            slot.* = .{ .task = spindle.executor.Task.init(runTask, null) };
        }
        return .{ .executor = executor, .slots = slots, .submission_capacity = submission_capacity };
    }

    pub fn custom(self: *Dispatcher) contract.Custom {
        return .{ .context = self, .dispatch_fn = dispatchErased };
    }

    fn dispatchErased(raw: *anyopaque, batch: contract.Batch) contract.Error!void {
        const self: *Dispatcher = @ptrCast(@alignCast(raw));
        return self.dispatch(batch);
    }

    /// Reuse order is intentionally explicit: completion wait, queue-release
    /// wait, reset, then install the next generation/context.
    pub fn dispatch(self: *Dispatcher, batch: contract.Batch) contract.Error!void {
        if (self.active.swap(true, .acq_rel)) return error.Reentrant;
        defer self.active.store(false, .release);
        if (batch.job_count > self.slots.len) return error.CapacityExceeded;
        if (batch.job_count > self.submission_capacity) return error.Backpressure;

        var index: u32 = 0;
        while (index < batch.job_count) : (index += 1) {
            const slot = &self.slots[index];
            prepare(slot) catch return error.WorkerFault;
            slot.context = .{ .batch = batch, .index = index };
            slot.task.context = &slot.context;
            slot.generation +%= 1;
        }
        // Capacity was preflighted above.  A failed submit means this batch is
        // failed; previously submitted jobs are drained before returning.
        var submitted: u32 = 0;
        while (submitted < batch.job_count) : (submitted += 1) {
            self.executor.submit(&self.slots[submitted].task, .{}) catch |err| {
                waitSubmitted(self.slots[0..submitted]) catch {};
                return switch (err) {
                    error.Backpressure => error.Backpressure,
                    error.Shutdown => error.Shutdown,
                    error.DuplicateSubmission, error.Rejected => error.WorkerFault,
                };
            };
        }
        waitSubmitted(self.slots[0..batch.job_count]) catch return error.WorkerFault;
        for (self.slots[0..batch.job_count]) |*slot| {
            if (slot.context.result.load(.acquire) != 0) return error.CallbackFailed;
            switch (slot.task.status()) {
                .completed => {},
                .cancelled => return error.Cancelled,
                else => return error.WorkerFault,
            }
        }
    }

    fn prepare(slot: *Slot) !void {
        switch (slot.task.status()) {
            .created => return,
            .completed, .cancelled, .failed => {
                try slot.task.wait();
                try slot.task.waitQueueReleased();
                try slot.task.reset();
            },
            else => return error.TaskInFlight,
        }
    }
    fn waitSubmitted(slots: []Slot) !void {
        for (slots) |*slot| {
            try slot.task.wait();
            try slot.task.waitQueueReleased();
        }
    }
    fn runTask(task: *spindle.executor.Task) void {
        const context: *Context = @ptrCast(@alignCast(task.context orelse {
            task.fail();
            return;
        }));
        context.batch.run(context.batch.context, context.index) catch {
            context.result.store(1, .release);
            task.fail();
        };
    }
};
