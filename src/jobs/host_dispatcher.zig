//! Adapter for Gravity's stable synchronous host batch ABI.  The host only
//! receives borrowed callbacks; this adapter owns exact-once accounting in a
//! caller-provided slab and performs no allocation during dispatch.
const std = @import("std");
const contract = @import("gravity_jobs");

pub const RunFn = *const fn (?*anyopaque, u32) callconv(.c) u32;
pub const DispatchFn = *const fn (?*anyopaque, u32, RunFn, ?*anyopaque) callconv(.c) u32;

pub const Dispatcher = struct {
    user: ?*anyopaque,
    dispatch_batch: DispatchFn,
    seen: []u8,
    active: std.atomic.Value(bool) = std.atomic.Value(bool).init(false),

    pub fn custom(self: *Dispatcher) contract.Custom {
        return .{ .context = self, .dispatch_fn = dispatchErased };
    }

    fn dispatchErased(raw: *anyopaque, batch: contract.Batch) contract.Error!void {
        const self: *Dispatcher = @ptrCast(@alignCast(raw));
        return self.dispatch(batch);
    }

    pub fn dispatch(self: *Dispatcher, batch: contract.Batch) contract.Error!void {
        if (self.active.swap(true, .acq_rel)) return error.Reentrant;
        defer self.active.store(false, .release);
        if (batch.job_count > self.seen.len) return error.CapacityExceeded;
        @memset(self.seen[0..batch.job_count], 0);
        var context = Context{ .batch = batch, .seen = self.seen[0..batch.job_count] };
        if (self.dispatch_batch(self.user, batch.job_count, run, &context) != 0 or context.failed) return error.CallbackFailed;
        for (context.seen) |count| if (count != 1) return error.CallbackFailed;
    }

    const Context = struct {
        batch: contract.Batch,
        seen: []u8,
        failed: bool = false,
    };
    fn run(raw: ?*anyopaque, index: u32) callconv(.c) u32 {
        const context: *Context = @ptrCast(@alignCast(raw orelse return 1));
        if (index >= context.seen.len or context.seen[index] != 0) {
            context.failed = true;
            return 1;
        }
        context.seen[index] = 1;
        context.batch.run(context.batch.context, index) catch {
            context.failed = true;
            return 1;
        };
        return 0;
    }
};
