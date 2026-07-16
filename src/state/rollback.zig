//! Fixed-capacity full-snapshot rollback history.
//!
//! Each slot owns one complete canonical snapshot and its canonical command
//! bytes.  The type deliberately has no allocator: callers reserve exactly
//! `window * snapshot_capacity` and `window * input_capacity` bytes.
const hash = @import("hash.zig");

pub const Error = error{ CapacityExceeded, InvalidWindow, SnapshotTooLarge, InputTooLarge, MissingTick };

pub const Record = struct {
    tick: u64,
    snapshot: []const u8,
    input: []const u8,
    state_hash: hash.Hash128,
};

/// A ring of full (not delta) snapshots.  Slots are selected by tick modulo
/// window, but the stored tick is always checked so stale data is never
/// mistaken for a valid rollback point after wraparound.
pub const Ring = struct {
    ticks: []u64,
    snapshot_lengths: []usize,
    input_lengths: []usize,
    hashes: []hash.Hash128,
    snapshots: []u8,
    inputs: []u8,
    snapshot_capacity: usize,
    input_capacity: usize,
    valid: []bool,

    pub fn init(ticks: []u64, snapshot_lengths: []usize, input_lengths: []usize, hashes: []hash.Hash128, snapshots: []u8, inputs: []u8, valid: []bool, snapshot_capacity: usize, input_capacity: usize) Error!Ring {
        const slot_count = ticks.len;
        if (slot_count == 0 or snapshot_lengths.len != slot_count or input_lengths.len != slot_count or hashes.len != slot_count or valid.len != slot_count) return error.InvalidWindow;
        if (snapshots.len != slot_count * snapshot_capacity or inputs.len != slot_count * input_capacity) return error.CapacityExceeded;
        @memset(valid, false);
        return .{ .ticks = ticks, .snapshot_lengths = snapshot_lengths, .input_lengths = input_lengths, .hashes = hashes, .snapshots = snapshots, .inputs = inputs, .snapshot_capacity = snapshot_capacity, .input_capacity = input_capacity, .valid = valid };
    }

    pub fn window(self: *const Ring) usize {
        return self.ticks.len;
    }

    pub fn save(self: *Ring, tick: u64, snapshot: []const u8, input: []const u8, state_hash: hash.Hash128) Error!void {
        if (snapshot.len > self.snapshot_capacity) return error.SnapshotTooLarge;
        if (input.len > self.input_capacity) return error.InputTooLarge;
        const slot = @as(usize, @intCast(tick % self.ticks.len));
        const snapshot_start = slot * self.snapshot_capacity;
        const input_start = slot * self.input_capacity;
        // Every fallible check occurs before this point.  The following copy
        // and metadata update is the ring's single commit point.
        @memcpy(self.snapshots[snapshot_start .. snapshot_start + snapshot.len], snapshot);
        @memcpy(self.inputs[input_start .. input_start + input.len], input);
        self.ticks[slot] = tick;
        self.snapshot_lengths[slot] = snapshot.len;
        self.input_lengths[slot] = input.len;
        self.hashes[slot] = state_hash;
        self.valid[slot] = true;
    }

    pub fn get(self: *const Ring, tick: u64) Error!Record {
        const slot = @as(usize, @intCast(tick % self.ticks.len));
        if (!self.valid[slot] or self.ticks[slot] != tick) return error.MissingTick;
        const snapshot_start = slot * self.snapshot_capacity;
        const input_start = slot * self.input_capacity;
        return .{ .tick = tick, .snapshot = self.snapshots[snapshot_start .. snapshot_start + self.snapshot_lengths[slot]], .input = self.inputs[input_start .. input_start + self.input_lengths[slot]], .state_hash = self.hashes[slot] };
    }
};
