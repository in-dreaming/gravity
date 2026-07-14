//! Allocation-free stable radix sorting. Scratch must match item length.
const std = @import("std");

pub const Error = error{ScratchLengthMismatch};

pub fn sortU32(comptime T: type, items: []T, scratch: []T, comptime key: fn (T) u32) Error!void {
    try sortUnsigned(T, items, scratch, u32, key);
}
pub fn sortU64(comptime T: type, items: []T, scratch: []T, comptime key: fn (T) u64) Error!void {
    try sortUnsigned(T, items, scratch, u64, key);
}
pub fn sortU128(comptime T: type, items: []T, scratch: []T, comptime key: fn (T) u128) Error!void {
    try sortUnsigned(T, items, scratch, u128, key);
}

fn sortUnsigned(comptime T: type, items: []T, scratch: []T, comptime U: type, comptime key: fn (T) U) Error!void {
    if (scratch.len != items.len) return error.ScratchLengthMismatch;
    var source = items;
    var destination = scratch;
    comptime var shift: usize = 0;
    inline while (shift < @bitSizeOf(U)) : (shift += 8) {
        var counts = [_]usize{0} ** 256;
        for (source) |item| counts[@as(u8, @truncate(key(item) >> shift))] += 1;
        var total: usize = 0;
        for (&counts) |*count| {
            const prior = count.*;
            count.* = total;
            total += prior;
        }
        for (source) |item| {
            const bucket: u8 = @truncate(key(item) >> shift);
            destination[counts[bucket]] = item;
            counts[bucket] += 1;
        }
        const next_source = destination;
        destination = source;
        source = next_source;
    }
    if (source.ptr != items.ptr) @memcpy(items, source);
}

/// Sorts lexicographic composite keys by applying stable least-significant key first.
pub fn sortComposite2(comptime T: type, items: []T, scratch: []T, comptime Primary: type, comptime Secondary: type, comptime primary: fn (T) Primary, comptime secondary: fn (T) Secondary) Error!void {
    switch (@bitSizeOf(Secondary)) {
        32 => try sortU32(T, items, scratch, secondary),
        64 => try sortU64(T, items, scratch, secondary),
        128 => try sortU128(T, items, scratch, secondary),
        else => @compileError("radix keys must be 32, 64, or 128 bits"),
    }
    switch (@bitSizeOf(Primary)) {
        32 => try sortU32(T, items, scratch, primary),
        64 => try sortU64(T, items, scratch, primary),
        128 => try sortU128(T, items, scratch, primary),
        else => @compileError("radix keys must be 32, 64, or 128 bits"),
    }
}
