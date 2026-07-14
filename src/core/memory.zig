//! Checked caller-memory layout and fixed containers used by World.
const std = @import("std");

pub const RegionSpec = struct { size: usize, alignment: usize };
pub const Region = struct { offset: usize, size: usize, alignment: usize };

pub const LayoutError = error{ InvalidAlignment, Overflow, InsufficientMemory, MisalignedMemory, CanaryCorrupt };

pub fn calculateLayout(specs: []const RegionSpec, regions: []Region) LayoutError!usize {
    if (regions.len != specs.len) return error.Overflow;
    var cursor: usize = 0;
    for (specs, 0..) |spec, index| {
        if (spec.alignment == 0 or !std.math.isPowerOfTwo(spec.alignment)) return error.InvalidAlignment;
        cursor = try alignForwardChecked(cursor, spec.alignment);
        regions[index] = .{ .offset = cursor, .size = spec.size, .alignment = spec.alignment };
        cursor = std.math.add(usize, cursor, spec.size) catch return error.Overflow;
    }
    return cursor;
}

pub const Arena = struct {
    memory: []u8,
    cursor: usize = 0,

    pub fn init(memory: []u8, required_alignment: usize) LayoutError!Arena {
        if (required_alignment == 0 or !std.math.isPowerOfTwo(required_alignment)) return error.InvalidAlignment;
        if (@intFromPtr(memory.ptr) % required_alignment != 0) return error.MisalignedMemory;
        return .{ .memory = memory };
    }

    pub fn allocate(self: *Arena, comptime T: type, count: usize) LayoutError![]T {
        const bytes = std.math.mul(usize, @sizeOf(T), count) catch return error.Overflow;
        const start = try alignForwardChecked(self.cursor, @alignOf(T));
        const end = std.math.add(usize, start, bytes) catch return error.Overflow;
        if (end > self.memory.len) return error.InsufficientMemory;
        self.cursor = end;
        const ptr: [*]align(@alignOf(T)) T = @ptrCast(@alignCast(self.memory.ptr + start));
        return ptr[0..count];
    }
};

fn alignForwardChecked(value: usize, alignment: usize) LayoutError!usize {
    const remainder = value % alignment;
    if (remainder == 0) return value;
    return std.math.add(usize, value, alignment - remainder) catch error.Overflow;
}

pub const Canary = struct {
    pub const byte: u8 = 0xA5;
    pub fn fill(memory: []u8) void {
        @memset(memory, byte);
    }
    pub fn validate(memory: []const u8) LayoutError!void {
        for (memory) |value| if (value != byte) return error.CanaryCorrupt;
    }
};

pub fn FixedVec(comptime T: type) type {
    return struct {
        items: []T,
        len: usize = 0,
        pub const Error = error{OutOfCapacity};
        pub fn init(items: []T) @This() {
            return .{ .items = items };
        }
        pub fn slice(self: *const @This()) []const T {
            return self.items[0..self.len];
        }
        pub fn mutableSlice(self: *@This()) []T {
            return self.items[0..self.len];
        }
        pub fn append(self: *@This(), value: T) Error!void {
            if (self.len == self.items.len) return error.OutOfCapacity;
            self.items[self.len] = value;
            self.len += 1;
        }
        pub fn pop(self: *@This()) ?T {
            if (self.len == 0) return null;
            self.len -= 1;
            return self.items[self.len];
        }
    };
}

pub const BitSet = struct {
    words: []usize,
    bit_count: usize,
    pub fn init(words: []usize, bit_count: usize) BitSet {
        @memset(words, 0);
        return .{ .words = words, .bit_count = bit_count };
    }
    pub fn set(self: *BitSet, index: usize) bool {
        if (index >= self.bit_count) return false;
        self.words[index / @bitSizeOf(usize)] |= @as(usize, 1) << @intCast(index % @bitSizeOf(usize));
        return true;
    }
    pub fn isSet(self: *const BitSet, index: usize) bool {
        return index < self.bit_count and (self.words[index / @bitSizeOf(usize)] & (@as(usize, 1) << @intCast(index % @bitSizeOf(usize)))) != 0;
    }
};
