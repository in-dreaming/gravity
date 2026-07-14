const std = @import("std");
const gravity = @import("gravity");
const ids = gravity.core.ids;
const memory = gravity.core.memory;
const radix = gravity.core.radix;
const config = gravity.core.config;

test "slot pool uses low index and rejects stale and double releases" {
    var values: [3]u32 = undefined;
    var generations: [3]u32 = undefined;
    var used: [3]bool = undefined;
    var retired: [3]bool = undefined;
    var pool = try ids.SlotPool(u32).init(&values, &generations, &used, &retired);
    const first = try pool.allocate(11);
    const second = try pool.allocate(22);
    try std.testing.expectEqual(@as(u32, 0), first.index());
    try std.testing.expectEqual(@as(u32, 1), second.index());
    try pool.release(first);
    try std.testing.expectError(error.InvalidId, pool.release(first));
    const recycled = try pool.allocate(33);
    try std.testing.expectEqual(@as(u32, 0), recycled.index());
    try std.testing.expectEqual(@as(u32, 1), recycled.generation());
    try std.testing.expect(pool.get(first) == null);
}

test "slot pool capacity failure and generation retirement are transactional" {
    var values: [1]u32 = undefined;
    var generations = [_]u32{std.math.maxInt(u32)};
    var used: [1]bool = .{false};
    var retired: [1]bool = .{false};
    var pool = ids.SlotPool(u32){ .values = &values, .generations = &generations, .occupied = &used, .retired = &retired };
    const id = try pool.allocate(9);
    try std.testing.expectError(error.OutOfCapacity, pool.allocate(10));
    try std.testing.expectEqual(@as(usize, 1), pool.live_count);
    try std.testing.expectEqual(@as(u32, 9), pool.get(id).?.*);
    try pool.release(id);
    try std.testing.expect(retired[0]);
    try std.testing.expectError(error.OutOfCapacity, pool.allocate(11));
}

test "fixed containers have transactional capacity failures" {
    var backing = [_]u32{0} ** 1;
    var vector = memory.FixedVec(u32).init(&backing);
    try vector.append(7);
    try std.testing.expectError(error.OutOfCapacity, vector.append(9));
    try std.testing.expectEqual(@as(usize, 1), vector.len);
    try std.testing.expectEqual(@as(u32, 7), vector.slice()[0]);
}

test "checked layout, arena alignment, bitset and canaries" {
    const specs = [_]memory.RegionSpec{ .{ .size = 3, .alignment = 1 }, .{ .size = 8, .alignment = 8 } };
    var regions: [2]memory.Region = undefined;
    try std.testing.expectEqual(@as(usize, 16), try memory.calculateLayout(&specs, &regions));
    try std.testing.expectEqual(@as(usize, 8), regions[1].offset);
    var bytes: [64]u8 align(8) = undefined;
    var arena = try memory.Arena.init(&bytes, 8);
    const ints = try arena.allocate(u32, 2);
    ints[0] = 1;
    try std.testing.expectEqual(@as(usize, 0), @intFromPtr(ints.ptr) % @alignOf(u32));
    var too_small: [7]u8 align(8) = undefined;
    var small_arena = try memory.Arena.init(&too_small, 8);
    try std.testing.expectError(error.InsufficientMemory, small_arena.allocate(u64, 1));
    try std.testing.expectError(error.MisalignedMemory, memory.Arena.init(bytes[1..], 8));
    var words: [1]usize = undefined;
    var bits = memory.BitSet.init(&words, 3);
    try std.testing.expect(bits.set(2));
    try std.testing.expect(bits.isSet(2));
    try std.testing.expect(!bits.set(3));
    var canary: [4]u8 = undefined;
    memory.Canary.fill(&canary);
    try memory.Canary.validate(&canary);
    canary[1] = 0;
    try std.testing.expectError(error.CanaryCorrupt, memory.Canary.validate(&canary));
}

const Entry = struct { primary: u64, secondary: u32, serial: u32 };
fn key32(value: Entry) u32 {
    return value.secondary;
}
fn key64(value: Entry) u64 {
    return value.primary;
}
fn key128(value: Entry) u128 {
    return (@as(u128, value.primary) << 32) | value.secondary;
}

test "radix sort is stable for 32 64 128 and composite keys" {
    var items = [_]Entry{ .{ .primary = 2, .secondary = 1, .serial = 0 }, .{ .primary = 1, .secondary = 2, .serial = 1 }, .{ .primary = 2, .secondary = 1, .serial = 2 }, .{ .primary = 1, .secondary = 1, .serial = 3 } };
    var scratch: [items.len]Entry = undefined;
    try radix.sortU32(Entry, &items, &scratch, key32);
    try std.testing.expectEqual(@as(u32, 0), items[0].serial);
    try std.testing.expectEqual(@as(u32, 2), items[1].serial);
    try radix.sortU64(Entry, &items, &scratch, key64);
    try std.testing.expectEqual(@as(u64, 1), items[0].primary);
    try radix.sortU128(Entry, &items, &scratch, key128);
    try std.testing.expectEqual(@as(u32, 3), items[0].serial);
    try radix.sortComposite2(Entry, &items, &scratch, u64, u32, key64, key32);
    try std.testing.expectEqual(@as(u32, 3), items[0].serial);
    try std.testing.expectEqual(@as(u32, 1), items[1].serial);
}

test "default config validates and excludes worker count" {
    const default = config.SimulationConfig.default;
    try default.validate();
    try std.testing.expectEqual(@as(u32, 8_192), default.capacities.body);
    try std.testing.expectEqual(@as(i64, 21_474_836), default.tolerances.linear_slop.raw);
    try std.testing.expectEqual(@as(i64, 599_690_565), default.tolerances.max_angular_correction.raw);
    const Visitor = struct {
        count: usize = 0,
        first: []const u8 = "",
        last: []const u8 = "",
        pub fn field(self: *@This(), name: []const u8, value: anytype) void {
            _ = value;
            if (self.count == 0) self.first = name;
            self.last = name;
            self.count += 1;
        }
    };
    var visitor = Visitor{};
    default.visitCanonical(&visitor);
    try std.testing.expectEqual(@as(usize, 44), visitor.count);
    try std.testing.expectEqualStrings("body", visitor.first);
    try std.testing.expectEqualStrings("features", visitor.last);
    var invalid = default;
    invalid.capacities.body = 0;
    try std.testing.expectError(error.InvalidCapacity, invalid.validate());
    invalid = default;
    invalid.features.reserved = 1;
    try std.testing.expectError(error.InvalidFeatureFlags, invalid.validate());
    invalid = default;
    invalid.iterations.velocity = 256;
    try std.testing.expectError(error.InvalidIteration, invalid.validate());
    invalid = default;
    invalid.iterations.position = 256;
    try std.testing.expectError(error.InvalidIteration, invalid.validate());
}

test "slot pool randomized reference model maintains IDs" {
    var values: [16]u32 = undefined;
    var generations: [16]u32 = undefined;
    var used: [16]bool = undefined;
    var retired: [16]bool = undefined;
    var pool = try ids.SlotPool(u32).init(&values, &generations, &used, &retired);
    var model: [16]?ids.Id = .{null} ** 16;
    var seed: u32 = 0x8bad_f00d;
    for (0..2_000) |_| {
        seed = seed *% 1_664_525 +% 1_013_904_223;
        const slot: usize = seed % model.len;
        if (model[slot]) |id| {
            try pool.release(id);
            model[slot] = null;
        } else if (pool.allocate(seed)) |id| {
            model[id.index()] = id;
            if (pool.get(id)) |value| value.* = seed;
        } else |err| try std.testing.expectEqual(error.OutOfCapacity, err);
    }
}
