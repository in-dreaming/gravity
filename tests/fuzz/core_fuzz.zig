const std = @import("std");
const gravity = @import("gravity");
const ids = gravity.core.ids;
const memory = gravity.core.memory;

test "bounded core property probes reject bad layout and stale IDs" {
    var seed: u64 = 0xd1ce_f00d_1234_5678;
    for (0..1_000) |_| {
        seed = seed *% 6_364_136_223_846_793_005 +% 1;
        var regions: [1]memory.Region = undefined;
        const alignment: usize = @as(usize, 1) << @intCast(seed % 8);
        _ = try memory.calculateLayout(&.{.{ .size = @truncate(seed), .alignment = alignment }}, &regions);
        try std.testing.expectEqual(@as(usize, 0), regions[0].offset % alignment);

        var values: [2]u8 = undefined;
        var generations: [2]u32 = undefined;
        var used: [2]bool = undefined;
        var retired: [2]bool = undefined;
        var pool = try ids.SlotPool(u8).init(&values, &generations, &used, &retired);
        const first = try pool.allocate(1);
        try pool.release(first);
        try std.testing.expect(pool.get(first) == null);
    }
}
