const std = @import("std");
const gravity = @import("gravity");

test "foundation exposes one coherent version tuple" {
    const metadata = gravity.buildMetadata();
    try std.testing.expect(metadata.commit.len >= 7);
    try std.testing.expectEqual(@as(u32, 1), metadata.abi);
    try std.testing.expectEqual(@as(u32, 1), metadata.protocol);
    try std.testing.expectEqual(@as(u32, 1), metadata.snapshot_format);
    try std.testing.expectEqual(@as(u32, 2), metadata.asset_format);
}
