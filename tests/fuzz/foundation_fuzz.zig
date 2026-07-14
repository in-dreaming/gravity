const std = @import("std");
const gravity = @import("gravity");

test "metadata remains valid across repeated property probes" {
    var index: usize = 0;
    while (index < 10_000) : (index += 1) {
        const metadata = gravity.buildMetadata();
        try std.testing.expect(metadata.commit.len >= 7);
        try std.testing.expectEqual(@as(u32, 1), metadata.abi);
        try std.testing.expectEqual(@as(u32, 1), metadata.protocol);
    }
}
