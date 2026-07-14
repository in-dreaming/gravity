const std = @import("std");
const gravity = @import("gravity");

test "metadata read is stable" {
    const first = gravity.buildMetadata();
    const second = gravity.buildMetadata();
    try std.testing.expectEqualStrings(first.commit, second.commit);
    try std.testing.expectEqualStrings(first.zig_version, second.zig_version);
    try std.testing.expectEqual(first.protocol, second.protocol);
}
