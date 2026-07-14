const std = @import("std");
const gravity = @import("gravity");

pub fn main() !void {
    const metadata = gravity.buildMetadata();
    std.debug.print("gravity state diff foundation: protocol v{d}\n", .{metadata.protocol});
}
