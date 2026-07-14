const std = @import("std");
const gravity = @import("gravity");

pub fn main() !void {
    const metadata = gravity.buildMetadata();
    std.debug.print("gravity replay foundation: snapshot format v{d}\n", .{metadata.snapshot_format});
}
