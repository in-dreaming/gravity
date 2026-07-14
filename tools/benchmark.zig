const std = @import("std");
const gravity = @import("gravity");

pub fn main() !void {
    const fp = gravity.math.fp;
    var checksum: u64 = 0;
    var status = fp.MathStatus{};
    var value = fp.Fp{ .raw = 5_123_456_789 };
    var index: u64 = 0;
    while (index < 1_000_000) : (index += 1) {
        const metadata = gravity.buildMetadata();
        checksum +%= metadata.protocol;
        checksum +%= metadata.asset_format;
        value = fp.Fp.mul(value, fp.Fp.one, &status);
    }
    if (checksum != 3_000_000 or status.fault != .none) return error.InvalidChecksum;
    std.debug.print("fixed-point benchmark: checksum {d}, raw {d}\n", .{ checksum, value.raw });
}
