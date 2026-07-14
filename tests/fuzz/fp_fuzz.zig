const std = @import("std");
const fp = @import("gravity").math.fp;

test "fixed integer property probes" {
    var state: u64 = 0x243f_6a88_85a3_08d3;
    var index: usize = 0;
    while (index < 100_000) : (index += 1) {
        state = state *% 2_862_933_555_777_941_757 +% 3_037_000_493;
        const raw: i64 = @intCast(state >> 2);
        const value = fp.Fp{ .raw = raw };
        var buffer: [64]u8 = undefined;
        const text = value.formatCanonical(&buffer).?;
        var status = fp.MathStatus{};
        try std.testing.expectEqual(value.raw, fp.Fp.parseCanonicalDecimal(text, &status).raw);
        try std.testing.expectEqual(fp.MathFault.none, status.fault);
    }
}
