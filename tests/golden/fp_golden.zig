const std = @import("std");
const fp = @import("gravity").math.fp;
pub const golden_vector_count = 10_000;

fn referenceRound(numerator: i128, denominator: i128) i128 {
    const negative = (numerator < 0) != (denominator < 0);
    const n = if (numerator < 0) -numerator else numerator;
    const d = if (denominator < 0) -denominator else denominator;
    var quotient = @divTrunc(n, d);
    const remainder = @rem(n, d);
    if (remainder > d - remainder or (remainder == d - remainder and (quotient & 1) == 1)) quotient += 1;
    return if (negative) -quotient else quotient;
}

test "10,000 golden Q32.32 arithmetic vectors" {
    var state: u64 = 0x6a09_e667_f3bc_c909;
    var hash: u64 = 0xcbf2_9ce4_8422_2325;
    var index: usize = 0;
    while (index < golden_vector_count) : (index += 1) {
        state = state *% 6_364_136_223_846_793_005 +% 1_442_695_040_888_963_407;
        const a: i64 = @intCast(state >> 1);
        state = state *% 6_364_136_223_846_793_005 +% 1_442_695_040_888_963_407;
        var b: i64 = @intCast(state >> 1);
        if (b == 0) b = 1;
        var status = fp.MathStatus{};
        const product = fp.Fp.mul(.{ .raw = a }, .{ .raw = b }, &status);
        const expected_product = referenceRound(@as(i128, a) * @as(i128, b), @as(i128, 1) << fp.fractional_bits);
        const expected_product_raw: i64 = if (expected_product > std.math.maxInt(i64)) std.math.maxInt(i64) else if (expected_product < std.math.minInt(i64)) std.math.minInt(i64) else @intCast(expected_product);
        try std.testing.expectEqual(expected_product_raw, product.raw);
        hash = (hash ^ @as(u64, @bitCast(product.raw))) *% 0x0000_0100_0000_01b3;
        status.clear();
        const quotient = fp.Fp.div(.{ .raw = a }, .{ .raw = b }, &status);
        const expected_quotient = referenceRound(@as(i128, a) << fp.fractional_bits, @as(i128, b));
        const expected_quotient_raw: i64 = if (expected_quotient > std.math.maxInt(i64)) std.math.maxInt(i64) else if (expected_quotient < std.math.minInt(i64)) std.math.minInt(i64) else @intCast(expected_quotient);
        try std.testing.expectEqual(expected_quotient_raw, quotient.raw);
        hash = (hash ^ @as(u64, @bitCast(quotient.raw))) *% 0x0000_0100_0000_01b3;
    }
    // This fixed FNV-1a digest is also compared by the three optimize modes.
    try std.testing.expectEqual(@as(u64, 0x7e3a_89fe_c3b0_e44c), hash);
}
