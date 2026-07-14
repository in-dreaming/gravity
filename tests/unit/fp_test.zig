const std = @import("std");
const fp = @import("gravity").math.fp;
const wide = @import("gravity").math.wide;
const envelope = @import("gravity").math.envelope;

test "Fp boundaries, division, and first fault" {
    var status = fp.MathStatus{};
    try std.testing.expectEqual(fp.Fp.max.raw, fp.Fp.add(fp.Fp.max, fp.Fp.one, &status).raw);
    try std.testing.expectEqual(fp.MathFault.overflow, status.fault);
    _ = fp.Fp.div(fp.Fp.one, fp.Fp.zero, &status);
    try std.testing.expectEqual(fp.MathFault.overflow, status.fault);
    status.clear();
    try std.testing.expectEqual(fp.Fp.max.raw, fp.Fp.div(fp.Fp.one, fp.Fp.zero, &status).raw);
    try std.testing.expectEqual(fp.MathFault.divide_by_zero, status.fault);
    status.clear();
    try std.testing.expectEqual(@as(i64, 0), fp.Fp.sqrt(fp.Fp{ .raw = -1 }, &status).raw);
    try std.testing.expectEqual(fp.MathFault.negative_sqrt, status.fault);
}

test "ties are rounded to even for signed quotient" {
    try std.testing.expectEqual(@as(i128, 2), fp.roundDivTiesEven(5, 2));
    try std.testing.expectEqual(@as(i128, 2), fp.roundDivTiesEven(3, 2));
    try std.testing.expectEqual(@as(i128, -2), fp.roundDivTiesEven(-5, 2));
    try std.testing.expectEqual(@as(i128, -2), fp.roundDivTiesEven(-3, 2));
    var status = fp.MathStatus{};
    try std.testing.expectEqual(@as(i64, 2_147_483_648), fp.Fp.fromRatio(1, 2, &status).raw);
    try std.testing.expectEqual(fp.Fp.min.raw, fp.Fp.fromRatio(-1, 0, &status).raw);
}

test "sqrt, decimal, wide dot, and envelope" {
    var status = fp.MathStatus{};
    const one_point_five = fp.Fp.parseCanonicalDecimal("1.5", &status);
    try std.testing.expectEqual(@as(i64, 6_442_450_944), one_point_five.raw);
    var buffer: [64]u8 = undefined;
    const text = one_point_five.formatCanonical(&buffer).?;
    try std.testing.expectEqualStrings("1.5", text);
    try std.testing.expectEqual(one_point_five.raw, fp.Fp.parseCanonicalDecimal(text, &status).raw);
    const half = fp.Fp{ .raw = fp.Fp.one.raw / 2 };
    try std.testing.expectEqual(@as(i64, 3 * (fp.Fp.one.raw / 4)), wide.dot3(.{ half, half, half }, .{ half, half, half }, &status).raw);
    try envelope.ProductEnvelope.product_default.validate();
    var invalid = envelope.ProductEnvelope.product_default;
    invalid.max_position = fp.Fp.max;
    try std.testing.expectError(error.InvalidEnvelope, invalid.validate());
}
