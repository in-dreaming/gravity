const std = @import("std");
const g = @import("gravity").math.geometry;
const fp = @import("gravity").math.fp;

fn expectNear(actual: fp.Fp, expected: fp.Fp, tolerance_raw: i64) !void {
    const difference = actual.raw -% expected.raw;
    const absolute = if (difference < 0) -difference else difference;
    try std.testing.expect(absolute <= tolerance_raw);
}

fn quarterTurnZ(status: *fp.MathStatus) g.Quat {
    return g.Quat.canonicalize(.{
        .z = fp.Fp.fromRatio(70_710_678, 100_000_000, status),
        .w = fp.Fp.fromRatio(70_710_678, 100_000_000, status),
    }, status);
}

test "quaternion canonicalization, composition, and matrix equivariance" {
    var s = fp.MathStatus{};
    const q = g.Quat{ .z = fp.Fp.fromRatio(1, 2, &s), .w = fp.Fp.fromRatio(-1, 2, &s) };
    const a = g.Quat.canonicalize(q, &s);
    const b = g.Quat.canonicalize(.{ .z = q.z.neg(&s), .w = q.w.neg(&s) }, &s);
    try std.testing.expectEqual(a.x.raw, b.x.raw);
    try std.testing.expectEqual(a.y.raw, b.y.raw);
    try std.testing.expectEqual(a.z.raw, b.z.raw);
    try std.testing.expectEqual(a.w.raw, b.w.raw);

    const turn = quarterTurnZ(&s);
    const twice = turn.mul(turn, &s);
    const v = g.Vec3.unit_x;
    const rotated = turn.rotate(v, &s);
    try expectNear(rotated.x, fp.Fp.zero, 16);
    try expectNear(rotated.y, fp.Fp.one, 16);
    try expectNear(turn.inverseRotate(rotated, &s).x, v.x, 32);
    try expectNear(turn.inverseRotate(rotated, &s).y, v.y, 32);
    try expectNear(twice.rotate(v, &s).x, fp.Fp{ .raw = -fp.Fp.one.raw }, 64);

    const matrix_rotated = turn.toMat3(&s).mulVec(v, &s);
    try expectNear(matrix_rotated.x, rotated.x, 32);
    try expectNear(matrix_rotated.y, rotated.y, 32);
    try expectNear(matrix_rotated.z, rotated.z, 32);
    try std.testing.expectEqual(fp.MathFault.none, s.fault);
}

test "long quaternion integration remains unit and canonical" {
    var s = fp.MathStatus{};
    var q = g.Quat.identity;
    const omega = g.Vec3{ .z = fp.Fp.one };
    const dt = fp.Fp.fromRatio(1, 60, &s);
    // One second at the engine tick rate exercises repeated normalize/canonicalize
    // in every optimization mode and cross-target validation run.
    for (0..60) |_| q = q.integrate(omega, dt, &s);
    const length_squared = q.x.mul(q.x, &s).add(q.y.mul(q.y, &s), &s).add(q.z.mul(q.z, &s), &s).add(q.w.mul(q.w, &s), &s);
    try expectNear(length_squared, fp.Fp.one, 32);
    try std.testing.expect(q.w.raw > 0 or (q.w.raw == 0 and (q.x.raw > 0 or (q.x.raw == 0 and (q.y.raw > 0 or (q.y.raw == 0 and q.z.raw >= 0))))));
    try std.testing.expectEqual(fp.MathFault.none, s.fault);
}

test "matrix inverse, symmetric inertia rotation, transform, and failures" {
    var s = fp.MathStatus{};
    const m = g.Mat3{ .m = .{ fp.Fp.fromInt(2), fp.Fp.one, fp.Fp.zero, fp.Fp.zero, fp.Fp.fromInt(3), fp.Fp.one, fp.Fp.one, fp.Fp.zero, fp.Fp.one } };
    const inverse = m.inverse(&s);
    try std.testing.expect(inverse.valid);
    const product = m.mul(inverse.value, &s);
    inline for (0..9) |i| try expectNear(product.m[i], g.Mat3.identity.m[i], 128);

    const turn = quarterTurnZ(&s);
    const inertia = g.SymmetricMat3{ .xx = fp.Fp.fromInt(2), .yy = fp.Fp.fromInt(3), .zz = fp.Fp.fromInt(4), .xy = fp.Fp.zero, .xz = fp.Fp.zero, .yz = fp.Fp.zero };
    const world_inertia = inertia.rotate(turn, &s);
    try expectNear(world_inertia.xx, inertia.yy, 64);
    try expectNear(world_inertia.yy, inertia.xx, 64);
    try expectNear(world_inertia.zz, inertia.zz, 64);

    const transform = g.Transform3{ .position = .{ .x = fp.Fp.fromInt(4), .y = fp.Fp.fromInt(-2) }, .orientation = turn };
    const point = g.Vec3{ .x = fp.Fp.fromInt(3), .y = fp.Fp.fromInt(1), .z = fp.Fp.fromInt(-5) };
    const round_trip = transform.inverseApply(transform.apply(point, &s), &s);
    try expectNear(round_trip.x, point.x, 64);
    try expectNear(round_trip.y, point.y, 64);
    try expectNear(round_trip.z, point.z, 64);

    const singular = g.Mat3{ .m = .{ fp.Fp.zero, fp.Fp.zero, fp.Fp.zero, fp.Fp.zero, fp.Fp.zero, fp.Fp.zero, fp.Fp.zero, fp.Fp.zero, fp.Fp.zero } };
    s.clear();
    try std.testing.expect(!singular.inverse(&s).valid);
    try std.testing.expectEqual(fp.MathFault.divide_by_zero, s.fault);
    s.clear();
    try std.testing.expect(!g.Vec3.zero.normalize(&s).valid);
    try std.testing.expect(s.fault != .none);
}

test "AABB touching and swept boundaries contain both endpoints" {
    var s = fp.MathStatus{};
    const a = g.Aabb3{ .min = g.Vec3.zero, .max = g.Vec3{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.one } };
    const touching = g.Aabb3{ .min = g.Vec3{ .x = fp.Fp.one }, .max = g.Vec3{ .x = fp.Fp.fromInt(2), .y = fp.Fp.one, .z = fp.Fp.one } };
    try std.testing.expect(a.overlaps(touching));
    const delta = g.Vec3{ .x = fp.Fp.fromInt(-3), .y = fp.Fp.fromInt(2), .z = fp.Fp.fromInt(-4) };
    const swept = a.swept(delta, &s);
    try std.testing.expectEqual(fp.Fp.fromInt(-3).raw, swept.min.x.raw);
    try std.testing.expectEqual(fp.Fp.fromInt(3).raw, swept.max.y.raw);
    try std.testing.expectEqual(fp.Fp.fromInt(-4).raw, swept.min.z.raw);
    try std.testing.expectEqual(fp.Fp.one.raw, swept.max.z.raw);
    try std.testing.expectEqual(fp.MathFault.none, s.fault);
}

test "CORDIC frozen table cardinal golden and hash" {
    const t = g.generateTrigTable();
    try std.testing.expect(@abs(t[0].sin.raw) <= 16);
    try std.testing.expect(@abs(t[0].cos.raw - fp.Fp.one.raw) <= 16);
    try std.testing.expect(@abs(t[256].sin.raw - fp.Fp.one.raw) <= 16);
    try std.testing.expect(@abs(t[256].cos.raw) <= 16);
    try std.testing.expect(@abs(t[512].sin.raw) <= 16);
    try std.testing.expect(@abs(t[512].cos.raw + fp.Fp.one.raw) <= 16);
    try std.testing.expect(@abs(t[768].sin.raw + fp.Fp.one.raw) <= 16);
    try std.testing.expect(@abs(t[768].cos.raw) <= 16);
    try std.testing.expectEqualSlices(u8, &.{ 0x26, 0xb1, 0x78, 0x52, 0xd0, 0x3c, 0x28, 0xd8, 0xca, 0xfb, 0x5b, 0x0c, 0x68, 0x66, 0xf2, 0x33, 0x10, 0xa9, 0x27, 0x35, 0x2b, 0xe8, 0x4f, 0xba, 0x92, 0x75, 0xb2, 0x1a, 0xff, 0xc7, 0x6a, 0x15 }, &g.trigTableHash(&t));
}
