const std = @import("std");
const gravity = @import("gravity");
const analytic = gravity.collision.analytic;
const fp = gravity.math.fp;
const g = gravity.math.geometry;
const ids = gravity.core.ids;
fn point(x: i32, y: i32, z: i32) g.Vec3 {
    return .{ .x = fp.Fp.fromInt(x), .y = fp.Fp.fromInt(y), .z = fp.Fp.fromInt(z) };
}
fn id(value: u32) ids.ColliderId {
    return ids.ColliderId.init(value, 0);
}
fn expectRawNear(expected: i64, actual: i64) !void {
    try std.testing.expect(@abs(@as(i128, expected) - actual) <= 2);
}

test "sphere pairs return signed separation witnesses and swap normals" {
    var status = fp.MathStatus{};
    const a = analytic.Sphere{ .center = point(0, 0, 0), .radius = fp.Fp.one };
    const b = analytic.Sphere{ .center = point(3, 0, 0), .radius = fp.Fp.one };
    const result = analytic.sphereSphere(a, b, g.Vec3.zero, id(1), id(2), &status);
    try std.testing.expectEqual(fp.Fp.one.raw, result.separation.raw);
    try expectRawNear(fp.Fp.one.raw, result.normal.x.raw);
    const swapped = analytic.sphereSphere(b, a, g.Vec3.zero, id(2), id(1), &status);
    try std.testing.expectEqual(-result.normal.x.raw, swapped.normal.x.raw);
}
test "capsule degeneracy and identical centers have stable fallbacks" {
    var status = fp.MathStatus{};
    const capsule = analytic.Capsule{ .segment = .{ .a = point(0, -1, 0), .b = point(0, 1, 0) }, .radius = fp.Fp.one };
    const sphere = analytic.Sphere{ .center = point(0, 0, 0), .radius = fp.Fp.one };
    const result = analytic.sphereCapsule(sphere, capsule, g.Vec3.zero, id(1), id(2), &status);
    try std.testing.expectEqual(fp.Fp{ .raw = -2 * fp.Fp.one.raw }, result.separation);
    try std.testing.expectEqual(fp.Fp.one.raw, result.normal.x.raw);
    const closest = analytic.closestSegments(.{ .a = point(0, 0, 0), .b = point(0, 0, 0) }, .{ .a = point(2, 0, 0), .b = point(2, 0, 0) }, &status);
    try std.testing.expectEqual(fp.Fp.fromInt(2).raw, closest.b.x.raw);
}
test "sphere box returns closest witness for separated and touching cases" {
    var status = fp.MathStatus{};
    const box = analytic.Obb{ .center = g.Vec3.zero, .half_extents = point(1, 1, 1), .orientation = g.Quat.identity };
    const sphere = analytic.Sphere{ .center = point(3, 0, 0), .radius = fp.Fp.one };
    const result = analytic.sphereBox(sphere, box, g.Vec3.zero, id(1), id(2), &status);
    try expectRawNear(fp.Fp.one.raw, result.separation.raw);
    try expectRawNear(-fp.Fp.one.raw, result.normal.x.raw);
    try std.testing.expectEqual(fp.Fp.one.raw, result.witness_b.x.raw);
    const inside = analytic.sphereBox(.{ .center = g.Vec3.zero, .radius = fp.Fp.one }, box, g.Vec3.zero, id(1), id(2), &status);
    try expectRawNear(fp.Fp.zero.raw, inside.separation.raw);
}
test "sphere triangle returns primitive witness and face fallback" {
    var status = fp.MathStatus{};
    const triangle = analytic.Triangle{ .a = point(-1, 0, -1), .b = point(1, 0, -1), .c = point(0, 0, 1) };
    const sphere = analytic.Sphere{ .center = point(0, 2, 0), .radius = fp.Fp.one };
    const separated = analytic.sphereTriangle(sphere, triangle, 17, g.Vec3.zero, id(1), id(2), &status);
    try expectRawNear(fp.Fp.one.raw, separated.separation.raw);
    try std.testing.expectEqual(@as(u32, 17), separated.feature_b.primitive);
    const overlap = analytic.sphereTriangle(.{ .center = point(0, 0, 0), .radius = fp.Fp.one }, triangle, 17, g.Vec3.zero, id(1), id(2), &status);
    try std.testing.expect(overlap.separation.raw < 0);
    try std.testing.expect(overlap.normal.y.raw != 0);
}
test "capsule triangle detects a segment piercing the face" {
    var status = fp.MathStatus{};
    const triangle = analytic.Triangle{ .a = point(-2, 0, -2), .b = point(2, 0, -2), .c = point(0, 0, 2) };
    const capsule = analytic.Capsule{ .segment = .{ .a = point(0, -1, 0), .b = point(0, 1, 0) }, .radius = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status) };
    const result = analytic.capsuleTriangle(capsule, triangle, 5, g.Vec3.zero, id(1), id(2), &status);
    try std.testing.expectEqual(-fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status).raw, result.separation.raw);
    try std.testing.expectEqual(@as(u32, 5), result.feature_b.primitive);
    try std.testing.expectEqual(fp.Fp.zero.raw, result.witness_b.y.raw);
}
test "capsule box uses segment witness instead of a bounding sphere" {
    var status = fp.MathStatus{};
    const box = analytic.Obb{ .center = g.Vec3.zero, .half_extents = point(1, 1, 1), .orientation = g.Quat.identity };
    const capsule = analytic.Capsule{ .segment = .{ .a = point(3, 0, 0), .b = point(5, 0, 0) }, .radius = fp.Fp.one };
    const result = analytic.capsuleBox(capsule, box, g.Vec3.zero, id(1), id(2), &status);
    try expectRawNear(fp.Fp.one.raw, result.separation.raw);
    try expectRawNear(-fp.Fp.one.raw, result.normal.x.raw);
}
test "box SAT finds face separation and overlap with ordered normal" {
    var status = fp.MathStatus{};
    const a = analytic.Obb{ .center = g.Vec3.zero, .half_extents = point(1, 1, 1), .orientation = g.Quat.identity };
    const separated = analytic.Obb{ .center = point(3, 0, 0), .half_extents = point(1, 1, 1), .orientation = g.Quat.identity };
    const result = analytic.boxBox(a, separated, g.Vec3.zero, id(1), id(2), &status);
    try expectRawNear(fp.Fp.one.raw, result.separation.raw);
    try expectRawNear(fp.Fp.one.raw, result.normal.x.raw);
    const overlap = analytic.Obb{ .center = point(1, 0, 0), .half_extents = point(1, 1, 1), .orientation = g.Quat.identity };
    try std.testing.expect((analytic.boxBox(a, overlap, g.Vec3.zero, id(1), id(2), &status)).separation.raw < 0);
}
test "analytic dispatcher preserves caller ordering under swaps" {
    var status = fp.MathStatus{};
    const sphere: analytic.QueryShape = .{ .sphere = .{ .center = point(3, 0, 0), .radius = fp.Fp.one } };
    const box: analytic.QueryShape = .{ .box = .{ .center = g.Vec3.zero, .half_extents = point(1, 1, 1), .orientation = g.Quat.identity } };
    const first = analytic.collide(sphere, box, g.Vec3.zero, id(1), id(2), &status).?;
    const second = analytic.collide(box, sphere, g.Vec3.zero, id(2), id(1), &status).?;
    try expectRawNear(-first.normal.x.raw, second.normal.x.raw);
    try std.testing.expectEqual(first.witness_a.x.raw, second.witness_b.x.raw);
}
test "rotated box SAT detects a cross-axis separation" {
    var status = fp.MathStatus{};
    const half_angle = g.pi.div(fp.Fp.fromInt(4), &status);
    const trig = g.cordic(half_angle);
    const rotated = g.Quat{ .z = trig.sin, .w = trig.cos };
    const a = analytic.Obb{ .center = g.Vec3.zero, .half_extents = point(2, 1, 1), .orientation = rotated };
    const b = analytic.Obb{ .center = point(0, 4, 0), .half_extents = point(1, 1, 1), .orientation = g.Quat.identity };
    const result = analytic.boxBox(a, b, g.Vec3.zero, id(1), id(2), &status);
    try std.testing.expect(result.separation.raw > 0);
}
test "point triangle closest covers face edge and vertex regions" {
    var status = fp.MathStatus{};
    const triangle = analytic.Triangle{ .a = point(0, 0, 0), .b = point(2, 0, 0), .c = point(0, 2, 0) };
    const face = analytic.closestPointTriangle(point(1, 1, 3), triangle, &status);
    try expectRawNear(fp.Fp.fromInt(1).raw, face.x.raw);
    try expectRawNear(fp.Fp.fromInt(1).raw, face.y.raw);
    const edge = analytic.closestPointTriangle(point(2, 2, 0), triangle, &status);
    try expectRawNear(fp.Fp.one.raw, edge.x.raw);
    try expectRawNear(fp.Fp.one.raw, edge.y.raw);
    const vertex = analytic.closestPointTriangle(point(-2, -1, 0), triangle, &status);
    try std.testing.expectEqual(fp.Fp.zero.raw, vertex.x.raw);
}
