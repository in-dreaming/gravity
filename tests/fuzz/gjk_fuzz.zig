//! Bounded deterministic corpus for Task 10 GJK classification invariants.
const std = @import("std");
const gravity = @import("gravity");
const gjk = gravity.collision.gjk;
const fp = gravity.math.fp;
const g = gravity.math.geometry;
const shapes = gravity.collision.shapes;
const asset_store = gravity.assets.store;

const Pair = struct { a: g.Vec3, b: g.Vec3, half: fp.Fp };
fn support(raw: *const anyopaque, direction: g.Vec3, status: *fp.MathStatus) gjk.SupportVertex {
    const pair: *const Pair = @ptrCast(@alignCast(raw));
    const a = pair.a.add(.{ .x = if (direction.x.raw >= 0) pair.half else pair.half.neg(status), .y = if (direction.y.raw >= 0) pair.half else pair.half.neg(status), .z = if (direction.z.raw >= 0) pair.half else pair.half.neg(status) }, status);
    const b = pair.b.add(.{ .x = if (direction.x.raw < 0) pair.half else pair.half.neg(status), .y = if (direction.y.raw < 0) pair.half else pair.half.neg(status), .z = if (direction.z.raw < 0) pair.half else pair.half.neg(status) }, status);
    return .{ .point = a.sub(b, status), .witness_a = a, .witness_b = b, .feature_a = 0, .feature_b = 0 };
}

test "bounded overlapping box corpus drives full GJK simplex through EPA" {
    var empty_assets: [0]asset_store.Asset = .{};
    const assets = asset_store.Store{ .assets = &empty_assets, .bytes = &.{}, .asset_set_hash = [_]u8{0} ** 32 };
    const box: shapes.Shape = .{ .box = .{ .half_extents = .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.one } } };
    for (0..5) |xi| for (0..5) |yi| for (0..5) |zi| {
        var status = fp.MathStatus{};
        var shape_pair = gjk.ShapePairContext{
            .shape_a = box,
            .shape_b = box,
            .assets = &assets,
            .transform_b = .{ .position = .{ .x = fp.Fp.fromRatio(@as(i64, @intCast(xi)) - 2, 2, &status), .y = fp.Fp.fromRatio(@as(i64, @intCast(yi)) - 2, 2, &status), .z = fp.Fp.fromRatio(@as(i64, @intCast(zi)) - 2, 2, &status) } },
        };
        const intersection = try gjk.intersectShapes(&shape_pair, g.Vec3.unit_x, &status);
        if (intersection.status != .intersecting or intersection.simplex_len != 4) continue;
        var vertices: [130]gjk.SupportVertex = undefined;
        var faces: [256]gjk.EpaFace = undefined;
        var visible: [256]bool = undefined;
        var horizon: [768]gjk.HorizonEdge = undefined;
        const penetration = gjk.epa(gjk.shapeSupportContext(&shape_pair), intersection.simplex[0..intersection.simplex_len], .{ .vertices = &vertices, .faces = &faces, .visible = &visible, .horizon = &horizon }, &status);
        try std.testing.expect(penetration.status == .converged or penetration.status == .non_convergent);
        if (penetration.status == .converged) try std.testing.expect(penetration.depth.raw >= 0);
    };
}

test "bounded axis-aligned box corpus agrees with interval oracle" {
    var status = fp.MathStatus{};
    for (0..7) |xi| for (0..7) |yi| for (0..7) |zi| {
        const x: i32 = @intCast(xi);
        const y: i32 = @intCast(yi);
        const z: i32 = @intCast(zi);
        const dx = x - 3;
        const dy = y - 3;
        const dz = z - 3;
        var pair = Pair{ .a = g.Vec3.zero, .b = .{ .x = fp.Fp.fromInt(dx), .y = fp.Fp.fromInt(dy), .z = fp.Fp.fromInt(dz) }, .half = fp.Fp.one };
        const result = gjk.intersect(.{ .ptr = &pair, .call = support }, g.Vec3.unit_x, &status);
        const expected = @abs(dx) <= 2 and @abs(dy) <= 2 and @abs(dz) <= 2;
        try std.testing.expectEqual(if (expected) gjk.Status.intersecting else gjk.Status.separated, result.status);
        const distance = gjk.distance(.{ .ptr = &pair, .call = support }, gjk.seedFromResult(result).direction, &status);
        try std.testing.expectEqual(result.status, distance.status);
        try std.testing.expect(distance.distance.raw >= 0);
    };
}
