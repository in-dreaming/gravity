//! Bounded BVH and mesh-mesh ordering/symmetry fuzz.
const std = @import("std");
const gravity = @import("gravity");
const fp = gravity.math.fp;
const g = gravity.math.geometry;
const baked = gravity.geometry.baked;
const mesh = gravity.collision.mesh;

fn next(seed: *u64) i32 {
    seed.* = seed.* *% 2_862_933_555_777_941_757 +% 3_037_000_493;
    return @intCast(@as(i64, @intCast((seed.* >> 32) % 17)) - 8);
}

fn point(seed: *u64) g.Vec3 {
    return .{ .x = fp.Fp.fromInt(next(seed)), .y = fp.Fp.fromInt(next(seed)), .z = fp.Fp.fromInt(next(seed)) };
}

test "degenerate and arbitrary triangle SAT remains symmetric" {
    var seed: u64 = 0x25_4d45_5348;
    for (0..20_000) |_| {
        const a = mesh.Triangle{ .a = point(&seed), .b = point(&seed), .c = point(&seed) };
        const b = mesh.Triangle{ .a = point(&seed), .b = point(&seed), .c = point(&seed) };
        try std.testing.expectEqual(mesh.trianglesOverlap(a, b), mesh.trianglesOverlap(b, a));
    }
}

test "mesh BVH pair candidates are stable and transactionally capacity checked" {
    const bounds = g.Aabb3{ .min = .{ .x = fp.Fp.fromInt(-8), .y = fp.Fp.fromInt(-8), .z = fp.Fp.fromInt(-8) }, .max = .{ .x = fp.Fp.fromInt(8), .y = fp.Fp.fromInt(8), .z = fp.Fp.fromInt(8) } };
    const nodes = [_]baked.BvhNode{baked.BvhNode.leaf(bounds, 0, 4)};
    const primitives = [_]u32{ 9, 1, 7, 3 };
    var work: [4]mesh.NodePair = undefined;
    var scratch_a: [16]mesh.PrimitivePair = undefined;
    var output_a: [16]mesh.PrimitivePair = undefined;
    var scratch_b: [16]mesh.PrimitivePair = undefined;
    var output_b: [16]mesh.PrimitivePair = undefined;
    const first = try mesh.traverseBvhPairs(&nodes, &primitives, &nodes, &primitives, &work, &scratch_a, &output_a);
    const second = try mesh.traverseBvhPairs(&nodes, &primitives, &nodes, &primitives, &work, &scratch_b, &output_b);
    try std.testing.expectEqualSlices(mesh.PrimitivePair, first, second);
    try std.testing.expectEqual(@as(usize, 16), first.len);
    for (first[1..], first[0 .. first.len - 1]) |right, left| try std.testing.expect(left.a < right.a or (left.a == right.a and left.b <= right.b));
    var sentinel = [_]mesh.PrimitivePair{.{ .a = 0xdeadbeef, .b = 0xdeadbeef }};
    try std.testing.expectError(error.CapacityExceeded, mesh.traverseBvhPairs(&nodes, &primitives, &nodes, &primitives, &work, &scratch_a, &sentinel));
    try std.testing.expectEqual(@as(u32, 0xdeadbeef), sentinel[0].a);
}
