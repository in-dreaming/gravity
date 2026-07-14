//! Bounded deterministic properties for Task 06 baked geometry.
const std = @import("std");
const baked = @import("gravity").geometry.baked;
const g = @import("gravity").math.geometry;
const fp = @import("gravity").math.fp;

test "BVH and welding remain canonical over bounded permutations" {
    const verts = [_]g.Vec3{
        .{ .x = fp.Fp.zero, .y = fp.Fp.zero, .z = fp.Fp.zero },
        .{ .x = fp.Fp.one, .y = fp.Fp.zero, .z = fp.Fp.zero },
        .{ .x = fp.Fp.zero, .y = fp.Fp.one, .z = fp.Fp.zero },
        .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.zero },
        .{ .x = fp.Fp.fromInt(2), .y = fp.Fp.zero, .z = fp.Fp.zero },
        .{ .x = fp.Fp.fromInt(2), .y = fp.Fp.one, .z = fp.Fp.zero },
    };
    const triangles = [_]baked.Triangle{ .{ .a = 0, .b = 1, .c = 2 }, .{ .a = 1, .b = 3, .c = 2 }, .{ .a = 1, .b = 4, .c = 3 }, .{ .a = 4, .b = 5, .c = 3 } };
    var expected: [4]u32 = undefined;
    var expected_nodes: [7]baked.BvhNode = undefined;
    const first = try baked.buildTriangleBvh(&verts, &triangles, &expected_nodes, &expected);
    var iteration: u32 = 0;
    while (iteration < 64) : (iteration += 1) {
        var ids: [4]u32 = undefined;
        var nodes: [7]baked.BvhNode = undefined;
        const result = try baked.buildTriangleBvh(&verts, &triangles, &nodes, &ids);
        try baked.validateBvh(result.nodes, result.primitives, triangles.len);
        try std.testing.expectEqualSlices(u32, first.primitives, result.primitives);
    }
}
