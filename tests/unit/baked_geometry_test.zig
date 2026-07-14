const std = @import("std");
const baked = @import("gravity").geometry.baked;
const g = @import("gravity").math.geometry;
const fp = @import("gravity").math.fp;

const values = [_]g.Vec3{ .{ .x = fp.Fp.zero, .y = fp.Fp.zero, .z = fp.Fp.zero }, .{ .x = fp.Fp.one, .y = fp.Fp.zero, .z = fp.Fp.zero }, .{ .x = fp.Fp.zero, .y = fp.Fp.one, .z = fp.Fp.zero } };
const triangles = [_]baked.Triangle{.{ .a = 0, .b = 1, .c = 2 }};
const nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = .{ .x = fp.Fp.zero, .y = fp.Fp.zero, .z = fp.Fp.zero }, .max = .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.zero } }, 0, 1)};
const primitives = [_]u32{0};
fn expectRawNear(expected: i64, actual: i64) !void {
    const delta = @as(i128, expected) - actual;
    try std.testing.expect(@abs(delta) <= 8);
}
fn mesh() baked.Mesh {
    return .{ .source_id = 42, .vertices = &values, .triangles = &triangles, .nodes = &nodes, .primitives = &primitives };
}
test "mesh bake is canonical and domain hashed" {
    const input = mesh();
    var a: [512]u8 = undefined;
    var b: [512]u8 = undefined;
    var scratch: [256]u8 = undefined;
    const first = try baked.encodeMesh(input, &a, &scratch);
    const second = try baked.encodeMesh(input, &b, &scratch);
    try std.testing.expectEqualSlices(u8, first.bytes, second.bytes);
    try std.testing.expectEqualSlices(u8, &first.content_hash, &second.content_hash);
}
test "mesh bake rejects invalid primitive and degenerate triangles" {
    var input = mesh();
    var bad = [_]u32{1};
    input.primitives = &bad;
    try std.testing.expectError(error.InvalidBvh, baked.validateMesh(input));
    input = mesh();
    var degenerate_triangles = [_]baked.Triangle{.{ .a = 0, .b = 0, .c = 2 }};
    input.triangles = &degenerate_triangles;
    try std.testing.expectError(error.DegenerateTriangle, baked.validateMesh(input));
}

test "deterministic bvh builder covers every primitive with adjacent child roots" {
    const verts = [_]g.Vec3{
        .{ .x = fp.Fp.zero, .y = fp.Fp.zero, .z = fp.Fp.zero }, .{ .x = fp.Fp.one, .y = fp.Fp.zero, .z = fp.Fp.zero }, .{ .x = fp.Fp.zero, .y = fp.Fp.one, .z = fp.Fp.zero }, .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.zero }, .{ .x = fp.Fp.fromInt(2), .y = fp.Fp.zero, .z = fp.Fp.zero }, .{ .x = fp.Fp.fromInt(2), .y = fp.Fp.one, .z = fp.Fp.zero }, .{ .x = fp.Fp.fromInt(3), .y = fp.Fp.zero, .z = fp.Fp.zero },
    };
    const tris = [_]baked.Triangle{ .{ .a = 0, .b = 1, .c = 2 }, .{ .a = 1, .b = 3, .c = 2 }, .{ .a = 1, .b = 4, .c = 5 }, .{ .a = 1, .b = 5, .c = 3 }, .{ .a = 4, .b = 6, .c = 5 } };
    var ids: [5]u32 = undefined;
    var nodes_buf: [9]baked.BvhNode = undefined;
    const first = try baked.buildTriangleBvh(&verts, &tris, &nodes_buf, &ids);
    try baked.validateBvh(first.nodes, first.primitives, tris.len);
    var ids_again: [5]u32 = undefined;
    var nodes_again: [9]baked.BvhNode = undefined;
    const second = try baked.buildTriangleBvh(&verts, &tris, &nodes_again, &ids_again);
    try std.testing.expectEqualSlices(u32, first.primitives, second.primitives);
}

test "compound bvh accepts the product child limit with deterministic leaves" {
    var children: [256]baked.CompoundChildBounds = undefined;
    for (&children, 0..) |*child, i| {
        const x = fp.Fp.fromInt(@intCast(i));
        child.* = .{ .ordinal = @intCast(i), .bounds = .{ .min = .{ .x = x, .y = fp.Fp.zero, .z = fp.Fp.zero }, .max = .{ .x = .{ .raw = x.raw + fp.Fp.one.raw }, .y = fp.Fp.one, .z = fp.Fp.one } } };
    }
    var node_buffer: [511]baked.BvhNode = undefined;
    var primitive_buffer: [256]u32 = undefined;
    const first = try baked.buildCompoundBvh(&children, &node_buffer, &primitive_buffer);
    try std.testing.expect(first.nodes.len > 1);
    try std.testing.expectEqual(@as(usize, 256), first.primitives.len);
    var seen: [256]bool = [_]bool{false} ** 256;
    for (first.primitives) |id| seen[id] = true;
    for (seen) |present| try std.testing.expect(present);
    for (first.nodes) |node| if (node.flags & baked.BvhNode.leaf_flag != 0) try std.testing.expect(node.count <= 4);
}

test "closed tetrahedron integrates full central symmetric inertia" {
    const verts = [_]g.Vec3{
        .{ .x = fp.Fp.zero, .y = fp.Fp.zero, .z = fp.Fp.zero },
        .{ .x = fp.Fp.one, .y = fp.Fp.zero, .z = fp.Fp.zero },
        .{ .x = fp.Fp.zero, .y = fp.Fp.one, .z = fp.Fp.zero },
        .{ .x = fp.Fp.zero, .y = fp.Fp.zero, .z = fp.Fp.one },
    };
    const tris = [_]baked.Triangle{ .{ .a = 0, .b = 2, .c = 1 }, .{ .a = 0, .b = 1, .c = 3 }, .{ .a = 0, .b = 3, .c = 2 }, .{ .a = 1, .b = 2, .c = 3 } };
    var status = fp.MathStatus{};
    const mass = try baked.meshMassProperties(&verts, &tris, &status);
    const one_sixth = fp.Fp.fromRatio(1, 6, &status);
    const quarter = fp.Fp.fromRatio(1, 4, &status);
    const one_80th = fp.Fp.fromRatio(1, 80, &status);
    const one_480th = fp.Fp.fromRatio(1, 480, &status);
    try expectRawNear(one_sixth.raw, mass.volume.raw);
    try expectRawNear(quarter.raw, mass.center.x.raw);
    try expectRawNear(quarter.raw, mass.center.y.raw);
    try expectRawNear(quarter.raw, mass.center.z.raw);
    try expectRawNear(one_80th.raw, mass.inertia.xx.raw);
    try expectRawNear(one_80th.raw, mass.inertia.yy.raw);
    try expectRawNear(one_80th.raw, mass.inertia.zz.raw);
    try expectRawNear(one_480th.raw, mass.inertia.xy.raw);
    try expectRawNear(one_480th.raw, mass.inertia.xz.raw);
    try expectRawNear(one_480th.raw, mass.inertia.yz.raw);
}

test "convex hull canonicalizes input and emits reciprocal half edges" {
    const input = [_]g.Vec3{
        .{ .x = fp.Fp.one, .y = fp.Fp.zero, .z = fp.Fp.zero },
        .{ .x = fp.Fp.zero, .y = fp.Fp.zero, .z = fp.Fp.one },
        .{ .x = fp.Fp.zero, .y = fp.Fp.zero, .z = fp.Fp.zero },
        .{ .x = fp.Fp.zero, .y = fp.Fp.one, .z = fp.Fp.zero },
        .{ .x = fp.Fp.one, .y = fp.Fp.zero, .z = fp.Fp.zero }, // duplicate
    };
    var vertices: [5]g.Vec3 = undefined;
    var triangles_out: [8]baked.Triangle = undefined;
    var faces: [8]baked.HullFace = undefined;
    var edges: [24]baked.HalfEdge = undefined;
    var status = fp.MathStatus{};
    const hull = try baked.buildConvexHull(&input, &vertices, &triangles_out, &faces, &edges, &status);
    try std.testing.expectEqual(@as(usize, 4), hull.vertices.len);
    try std.testing.expectEqual(@as(usize, 4), hull.faces.len);
    try baked.validateHullTopology(hull.vertices, hull.faces, hull.half_edges);
    try expectRawNear(fp.Fp.fromRatio(1, 6, &status).raw, hull.mass.volume.raw);
    var encoded: [2048]u8 = undefined;
    var scratch: [512]u8 = undefined;
    const bytes = try baked.encodeConvexHull(hull, 17, &encoded, &scratch);
    try std.testing.expectEqual(.convex_hull, (try baked.validateEncoded(bytes.bytes)).kind);
}

test "convex hull groups coplanar cube facets into canonical polygon faces" {
    const zero = fp.Fp.zero;
    const one = fp.Fp.one;
    // Deliberately not lexicographic: the baked topology must not depend on
    // source ordering, and each square remains one four-edge face.
    const input = [_]g.Vec3{
        .{ .x = one, .y = one, .z = one },   .{ .x = zero, .y = zero, .z = zero },
        .{ .x = one, .y = zero, .z = one },  .{ .x = zero, .y = one, .z = zero },
        .{ .x = one, .y = one, .z = zero },  .{ .x = zero, .y = zero, .z = one },
        .{ .x = one, .y = zero, .z = zero }, .{ .x = zero, .y = one, .z = one },
    };
    var vertices: [8]g.Vec3 = undefined;
    var triangles_out: [12]baked.Triangle = undefined;
    var faces: [6]baked.HullFace = undefined;
    var edges: [24]baked.HalfEdge = undefined;
    var status = fp.MathStatus{};
    const hull = try baked.buildConvexHull(&input, &vertices, &triangles_out, &faces, &edges, &status);
    try std.testing.expectEqual(@as(usize, 6), hull.faces.len);
    try std.testing.expectEqual(@as(usize, 12), hull.triangles.len);
    try std.testing.expectEqual(@as(usize, 24), hull.half_edges.len);
    for (hull.faces) |face| try std.testing.expectEqual(@as(u32, 4), face.half_edge_count);
    try baked.validateHullTopology(hull.vertices, hull.faces, hull.half_edges);
    try expectRawNear(fp.Fp.one.raw, hull.mass.volume.raw);
}

test "height field tiles preserve row-major minmax, holes, materials and canonical bytes" {
    const samples = [_]fp.Fp{
        fp.Fp.fromInt(3), fp.Fp.fromInt(-2), fp.Fp.fromInt(7),
        fp.Fp.fromInt(4), fp.Fp.fromInt(9),  fp.Fp.fromInt(1),
        fp.Fp.fromInt(5), fp.Fp.fromInt(0),  fp.Fp.fromInt(6),
    };
    const cells = [_]baked.HeightCell{ .{ .material_id = 11 }, .{ .hole = true, .material_id = 12 }, .{ .material_id = 13 }, .{ .material_id = 14 } };
    var tile_nodes: [1]baked.BvhNode = undefined;
    const tiles = try baked.buildHeightFieldTiles(3, 3, &samples, &cells, &tile_nodes);
    try std.testing.expectEqual(@as(usize, 1), tiles.len);
    try std.testing.expectEqual(fp.Fp.fromInt(-2).raw, tiles[0].bounds.min.y.raw);
    try std.testing.expectEqual(fp.Fp.fromInt(9).raw, tiles[0].bounds.max.y.raw);
    const field = baked.HeightField{ .source_id = 99, .width = 3, .height = 3, .samples = &samples, .cells = &cells, .tile_nodes = tiles };
    var a: [1024]u8 = undefined;
    var b: [1024]u8 = undefined;
    var scratch: [256]u8 = undefined;
    const first = try baked.encodeHeightField(field, &a, &scratch);
    const second = try baked.encodeHeightField(field, &b, &scratch);
    try std.testing.expectEqualSlices(u8, first.bytes, second.bytes);
    try std.testing.expectEqual(.height_field, (try baked.validateEncoded(first.bytes)).kind);
}

test "mesh welding canonicalizes order and dynamic mass requires override for open mesh" {
    const verts = [_]g.Vec3{
        .{ .x = fp.Fp.one, .y = fp.Fp.zero, .z = fp.Fp.zero },
        .{ .x = fp.Fp.zero, .y = fp.Fp.one, .z = fp.Fp.zero },
        .{ .x = fp.Fp.zero, .y = fp.Fp.zero, .z = fp.Fp.zero },
        .{ .x = fp.Fp{ .raw = fp.Fp.one.raw + 1 }, .y = fp.Fp.zero, .z = fp.Fp.zero },
    };
    const source = [_]baked.Triangle{.{ .a = 3, .b = 1, .c = 2 }};
    var work: [4]baked.VertexWeldRef = undefined;
    var remap: [4]u32 = undefined;
    var out_verts: [4]g.Vec3 = undefined;
    var out_tris: [1]baked.Triangle = undefined;
    const welded = try baked.weldMesh(&verts, &source, &work, &remap, &out_verts, &out_tris);
    try std.testing.expectEqual(@as(usize, 3), welded.vertices.len);
    var adjacency: [3]baked.MeshAdjacency = undefined;
    var status = fp.MathStatus{};
    try std.testing.expectError(error.InvalidMass, baked.dynamicMeshMass(welded.vertices, welded.triangles, &adjacency, null, &status));
    const override = baked.MassProperties{ .volume = fp.Fp.one, .center = g.Vec3.zero, .inertia = .{ .xx = fp.Fp.one, .yy = fp.Fp.one, .zz = fp.Fp.one, .xy = fp.Fp.zero, .xz = fp.Fp.zero, .yz = fp.Fp.zero } };
    _ = try baked.dynamicMeshMass(welded.vertices, welded.triangles, &adjacency, override, &status);
}

test "self intersecting nonadjacent triangles are rejected by integer SAT" {
    const verts = [_]g.Vec3{
        .{ .x = fp.Fp.fromInt(-2), .y = fp.Fp.fromInt(-1), .z = fp.Fp.zero }, .{ .x = fp.Fp.fromInt(2), .y = fp.Fp.fromInt(-1), .z = fp.Fp.zero }, .{ .x = fp.Fp.zero, .y = fp.Fp.fromInt(2), .z = fp.Fp.zero },
        .{ .x = fp.Fp.zero, .y = fp.Fp.zero, .z = fp.Fp.fromInt(-1) },        .{ .x = fp.Fp.zero, .y = fp.Fp.zero, .z = fp.Fp.fromInt(1) },        .{ .x = fp.Fp.one, .y = fp.Fp.zero, .z = fp.Fp.zero },
    };
    const tris = [_]baked.Triangle{ .{ .a = 0, .b = 1, .c = 2 }, .{ .a = 3, .b = 4, .c = 5 } };
    try std.testing.expectError(error.InvalidTopology, baked.validateMeshSelfIntersection(&verts, &tris));
}
