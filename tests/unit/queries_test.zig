const std = @import("std");
const gravity = @import("gravity");
const fp = gravity.math.fp;
const g = gravity.math.geometry;
const queries = gravity.query.queries;
const baked = gravity.geometry.baked;
const store = gravity.assets.store;

fn oneTriangleStore(memory: []u8) !store.Store {
    const vertices = [_]g.Vec3{ .{}, .{ .x = .one }, .{ .y = .one } };
    const triangles = [_]baked.Triangle{.{ .a = 0, .b = 1, .c = 2 }};
    const nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = .{}, .max = .{ .x = .one, .y = .one } }, 0, 1)};
    const primitives = [_]u32{0};
    var encoded: [512]u8 = undefined;
    var scratch: [256]u8 = undefined;
    const bytes = (try baked.encodeMesh(.{ .source_id = 17, .vertices = &vertices, .triangles = &triangles, .nodes = &nodes, .primitives = &primitives }, &encoded, &scratch)).bytes;
    const inputs = [_][]const u8{bytes};
    return store.Store.init(memory, &inputs);
}

fn oneTriangleCompoundStore(memory: []u8) !store.Store {
    const vertices = [_]g.Vec3{ .{}, .{ .x = .one }, .{ .y = .one } };
    const triangles = [_]baked.Triangle{.{ .a = 0, .b = 1, .c = 2 }};
    const nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = .{}, .max = .{ .x = .one, .y = .one } }, 0, 1)};
    const primitives = [_]u32{0};
    var mesh_bytes: [512]u8 = undefined;
    var mesh_scratch: [256]u8 = undefined;
    const encoded_mesh = try baked.encodeMesh(.{ .source_id = 27, .vertices = &vertices, .triangles = &triangles, .nodes = &nodes, .primitives = &primitives }, &mesh_bytes, &mesh_scratch);
    const children = [_]baked.CompoundChild{.{ .ordinal = 0, .content_hash = encoded_mesh.content_hash, .translation = .{}, .rotation = g.Quat.identity }};
    var compound_bytes: [512]u8 = undefined;
    var compound_scratch: [256]u8 = undefined;
    const encoded_compound = try baked.encodeCompound(.{ .source_id = 28, .children = &children, .nodes = &nodes }, &compound_bytes, &compound_scratch);
    const inputs = [_][]const u8{ encoded_mesh.bytes, encoded_compound.bytes };
    return store.Store.init(memory, &inputs);
}

fn surfaceCasterStore(memory: []u8) !store.Store {
    const mesh_vertices = [_]g.Vec3{ .{}, .{ .x = .one }, .{ .y = .one } };
    const mesh_triangles = [_]baked.Triangle{.{ .a = 0, .b = 1, .c = 2 }};
    const mesh_nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = .{}, .max = .{ .x = .one, .y = .one } }, 0, 1)};
    const mesh_primitives = [_]u32{0};
    var mesh_bytes: [512]u8 = undefined;
    var mesh_scratch: [256]u8 = undefined;
    const encoded_mesh = try baked.encodeMesh(.{ .source_id = 31, .vertices = &mesh_vertices, .triangles = &mesh_triangles, .nodes = &mesh_nodes, .primitives = &mesh_primitives }, &mesh_bytes, &mesh_scratch);
    const input = [_]g.Vec3{ .{}, .{ .x = .one }, .{ .y = .one }, .{ .z = .one } };
    var hull_vertices: [4]g.Vec3 = undefined;
    var hull_triangles: [4]baked.Triangle = undefined;
    var hull_faces: [4]baked.HullFace = undefined;
    var hull_edges: [12]baked.HalfEdge = undefined;
    var status = fp.MathStatus{};
    const hull = try baked.buildConvexHull(&input, &hull_vertices, &hull_triangles, &hull_faces, &hull_edges, &status);
    var hull_bytes: [2048]u8 = undefined;
    var hull_scratch: [512]u8 = undefined;
    const encoded_hull = try baked.encodeConvexHull(hull, 32, &hull_bytes, &hull_scratch);
    const children = [_]baked.CompoundChild{.{ .ordinal = 0, .content_hash = encoded_hull.content_hash, .translation = .{}, .rotation = g.Quat.identity }};
    const compound_nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = .{}, .max = .{ .x = .one, .y = .one, .z = .one } }, 0, 1)};
    var compound_bytes: [512]u8 = undefined;
    var compound_scratch: [256]u8 = undefined;
    const encoded_compound = try baked.encodeCompound(.{ .source_id = 33, .children = &children, .nodes = &compound_nodes }, &compound_bytes, &compound_scratch);
    const inputs = [_][]const u8{ encoded_mesh.bytes, encoded_hull.bytes, encoded_compound.bytes };
    return store.Store.init(memory, &inputs);
}

fn oneCellHeightStore(memory: []u8, hole: bool) !store.Store {
    const samples = [_]fp.Fp{ .zero, .zero, .zero, .zero };
    const cells = [_]baked.HeightCell{.{ .hole = hole }};
    var tiles: [1]baked.BvhNode = undefined;
    const tile_nodes = try baked.buildHeightFieldTiles(2, 2, &samples, &cells, &tiles);
    var encoded: [512]u8 = undefined;
    var scratch: [256]u8 = undefined;
    const bytes = (try baked.encodeHeightField(.{ .source_id = 19, .width = 2, .height = 2, .samples = &samples, .cells = &cells, .tile_nodes = tile_nodes }, &encoded, &scratch)).bytes;
    const inputs = [_][]const u8{bytes};
    return store.Store.init(memory, &inputs);
}

fn tetraHullStore(memory: []u8) !store.Store {
    const input = [_]g.Vec3{ .{}, .{ .x = .one }, .{ .y = .one }, .{ .z = .one } };
    var vertices: [4]g.Vec3 = undefined;
    var triangles: [4]baked.Triangle = undefined;
    var faces: [4]baked.HullFace = undefined;
    var edges: [12]baked.HalfEdge = undefined;
    var status = fp.MathStatus{};
    const hull = try baked.buildConvexHull(&input, &vertices, &triangles, &faces, &edges, &status);
    var encoded: [2048]u8 = undefined;
    var scratch: [512]u8 = undefined;
    const bytes = (try baked.encodeConvexHull(hull, 21, &encoded, &scratch)).bytes;
    const inputs = [_][]const u8{bytes};
    return store.Store.init(memory, &inputs);
}

test "ray sphere covers inside tangent miss and zero length" {
    var status = fp.MathStatus{};
    const sphere = g.Vec3.zero;
    const hit = queries.raySphere(.{ .origin = .{ .x = fp.Fp.fromInt(-2) }, .delta = .{ .x = fp.Fp.fromInt(4) } }, sphere, .one, &status).?;
    try std.testing.expectEqual(fp.Fp.fromRatio(1, 4, &status).raw, hit.fraction.raw);
    try std.testing.expect(queries.raySphere(.{ .origin = .{ .y = fp.Fp.fromInt(2) }, .delta = .{ .x = .one } }, sphere, .one, &status) == null);
    try std.testing.expectEqual(fp.Fp.zero.raw, queries.raySphere(.{ .origin = .{}, .delta = .{} }, sphere, .one, &status).?.fraction.raw);
}

test "ray AABB handles tangent parallel and stable hit publication" {
    var status = fp.MathStatus{};
    const bounds = g.Aabb3{ .min = .{ .x = fp.Fp.fromInt(-1), .y = fp.Fp.fromInt(-1), .z = fp.Fp.fromInt(-1) }, .max = .{ .x = .one, .y = .one, .z = .one } };
    try std.testing.expectEqual(fp.Fp.fromRatio(1, 4, &status).raw, queries.rayAabb(.{ .origin = .{ .x = fp.Fp.fromInt(-2) }, .delta = .{ .x = fp.Fp.fromInt(4) } }, bounds, &status).?.raw);
    try std.testing.expect(queries.rayAabb(.{ .origin = .{ .x = fp.Fp.fromInt(-2), .y = fp.Fp.fromInt(2) }, .delta = .{ .x = fp.Fp.fromInt(4) } }, bounds, &status) == null);
    var hits = [_]queries.Hit{
        .{ .fraction = .one, .collider = gravity.core.ids.ColliderId.init(1, 0), .point = .{}, .normal = .{} },
        .{ .fraction = .one, .collider = gravity.core.ids.ColliderId.init(0, 0), .point = .{}, .normal = .{} },
    };
    var output: [2]queries.Hit = undefined;
    const published = try queries.publish(.all, &hits, &output);
    try std.testing.expectEqual(@as(u32, 0), published.hits[0].collider.index());
    var small = [_]queries.Hit{output[0]};
    const insufficient = try queries.publish(.all, &hits, &small);
    try std.testing.expectEqual(@as(usize, 2), insufficient.required);
    try std.testing.expectEqual(@as(usize, 0), insufficient.hits.len);
    try std.testing.expectEqualDeep(output[0], small[0]);
}

test "ray box and capsule retain exact primitive boundaries" {
    var status = fp.MathStatus{};
    const ray = queries.Ray{ .origin = .{ .x = fp.Fp.fromInt(-2) }, .delta = .{ .x = fp.Fp.fromInt(4) } };
    const box = queries.rayBox(ray, .{}, .{ .x = .one, .y = .one, .z = .one }, &status).?;
    try std.testing.expectEqual(fp.Fp.fromRatio(1, 4, &status).raw, box.fraction.raw);
    try std.testing.expect(box.normal.x.raw < 0);
    const capsule = queries.rayCapsule(ray, .{}, .one, .zero, &status).?;
    try std.testing.expectEqual(fp.Fp.fromRatio(1, 4, &status).raw, capsule.fraction.raw);
    try std.testing.expect(capsule.normal.x.raw < 0);
    try std.testing.expectEqual(fp.Fp.zero.raw, queries.rayCapsule(.{ .origin = .{}, .delta = .{ .x = .one } }, .{}, .one, .zero, &status).?.fraction.raw);
}

test "point primitive overlap is closed on boundaries" {
    var status = fp.MathStatus{};
    try std.testing.expect(try queries.pointOverlapsPrimitive(.{ .x = .one }, .{ .sphere = .{ .radius = .one } }, .{}, &status));
    try std.testing.expect(!(try queries.pointOverlapsPrimitive(.{ .x = fp.Fp.fromInt(2) }, .{ .box = .{ .half_extents = .{ .x = .one, .y = .one, .z = .one } } }, .{}, &status)));
    try std.testing.expect(try queries.pointOverlapsPrimitive(.{ .y = fp.Fp.fromInt(2) }, .{ .capsule = .{ .radius = .one, .half_height = .one } }, .{}, &status));
}

test "mesh ray uses the baked BVH and exact triangle test" {
    var memory: [1024]u8 align(@alignOf(store.Asset)) = undefined;
    const assets = try oneTriangleStore(&memory);
    const view = try gravity.assets.runtime_view.find(&assets, 17);
    var status = fp.MathStatus{};
    var work: [2]u32 = undefined;
    var hits: [1]queries.MeshRayHit = undefined;
    const result = try queries.rayMesh(.{ .origin = .{ .x = fp.Fp.fromRatio(1, 4, &status), .y = fp.Fp.fromRatio(1, 4, &status), .z = .one }, .delta = .{ .z = fp.Fp.fromInt(-2) } }, view, .{}, &work, &hits, &status);
    try std.testing.expectEqual(@as(usize, 1), result.len);
    try std.testing.expectEqual(@as(u32, 0), result[0].primitive);
    try std.testing.expectEqual(fp.Fp.fromRatio(1, 2, &status).raw, result[0].fraction.raw);
}

test "unified ray publication preserves mesh primitive identity" {
    var memory: [1024]u8 align(@alignOf(store.Asset)) = undefined;
    const assets = try oneTriangleStore(&memory);
    var collider = gravity.collision.shapes.Collider{ .body = gravity.core.ids.BodyId.init(0, 0), .shape = .{ .triangle_mesh = .{ .source_id = 17 } } };
    const items = [_]queries.Item{.{ .id = gravity.core.ids.ColliderId.init(4, 1), .collider = &collider, .transform = .{} }};
    var status = fp.MathStatus{};
    var nodes: [2]u32 = undefined;
    var mesh_hits: [1]queries.MeshRayHit = undefined;
    var height_triangles: [1]gravity.collision.mesh.HeightTriangle = undefined;
    var leaves: [1]gravity.collision.shapes.CompoundLeaf = undefined;
    var candidates: [1]queries.Hit = undefined;
    var output: [1]queries.Hit = undefined;
    const result = try queries.rayShapes(.{ .origin = .{ .x = fp.Fp.fromRatio(1, 4, &status), .y = fp.Fp.fromRatio(1, 4, &status), .z = .one }, .delta = .{ .z = fp.Fp.fromInt(-2) } }, .{}, &items, &assets, .{ .bvh_nodes = &nodes, .mesh_hits = &mesh_hits, .height_triangles = &height_triangles, .compound_leaves = &leaves }, &candidates, &output, .closest, &status);
    try std.testing.expectEqual(@as(usize, 1), result.hits.len);
    try std.testing.expectEqual(@as(u32, 0), result.hits[0].primitive);
    try std.testing.expectEqual(@as(u32, 4), result.hits[0].collider.index());

    var no_candidates: [0]queries.Hit = .{};
    const insufficient = try queries.rayShapes(.{ .origin = .{ .x = fp.Fp.fromRatio(1, 4, &status), .y = fp.Fp.fromRatio(1, 4, &status), .z = .one }, .delta = .{ .z = fp.Fp.fromInt(-2) } }, .{}, &items, &assets, .{ .bvh_nodes = &nodes, .mesh_hits = &mesh_hits, .height_triangles = &height_triangles, .compound_leaves = &leaves }, &no_candidates, &output, .all, &status);
    try std.testing.expectEqual(@as(usize, 1), insufficient.required);
    try std.testing.expectEqual(@as(usize, 0), insufficient.hits.len);
}

test "heightfield ray traverses tiles and respects holes" {
    var memory: [1024]u8 align(@alignOf(store.Asset)) = undefined;
    const assets = try oneCellHeightStore(&memory, false);
    const view = try gravity.assets.runtime_view.find(&assets, 19);
    var status = fp.MathStatus{};
    var nodes: [1]u32 = undefined;
    var triangles: [2]gravity.collision.mesh.HeightTriangle = undefined;
    var hits: [2]queries.MeshRayHit = undefined;
    const ray = queries.Ray{ .origin = .{ .x = fp.Fp.fromRatio(1, 4, &status), .y = .one, .z = fp.Fp.fromRatio(1, 4, &status) }, .delta = .{ .y = fp.Fp.fromInt(-2) } };
    const result = try queries.rayHeightfield(ray, view, .{}, &nodes, &triangles, &hits, &status);
    try std.testing.expectEqual(@as(usize, 1), result.len);
    try std.testing.expectEqual(@as(u32, 0), result[0].primitive);
    const holes = try oneCellHeightStore(&memory, true);
    const hole_view = try gravity.assets.runtime_view.find(&holes, 19);
    try std.testing.expectEqual(@as(usize, 0), (try queries.rayHeightfield(ray, hole_view, .{}, &nodes, &triangles, &hits, &status)).len);
}

test "point overlap dispatches exact mesh and heightfield surfaces through BVHs" {
    var memory: [1024]u8 align(@alignOf(store.Asset)) = undefined;
    const mesh_assets = try oneTriangleStore(&memory);
    var status = fp.MathStatus{};
    var nodes: [2]u32 = undefined;
    var mesh_hits: [1]queries.MeshRayHit = undefined;
    var triangles: [2]gravity.collision.mesh.HeightTriangle = undefined;
    var leaves: [1]gravity.collision.shapes.CompoundLeaf = undefined;
    const workspace = queries.RayWorkspace{ .bvh_nodes = &nodes, .mesh_hits = &mesh_hits, .height_triangles = &triangles, .compound_leaves = &leaves };
    const mesh_shape = gravity.collision.shapes.Shape{ .triangle_mesh = .{ .source_id = 17 } };
    try std.testing.expect(try queries.pointOverlapsShape(.{ .x = fp.Fp.fromRatio(1, 4, &status), .y = fp.Fp.fromRatio(1, 4, &status) }, mesh_shape, .{}, &mesh_assets, workspace, &status));
    try std.testing.expect(!(try queries.pointOverlapsShape(.{ .x = fp.Fp.fromRatio(1, 4, &status), .y = fp.Fp.fromRatio(1, 4, &status), .z = fp.Fp.fromRatio(1, 8, &status) }, mesh_shape, .{}, &mesh_assets, workspace, &status)));

    const height_assets = try oneCellHeightStore(&memory, false);
    const height_shape = gravity.collision.shapes.Shape{ .height_field = .{ .source_id = 19 } };
    try std.testing.expect(try queries.pointOverlapsShape(.{ .x = fp.Fp.fromRatio(1, 4, &status), .z = fp.Fp.fromRatio(1, 4, &status) }, height_shape, .{}, &height_assets, workspace, &status));
    const holes = try oneCellHeightStore(&memory, true);
    try std.testing.expect(!(try queries.pointOverlapsShape(.{ .x = fp.Fp.fromRatio(1, 4, &status), .z = fp.Fp.fromRatio(1, 4, &status) }, height_shape, .{}, &holes, workspace, &status)));
}

test "convex hull ray is exact and reports inside at zero fraction" {
    var memory: [4096]u8 align(@alignOf(store.Asset)) = undefined;
    const assets = try tetraHullStore(&memory);
    const view = try gravity.assets.runtime_view.find(&assets, 21);
    var status = fp.MathStatus{};
    const ray = queries.Ray{ .origin = .{ .x = fp.Fp.fromInt(-1), .y = fp.Fp.fromRatio(1, 10, &status), .z = fp.Fp.fromRatio(1, 10, &status) }, .delta = .{ .x = fp.Fp.fromInt(3) } };
    const hit = (try queries.rayConvexHull(ray, view, .{}, &status)).?;
    try std.testing.expectEqual(fp.Fp.fromRatio(1, 3, &status).raw, hit.fraction.raw);
    const inside = (try queries.rayConvexHull(.{ .origin = .{ .x = fp.Fp.fromRatio(1, 10, &status), .y = fp.Fp.fromRatio(1, 10, &status), .z = fp.Fp.fromRatio(1, 10, &status) }, .delta = .{} }, view, .{}, &status)).?;
    try std.testing.expectEqual(fp.Fp.zero.raw, inside.fraction.raw);
}

test "convex point overlap tests hull volume instead of its bounds" {
    var memory: [4096]u8 align(@alignOf(store.Asset)) = undefined;
    const assets = try tetraHullStore(&memory);
    var status = fp.MathStatus{};
    var leaves: [1]gravity.collision.shapes.CompoundLeaf = undefined;
    const hull = gravity.collision.shapes.Shape{ .convex_hull = .{ .source_id = 21 } };
    try std.testing.expect(try queries.pointOverlapsConvex(.{ .x = fp.Fp.fromRatio(1, 10, &status), .y = fp.Fp.fromRatio(1, 10, &status), .z = fp.Fp.fromRatio(1, 10, &status) }, hull, .{}, &assets, &leaves, &status));
    try std.testing.expect(!(try queries.pointOverlapsConvex(.{ .x = .one, .y = .one, .z = .one }, hull, .{}, &assets, &leaves, &status)));
}

test "AABB overlap uses exact convex geometry and retains touching" {
    var memory: [4096]u8 align(@alignOf(store.Asset)) = undefined;
    const assets = try tetraHullStore(&memory);
    var status = fp.MathStatus{};
    const bounds = g.Aabb3{ .min = .{ .x = .one, .y = .zero, .z = .zero }, .max = .{ .x = fp.Fp.fromInt(2), .y = .one, .z = .one } };
    try std.testing.expect(try queries.aabbOverlapsConvex(bounds, .{ .sphere = .{ .radius = .one } }, .{}, &assets, &status));
    const separate = g.Aabb3{ .min = .{ .x = fp.Fp.fromInt(3), .y = .zero, .z = .zero }, .max = .{ .x = fp.Fp.fromInt(4), .y = .one, .z = .one } };
    try std.testing.expect(!(try queries.aabbOverlapsConvex(separate, .{ .sphere = .{ .radius = .one } }, .{}, &assets, &status)));
}

test "shape and AABB overlaps use exact surface triangles" {
    var memory: [1024]u8 align(@alignOf(store.Asset)) = undefined;
    const assets = try oneTriangleStore(&memory);
    var status = fp.MathStatus{};
    var leaves: [1]gravity.collision.shapes.CompoundLeaf = undefined;
    var mesh_nodes: [1]baked.BvhNode = undefined;
    var mesh_primitives: [1]u32 = undefined;
    var mesh_work: [2]gravity.collision.mesh.NodePair = undefined;
    var mesh_scratch: [2]gravity.collision.mesh.PrimitivePair = undefined;
    var mesh_pairs: [2]gravity.collision.mesh.PrimitivePair = undefined;
    var mesh_intersections: [1]u32 = undefined;
    var tile_nodes: [1]baked.BvhNode = undefined;
    var height_work: [2]gravity.collision.mesh.NodePair = undefined;
    var height_scratch: [2]gravity.collision.mesh.PrimitivePair = undefined;
    var height_pairs: [2]gravity.collision.mesh.PrimitivePair = undefined;
    var height_triangles: [2]gravity.collision.mesh.HeightTriangle = undefined;
    var height_intersections: [2]u32 = undefined;
    const workspace = queries.SurfaceOverlapWorkspace{
        .compound_leaves = &leaves,
        .mesh = .{ .nodes = &mesh_nodes, .primitives = &mesh_primitives, .work = &mesh_work, .pair_scratch = &mesh_scratch, .pair_output = &mesh_pairs, .intersections = &mesh_intersections },
        .heightfield = .{ .tile_nodes = &tile_nodes, .work = &height_work, .pair_scratch = &height_scratch, .pair_output = &height_pairs, .triangles = &height_triangles, .intersections = &height_intersections },
    };
    const box = gravity.collision.shapes.Shape{ .box = .{ .half_extents = .{ .x = fp.Fp.fromRatio(1, 4, &status), .y = fp.Fp.fromRatio(1, 4, &status), .z = fp.Fp.fromRatio(1, 4, &status) } } };
    const mesh_shape = gravity.collision.shapes.Shape{ .triangle_mesh = .{ .source_id = 17 } };
    try std.testing.expect(try queries.convexOverlapsShape(box, .{ .position = .{ .x = fp.Fp.fromRatio(1, 4, &status), .y = fp.Fp.fromRatio(1, 4, &status) } }, mesh_shape, .{}, &assets, workspace, &status));
    const bounds = g.Aabb3{ .min = .{ .x = .zero, .y = .zero, .z = fp.Fp.fromRatio(-1, 8, &status) }, .max = .{ .x = fp.Fp.fromRatio(1, 2, &status), .y = fp.Fp.fromRatio(1, 2, &status), .z = fp.Fp.fromRatio(1, 8, &status) } };
    try std.testing.expect(try queries.aabbOverlapsShape(bounds, mesh_shape, .{}, &assets, workspace, &status));
    try std.testing.expect(!(try queries.convexOverlapsShape(box, .{ .position = .{ .x = fp.Fp.fromInt(4) } }, mesh_shape, .{}, &assets, workspace, &status)));
}

test "convex shape cast has stable hit and miss outcomes" {
    var memory: [4096]u8 align(@alignOf(store.Asset)) = undefined;
    const assets = try tetraHullStore(&memory);
    var status = fp.MathStatus{};
    var leaves: [1]gravity.collision.shapes.CompoundLeaf = undefined;
    const hit = try queries.convexShapeCast(.{ .sphere = .{ .radius = .one } }, .{ .position = .{ .x = fp.Fp.fromInt(-3) } }, .{ .x = fp.Fp.fromInt(6) }, .{ .sphere = .{ .radius = .one } }, .{}, &assets, .{ .compound_leaves = &leaves }, &status);
    try std.testing.expectEqual(queries.ShapeCastStatus.hit, hit.status);
    try std.testing.expectEqual(fp.Fp.fromRatio(1, 6, &status).raw, hit.fraction.raw);
    const miss = try queries.convexShapeCast(.{ .sphere = .{ .radius = .one } }, .{ .position = .{ .x = fp.Fp.fromInt(-3) } }, .{ .x = fp.Fp.fromInt(-1) }, .{ .sphere = .{ .radius = .one } }, .{}, &assets, .{ .compound_leaves = &leaves }, &status);
    try std.testing.expectEqual(queries.ShapeCastStatus.miss, miss.status);
    try std.testing.expect(try queries.convexOverlapsResolved(.{ .sphere = .{ .radius = .one } }, .{}, .{ .box = .{ .half_extents = .{ .x = .one, .y = .one, .z = .one } } }, .{ .position = .{ .x = fp.Fp.fromInt(1) } }, &assets, &leaves, &status));
}

test "convex surface shape cast advances from exact mesh distance" {
    var memory: [1024]u8 align(@alignOf(store.Asset)) = undefined;
    const assets = try oneTriangleStore(&memory);
    var status = fp.MathStatus{};
    var leaves: [1]gravity.collision.shapes.CompoundLeaf = undefined;
    var mesh_nodes: [1]baked.BvhNode = undefined;
    var mesh_primitives: [1]u32 = undefined;
    var mesh_stack: [1]u32 = undefined;
    var height_stack: [1]u32 = undefined;
    var height_triangles: [2]gravity.collision.mesh.HeightTriangle = undefined;
    const workspace = queries.SurfaceCastWorkspace{
        .compound_leaves = &leaves,
        .mesh = .{ .nodes = &mesh_nodes, .primitives = &mesh_primitives, .stack = &mesh_stack },
        .heightfield = .{ .stack = &height_stack, .triangles = &height_triangles },
    };
    const hit = try queries.convexShapeCastSurface(.{ .sphere = .{ .radius = fp.Fp.fromRatio(1, 4, &status) } }, .{ .position = .{ .x = fp.Fp.fromRatio(1, 4, &status), .y = fp.Fp.fromRatio(1, 4, &status), .z = fp.Fp.fromInt(2) } }, .{ .z = fp.Fp.fromInt(-4) }, .{ .triangle_mesh = .{ .source_id = 17 } }, .{}, &assets, workspace, &status);
    try std.testing.expectEqual(queries.ShapeCastStatus.hit, hit.status);
    // Conservative advancement terminates at the frozen one-raw contact
    // threshold; retain the exact, bounded fixed-point convergence envelope.
    const expected = fp.Fp.fromRatio(7, 16, &status).raw;
    try std.testing.expect(hit.fraction.raw >= expected and hit.fraction.raw <= expected + 2048);
    const miss = try queries.convexShapeCastSurface(.{ .sphere = .{ .radius = fp.Fp.fromRatio(1, 4, &status) } }, .{ .position = .{ .x = fp.Fp.fromInt(3), .z = fp.Fp.fromInt(2) } }, .{ .z = fp.Fp.fromInt(-4) }, .{ .triangle_mesh = .{ .source_id = 17 } }, .{}, &assets, workspace, &status);
    try std.testing.expectEqual(queries.ShapeCastStatus.miss, miss.status);

    const compound_assets = try oneTriangleCompoundStore(&memory);
    const compound_hit = try queries.convexShapeCastSurface(.{ .sphere = .{ .radius = fp.Fp.fromRatio(1, 4, &status) } }, .{ .position = .{ .x = fp.Fp.fromRatio(1, 4, &status), .y = fp.Fp.fromRatio(1, 4, &status), .z = fp.Fp.fromInt(2) } }, .{ .z = fp.Fp.fromInt(-4) }, .{ .compound = .{ .source_id = 28 } }, .{}, &compound_assets, workspace, &status);
    try std.testing.expectEqual(queries.ShapeCastStatus.hit, compound_hit.status);

    const box_hit = try queries.convexShapeCastSurface(.{ .box = .{ .half_extents = .{ .x = fp.Fp.fromRatio(1, 4, &status), .y = fp.Fp.fromRatio(1, 4, &status), .z = fp.Fp.fromRatio(1, 4, &status) } } }, .{ .position = .{ .x = fp.Fp.fromRatio(1, 4, &status), .y = fp.Fp.fromRatio(1, 4, &status), .z = fp.Fp.fromInt(2) } }, .{ .z = fp.Fp.fromInt(-4) }, .{ .triangle_mesh = .{ .source_id = 27 } }, .{}, &compound_assets, workspace, &status);
    try std.testing.expectEqual(queries.ShapeCastStatus.hit, box_hit.status);
    const capsule_hit = try queries.convexShapeCastSurface(.{ .capsule = .{ .radius = fp.Fp.fromRatio(1, 4, &status), .half_height = fp.Fp.fromRatio(1, 4, &status) } }, .{ .position = .{ .x = fp.Fp.fromRatio(1, 4, &status), .y = fp.Fp.fromRatio(1, 4, &status), .z = fp.Fp.fromInt(2) } }, .{ .z = fp.Fp.fromInt(-4) }, .{ .triangle_mesh = .{ .source_id = 27 } }, .{}, &compound_assets, workspace, &status);
    try std.testing.expectEqual(queries.ShapeCastStatus.hit, capsule_hit.status);

    var caster_memory: [8192]u8 align(@alignOf(store.Asset)) = undefined;
    const caster_assets = try surfaceCasterStore(&caster_memory);
    const hull_start = g.Transform3{ .position = .{ .x = fp.Fp.fromRatio(1, 10, &status), .y = fp.Fp.fromRatio(1, 10, &status), .z = fp.Fp.fromInt(2) } };
    const hull_hit = try queries.convexShapeCastSurface(.{ .convex_hull = .{ .source_id = 32 } }, hull_start, .{ .z = fp.Fp.fromInt(-4) }, .{ .triangle_mesh = .{ .source_id = 31 } }, .{}, &caster_assets, workspace, &status);
    try std.testing.expectEqual(queries.ShapeCastStatus.hit, hull_hit.status);
    const compound_caster_hit = try queries.convexShapeCastSurface(.{ .compound = .{ .source_id = 33 } }, hull_start, .{ .z = fp.Fp.fromInt(-4) }, .{ .triangle_mesh = .{ .source_id = 31 } }, .{}, &caster_assets, workspace, &status);
    try std.testing.expectEqual(queries.ShapeCastStatus.hit, compound_caster_hit.status);
}

test "convex surface shape cast exposes heightfield nonconvergence and holes" {
    var memory: [1024]u8 align(@alignOf(store.Asset)) = undefined;
    const assets = try oneCellHeightStore(&memory, false);
    var status = fp.MathStatus{};
    var leaves: [1]gravity.collision.shapes.CompoundLeaf = undefined;
    var mesh_nodes: [1]baked.BvhNode = undefined;
    var mesh_primitives: [1]u32 = undefined;
    var mesh_stack: [1]u32 = undefined;
    var height_stack: [1]u32 = undefined;
    var height_triangles: [2]gravity.collision.mesh.HeightTriangle = undefined;
    const workspace = queries.SurfaceCastWorkspace{
        .compound_leaves = &leaves,
        .mesh = .{ .nodes = &mesh_nodes, .primitives = &mesh_primitives, .stack = &mesh_stack },
        .heightfield = .{ .stack = &height_stack, .triangles = &height_triangles },
    };
    const caster = gravity.collision.shapes.Shape{ .sphere = .{ .radius = fp.Fp.fromRatio(1, 4, &status) } };
    const target = gravity.collision.shapes.Shape{ .height_field = .{ .source_id = 19 } };
    const hit = try queries.convexShapeCastSurface(caster, .{ .position = .{ .x = fp.Fp.fromRatio(1, 4, &status), .y = fp.Fp.fromInt(2), .z = fp.Fp.fromRatio(1, 4, &status) } }, .{ .y = fp.Fp.fromInt(-4) }, target, .{}, &assets, workspace, &status);
    // Coplanar sphere--triangle GJK has a documented explicit failure mode;
    // it must not silently publish a fabricated HeightField hit.
    try std.testing.expectEqual(queries.ShapeCastStatus.non_convergent, hit.status);
    const holes = try oneCellHeightStore(&memory, true);
    const miss = try queries.convexShapeCastSurface(caster, .{ .position = .{ .x = fp.Fp.fromRatio(1, 4, &status), .y = fp.Fp.fromInt(2), .z = fp.Fp.fromRatio(1, 4, &status) } }, .{ .y = fp.Fp.fromInt(-4) }, target, .{}, &holes, workspace, &status);
    try std.testing.expectEqual(queries.ShapeCastStatus.miss, miss.status);
}

test "primitive query modes all derive from one stable sorted hit set" {
    var status = fp.MathStatus{};
    var late = gravity.collision.shapes.Collider{ .body = gravity.core.ids.BodyId.init(1, 0), .shape = .{ .sphere = .{ .radius = .one } } };
    var early = gravity.collision.shapes.Collider{ .body = gravity.core.ids.BodyId.init(2, 0), .shape = .{ .sphere = .{ .radius = .one } } };
    const items = [_]queries.Item{
        .{ .id = gravity.core.ids.ColliderId.init(1, 0), .collider = &late, .transform = .{ .position = .{ .x = fp.Fp.fromInt(2) } } },
        .{ .id = gravity.core.ids.ColliderId.init(0, 0), .collider = &early, .transform = .{} },
    };
    const ray = queries.Ray{ .origin = .{ .x = fp.Fp.fromInt(-2) }, .delta = .{ .x = fp.Fp.fromInt(6) } };
    var candidates: [2]queries.Hit = undefined;
    var all: [2]queries.Hit = undefined;
    const full = try queries.rayPrimitives(ray, .{}, &items, &candidates, &all, .all, &status);
    try std.testing.expectEqual(@as(usize, 2), full.hits.len);
    try std.testing.expectEqual(@as(u32, 0), full.hits[0].collider.index());
    var one: [1]queries.Hit = undefined;
    const any = try queries.rayPrimitives(ray, .{}, &items, &candidates, &one, .any, &status);
    const closest = try queries.rayPrimitives(ray, .{}, &items, &candidates, &one, .closest, &status);
    try std.testing.expectEqual(full.hits[0].collider.value, any.hits[0].collider.value);
    try std.testing.expectEqual(full.hits[0].collider.value, closest.hits[0].collider.value);
    var no_candidates: [0]queries.Hit = .{};
    const required = try queries.rayPrimitives(ray, .{}, &items, &no_candidates, &one, .all, &status);
    try std.testing.expectEqual(@as(usize, 2), required.required);
    try std.testing.expectEqual(@as(usize, 0), required.hits.len);
}

test "unified primitive ray matches the brute primitive oracle" {
    var status = fp.MathStatus{};
    var sphere = gravity.collision.shapes.Collider{ .body = gravity.core.ids.BodyId.init(1, 0), .shape = .{ .sphere = .{ .radius = .one } } };
    var box = gravity.collision.shapes.Collider{ .body = gravity.core.ids.BodyId.init(2, 0), .shape = .{ .box = .{ .half_extents = .{ .x = .one, .y = .one, .z = .one } } } };
    const items = [_]queries.Item{
        .{ .id = gravity.core.ids.ColliderId.init(4, 0), .collider = &sphere, .transform = .{ .position = .{ .x = fp.Fp.fromInt(3) } } },
        .{ .id = gravity.core.ids.ColliderId.init(2, 0), .collider = &box, .transform = .{} },
    };
    const ray = queries.Ray{ .origin = .{ .x = fp.Fp.fromInt(-3) }, .delta = .{ .x = fp.Fp.fromInt(8) } };
    var oracle_candidates: [2]queries.Hit = undefined;
    var oracle_output: [2]queries.Hit = undefined;
    const oracle = try queries.rayPrimitives(ray, .{}, &items, &oracle_candidates, &oracle_output, .all, &status);
    var nodes: [1]u32 = undefined;
    var mesh_hits: [1]queries.MeshRayHit = undefined;
    var height_triangles: [2]gravity.collision.mesh.HeightTriangle = undefined;
    var leaves: [1]gravity.collision.shapes.CompoundLeaf = undefined;
    var candidates: [2]queries.Hit = undefined;
    var output: [2]queries.Hit = undefined;
    // Primitive-only items make `rayPrimitives` the direct brute oracle for
    // the generalized query dispatch and frozen ordering contract.
    var empty_memory: [@sizeOf(store.Asset)]u8 align(@alignOf(store.Asset)) = undefined;
    const empty_assets = try store.Store.init(&empty_memory, &.{});
    const unified = try queries.rayShapes(ray, .{}, &items, &empty_assets, .{ .bvh_nodes = &nodes, .mesh_hits = &mesh_hits, .height_triangles = &height_triangles, .compound_leaves = &leaves }, &candidates, &output, .all, &status);
    try std.testing.expectEqual(oracle.hits.len, unified.hits.len);
    for (oracle.hits, unified.hits) |expected, actual| try std.testing.expectEqualDeep(expected, actual);
}
