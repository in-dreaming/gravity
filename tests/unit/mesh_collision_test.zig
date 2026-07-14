const std = @import("std");
const gravity = @import("gravity");
const mesh = gravity.collision.mesh;
const baked = gravity.geometry.baked;
const fp = gravity.math.fp;
const g = gravity.math.geometry;
const asset_store = gravity.assets.store;
const runtime_view = gravity.assets.runtime_view;

fn bounds(min_x: i32, max_x: i32) g.Aabb3 {
    return .{ .min = .{ .x = fp.Fp.fromInt(min_x) }, .max = .{ .x = fp.Fp.fromInt(max_x) } };
}
fn inner(value: g.Aabb3) baked.BvhNode {
    return .{ .bounds = value, .first = 1, .count = 0, .axis = 0, .flags = 0 };
}
test "ordered BVH pair traversal matches overlapping leaf oracle transactionally" {
    const a_nodes = [_]baked.BvhNode{ inner(bounds(0, 3)), baked.BvhNode.leaf(bounds(0, 1), 0, 1), baked.BvhNode.leaf(bounds(2, 3), 1, 1) };
    const b_nodes = [_]baked.BvhNode{ inner(bounds(0, 4)), baked.BvhNode.leaf(bounds(1, 2), 0, 1), baked.BvhNode.leaf(bounds(3, 4), 1, 1) };
    const a_primitives = [_]u32{ 4, 9 };
    const b_primitives = [_]u32{ 2, 7 };
    var work: [8]mesh.NodePair = undefined;
    var scratch: [8]mesh.PrimitivePair = undefined;
    var output: [8]mesh.PrimitivePair = undefined;
    const pairs = try mesh.traverseBvhPairs(&a_nodes, &a_primitives, &b_nodes, &b_primitives, &work, &scratch, &output);
    try std.testing.expectEqual(@as(usize, 3), pairs.len);
    try std.testing.expectEqual(@as(u32, 4), pairs[0].a);
    try std.testing.expectEqual(@as(u32, 2), pairs[0].b);
    try std.testing.expectEqual(@as(u32, 9), pairs[1].a);
    try std.testing.expectEqual(@as(u32, 2), pairs[1].b);
    try std.testing.expectEqual(@as(u32, 9), pairs[2].a);
    try std.testing.expectEqual(@as(u32, 7), pairs[2].b);
    var small_output = [_]mesh.PrimitivePair{.{ .a = 99, .b = 99 }};
    try std.testing.expectError(error.CapacityExceeded, mesh.traverseBvhPairs(&a_nodes, &a_primitives, &b_nodes, &b_primitives, &work, &scratch, &small_output));
    try std.testing.expectEqual(@as(u32, 99), small_output[0].a);
}

test "triangle SAT handles coplanar overlap and separation exactly" {
    const a = mesh.Triangle{ .a = .{}, .b = .{ .x = fp.Fp.fromInt(2) }, .c = .{ .y = fp.Fp.fromInt(2) } };
    const overlap = mesh.Triangle{ .a = .{ .x = fp.Fp.one, .y = fp.Fp.one }, .b = .{ .x = fp.Fp.fromInt(3), .y = fp.Fp.one }, .c = .{ .x = fp.Fp.one, .y = fp.Fp.fromInt(3) } };
    const separate = mesh.Triangle{ .a = .{ .x = fp.Fp.fromInt(4) }, .b = .{ .x = fp.Fp.fromInt(5) }, .c = .{ .x = fp.Fp.fromInt(4), .y = fp.Fp.one } };
    try std.testing.expect(mesh.trianglesOverlap(a, overlap));
    try std.testing.expect(!mesh.trianglesOverlap(a, separate));
    try std.testing.expect(mesh.trianglesOverlap(overlap, a));
}

test "runtime convex triangle GJK adapter classifies overlap and separation" {
    const vertices = [_]g.Vec3{ .{}, .{ .x = fp.Fp.one }, .{ .y = fp.Fp.one } };
    const triangles = [_]baked.Triangle{.{ .a = 0, .b = 1, .c = 2 }};
    const nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = g.Vec3.zero, .max = .{ .x = fp.Fp.one, .y = fp.Fp.one } }, 0, 1)};
    const primitives = [_]u32{0};
    var bytes: [512]u8 = undefined;
    var scratch: [256]u8 = undefined;
    const encoded = try baked.encodeMesh(.{ .source_id = 60, .vertices = &vertices, .triangles = &triangles, .nodes = &nodes, .primitives = &primitives }, &bytes, &scratch);
    const inputs = [_][]const u8{encoded.bytes};
    var memory: [1024]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&memory, &inputs);
    var status = fp.MathStatus{};
    var overlap = mesh.ConvexTriangleContext{ .shape = .{ .box = .{ .half_extents = .{ .x = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status), .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status), .z = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status) } } }, .assets = &assets, .transform = .{ .position = .{ .x = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status), .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status) } }, .triangle = .{ .a = vertices[0], .b = vertices[1], .c = vertices[2] } };
    try std.testing.expectEqual(gravity.collision.gjk.Status.intersecting, (try mesh.convexTriangleIntersect(&overlap, g.Vec3.unit_z, &status)).status);
    overlap.transform.position.z = fp.Fp.fromInt(5);
    try std.testing.expectEqual(gravity.collision.gjk.Status.separated, (try mesh.convexTriangleIntersect(&overlap, g.Vec3.unit_z, &status)).status);
    const distance = try mesh.convexTriangleDistance(&overlap, g.Vec3.unit_z, &status);
    try std.testing.expectEqual(gravity.collision.gjk.Status.separated, distance.status);
    try std.testing.expect(distance.distance.raw > 0);
    var distance_nodes: [1]baked.BvhNode = undefined;
    var distance_primitives: [1]u32 = undefined;
    var distance_stack: [1]u32 = undefined;
    const surface_distance = try mesh.convexMeshDistance(overlap.shape, &assets, overlap.transform, try runtime_view.find(&assets, 60), .{}, .{ .nodes = &distance_nodes, .primitives = &distance_primitives, .stack = &distance_stack }, &status);
    try std.testing.expectEqual(gravity.collision.gjk.Status.separated, surface_distance.status);
    try std.testing.expectEqual(@as(u32, 0), surface_distance.feature_b);
}

test "runtime convex triangle contact publishes EPA witness for a full simplex" {
    const vertices = [_]g.Vec3{ .{ .x = fp.Fp.fromInt(-2), .z = fp.Fp.fromInt(-2) }, .{ .x = fp.Fp.fromInt(2), .z = fp.Fp.fromInt(-2) }, .{ .z = fp.Fp.fromInt(2) } };
    const triangles = [_]baked.Triangle{.{ .a = 0, .b = 1, .c = 2 }};
    const nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = .{ .x = fp.Fp.fromInt(-2), .z = fp.Fp.fromInt(-2) }, .max = .{ .x = fp.Fp.fromInt(2), .z = fp.Fp.fromInt(2) } }, 0, 1)};
    const primitives = [_]u32{0};
    var bytes: [512]u8 = undefined;
    var scratch: [256]u8 = undefined;
    const encoded = try baked.encodeMesh(.{ .source_id = 61, .vertices = &vertices, .triangles = &triangles, .nodes = &nodes, .primitives = &primitives }, &bytes, &scratch);
    const inputs = [_][]const u8{encoded.bytes};
    var memory: [1024]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&memory, &inputs);
    var status = fp.MathStatus{};
    var context = mesh.ConvexTriangleContext{ .shape = .{ .box = .{ .half_extents = .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.one } } }, .assets = &assets, .transform = .{ .position = .{ .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status) } }, .triangle = .{ .a = vertices[0], .b = vertices[1], .c = vertices[2] } };
    var epa_vertices: [32]gravity.collision.gjk.SupportVertex = undefined;
    var epa_faces: [64]gravity.collision.gjk.EpaFace = undefined;
    var visible: [64]bool = undefined;
    var horizon: [64]gravity.collision.gjk.HorizonEdge = undefined;
    const result = try mesh.convexTriangleContact(&context, g.Vec3.unit_y, .{ .epa = .{ .vertices = &epa_vertices, .faces = &epa_faces, .visible = &visible, .horizon = &horizon } }, &status);
    try std.testing.expectEqual(gravity.collision.gjk.Status.intersecting, result.gjk.status);
    try std.testing.expect(result.epa != null);
    try std.testing.expect(result.contact != null);
    try std.testing.expect(result.contact.?.separation.raw < 0);
    context.transform.position.y = fp.Fp.one;
    const touching = try mesh.convexTriangleContact(&context, g.Vec3.unit_y, .{ .epa = .{ .vertices = &epa_vertices, .faces = &epa_faces, .visible = &visible, .horizon = &horizon } }, &status);
    try std.testing.expectEqual(gravity.collision.gjk.Status.intersecting, touching.gjk.status);
    try std.testing.expect(touching.contact != null);
    try std.testing.expect(touching.contact.?.separation.raw <= 0);
}

test "runtime convex mesh patch reduces EPA triangle contact" {
    const vertices = [_]g.Vec3{ .{ .x = fp.Fp.fromInt(-2), .z = fp.Fp.fromInt(-2) }, .{ .x = fp.Fp.fromInt(2), .z = fp.Fp.fromInt(-2) }, .{ .z = fp.Fp.fromInt(2) } };
    const triangles = [_]baked.Triangle{.{ .a = 0, .b = 1, .c = 2 }};
    const nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = .{ .x = fp.Fp.fromInt(-2), .z = fp.Fp.fromInt(-2) }, .max = .{ .x = fp.Fp.fromInt(2), .z = fp.Fp.fromInt(2) } }, 0, 1)};
    const primitives = [_]u32{0};
    var bytes: [512]u8 = undefined;
    var scratch: [256]u8 = undefined;
    const encoded = try baked.encodeMesh(.{ .source_id = 62, .vertices = &vertices, .triangles = &triangles, .nodes = &nodes, .primitives = &primitives }, &bytes, &scratch);
    const inputs = [_][]const u8{encoded.bytes};
    var memory: [1024]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&memory, &inputs);
    var status = fp.MathStatus{};
    var query_nodes: [1]baked.BvhNode = undefined;
    var query_primitives: [1]u32 = undefined;
    var work: [1]mesh.NodePair = undefined;
    var pair_scratch: [1]mesh.PrimitivePair = undefined;
    var pair_output: [1]mesh.PrimitivePair = undefined;
    var hits: [1]u32 = undefined;
    var epa_vertices: [32]gravity.collision.gjk.SupportVertex = undefined;
    var epa_faces: [64]gravity.collision.gjk.EpaFace = undefined;
    var visible: [64]bool = undefined;
    var horizon: [64]gravity.collision.gjk.HorizonEdge = undefined;
    var contacts: [1]gravity.collision.gjk.ContactPoint = undefined;
    const patch = try mesh.convexMeshPatch(.{ .box = .{ .half_extents = .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.one } } }, &assets, .{ .position = .{ .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status) } }, try runtime_view.find(&assets, 62), .{}, .{ .query = .{ .nodes = &query_nodes, .primitives = &query_primitives, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .intersections = &hits }, .triangle = .{ .epa = .{ .vertices = &epa_vertices, .faces = &epa_faces, .visible = &visible, .horizon = &horizon } }, .contacts = &contacts }, &status);
    try std.testing.expectEqual(@as(u8, 1), patch.len);
    try std.testing.expectEqual(@as(u32, 0), patch.points[0].feature_b);
    try std.testing.expect(patch.points[0].separation.raw < 0);
}

test "runtime convex traversal classifies mesh and heightfield features" {
    const vertices = [_]g.Vec3{ .{}, .{ .x = fp.Fp.one }, .{ .z = fp.Fp.one } };
    const triangles = [_]baked.Triangle{.{ .a = 0, .b = 1, .c = 2 }};
    const nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = g.Vec3.zero, .max = .{ .x = fp.Fp.one, .z = fp.Fp.one } }, 0, 1)};
    const primitives = [_]u32{0};
    const samples = [_]fp.Fp{ fp.Fp.zero, fp.Fp.zero, fp.Fp.zero, fp.Fp.zero };
    const cells = [_]baked.HeightCell{.{ .material_id = 4 }};
    var height_nodes: [1]baked.BvhNode = undefined;
    const tiles = try baked.buildHeightFieldTiles(2, 2, &samples, &cells, &height_nodes);
    var mesh_bytes: [512]u8 = undefined;
    var mesh_scratch: [256]u8 = undefined;
    const encoded_mesh = try baked.encodeMesh(.{ .source_id = 70, .vertices = &vertices, .triangles = &triangles, .nodes = &nodes, .primitives = &primitives }, &mesh_bytes, &mesh_scratch);
    var height_bytes: [1024]u8 = undefined;
    var height_scratch: [512]u8 = undefined;
    const encoded_height = try baked.encodeHeightField(.{ .source_id = 71, .width = 2, .height = 2, .samples = &samples, .cells = &cells, .tile_nodes = tiles }, &height_bytes, &height_scratch);
    const inputs = [_][]const u8{ encoded_mesh.bytes, encoded_height.bytes };
    var memory: [4096]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&memory, &inputs);
    var status = fp.MathStatus{};
    const shape = gravity.collision.shapes.Shape{ .box = .{ .half_extents = .{ .x = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status), .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status), .z = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status) } } };
    const transform = g.Transform3{ .position = .{ .x = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status), .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(8), &status), .z = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status) } };
    var mesh_nodes: [1]baked.BvhNode = undefined;
    var mesh_primitives: [1]u32 = undefined;
    var work: [1]mesh.NodePair = undefined;
    var pair_scratch: [1]mesh.PrimitivePair = undefined;
    var pair_output: [1]mesh.PrimitivePair = undefined;
    var intersections: [1]u32 = undefined;
    const mesh_hits = try mesh.convexMeshIntersections(shape, &assets, transform, try runtime_view.find(&assets, 70), .{}, .{ .nodes = &mesh_nodes, .primitives = &mesh_primitives, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .intersections = &intersections }, &status);
    try std.testing.expectEqualSlices(u32, &[_]u32{0}, mesh_hits);
    var tile_nodes: [1]baked.BvhNode = undefined;
    var height_triangles: [2]mesh.HeightTriangle = undefined;
    var height_work: [1]mesh.NodePair = undefined;
    var height_scratch_pairs: [1]mesh.PrimitivePair = undefined;
    var height_pair_output: [1]mesh.PrimitivePair = undefined;
    var height_hits_output: [2]u32 = undefined;
    const height_hits = try mesh.convexHeightfieldIntersections(shape, &assets, transform, try runtime_view.find(&assets, 71), .{}, .{ .tile_nodes = &tile_nodes, .work = &height_work, .pair_scratch = &height_scratch_pairs, .pair_output = &height_pair_output, .triangles = &height_triangles, .intersections = &height_hits_output }, &status);
    try std.testing.expectEqualSlices(u32, &[_]u32{ 0, 1 }, height_hits);
    var no_hits: [1]u32 = .{99};
    const far = g.Transform3{ .position = .{ .x = fp.Fp.fromInt(4) } };
    try std.testing.expectEqual(@as(usize, 0), (try mesh.convexMeshIntersections(shape, &assets, far, try runtime_view.find(&assets, 70), .{}, .{ .nodes = &mesh_nodes, .primitives = &mesh_primitives, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .intersections = &no_hits }, &status)).len);
    try std.testing.expectEqual(@as(u32, 99), no_hits[0]);
}

test "runtime convex heightfield patch reduces EPA triangle contacts" {
    const samples = [_]fp.Fp{ fp.Fp.zero, fp.Fp.zero, fp.Fp.zero, fp.Fp.zero };
    const cells = [_]baked.HeightCell{.{ .material_id = 4 }};
    var tile_nodes: [1]baked.BvhNode = undefined;
    const tiles = try baked.buildHeightFieldTiles(2, 2, &samples, &cells, &tile_nodes);
    var bytes: [1024]u8 = undefined;
    var scratch: [512]u8 = undefined;
    const encoded = try baked.encodeHeightField(.{ .source_id = 72, .width = 2, .height = 2, .samples = &samples, .cells = &cells, .tile_nodes = tiles }, &bytes, &scratch);
    const inputs = [_][]const u8{encoded.bytes};
    var memory: [2048]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&memory, &inputs);
    var status = fp.MathStatus{};
    var query_tiles: [1]baked.BvhNode = undefined;
    var work: [1]mesh.NodePair = undefined;
    var pair_scratch: [1]mesh.PrimitivePair = undefined;
    var pair_output: [1]mesh.PrimitivePair = undefined;
    var expanded: [2]mesh.HeightTriangle = undefined;
    var hits: [2]u32 = undefined;
    var epa_vertices: [32]gravity.collision.gjk.SupportVertex = undefined;
    var epa_faces: [64]gravity.collision.gjk.EpaFace = undefined;
    var visible: [64]bool = undefined;
    var horizon: [64]gravity.collision.gjk.HorizonEdge = undefined;
    var contacts: [2]gravity.collision.gjk.ContactPoint = undefined;
    const patch = try mesh.convexHeightfieldPatch(.{ .box = .{ .half_extents = .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.one } } }, &assets, .{ .position = .{ .x = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status), .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status), .z = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status) } }, try runtime_view.find(&assets, 72), .{}, .{ .query = .{ .tile_nodes = &query_tiles, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .triangles = &expanded, .intersections = &hits }, .triangle = .{ .epa = .{ .vertices = &epa_vertices, .faces = &epa_faces, .visible = &visible, .horizon = &horizon } }, .contacts = &contacts }, &status);
    try std.testing.expect(patch.len > 0);
    for (patch.points[0..patch.len]) |point| try std.testing.expect(point.separation.raw < 0);
}

test "mesh traversal buffers load from a validated runtime asset view" {
    const vertices = [_]g.Vec3{ .{}, .{ .x = fp.Fp.one }, .{ .y = fp.Fp.one } };
    const triangles = [_]baked.Triangle{.{ .a = 0, .b = 1, .c = 2 }};
    const nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = g.Vec3.zero, .max = .{ .x = fp.Fp.one, .y = fp.Fp.one } }, 0, 1)};
    const primitives = [_]u32{0};
    var bytes: [512]u8 = undefined;
    var scratch: [256]u8 = undefined;
    const encoded = try baked.encodeMesh(.{ .source_id = 17, .vertices = &vertices, .triangles = &triangles, .nodes = &nodes, .primitives = &primitives }, &bytes, &scratch);
    const inputs = [_][]const u8{encoded.bytes};
    var memory: [1024]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&memory, &inputs);
    var loaded_nodes: [1]baked.BvhNode = undefined;
    var loaded_primitives: [1]u32 = undefined;
    const loaded = try mesh.loadMeshBvh(try runtime_view.find(&assets, 17), &loaded_nodes, &loaded_primitives);
    try std.testing.expectEqual(@as(usize, 1), loaded.nodes.len);
    try std.testing.expectEqual(@as(u32, 0), loaded.primitives[0]);
    var work: [1]mesh.NodePair = undefined;
    var pair_scratch: [1]mesh.PrimitivePair = undefined;
    var pair_output: [1]mesh.PrimitivePair = undefined;
    var contacts: [1]gravity.collision.gjk.ContactPoint = undefined;
    var status = fp.MathStatus{};
    const patch = try mesh.sphereMeshPatch(try runtime_view.find(&assets, 17), .{ .center = .{ .x = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status), .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status), .z = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status) }, .radius = fp.Fp.one }, .{ .nodes = &loaded_nodes, .primitives = &loaded_primitives, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .contacts = &contacts }, &status);
    try std.testing.expectEqual(@as(u8, 1), patch.len);
    try std.testing.expectEqual(@as(u32, 0), patch.points[0].feature_b);
    const capsule_patch = try mesh.capsuleMeshPatch(try runtime_view.find(&assets, 17), .{ .segment = .{ .a = .{ .x = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status), .y = fp.Fp.fromInt(-1), .z = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status) }, .b = .{ .x = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status), .y = fp.Fp.one, .z = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status) } }, .radius = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status) }, .{ .nodes = &loaded_nodes, .primitives = &loaded_primitives, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .contacts = &contacts }, &status);
    try std.testing.expectEqual(@as(u8, 1), capsule_patch.len);
    const moved_patch = try mesh.sphereMeshPatchTransformed(try runtime_view.find(&assets, 17), .{ .position = .{ .x = fp.Fp.fromInt(5) } }, .{ .center = .{ .x = fp.Fp.fromInt(5).add(fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status), &status), .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status), .z = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status) }, .radius = fp.Fp.one }, .{ .nodes = &loaded_nodes, .primitives = &loaded_primitives, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .contacts = &contacts }, &status);
    try std.testing.expectEqual(@as(u8, 1), moved_patch.len);
}

test "height tile expansion skips holes and preserves cell material" {
    var status = fp.MathStatus{};
    const samples = [_]fp.Fp{ fp.Fp.zero, fp.Fp.one, fp.Fp.fromInt(2), fp.Fp.fromInt(3) };
    const cells = [_]baked.HeightCell{.{ .material_id = 23 }};
    var tiles: [1]baked.BvhNode = undefined;
    const tile_nodes = try baked.buildHeightFieldTiles(2, 2, &samples, &cells, &tiles);
    var bytes: [1024]u8 = undefined;
    var scratch: [512]u8 = undefined;
    const encoded = try baked.encodeHeightField(.{ .source_id = 19, .width = 2, .height = 2, .samples = &samples, .cells = &cells, .tile_nodes = tile_nodes }, &bytes, &scratch);
    const inputs = [_][]const u8{encoded.bytes};
    var memory: [2048]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&memory, &inputs);
    var triangles: [2]mesh.HeightTriangle = undefined;
    const expanded = try mesh.expandHeightTile(try runtime_view.find(&assets, 19), 0, &triangles);
    try std.testing.expectEqual(@as(usize, 2), expanded.len);
    try std.testing.expectEqual(@as(u32, 23), expanded[0].material_id);
    try std.testing.expectEqual(@as(u1, 1), expanded[1].half);
    var query_tile_nodes: [1]baked.BvhNode = undefined;
    var work: [1]mesh.NodePair = undefined;
    var pair_scratch: [1]mesh.PrimitivePair = undefined;
    var pair_output: [1]mesh.PrimitivePair = undefined;
    var contacts: [2]gravity.collision.gjk.ContactPoint = undefined;
    const patch = try mesh.sphereHeightfieldPatch(try runtime_view.find(&assets, 19), .{ .center = .{ .x = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status), .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status), .z = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status) }, .radius = fp.Fp.one }, .{ .tile_nodes = &query_tile_nodes, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .triangles = &triangles, .contacts = &contacts }, &status);
    try std.testing.expect(patch.len > 0);
    const moved = try mesh.sphereHeightfieldPatchTransformed(try runtime_view.find(&assets, 19), .{ .position = .{ .x = fp.Fp.fromInt(4) } }, .{ .center = .{ .x = fp.Fp.fromInt(4).add(fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status), &status), .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status), .z = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status) }, .radius = fp.Fp.one }, .{ .tile_nodes = &query_tile_nodes, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .triangles = &triangles, .contacts = &contacts }, &status);
    try std.testing.expect(moved.len > 0);
}

test "sphere triangle candidates merge into a stable mesh patch" {
    var status = fp.MathStatus{};
    const triangles = [_]mesh.Triangle{
        .{ .a = .{ .x = fp.Fp.fromInt(-2), .z = fp.Fp.fromInt(-1) }, .b = .{ .z = fp.Fp.fromInt(-1) }, .c = .{ .x = fp.Fp.fromInt(-1), .z = fp.Fp.one } },
        .{ .a = .{ .z = fp.Fp.fromInt(-1) }, .b = .{ .x = fp.Fp.fromInt(2), .z = fp.Fp.fromInt(-1) }, .c = .{ .x = fp.Fp.one, .z = fp.Fp.one } },
    };
    const triangle_ids = [_]u32{ 4, 7 };
    var scratch: [2]gravity.collision.gjk.ContactPoint = undefined;
    const patch = try mesh.sphereTrianglePatch(.{ .center = .{ .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status) }, .radius = fp.Fp.one }, &triangles, &triangle_ids, &scratch, &status);
    try std.testing.expectEqual(@as(u8, 2), patch.len);
    try std.testing.expect(patch.points[0].separation.raw < 0);
    try std.testing.expect(patch.points[1].separation.raw < 0);
}

test "coplanar adjacent mesh triangles suppress duplicate seam contact" {
    const vertices = [_]g.Vec3{ .{ .x = fp.Fp.fromInt(-1), .z = fp.Fp.fromInt(-1) }, .{ .x = fp.Fp.one, .z = fp.Fp.fromInt(-1) }, .{ .x = fp.Fp.one, .z = fp.Fp.one }, .{ .x = fp.Fp.fromInt(-1), .z = fp.Fp.one } };
    const triangles = [_]baked.Triangle{ .{ .a = 0, .b = 1, .c = 2 }, .{ .a = 0, .b = 2, .c = 3 } };
    const nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = vertices[0], .max = vertices[2] }, 0, 2)};
    const primitives = [_]u32{ 0, 1 };
    var bytes: [1024]u8 = undefined;
    var scratch: [512]u8 = undefined;
    const encoded = try baked.encodeMesh(.{ .source_id = 31, .vertices = &vertices, .triangles = &triangles, .nodes = &nodes, .primitives = &primitives }, &bytes, &scratch);
    const inputs = [_][]const u8{encoded.bytes};
    var memory: [2048]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&memory, &inputs);
    var mesh_nodes: [1]baked.BvhNode = undefined;
    var mesh_primitives: [2]u32 = undefined;
    var work: [1]mesh.NodePair = undefined;
    var pair_scratch: [2]mesh.PrimitivePair = undefined;
    var pair_output: [2]mesh.PrimitivePair = undefined;
    var contacts: [2]gravity.collision.gjk.ContactPoint = undefined;
    var status = fp.MathStatus{};
    const patch = try mesh.sphereMeshPatch(try runtime_view.find(&assets, 31), .{ .center = .{ .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status) }, .radius = fp.Fp.one }, .{ .nodes = &mesh_nodes, .primitives = &mesh_primitives, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .contacts = &contacts }, &status);
    try std.testing.expectEqual(@as(u8, 1), patch.len);
    try std.testing.expectEqual(@as(u32, 0), patch.points[0].feature_b);
}

test "sharp adjacent mesh triangles retain distinct edge contacts" {
    const vertices = [_]g.Vec3{ .{ .x = fp.Fp.fromInt(-1), .z = fp.Fp.fromInt(-1) }, .{ .x = fp.Fp.one, .z = fp.Fp.fromInt(-1) }, .{ .x = fp.Fp.one, .z = fp.Fp.one }, .{ .y = fp.Fp.one } };
    const triangles = [_]baked.Triangle{ .{ .a = 0, .b = 1, .c = 2 }, .{ .a = 0, .b = 2, .c = 3 } };
    const nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = .{ .x = fp.Fp.fromInt(-1), .z = fp.Fp.fromInt(-1) }, .max = .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.one } }, 0, 2)};
    const primitives = [_]u32{ 0, 1 };
    var bytes: [1024]u8 = undefined;
    var scratch: [512]u8 = undefined;
    const encoded = try baked.encodeMesh(.{ .source_id = 32, .vertices = &vertices, .triangles = &triangles, .nodes = &nodes, .primitives = &primitives }, &bytes, &scratch);
    const inputs = [_][]const u8{encoded.bytes};
    var memory: [2048]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&memory, &inputs);
    var mesh_nodes: [1]baked.BvhNode = undefined;
    var mesh_primitives: [2]u32 = undefined;
    var work: [1]mesh.NodePair = undefined;
    var pair_scratch: [2]mesh.PrimitivePair = undefined;
    var pair_output: [2]mesh.PrimitivePair = undefined;
    var contacts: [2]gravity.collision.gjk.ContactPoint = undefined;
    var status = fp.MathStatus{};
    const patch = try mesh.sphereMeshPatch(try runtime_view.find(&assets, 32), .{ .center = .{ .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status) }, .radius = fp.Fp.one }, .{ .nodes = &mesh_nodes, .primitives = &mesh_primitives, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .contacts = &contacts }, &status);
    try std.testing.expectEqual(@as(u8, 2), patch.len);
    try std.testing.expect((patch.points[0].feature_b == 0 and patch.points[1].feature_b == 1) or (patch.points[0].feature_b == 1 and patch.points[1].feature_b == 0));
}

test "mesh mesh BVH SAT overlaps match a triangle soup brute oracle" {
    const vertices_a = [_]g.Vec3{ .{}, .{ .x = fp.Fp.fromInt(2) }, .{ .y = fp.Fp.fromInt(2) }, .{ .x = fp.Fp.fromInt(4) }, .{ .x = fp.Fp.fromInt(4), .y = fp.Fp.fromInt(2) } };
    const triangles_a = [_]baked.Triangle{ .{ .a = 0, .b = 1, .c = 2 }, .{ .a = 1, .b = 3, .c = 4 } };
    const nodes_a = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = g.Vec3.zero, .max = .{ .x = fp.Fp.fromInt(4), .y = fp.Fp.fromInt(2) } }, 0, 2)};
    const vertices_b = [_]g.Vec3{ .{ .x = fp.Fp.one, .y = fp.Fp.one }, .{ .x = fp.Fp.fromInt(3), .y = fp.Fp.one }, .{ .x = fp.Fp.one, .y = fp.Fp.fromInt(3) }, .{ .x = fp.Fp.fromInt(9) }, .{ .x = fp.Fp.fromInt(10) }, .{ .x = fp.Fp.fromInt(9), .y = fp.Fp.one } };
    const triangles_b = [_]baked.Triangle{ .{ .a = 0, .b = 1, .c = 2 }, .{ .a = 3, .b = 4, .c = 5 } };
    const nodes_b = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = .{ .x = fp.Fp.one, .y = fp.Fp.one }, .max = .{ .x = fp.Fp.fromInt(10), .y = fp.Fp.fromInt(3) } }, 0, 2)};
    const primitives = [_]u32{ 0, 1 };
    var bytes_a: [1024]u8 = undefined;
    var bytes_b: [1024]u8 = undefined;
    var bake_scratch: [512]u8 = undefined;
    const encoded_a = try baked.encodeMesh(.{ .source_id = 41, .vertices = &vertices_a, .triangles = &triangles_a, .nodes = &nodes_a, .primitives = &primitives }, &bytes_a, &bake_scratch);
    const encoded_b = try baked.encodeMesh(.{ .source_id = 42, .vertices = &vertices_b, .triangles = &triangles_b, .nodes = &nodes_b, .primitives = &primitives }, &bytes_b, &bake_scratch);
    const inputs = [_][]const u8{ encoded_a.bytes, encoded_b.bytes };
    var memory: [4096]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&memory, &inputs);
    const view_a = try runtime_view.find(&assets, 41);
    const view_b = try runtime_view.find(&assets, 42);
    var loaded_nodes_a: [1]baked.BvhNode = undefined;
    var loaded_primitives_a: [2]u32 = undefined;
    var loaded_nodes_b: [1]baked.BvhNode = undefined;
    var loaded_primitives_b: [2]u32 = undefined;
    var work: [1]mesh.NodePair = undefined;
    var pair_scratch: [4]mesh.PrimitivePair = undefined;
    var pair_output: [4]mesh.PrimitivePair = undefined;
    var overlaps: [4]mesh.PrimitivePair = undefined;
    const actual = try mesh.meshMeshOverlaps(view_a, view_b, .{ .nodes_a = &loaded_nodes_a, .primitives_a = &loaded_primitives_a, .nodes_b = &loaded_nodes_b, .primitives_b = &loaded_primitives_b, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .overlaps = &overlaps });
    var patch_contacts: [4]gravity.collision.gjk.ContactPoint = undefined;
    const patch = try mesh.meshMeshPatch(view_a, view_b, .{ .query = .{ .nodes_a = &loaded_nodes_a, .primitives_a = &loaded_primitives_a, .nodes_b = &loaded_nodes_b, .primitives_b = &loaded_primitives_b, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .overlaps = &overlaps }, .contacts = &patch_contacts });
    try std.testing.expect(patch.len > 0);
    for (patch.points[0..patch.len]) |point| try std.testing.expect(point.separation.raw <= 0);
    var no_patch_contacts: [0]gravity.collision.gjk.ContactPoint = .{};
    try std.testing.expectError(error.CapacityExceeded, mesh.meshMeshPatch(view_a, view_b, .{ .query = .{ .nodes_a = &loaded_nodes_a, .primitives_a = &loaded_primitives_a, .nodes_b = &loaded_nodes_b, .primitives_b = &loaded_primitives_b, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .overlaps = &overlaps }, .contacts = &no_patch_contacts }));
    var expected: [4]mesh.PrimitivePair = undefined;
    var expected_count: usize = 0;
    for (triangles_a, 0..) |left, left_id| for (triangles_b, 0..) |right, right_id| {
        if (mesh.trianglesOverlap(.{ .a = vertices_a[left.a], .b = vertices_a[left.b], .c = vertices_a[left.c] }, .{ .a = vertices_b[right.a], .b = vertices_b[right.b], .c = vertices_b[right.c] })) {
            expected[expected_count] = .{ .a = @intCast(left_id), .b = @intCast(right_id) };
            expected_count += 1;
        }
    };
    try std.testing.expectEqual(expected_count, actual.len);
    for (actual, expected[0..expected_count]) |got, want| {
        try std.testing.expectEqual(want.a, got.a);
        try std.testing.expectEqual(want.b, got.b);
    }
    var too_small = [_]mesh.PrimitivePair{.{ .a = 99, .b = 99 }};
    try std.testing.expectError(error.CapacityExceeded, mesh.meshMeshOverlaps(view_a, view_b, .{ .nodes_a = &loaded_nodes_a, .primitives_a = &loaded_primitives_a, .nodes_b = &loaded_nodes_b, .primitives_b = &loaded_primitives_b, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .overlaps = &too_small }));
    try std.testing.expectEqual(@as(u32, 99), too_small[0].a);
    var moved_nodes_a: [1]baked.BvhNode = undefined;
    var moved_primitives_a: [2]u32 = undefined;
    var moved_nodes_b: [1]baked.BvhNode = undefined;
    var moved_primitives_b: [2]u32 = undefined;
    var moved_status = fp.MathStatus{};
    const moved = try mesh.meshMeshOverlapsTransformed(view_a, .{}, view_b, .{ .position = .{ .x = fp.Fp.fromInt(-8) } }, .{ .nodes_a = &moved_nodes_a, .primitives_a = &moved_primitives_a, .nodes_b = &moved_nodes_b, .primitives_b = &moved_primitives_b, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .overlaps = &overlaps }, &moved_status);
    try std.testing.expectEqual(@as(usize, 2), moved.len);
    try std.testing.expectEqual(@as(u32, 0), moved[0].a);
    try std.testing.expectEqual(@as(u32, 1), moved[0].b);
    try std.testing.expectEqual(@as(u32, 1), moved[1].a);
    try std.testing.expectEqual(@as(u32, 1), moved[1].b);
    const moved_patch = try mesh.meshMeshPatchTransformed(view_a, .{}, view_b, .{ .position = .{ .x = fp.Fp.fromInt(-8) } }, .{ .query = .{ .nodes_a = &moved_nodes_a, .primitives_a = &moved_primitives_a, .nodes_b = &moved_nodes_b, .primitives_b = &moved_primitives_b, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .overlaps = &overlaps }, .contacts = &patch_contacts }, &moved_status);
    try std.testing.expect(patch.len > 0);
    try std.testing.expect(moved_patch.len > 0);
}

test "compound surface traversal preserves mesh child path and transform" {
    const vertices = [_]g.Vec3{ .{}, .{ .x = fp.Fp.one }, .{ .y = fp.Fp.one } };
    const triangles = [_]baked.Triangle{.{ .a = 0, .b = 1, .c = 2 }};
    const mesh_nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = g.Vec3.zero, .max = .{ .x = fp.Fp.one, .y = fp.Fp.one } }, 0, 1)};
    const primitives = [_]u32{0};
    var mesh_bytes: [512]u8 = undefined;
    var bake_scratch: [256]u8 = undefined;
    const encoded_mesh = try baked.encodeMesh(.{ .source_id = 51, .vertices = &vertices, .triangles = &triangles, .nodes = &mesh_nodes, .primitives = &primitives }, &mesh_bytes, &bake_scratch);
    const children = [_]baked.CompoundChild{.{ .ordinal = 0, .content_hash = encoded_mesh.content_hash, .translation = .{ .x = fp.Fp.fromInt(3) }, .rotation = g.Quat.identity }};
    const compound_nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = .{ .x = fp.Fp.fromInt(3) }, .max = .{ .x = fp.Fp.fromInt(4), .y = fp.Fp.one } }, 0, 1)};
    var compound_bytes: [512]u8 = undefined;
    var compound_scratch: [256]u8 = undefined;
    const encoded_compound = try baked.encodeCompound(.{ .source_id = 52, .children = &children, .nodes = &compound_nodes }, &compound_bytes, &compound_scratch);
    const inputs = [_][]const u8{ encoded_mesh.bytes, encoded_compound.bytes };
    var memory: [2048]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&memory, &inputs);
    var leaves: [1]gravity.collision.shapes.CompoundLeaf = undefined;
    var loaded_nodes: [1]baked.BvhNode = undefined;
    var loaded_primitives: [1]u32 = undefined;
    var work: [1]mesh.NodePair = undefined;
    var pair_scratch: [1]mesh.PrimitivePair = undefined;
    var pair_output: [1]mesh.PrimitivePair = undefined;
    var contacts: [1]gravity.collision.gjk.ContactPoint = undefined;
    var tile_nodes: [1]baked.BvhNode = undefined;
    var height_triangles: [1]mesh.HeightTriangle = undefined;
    var height_contacts: [1]gravity.collision.gjk.ContactPoint = undefined;
    var merged: [4]gravity.collision.gjk.ContactPoint = undefined;
    var status = fp.MathStatus{};
    const patch = try mesh.sphereCompoundSurfacePatch(.{ .compound = .{ .source_id = 52 } }, &assets, .{}, .{ .center = .{ .x = fp.Fp.fromInt(3).add(fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status), &status), .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status), .z = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status) }, .radius = fp.Fp.one }, .{ .leaves = &leaves, .mesh = .{ .nodes = &loaded_nodes, .primitives = &loaded_primitives, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .contacts = &contacts }, .heightfield = .{ .tile_nodes = &tile_nodes, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .triangles = &height_triangles, .contacts = &height_contacts }, .merged = &merged }, &status);
    try std.testing.expectEqual(@as(u8, 1), patch.len);
    try std.testing.expectEqual(@as(u8, 1), patch.points[0].path_b.len);
    try std.testing.expectEqual(@as(u32, 0), patch.points[0].path_b.values[0]);
    var convex_hits: [1]u32 = undefined;
    var epa_vertices: [32]gravity.collision.gjk.SupportVertex = undefined;
    var epa_faces: [64]gravity.collision.gjk.EpaFace = undefined;
    var visible: [64]bool = undefined;
    var horizon: [64]gravity.collision.gjk.HorizonEdge = undefined;
    var convex_contacts: [1]gravity.collision.gjk.ContactPoint = undefined;
    var empty_tiles: [0]baked.BvhNode = .{};
    var empty_triangles: [0]mesh.HeightTriangle = .{};
    var empty_hits: [0]u32 = .{};
    var empty_contacts: [0]gravity.collision.gjk.ContactPoint = .{};
    const convex_patch = try mesh.convexCompoundSurfacePatch(.{ .compound = .{ .source_id = 52 } }, &assets, .{}, .{ .box = .{ .half_extents = .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.one } } }, .{ .position = .{ .x = fp.Fp.fromInt(3).add(fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status), &status), .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status), .z = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status) } }, .{ .leaves = &leaves, .mesh = .{ .query = .{ .nodes = &loaded_nodes, .primitives = &loaded_primitives, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .intersections = &convex_hits }, .triangle = .{ .epa = .{ .vertices = &epa_vertices, .faces = &epa_faces, .visible = &visible, .horizon = &horizon } }, .contacts = &convex_contacts }, .heightfield = .{ .query = .{ .tile_nodes = &empty_tiles, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .triangles = &empty_triangles, .intersections = &empty_hits }, .triangle = .{ .epa = .{ .vertices = &epa_vertices, .faces = &epa_faces, .visible = &visible, .horizon = &horizon } }, .contacts = &empty_contacts }, .merged = &merged }, &status);
    try std.testing.expectEqual(@as(u8, 1), convex_patch.len);
    try std.testing.expectEqual(@as(u8, 1), convex_patch.points[0].path_b.len);
    try std.testing.expectEqual(@as(u32, 0), convex_patch.points[0].path_b.values[0]);
    var no_merged: [0]gravity.collision.gjk.ContactPoint = .{};
    try std.testing.expectError(error.CapacityExceeded, mesh.sphereCompoundSurfacePatch(.{ .compound = .{ .source_id = 52 } }, &assets, .{}, .{ .center = .{ .x = fp.Fp.fromInt(3).add(fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status), &status), .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status), .z = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status) }, .radius = fp.Fp.one }, .{ .leaves = &leaves, .mesh = .{ .nodes = &loaded_nodes, .primitives = &loaded_primitives, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .contacts = &contacts }, .heightfield = .{ .tile_nodes = &tile_nodes, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .triangles = &height_triangles, .contacts = &height_contacts }, .merged = &no_merged }, &status));
}
