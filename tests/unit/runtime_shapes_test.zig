const std = @import("std");
const shapes = @import("gravity").collision.shapes;
const fp = @import("gravity").math.fp;
const geometry = @import("gravity").math.geometry;
const ids = @import("gravity").core.ids;

fn collider(shape: shapes.Shape, body: u32) shapes.Collider {
    return .{ .body = ids.BodyId.init(body, 0), .shape = shape };
}
fn expectRawNear(expected: i64, actual: i64) !void {
    const delta = @as(i128, expected) - actual;
    try std.testing.expect(@abs(delta) <= 512);
}

test "primitive bounds and support use exact fixed-point extrema" {
    var status = fp.MathStatus{};
    var ignored_assets: [0]@import("gravity").assets.store.Asset = .{};
    const store = @import("gravity").assets.store.Store{ .assets = &ignored_assets, .bytes = &.{}, .asset_set_hash = [_]u8{0} ** 32 };
    const box: shapes.Shape = .{ .box = .{ .half_extents = .{ .x = fp.Fp.fromInt(2), .y = fp.Fp.fromInt(3), .z = fp.Fp.fromInt(4) } } };
    const bounds = try shapes.localAabb(box, &store, &status);
    try std.testing.expectEqual(-fp.Fp.fromInt(2).raw, bounds.min.x.raw);
    const point = try shapes.support(box, &store, .{ .x = fp.Fp.one, .y = fp.Fp{ .raw = -1 }, .z = fp.Fp.zero }, &status);
    try std.testing.expectEqual(fp.Fp.fromInt(2).raw, point.point.x.raw);
    try std.testing.expectEqual(-fp.Fp.fromInt(3).raw, point.point.y.raw);
}

test "filter is symmetric and preserves sensor overlap semantics" {
    const sphere: shapes.Shape = .{ .sphere = .{ .radius = fp.Fp.one } };
    var a = collider(sphere, 1);
    var b = collider(sphere, 2);
    try std.testing.expectEqual(shapes.FilterResult.contact, shapes.filter(&a, .dynamic, &b, .static));
    b.sensor = true;
    try std.testing.expectEqual(shapes.FilterResult.overlap, shapes.filter(&a, .dynamic, &b, .static));
    a.group = -7;
    b.group = -7;
    try std.testing.expectEqual(shapes.FilterResult.ignore, shapes.filter(&a, .dynamic, &b, .static));
    a.group = 9;
    b.group = 9;
    b.sensor = false;
    try std.testing.expectEqual(shapes.FilterResult.ignore, shapes.filter(&a, .static, &b, .kinematic));
    a.mask = 0;
    b.mask = 0;
    try std.testing.expectEqual(shapes.FilterResult.contact, shapes.filter(&a, .dynamic, &b, .static));
    try std.testing.expectEqual(shapes.filter(&a, .dynamic, &b, .static), shapes.filter(&b, .static, &a, .dynamic));
}

test "dynamic height field is rejected before publication" {
    const shape: shapes.Shape = .{ .height_field = .{ .asset = ids.AssetId.init(1, 0) } };
    try std.testing.expectError(error.InvalidBodyShape, shapes.validateBodyShape(shape, .dynamic));
    try shapes.validateBodyShape(shape, .static);
    _ = geometry.Transform3{};
}

test "shape revision is an explicit derived-cache invalidation key" {
    const initial: shapes.Shape = .{ .convex_hull = .{ .source_id = 19, .revision = 4 } };
    const changed: shapes.Shape = .{ .convex_hull = .{ .source_id = 19, .revision = 5 } };
    const key = shapes.cacheKey(initial);
    try std.testing.expect(shapes.cacheValid(key, initial));
    try std.testing.expect(!shapes.cacheValid(key, changed));
}

test "primitive mass tensors use central axes and explicit mesh override" {
    var status = fp.MathStatus{};
    var empty_assets: [0]@import("gravity").assets.store.Asset = .{};
    const store = @import("gravity").assets.store.Store{ .assets = &empty_assets, .bytes = &.{}, .asset_set_hash = [_]u8{0} ** 32 };
    const box: shapes.Shape = .{ .box = .{ .half_extents = .{ .x = fp.Fp.one, .y = fp.Fp.fromInt(2), .z = fp.Fp.fromInt(3) } } };
    const properties = try shapes.massProperties(box, &store, fp.Fp.one, null, &status);
    try std.testing.expectEqual(fp.Fp.fromInt(48).raw, properties.mass.raw);
    try expectRawNear(fp.Fp.fromInt(208).raw, properties.inertia.xx.raw);
    try expectRawNear(fp.Fp.fromInt(160).raw, properties.inertia.yy.raw);
    try expectRawNear(fp.Fp.fromInt(80).raw, properties.inertia.zz.raw);
    const mesh: shapes.Shape = .{ .triangle_mesh = .{ .source_id = 99 } };
    try std.testing.expectError(error.MissingMass, shapes.massProperties(mesh, &store, fp.Fp.one, null, &status));
    const override = shapes.MassOverride{ .mass = fp.Fp.one, .center = geometry.Vec3.zero, .inertia = .{ .xx = fp.Fp.one, .yy = fp.Fp.one, .zz = fp.Fp.one, .xy = fp.Fp.zero, .xz = fp.Fp.zero, .yz = fp.Fp.zero } };
    try std.testing.expectEqual(fp.Fp.one.raw, (try shapes.massProperties(mesh, &store, fp.Fp.one, override, &status)).mass.raw);
}

test "compound traversal transforms AABB and records stable child path" {
    const baked = @import("gravity").geometry.baked;
    const asset_store = @import("gravity").assets.store;
    const hash = @import("gravity").state.hash;
    const vertices = [_]geometry.Vec3{ .{ .x = fp.Fp.zero, .y = fp.Fp.zero, .z = fp.Fp.zero }, .{ .x = fp.Fp.one, .y = fp.Fp.zero, .z = fp.Fp.zero }, .{ .x = fp.Fp.zero, .y = fp.Fp.one, .z = fp.Fp.zero } };
    const triangles = [_]baked.Triangle{.{ .a = 0, .b = 1, .c = 2 }};
    const nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = vertices[0], .max = vertices[2] }, 0, 1)};
    const primitives = [_]u32{0};
    var mesh_bytes: [512]u8 = undefined;
    var mesh_scratch: [256]u8 = undefined;
    const mesh = try baked.encodeMesh(.{ .source_id = 3, .vertices = &vertices, .triangles = &triangles, .nodes = &nodes, .primitives = &primitives }, &mesh_bytes, &mesh_scratch);
    const mesh_hash = hash.oneShot256(.asset, mesh.bytes);
    const children = [_]baked.CompoundChild{.{ .ordinal = 0, .content_hash = mesh_hash, .translation = .{ .x = fp.Fp.fromInt(10) }, .rotation = geometry.Quat.identity }};
    const compound_nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = .{ .x = fp.Fp.fromInt(10), .y = fp.Fp.zero, .z = fp.Fp.zero }, .max = .{ .x = fp.Fp.fromInt(11), .y = fp.Fp.one, .z = fp.Fp.zero } }, 0, 1)};
    var compound_bytes: [512]u8 = undefined;
    var compound_scratch: [256]u8 = undefined;
    const compound = try baked.encodeCompound(.{ .source_id = 7, .children = &children, .nodes = &compound_nodes }, &compound_bytes, &compound_scratch);
    const inputs = [_][]const u8{ mesh.bytes, compound.bytes };
    var memory: [2048]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&memory, &inputs);
    var status = fp.MathStatus{};
    const shape: shapes.Shape = .{ .compound = .{ .source_id = 7 } };
    const bounds = try shapes.localAabb(shape, &assets, &status);
    try std.testing.expectEqual(fp.Fp.fromInt(10).raw, bounds.min.x.raw);
    const point = try shapes.support(shape, &assets, geometry.Vec3.unit_x, &status);
    try std.testing.expectEqual(fp.Fp.fromInt(11).raw, point.point.x.raw);
    try std.testing.expectEqual(@as(u8, 1), point.child_path.len);
    try std.testing.expectEqual(@as(u32, 0), point.child_path.values[0]);
    const mesh_shape: shapes.Shape = .{ .triangle_mesh = .{ .source_id = 3 } };
    try std.testing.expectEqual(@as(u32, 2), (try shapes.primitive(mesh_shape, &assets, 0)).c);
    try std.testing.expectEqual(fp.Fp.one.raw, (try shapes.vertex(mesh_shape, &assets, 1, &status)).x.raw);
    try std.testing.expectError(error.InvalidShape, shapes.localAabb(.{ .convex_hull = .{ .source_id = 3 } }, &assets, &status));
}

test "compound mass combines rotated child tensors with parallel-axis offsets" {
    const baked = @import("gravity").geometry.baked;
    const asset_store = @import("gravity").assets.store;
    const hash = @import("gravity").state.hash;
    const points = [_]geometry.Vec3{ .{ .x = fp.Fp.zero, .y = fp.Fp.zero, .z = fp.Fp.zero }, .{ .x = fp.Fp.one, .y = fp.Fp.zero, .z = fp.Fp.zero }, .{ .x = fp.Fp.zero, .y = fp.Fp.one, .z = fp.Fp.zero }, .{ .x = fp.Fp.zero, .y = fp.Fp.zero, .z = fp.Fp.one } };
    var vertices: [4]geometry.Vec3 = undefined;
    var triangles: [4]baked.Triangle = undefined;
    var faces: [4]baked.HullFace = undefined;
    var edges: [12]baked.HalfEdge = undefined;
    var status = fp.MathStatus{};
    const hull = try baked.buildConvexHull(&points, &vertices, &triangles, &faces, &edges, &status);
    var hull_bytes: [2048]u8 = undefined;
    var hull_scratch: [1024]u8 = undefined;
    const encoded_hull = try baked.encodeConvexHull(hull, 3, &hull_bytes, &hull_scratch);
    const hull_hash = hash.oneShot256(.asset, encoded_hull.bytes);
    const children = [_]baked.CompoundChild{
        .{ .ordinal = 0, .content_hash = hull_hash, .translation = geometry.Vec3.zero, .rotation = geometry.Quat.identity },
        .{ .ordinal = 1, .content_hash = hull_hash, .translation = .{ .x = fp.Fp.fromInt(2) }, .rotation = geometry.Quat.identity },
    };
    const nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = geometry.Vec3.zero, .max = .{ .x = fp.Fp.fromInt(3), .y = fp.Fp.one, .z = fp.Fp.one } }, 0, 2)};
    var compound_bytes: [1024]u8 = undefined;
    var compound_scratch: [512]u8 = undefined;
    const encoded_compound = try baked.encodeCompound(.{ .source_id = 7, .children = &children, .nodes = &nodes }, &compound_bytes, &compound_scratch);
    const inputs = [_][]const u8{ encoded_hull.bytes, encoded_compound.bytes };
    var memory: [4096]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&memory, &inputs);
    const hull_shape: shapes.Shape = .{ .convex_hull = .{ .source_id = 3 } };
    try std.testing.expectEqual(fp.Fp.one.raw, (try shapes.support(hull_shape, &assets, geometry.Vec3.unit_x, &status)).point.x.raw);
    try std.testing.expectEqual(@as(u32, 3), (try shapes.hullFace(hull_shape, &assets, 0)).half_edge_count);
    try std.testing.expect((try shapes.hullHalfEdge(hull_shape, &assets, 0)).origin < 4);
    const properties = try shapes.massProperties(.{ .compound = .{ .source_id = 7 } }, &assets, fp.Fp.one, null, &status);
    try expectRawNear(fp.Fp.fromRatio(1, 3, &status).raw, properties.mass.raw);
    try expectRawNear(fp.Fp.fromRatio(5, 4, &status).raw, properties.center.x.raw);
    // Translation is along X, so the Y principal moment gains m_total * 1^2.
    try std.testing.expect(properties.inertia.yy.raw > fp.Fp.fromRatio(1, 3, &status).raw);
    var leaves: [2]shapes.CompoundLeaf = undefined;
    const resolved = try shapes.collectCompoundLeaves(.{ .compound = .{ .source_id = 7 } }, &assets, .{ .position = .{ .x = fp.Fp.fromInt(10) } }, &leaves, &status);
    try std.testing.expectEqual(@as(usize, 2), resolved.len);
    try std.testing.expectEqual(@as(u8, 1), resolved[1].path.len);
    try std.testing.expectEqual(@as(u32, 1), resolved[1].path.values[0]);
    try std.testing.expectEqual(fp.Fp.fromInt(12).raw, resolved[1].transform.position.x.raw);
    var tiny: [1]shapes.CompoundLeaf = undefined;
    try std.testing.expectError(error.CapacityExceeded, shapes.collectCompoundLeaves(.{ .compound = .{ .source_id = 7 } }, &assets, .{}, &tiny, &status));
}

test "height field runtime bounds and support read canonical samples" {
    const baked = @import("gravity").geometry.baked;
    const asset_store = @import("gravity").assets.store;
    const samples = [_]fp.Fp{ fp.Fp.fromInt(-1), fp.Fp.fromInt(2), fp.Fp.zero, fp.Fp.fromInt(3) };
    const cells = [_]baked.HeightCell{.{}};
    var tiles: [1]baked.BvhNode = undefined;
    const tile_nodes = try baked.buildHeightFieldTiles(2, 2, &samples, &cells, &tiles);
    var bytes: [1024]u8 = undefined;
    var scratch: [512]u8 = undefined;
    const encoded = try baked.encodeHeightField(.{ .source_id = 11, .width = 2, .height = 2, .samples = &samples, .cells = &cells, .tile_nodes = tile_nodes }, &bytes, &scratch);
    const inputs = [_][]const u8{encoded.bytes};
    var memory: [2048]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&memory, &inputs);
    var status = fp.MathStatus{};
    const shape: shapes.Shape = .{ .height_field = .{ .source_id = 11 } };
    try std.testing.expectEqual(fp.Fp.fromInt(-1).raw, (try shapes.localAabb(shape, &assets, &status)).min.y.raw);
    const point = try shapes.support(shape, &assets, geometry.Vec3.unit_y, &status);
    try std.testing.expectEqual(fp.Fp.fromInt(3).raw, point.point.y.raw);
    try std.testing.expectEqual(@as(u32, 3), switch (point.feature) {
        .vertex => |id| id,
        else => unreachable,
    });
}
