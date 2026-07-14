const std = @import("std");
const gravity = @import("gravity");
const gjk = gravity.collision.gjk;
const shapes = gravity.collision.shapes;
const asset_store = gravity.assets.store;
const analytic = gravity.collision.analytic;
const ids = gravity.core.ids;
const fp = gravity.math.fp;
const g = gravity.math.geometry;
const Pair = struct { a: g.Vec3, b: g.Vec3, half: fp.Fp };
fn support(raw: *const anyopaque, direction: g.Vec3, status: *fp.MathStatus) gjk.SupportVertex {
    const pair: *const Pair = @ptrCast(@alignCast(raw));
    const witness_a = pair.a.add(.{ .x = if (direction.x.raw >= 0) pair.half else pair.half.neg(status), .y = if (direction.y.raw >= 0) pair.half else pair.half.neg(status), .z = if (direction.z.raw >= 0) pair.half else pair.half.neg(status) }, status);
    const witness_b = pair.b.add(.{ .x = if (direction.x.raw < 0) pair.half else pair.half.neg(status), .y = if (direction.y.raw < 0) pair.half else pair.half.neg(status), .z = if (direction.z.raw < 0) pair.half else pair.half.neg(status) }, status);
    return .{ .point = witness_a.sub(witness_b, status), .witness_a = witness_a, .witness_b = witness_b, .feature_a = 0, .feature_b = 0 };
}
test "GJK classifies separated and intersecting Minkowski boxes" {
    var status = fp.MathStatus{};
    var separated = Pair{ .a = g.Vec3.zero, .b = .{ .x = fp.Fp.fromInt(3) }, .half = fp.Fp.one };
    const context = gjk.SupportContext{ .ptr = &separated, .call = support };
    try std.testing.expectEqual(gjk.Status.separated, gjk.intersect(context, g.Vec3.unit_x, &status).status);
    var overlap = Pair{ .a = g.Vec3.zero, .b = .{ .x = fp.Fp.one }, .half = fp.Fp.one };
    const result = gjk.intersect(.{ .ptr = &overlap, .call = support }, g.Vec3.unit_x, &status);
    try std.testing.expectEqual(gjk.Status.intersecting, result.status);
}

test "GJK distance returns stable separation witnesses and seed" {
    var status = fp.MathStatus{};
    var separated = Pair{ .a = g.Vec3.zero, .b = .{ .x = fp.Fp.fromInt(3) }, .half = fp.Fp.one };
    const result = gjk.distance(.{ .ptr = &separated, .call = support }, g.Vec3.unit_x, &status);
    try std.testing.expectEqual(gjk.Status.separated, result.status);
    try std.testing.expectEqual(fp.Fp.one.raw, result.distance.raw);
    try std.testing.expectEqual(fp.Fp.one.neg(&status).raw, result.witness_a.sub(result.witness_b, &status).x.raw);
    try std.testing.expect(gjk.seedFromResult(result).direction.x.raw != 0);
    var overlap = Pair{ .a = g.Vec3.zero, .b = .{ .x = fp.Fp.one }, .half = fp.Fp.one };
    try std.testing.expectEqual(gjk.Status.intersecting, gjk.distance(.{ .ptr = &overlap, .call = support }, g.Vec3.unit_x, &status).status);
}

test "runtime shape adapter preserves world transforms and errors" {
    var status = fp.MathStatus{};
    var empty_assets: [0]asset_store.Asset = .{};
    const assets = asset_store.Store{ .assets = &empty_assets, .bytes = &.{}, .asset_set_hash = [_]u8{0} ** 32 };
    const box: shapes.Shape = .{ .box = .{ .half_extents = .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.one } } };
    var pair = gjk.ShapePairContext{ .shape_a = box, .shape_b = box, .assets = &assets, .transform_b = .{ .position = .{ .x = fp.Fp.fromInt(3) } } };
    try std.testing.expectEqual(gjk.Status.separated, (try gjk.intersectShapes(&pair, g.Vec3.unit_x, &status)).status);
    pair.transform_b.position.x = fp.Fp.one;
    try std.testing.expectEqual(gjk.Status.intersecting, (try gjk.intersectShapes(&pair, g.Vec3.unit_x, &status)).status);
    pair.shape_a = .{ .convex_hull = .{ .source_id = 99 } };
    try std.testing.expectError(error.MissingAsset, gjk.intersectShapes(&pair, g.Vec3.unit_x, &status));
}

test "runtime GJK distance agrees with analytic box separation" {
    var status = fp.MathStatus{};
    var empty_assets: [0]asset_store.Asset = .{};
    const assets = asset_store.Store{ .assets = &empty_assets, .bytes = &.{}, .asset_set_hash = [_]u8{0} ** 32 };
    const box: shapes.Shape = .{ .box = .{ .half_extents = .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.one } } };
    var pair = gjk.ShapePairContext{ .shape_a = box, .shape_b = box, .assets = &assets, .transform_b = .{ .position = .{ .x = fp.Fp.fromInt(3) } } };
    const result = try gjk.distanceShapes(&pair, .{}, &status);
    const oracle = analytic.collide(.{ .box = .{ .center = g.Vec3.zero, .half_extents = .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.one }, .orientation = g.Quat.identity } }, .{ .box = .{ .center = .{ .x = fp.Fp.fromInt(3) }, .half_extents = .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.one }, .orientation = g.Quat.identity } }, g.Vec3.zero, ids.ColliderId.init(1, 0), ids.ColliderId.init(2, 0), &status).?;
    try std.testing.expectEqual(gjk.Status.separated, result.status);
    try std.testing.expectEqual(oracle.separation.raw, result.distance.raw);
    pair.transform_b.position.x = fp.Fp.one;
    const overlap = try gjk.distanceShapes(&pair, gjk.seedFromResult(result), &status);
    const overlap_oracle = analytic.collide(.{ .box = .{ .center = g.Vec3.zero, .half_extents = .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.one }, .orientation = g.Quat.identity } }, .{ .box = .{ .center = .{ .x = fp.Fp.one }, .half_extents = .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.one }, .orientation = g.Quat.identity } }, g.Vec3.zero, ids.ColliderId.init(1, 0), ids.ColliderId.init(2, 0), &status).?;
    try std.testing.expectEqual(gjk.Status.intersecting, overlap.status);
    try std.testing.expect(overlap_oracle.separation.raw < 0);
}

test "EPA closest face uses stable total key" {
    const face = gjk.EpaFace{ .vertices = .{ 7, 2, 5 }, .normal = g.Vec3.unit_x, .distance = fp.Fp.one };
    const earlier = gjk.EpaFace{ .vertices = .{ 4, 1, 3 }, .normal = g.Vec3.unit_x, .distance = fp.Fp.one };
    const farther = gjk.EpaFace{ .vertices = .{ 0, 1, 2 }, .normal = g.Vec3.unit_y, .distance = fp.Fp.fromInt(2) };
    const faces = [_]gjk.EpaFace{ face, farther, earlier };
    try std.testing.expectEqual(@as(?usize, 2), gjk.closestEpaFace(&faces));
}
test "EPA pool reports fixed capacity without truncation" {
    var storage: [1]gjk.EpaFace = undefined;
    var pool = gjk.EpaPool{ .faces = &storage };
    const face = gjk.EpaFace{ .vertices = .{ 0, 1, 2 }, .normal = g.Vec3.unit_x, .distance = fp.Fp.one };
    try pool.append(face);
    try std.testing.expectError(error.CapacityExceeded, pool.append(face));
    try std.testing.expectEqual(@as(usize, 1), pool.active().len);
    try std.testing.expectEqual(@as(?usize, 0), pool.closest());
}
test "EPA horizon removes internal reverse edges in stable order" {
    const faces = [_]gjk.EpaFace{
        .{ .vertices = .{ 0, 1, 2 }, .normal = g.Vec3.unit_x, .distance = fp.Fp.one },
        .{ .vertices = .{ 2, 1, 3 }, .normal = g.Vec3.unit_y, .distance = fp.Fp.one },
    };
    const visible = [_]bool{ true, true };
    var edges: [6]gjk.HorizonEdge = undefined;
    const result = try gjk.collectHorizon(&faces, &visible, &edges);
    try std.testing.expectEqual(@as(usize, 4), result.len);
    try std.testing.expectEqual(@as(u16, 0), result[0].from);
}
test "EPA expands a fixed tetrahedron and returns witnesses" {
    var status = fp.MathStatus{};
    var pair = Pair{ .a = g.Vec3.zero, .b = .{ .x = fp.Fp.one }, .half = fp.Fp.one };
    const vertex = struct {
        fn make(x: i32, y: i32, z: i32) gjk.SupportVertex {
            const point = g.Vec3{ .x = fp.Fp.fromInt(x), .y = fp.Fp.fromInt(y), .z = fp.Fp.fromInt(z) };
            return .{ .point = point, .witness_a = point, .witness_b = g.Vec3.zero, .feature_a = @intCast(x + 2), .feature_b = @intCast(y + 2) };
        }
    }.make;
    const tetra = [_]gjk.SupportVertex{ vertex(-1, -1, -1), vertex(1, -1, 1), vertex(-1, 1, 1), vertex(1, 1, -1) };
    var vertices: [130]gjk.SupportVertex = undefined;
    var faces: [256]gjk.EpaFace = undefined;
    var visible: [256]bool = undefined;
    var horizon: [768]gjk.HorizonEdge = undefined;
    const result = gjk.epa(.{ .ptr = &pair, .call = support }, &tetra, .{ .vertices = &vertices, .faces = &faces, .visible = &visible, .horizon = &horizon }, &status);
    try std.testing.expectEqual(gjk.EpaStatus.converged, result.status);
    try std.testing.expect(result.depth.raw > 0);
    try std.testing.expect(result.witness_a.sub(result.witness_b, &status).dot(result.normal, &status).raw > 0);
}
test "patch reduction keeps four deterministic coverage points" {
    var status = fp.MathStatus{};
    const contact = struct {
        fn make(x: i32, y: i32, separation: i32, feature: u32) gjk.ContactPoint {
            const point = g.Vec3{ .x = fp.Fp.fromInt(x), .y = fp.Fp.fromInt(y) };
            return .{ .point_a = point, .point_b = point, .separation = fp.Fp.fromInt(separation), .feature_a = feature, .feature_b = feature };
        }
    }.make;
    const candidates = [_]gjk.ContactPoint{ contact(-1, -1, -1, 4), contact(1, -1, -1, 3), contact(1, 1, -2, 2), contact(-1, 1, -1, 1), contact(0, 0, -1, 0) };
    const patch = gjk.reducePatch(&candidates, &status);
    try std.testing.expectEqual(@as(u8, 4), patch.len);
    try std.testing.expectEqual(@as(u32, 2), patch.points[0].feature_a);
    for (patch.points[0..patch.len], 0..) |left, i| for (patch.points[0..patch.len], 0..) |right, j| if (i != j) try std.testing.expect(left.feature_a != right.feature_a);
}

test "face clipping retains boundary and projects deterministic contacts" {
    var status = fp.MathStatus{};
    const input = [_]gjk.ClipVertex{
        .{ .point = .{ .x = fp.Fp.fromInt(-2), .y = fp.Fp.fromInt(-1) }, .feature = 4 },
        .{ .point = .{ .x = fp.Fp.fromInt(2), .y = fp.Fp.fromInt(-1) }, .feature = 5 },
        .{ .point = .{ .x = fp.Fp.fromInt(2), .y = fp.Fp.fromInt(1) }, .feature = 6 },
        .{ .point = .{ .x = fp.Fp.fromInt(-2), .y = fp.Fp.fromInt(1) }, .feature = 7 },
    };
    var clipped_storage: [8]gjk.ClipVertex = undefined;
    const clipped = try gjk.clipPolygonAgainstPlane(&input, g.Vec3.unit_x, fp.Fp.one, &clipped_storage, &status);
    try std.testing.expectEqual(@as(usize, 4), clipped.len);
    for (clipped) |point| try std.testing.expect(point.point.x.raw <= fp.Fp.one.raw);
    var contacts_storage: [8]gjk.ContactPoint = undefined;
    const contacts = try gjk.contactsFromIncident(g.Vec3.unit_x, fp.Fp.zero, 12, clipped, &contacts_storage, &status);
    try std.testing.expectEqual(@as(usize, 2), contacts.len);
    for (contacts) |contact| {
        try std.testing.expect(contact.separation.raw <= 0);
        try std.testing.expectEqual(fp.Fp.zero.raw, contact.point_a.x.raw);
        try std.testing.expectEqual(@as(u32, 12), contact.feature_a);
    }
}

test "box reference and incident faces produce a four-point patch" {
    var status = fp.MathStatus{};
    var empty_assets: [0]asset_store.Asset = .{};
    const assets = asset_store.Store{ .assets = &empty_assets, .bytes = &.{}, .asset_set_hash = [_]u8{0} ** 32 };
    const box: shapes.Shape = .{ .box = .{ .half_extents = .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.one } } };
    var reference_storage: [8]gjk.ClipVertex = undefined;
    var incident_storage: [8]gjk.ClipVertex = undefined;
    const reference = try gjk.referenceFace(box, &assets, .{}, g.Vec3.unit_x, &reference_storage, &status);
    const incident = try gjk.referenceFace(box, &assets, .{ .position = .{ .x = fp.Fp.fromInt(3).div(fp.Fp.fromInt(2), &status) } }, .{ .x = fp.Fp.one.neg(&status) }, &incident_storage, &status);
    var scratch_a: [16]gjk.ClipVertex = undefined;
    var scratch_b: [16]gjk.ClipVertex = undefined;
    var contacts: [16]gjk.ContactPoint = undefined;
    const patch = try gjk.clipFacePair(reference, incident, &scratch_a, &scratch_b, &contacts, &status);
    try std.testing.expectEqual(@as(u8, 4), patch.len);
    for (patch.points[0..patch.len]) |contact| {
        try std.testing.expectEqual(-fp.Fp.fromInt(1).div(fp.Fp.fromInt(2), &status).raw, contact.separation.raw);
        try std.testing.expectEqual(fp.Fp.one.raw, contact.point_a.x.raw);
    }
}

test "runtime box pair flows from GJK simplex into EPA" {
    var status = fp.MathStatus{};
    var empty_assets: [0]asset_store.Asset = .{};
    const assets = asset_store.Store{ .assets = &empty_assets, .bytes = &.{}, .asset_set_hash = [_]u8{0} ** 32 };
    const box: shapes.Shape = .{ .box = .{ .half_extents = .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.one } } };
    var pair = gjk.ShapePairContext{ .shape_a = box, .shape_b = box, .assets = &assets, .transform_b = .{ .position = .{ .x = fp.Fp.one } } };
    const result = try gjk.intersectShapes(&pair, g.Vec3.unit_x, &status);
    try std.testing.expectEqual(gjk.Status.intersecting, result.status);
    try std.testing.expectEqual(@as(u8, 4), result.simplex_len);
    var vertices: [130]gjk.SupportVertex = undefined;
    var faces: [256]gjk.EpaFace = undefined;
    var visible: [256]bool = undefined;
    var horizon: [768]gjk.HorizonEdge = undefined;
    pair.failure = null;
    const penetration = gjk.epa(gjk.shapeSupportContext(&pair), result.simplex[0..result.simplex_len], .{ .vertices = &vertices, .faces = &faces, .visible = &visible, .horizon = &horizon }, &status);
    try std.testing.expect(pair.failure == null);
    try std.testing.expectEqual(gjk.EpaStatus.converged, penetration.status);
    try std.testing.expect(penetration.depth.raw > 0);
}

test "runtime convex manifold selects clipped box patch" {
    var status = fp.MathStatus{};
    var empty_assets: [0]asset_store.Asset = .{};
    const assets = asset_store.Store{ .assets = &empty_assets, .bytes = &.{}, .asset_set_hash = [_]u8{0} ** 32 };
    const box: shapes.Shape = .{ .box = .{ .half_extents = .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.one } } };
    var pair = gjk.ShapePairContext{ .shape_a = box, .shape_b = box, .assets = &assets, .transform_b = .{ .position = .{ .x = fp.Fp.one } } };
    var vertices: [130]gjk.SupportVertex = undefined;
    var faces: [256]gjk.EpaFace = undefined;
    var visible: [256]bool = undefined;
    var horizon: [768]gjk.HorizonEdge = undefined;
    var reference: [8]gjk.ClipVertex = undefined;
    var incident: [8]gjk.ClipVertex = undefined;
    var scratch_a: [16]gjk.ClipVertex = undefined;
    var scratch_b: [16]gjk.ClipVertex = undefined;
    var contacts: [16]gjk.ContactPoint = undefined;
    const result = try gjk.collideShapes(&pair, g.Vec3.unit_x, .{ .epa = .{ .vertices = &vertices, .faces = &faces, .visible = &visible, .horizon = &horizon }, .reference = &reference, .incident = &incident, .scratch_a = &scratch_a, .scratch_b = &scratch_b, .contacts = &contacts }, &status);
    try std.testing.expectEqual(gjk.Status.intersecting, result.gjk.status);
    try std.testing.expectEqual(gjk.EpaStatus.converged, result.epa.?.status);
    try std.testing.expectEqual(@as(u8, 4), result.patch.len);
}

test "smooth convex pair uses analytic witness contact" {
    var status = fp.MathStatus{};
    var empty_assets: [0]asset_store.Asset = .{};
    const assets = asset_store.Store{ .assets = &empty_assets, .bytes = &.{}, .asset_set_hash = [_]u8{0} ** 32 };
    const sphere: shapes.Shape = .{ .sphere = .{ .radius = fp.Fp.one } };
    var pair = gjk.ShapePairContext{ .shape_a = sphere, .shape_b = sphere, .assets = &assets, .transform_b = .{ .position = .{ .x = fp.Fp.one } } };
    var vertices: [130]gjk.SupportVertex = undefined;
    var faces: [256]gjk.EpaFace = undefined;
    var visible: [256]bool = undefined;
    var horizon: [768]gjk.HorizonEdge = undefined;
    var reference: [8]gjk.ClipVertex = undefined;
    var incident: [8]gjk.ClipVertex = undefined;
    var scratch_a: [16]gjk.ClipVertex = undefined;
    var scratch_b: [16]gjk.ClipVertex = undefined;
    var contacts: [16]gjk.ContactPoint = undefined;
    const result = try gjk.collideShapes(&pair, g.Vec3.unit_x, .{ .epa = .{ .vertices = &vertices, .faces = &faces, .visible = &visible, .horizon = &horizon }, .reference = &reference, .incident = &incident, .scratch_a = &scratch_a, .scratch_b = &scratch_b, .contacts = &contacts }, &status);
    try std.testing.expectEqual(gjk.Status.intersecting, result.gjk.status);
    try std.testing.expect(result.epa == null);
    try std.testing.expectEqual(@as(u8, 1), result.patch.len);
    try std.testing.expect(result.patch.points[0].separation.raw < 0);
}

test "convex hull reference face follows baked half-edge winding" {
    const baked = gravity.geometry.baked;
    var status = fp.MathStatus{};
    const points = [_]g.Vec3{ .{}, .{ .x = fp.Fp.one }, .{ .y = fp.Fp.one }, .{ .z = fp.Fp.one } };
    var vertices: [4]g.Vec3 = undefined;
    var triangles: [4]baked.Triangle = undefined;
    var faces: [4]baked.HullFace = undefined;
    var edges: [12]baked.HalfEdge = undefined;
    const hull = try baked.buildConvexHull(&points, &vertices, &triangles, &faces, &edges, &status);
    var bytes: [2048]u8 = undefined;
    var scratch: [1024]u8 = undefined;
    const encoded = try baked.encodeConvexHull(hull, 42, &bytes, &scratch);
    const input = [_][]const u8{encoded.bytes};
    var memory: [4096]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&memory, &input);
    var output: [8]gjk.ClipVertex = undefined;
    const face = try gjk.referenceFace(.{ .convex_hull = .{ .source_id = 42 } }, &assets, .{}, g.Vec3.unit_x, &output, &status);
    try std.testing.expectEqual(@as(usize, 3), face.vertices.len);
    try std.testing.expect(face.normal.dot(g.Vec3.unit_x, &status).raw > 0);
}

test "compound traversal selects the intersecting child rather than its convex hull" {
    const baked = gravity.geometry.baked;
    const hash = gravity.state.hash;
    var status = fp.MathStatus{};
    const points = [_]g.Vec3{ .{}, .{ .x = fp.Fp.one }, .{ .y = fp.Fp.one }, .{ .z = fp.Fp.one } };
    var vertices: [4]g.Vec3 = undefined;
    var triangles: [4]baked.Triangle = undefined;
    var hull_faces: [4]baked.HullFace = undefined;
    var edges: [12]baked.HalfEdge = undefined;
    const hull = try baked.buildConvexHull(&points, &vertices, &triangles, &hull_faces, &edges, &status);
    var hull_bytes: [2048]u8 = undefined;
    var hull_scratch: [1024]u8 = undefined;
    const encoded_hull = try baked.encodeConvexHull(hull, 3, &hull_bytes, &hull_scratch);
    const hull_hash = hash.oneShot256(.asset, encoded_hull.bytes);
    const children = [_]baked.CompoundChild{ .{ .ordinal = 0, .content_hash = hull_hash, .translation = .{}, .rotation = g.Quat.identity }, .{ .ordinal = 1, .content_hash = hull_hash, .translation = .{ .x = fp.Fp.fromInt(2) }, .rotation = g.Quat.identity } };
    const nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = g.Vec3.zero, .max = .{ .x = fp.Fp.fromInt(3), .y = fp.Fp.one, .z = fp.Fp.one } }, 0, 2)};
    var compound_bytes: [1024]u8 = undefined;
    var compound_scratch: [512]u8 = undefined;
    const encoded_compound = try baked.encodeCompound(.{ .source_id = 7, .children = &children, .nodes = &nodes }, &compound_bytes, &compound_scratch);
    const inputs = [_][]const u8{ encoded_hull.bytes, encoded_compound.bytes };
    var memory: [4096]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&memory, &inputs);
    const box: shapes.Shape = .{ .box = .{ .half_extents = .{ .x = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status), .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status), .z = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status) } } };
    var pair = gjk.ShapePairContext{ .shape_a = .{ .compound = .{ .source_id = 7 } }, .shape_b = box, .assets = &assets, .transform_b = .{ .position = .{ .x = fp.Fp.fromInt(9).div(fp.Fp.fromInt(4), &status), .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status), .z = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status) } } };
    var epa_vertices: [130]gjk.SupportVertex = undefined;
    var epa_faces: [256]gjk.EpaFace = undefined;
    var visible: [256]bool = undefined;
    var horizon: [768]gjk.HorizonEdge = undefined;
    var reference: [8]gjk.ClipVertex = undefined;
    var incident: [8]gjk.ClipVertex = undefined;
    var scratch_a: [16]gjk.ClipVertex = undefined;
    var scratch_b: [16]gjk.ClipVertex = undefined;
    var contacts: [16]gjk.ContactPoint = undefined;
    const result = try gjk.collideShapes(&pair, g.Vec3.unit_x, .{ .epa = .{ .vertices = &epa_vertices, .faces = &epa_faces, .visible = &visible, .horizon = &horizon }, .reference = &reference, .incident = &incident, .scratch_a = &scratch_a, .scratch_b = &scratch_b, .contacts = &contacts }, &status);
    try std.testing.expectEqual(gjk.Status.intersecting, result.gjk.status);
    try std.testing.expectEqual(@as(u8, 1), result.path_a.len);
    try std.testing.expectEqual(@as(u32, 1), result.path_a.values[0]);
    pair.transform_b.position.x = fp.Fp.fromInt(3).div(fp.Fp.fromInt(2), &status);
    const gap = try gjk.collideShapes(&pair, g.Vec3.unit_x, .{ .epa = .{ .vertices = &epa_vertices, .faces = &epa_faces, .visible = &visible, .horizon = &horizon }, .reference = &reference, .incident = &incident, .scratch_a = &scratch_a, .scratch_b = &scratch_b, .contacts = &contacts }, &status);
    try std.testing.expectEqual(gjk.Status.separated, gap.gjk.status);
    pair.shape_b = .{ .box = .{ .half_extents = .{ .x = fp.Fp.one, .y = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status), .z = fp.Fp.fromInt(1).div(fp.Fp.fromInt(4), &status) } } };
    const combined = try gjk.collideShapes(&pair, g.Vec3.unit_x, .{ .epa = .{ .vertices = &epa_vertices, .faces = &epa_faces, .visible = &visible, .horizon = &horizon }, .reference = &reference, .incident = &incident, .scratch_a = &scratch_a, .scratch_b = &scratch_b, .contacts = &contacts }, &status);
    try std.testing.expectEqual(gjk.Status.intersecting, combined.gjk.status);
    var saw_zero = false;
    var saw_one = false;
    for (combined.patch.points[0..combined.patch.len]) |point| if (point.path_a.len == 1) {
        saw_zero = saw_zero or point.path_a.values[0] == 0;
        saw_one = saw_one or point.path_a.values[0] == 1;
    };
    try std.testing.expect(saw_zero and saw_one);
}
