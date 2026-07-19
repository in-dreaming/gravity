//! Deterministic, caller-buffered mesh BVH traversal primitives for Task 11.
const std = @import("std");
const baked = @import("../geometry/baked.zig");
const runtime_view = @import("../assets/runtime_view.zig");
const analytic = @import("analytic.zig");
const gjk = @import("gjk.zig");
const fp = @import("../math/fp.zig");
const ids = @import("../core/ids.zig");
const shapes = @import("shapes.zig");
const store = @import("../assets/store.zig");

pub const Error = error{ CapacityExceeded, InvalidBvh, NoSurface };
pub const RuntimeError = runtime_view.Error || Error;
pub const NodePair = struct { a: u32, b: u32 };
pub const PrimitivePair = struct { a: u32, b: u32 };
pub const Triangle = struct { a: @import("../math/geometry.zig").Vec3, b: @import("../math/geometry.zig").Vec3, c: @import("../math/geometry.zig").Vec3 };
pub const HeightTriangle = struct { triangle: Triangle, cell: u32, half: u1, material_id: u32 };
pub const ContactError = error{CapacityExceeded};
pub const ConvexSurfaceError = RuntimeError || shapes.Error;
pub const ConvexTriangleContext = struct {
    shape: shapes.Shape,
    assets: *const store.Store,
    transform: @import("../math/geometry.zig").Transform3 = .{},
    triangle: Triangle,
    failure: ?shapes.Error = null,
};
/// Fixed storage for extracting a penetration witness from an intersecting
/// convex--triangle GJK simplex. It is intentionally separate from traversal
/// storage because EPA has a bounded, caller-visible capacity contract.
pub const ConvexTriangleWorkspace = struct { epa: gjk.EpaWorkspace };
pub const ConvexTriangleContact = struct {
    gjk: gjk.Result,
    epa: ?gjk.EpaResult = null,
    contact: ?gjk.ContactPoint = null,
};
pub const SphereMeshWorkspace = struct {
    nodes: []baked.BvhNode,
    primitives: []u32,
    work: []NodePair,
    pair_scratch: []PrimitivePair,
    pair_output: []PrimitivePair,
    contacts: []gjk.ContactPoint,
};
pub const SphereHeightfieldWorkspace = struct {
    tile_nodes: []baked.BvhNode,
    work: []NodePair,
    pair_scratch: []PrimitivePair,
    pair_output: []PrimitivePair,
    triangles: []HeightTriangle,
    contacts: []gjk.ContactPoint,
};
/// Caller-owned state for a runtime convex shape against one immutable mesh.
/// `intersections` receives canonical triangle IDs only after every candidate
/// has been classified, so a capacity fault cannot publish a partial result.
pub const ConvexMeshWorkspace = struct {
    nodes: []baked.BvhNode,
    primitives: []u32,
    work: []NodePair,
    pair_scratch: []PrimitivePair,
    pair_output: []PrimitivePair,
    intersections: []u32,
};
/// HeightField equivalent of `ConvexMeshWorkspace`. Feature IDs encode the
/// canonical `(cell << 1) | half` triangle identity.
pub const ConvexHeightfieldWorkspace = struct {
    tile_nodes: []baked.BvhNode,
    work: []NodePair,
    pair_scratch: []PrimitivePair,
    pair_output: []PrimitivePair,
    triangles: []HeightTriangle,
    intersections: []u32,
};
/// Caller-owned fixed storage for a closest convex--mesh surface witness.
/// `stack` is a deterministic BVH node stack, not an allocator-backed queue.
pub const ConvexMeshDistanceWorkspace = struct {
    nodes: []baked.BvhNode,
    primitives: []u32,
    stack: []u32,
};
/// HeightField counterpart of `ConvexMeshDistanceWorkspace`; expanded tile
/// triangles retain their canonical cell/half identity while being scored.
pub const ConvexHeightfieldDistanceWorkspace = struct {
    stack: []u32,
    triangles: []HeightTriangle,
};
/// Mesh patch storage for a runtime convex shape. The nested query workspace
/// owns BVH state; the nested triangle workspace owns bounded EPA state.
pub const ConvexMeshPatchWorkspace = struct {
    query: ConvexMeshWorkspace,
    triangle: ConvexTriangleWorkspace,
    contacts: []gjk.ContactPoint,
};
pub const ConvexHeightfieldPatchWorkspace = struct {
    query: ConvexHeightfieldWorkspace,
    triangle: ConvexTriangleWorkspace,
    contacts: []gjk.ContactPoint,
};
pub const ConvexContactError = ConvexSurfaceError;
/// Caller-owned storage for one discrete immutable mesh pair query. The
/// returned primitive pairs are ordered `(triangle_a, triangle_b)`.
pub const MeshMeshWorkspace = struct {
    nodes_a: []baked.BvhNode,
    primitives_a: []u32,
    nodes_b: []baked.BvhNode,
    primitives_b: []u32,
    work: []NodePair,
    pair_scratch: []PrimitivePair,
    pair_output: []PrimitivePair,
    overlaps: []PrimitivePair,
};
/// Mesh--Mesh contact storage. The nested query workspace retains the exact
/// ordered SAT overlap set; `contacts` is only published after all witnesses
/// have been generated and capacity checked.
pub const MeshMeshPatchWorkspace = struct {
    query: MeshMeshWorkspace,
    contacts: []gjk.ContactPoint,
};
/// Scratch for a Sphere against a Compound whose terminal leaves are mesh or
/// HeightField assets. `merged` must hold up to four points per leaf.
pub const SphereCompoundSurfaceWorkspace = struct {
    leaves: []shapes.CompoundLeaf,
    mesh: SphereMeshWorkspace,
    heightfield: SphereHeightfieldWorkspace,
    merged: []gjk.ContactPoint,
};
/// Caller-owned aggregation state for convex shapes against Compound surface
/// leaves. Terminal primitive children are intentionally dispatched by the
/// regular convex narrow phase rather than this mesh/HeightField-only path.
pub const ConvexCompoundSurfaceWorkspace = struct {
    leaves: []shapes.CompoundLeaf,
    mesh: ConvexMeshPatchWorkspace,
    heightfield: ConvexHeightfieldPatchWorkspace,
    merged: []gjk.ContactPoint,
};
pub const CompoundError = RuntimeError || shapes.Error || ContactError;
pub const ConvexCompoundError = ConvexContactError || shapes.Error;

/// Exact 11-axis triangle SAT. It preserves coplanar overlap because the
/// underlying fixed-width projections treat shared boundaries as contacts.
pub fn trianglesOverlap(a: Triangle, b: Triangle) bool {
    const vertices = [_]@import("../math/geometry.zig").Vec3{ a.a, a.b, a.c, b.a, b.b, b.c };
    return baked.trianglesOverlapSat(&vertices, .{ .a = 0, .b = 1, .c = 2 }, .{ .a = 3, .b = 4, .c = 5 });
}

/// Deterministic zero-distance witness for two intersecting triangles. The
/// candidate order is vertex--face A/B followed by the nine edge pairs; this
/// covers containment, coplanar crossings and skew edge intersections without
/// relying on floating-point clipping.
pub fn triangleTriangleContact(a: Triangle, b: Triangle, feature_a: u32, feature_b: u32, status: *fp.MathStatus) ?gjk.ContactPoint {
    if (!trianglesOverlap(a, b)) return null;
    var best_a = a.a;
    var best_b = analytic.closestPointTriangle(a.a, .{ .a = b.a, .b = b.b, .c = b.c }, status);
    var best_d2 = best_a.sub(best_b, status).lengthSquared(status);
    const Consider = struct {
        fn candidate(best_a_: *@import("../math/geometry.zig").Vec3, best_b_: *@import("../math/geometry.zig").Vec3, best_d2_: *fp.Fp, candidate_a: @import("../math/geometry.zig").Vec3, candidate_b: @import("../math/geometry.zig").Vec3, status_: *fp.MathStatus) void {
            const distance = candidate_a.sub(candidate_b, status_).lengthSquared(status_);
            if (distance.raw < best_d2_.*.raw) {
                best_a_.* = candidate_a;
                best_b_.* = candidate_b;
                best_d2_.* = distance;
            }
        }
    };
    for ([_]@import("../math/geometry.zig").Vec3{ a.b, a.c }) |vertex| Consider.candidate(&best_a, &best_b, &best_d2, vertex, analytic.closestPointTriangle(vertex, .{ .a = b.a, .b = b.b, .c = b.c }, status), status);
    for ([_]@import("../math/geometry.zig").Vec3{ b.a, b.b, b.c }) |vertex| Consider.candidate(&best_a, &best_b, &best_d2, analytic.closestPointTriangle(vertex, .{ .a = a.a, .b = a.b, .c = a.c }, status), vertex, status);
    const edges_a = [_]analytic.Segment{ .{ .a = a.a, .b = a.b }, .{ .a = a.b, .b = a.c }, .{ .a = a.c, .b = a.a } };
    const edges_b = [_]analytic.Segment{ .{ .a = b.a, .b = b.b }, .{ .a = b.b, .b = b.c }, .{ .a = b.c, .b = b.a } };
    for (edges_a) |edge_a| for (edges_b) |edge_b| {
        const closest = analytic.closestSegments(edge_a, edge_b, status);
        Consider.candidate(&best_a, &best_b, &best_d2, closest.a, closest.b, status);
    };
    const center_a = a.a.add(a.b, status).add(a.c, status).scale(fp.Fp.fromRatio(1, 3, status), status);
    const center_b = b.a.add(b.b, status).add(b.c, status).scale(fp.Fp.fromRatio(1, 3, status), status);
    const face = a.b.sub(a.a, status).cross(a.c.sub(a.a, status), status).normalize(status);
    var normal = if (face.valid) face.value else @import("../math/geometry.zig").Vec3.unit_x;
    if (center_b.sub(center_a, status).dot(normal, status).raw < 0) normal = normal.scale(fp.Fp.fromInt(-1), status);
    return .{ .point_a = best_a, .point_b = best_b, .normal = normal, .separation = if (best_d2.raw == 0) fp.Fp.zero else best_d2.sqrt(status).neg(status), .feature_a = feature_a, .feature_b = feature_b };
}

/// GJK classification for any runtime convex shape against one world-space
/// triangle. The triangle support tie-break is vertex order, making the
/// adapter deterministic and reusable by mesh and HeightField traversal.
pub fn convexTriangleIntersect(context: *ConvexTriangleContext, initial_direction: @import("../math/geometry.zig").Vec3, status: *fp.MathStatus) shapes.Error!gjk.Result {
    context.failure = null;
    const support: gjk.SupportContext = .{ .ptr = context, .call = convexTriangleSupport };
    const result = gjk.intersect(support, initial_direction, status);
    if (context.failure) |err| return err;
    return result;
}

/// Fixed-iteration GJK distance between a runtime convex shape and one
/// world-space triangle. The witness pair is suitable for conservative
/// advancement; a non-convergent GJK status remains explicit to the caller.
/// This shares the exact support and feature ordering used by the discrete
/// convex--triangle classifier, so CCD cannot disagree with overlap solely
/// because it chose a different surface representation.
pub fn convexTriangleDistance(context: *ConvexTriangleContext, initial_direction: @import("../math/geometry.zig").Vec3, status: *fp.MathStatus) shapes.Error!gjk.Result {
    context.failure = null;
    const support: gjk.SupportContext = .{ .ptr = context, .call = convexTriangleSupport };
    const result = gjk.distance(support, initial_direction, status);
    if (context.failure) |err| return err;
    return result;
}

/// Produces one signed penetration witness for an intersecting convex--
/// triangle pair. EPA supplies the exact polytope witness when the GJK
/// simplex is tetrahedral; coplanar and touching simplices use the canonical
/// triangle closest point plus convex support in that direction.
pub fn convexTriangleContact(context: *ConvexTriangleContext, initial_direction: @import("../math/geometry.zig").Vec3, workspace: ConvexTriangleWorkspace, status: *fp.MathStatus) shapes.Error!ConvexTriangleContact {
    const intersection = try convexTriangleIntersect(context, initial_direction, status);
    var result: ConvexTriangleContact = .{ .gjk = intersection };
    if (intersection.status != .intersecting) return result;
    if (intersection.simplex_len != 4) {
        result.contact = try convexTriangleFallbackContact(context, status);
        return result;
    }
    const support: gjk.SupportContext = .{ .ptr = context, .call = convexTriangleSupport };
    const epa_result = gjk.epa(support, intersection.simplex[0..intersection.simplex_len], workspace.epa, status);
    result.epa = epa_result;
    result.contact = if (epa_result.status == .converged) .{
        .point_a = epa_result.witness_a,
        .point_b = epa_result.witness_b,
        .normal = epa_result.normal,
        .separation = epa_result.depth.neg(status),
        .feature_a = epa_result.feature_a,
        .feature_b = epa_result.feature_b,
    } else try convexTriangleFallbackContact(context, status);
    return result;
}

fn convexTriangleFallbackContact(context: *ConvexTriangleContext, status: *fp.MathStatus) shapes.Error!gjk.ContactPoint {
    const triangle: analytic.Triangle = .{ .a = context.triangle.a, .b = context.triangle.b, .c = context.triangle.c };
    const point_b = analytic.closestPointTriangle(context.transform.position, triangle, status);
    var direction = point_b.sub(context.transform.position, status);
    if (!direction.normalize(status).valid) {
        const normal = context.triangle.b.sub(context.triangle.a, status).cross(context.triangle.c.sub(context.triangle.a, status), status).normalize(status);
        direction = if (normal.valid) normal.value else @import("../math/geometry.zig").Vec3.unit_x;
    }
    const unit = direction.normalize(status);
    const support_a = try shapes.support(context.shape, context.assets, context.transform.orientation.inverseRotate(unit.value, status), status);
    const point_a = context.transform.apply(support_a.point, status);
    const signed = point_b.sub(point_a, status).dot(unit.value, status);
    return .{
        .point_a = point_a,
        .point_b = point_b,
        .normal = unit.value,
        .separation = if (signed.raw > 0) fp.Fp.zero else signed,
        .feature_a = shapeFeatureKey(support_a.feature),
        .feature_b = 0,
    };
}

fn shapeFeatureKey(feature: shapes.Feature) u32 {
    return switch (feature) {
        .vertex => |value| value,
        .edge => |value| value | 0x4000_0000,
        .face => |value| value | 0x8000_0000,
        .primitive => |value| value | 0xc000_0000,
    };
}

fn convexTriangleSupport(raw: *const anyopaque, direction: @import("../math/geometry.zig").Vec3, status: *fp.MathStatus) gjk.SupportVertex {
    const context: *ConvexTriangleContext = @ptrCast(@alignCast(@constCast(raw)));
    const local = context.transform.orientation.inverseRotate(direction, status);
    const support_a = shapes.support(context.shape, context.assets, local, status) catch |err| {
        context.failure = err;
        return .{ .point = @import("../math/geometry.zig").Vec3.zero, .witness_a = @import("../math/geometry.zig").Vec3.zero, .witness_b = @import("../math/geometry.zig").Vec3.zero, .feature_a = 0, .feature_b = 0 };
    };
    const witness_a = context.transform.apply(support_a.point, status);
    const reverse = @import("../math/geometry.zig").Vec3{ .x = direction.x.neg(status), .y = direction.y.neg(status), .z = direction.z.neg(status) };
    var witness_b = context.triangle.a;
    var best = witness_b.dot(reverse, status);
    inline for ([_]@import("../math/geometry.zig").Vec3{ context.triangle.b, context.triangle.c }) |candidate| {
        const score = candidate.dot(reverse, status);
        if (score.raw > best.raw) {
            best = score;
            witness_b = candidate;
        }
    }
    return .{ .point = witness_a.sub(witness_b, status), .witness_a = witness_a, .witness_b = witness_b, .feature_a = switch (support_a.feature) {
        .vertex => |v| v,
        .edge => |v| v,
        .face => |v| v,
        .primitive => |v| v,
    }, .feature_b = if (witness_b.x.raw == context.triangle.a.x.raw and witness_b.y.raw == context.triangle.a.y.raw and witness_b.z.raw == context.triangle.a.z.raw) 0 else if (witness_b.x.raw == context.triangle.b.x.raw and witness_b.y.raw == context.triangle.b.y.raw and witness_b.z.raw == context.triangle.b.z.raw) 1 else 2 };
}

/// Copies zero-copy view accessors into caller-owned traversal buffers. It
/// validates the expected mesh kind and never allocates.
pub fn loadMeshBvh(view: runtime_view.View, nodes: []baked.BvhNode, primitives: []u32) RuntimeError!struct { nodes: []const baked.BvhNode, primitives: []const u32 } {
    if (view.header.kind != .triangle_mesh or view.nodeCount() == 0 or view.primitiveCount() != view.triangleCount()) return error.InvalidBvh;
    if (nodes.len < view.nodeCount() or primitives.len < view.primitiveCount()) return error.CapacityExceeded;
    for (nodes[0..view.nodeCount()], 0..) |*node, i| node.* = try view.node(i);
    for (primitives[0..view.primitiveCount()], 0..) |*primitive, i| primitive.* = try view.primitive(i);
    return .{ .nodes = nodes[0..view.nodeCount()], .primitives = primitives[0..view.primitiveCount()] };
}

/// Expands one canonical heightfield tile into its non-hole triangles. Output
/// order is `(cell row-major, half 0 then half 1)` and capacity is checked
/// before publication so callers never observe a truncated tile.
pub fn expandHeightTile(view: runtime_view.View, tile: u32, output: []HeightTriangle) RuntimeError![]const HeightTriangle {
    if (view.header.kind != .height_field) return error.InvalidBvh;
    const dimensions = try view.heightDimensions();
    const cells_x = dimensions.width - 1;
    const cells_z = dimensions.height - 1;
    const tiles_x = (cells_x + baked.heightfield_tile_axis - 1) / baked.heightfield_tile_axis;
    const tiles_z = (cells_z + baked.heightfield_tile_axis - 1) / baked.heightfield_tile_axis;
    if (tile >= @as(u64, tiles_x) * tiles_z) return error.InvalidBvh;
    const x0 = (tile % tiles_x) * baked.heightfield_tile_axis;
    const z0 = (tile / tiles_x) * baked.heightfield_tile_axis;
    const x1 = @min(x0 + baked.heightfield_tile_axis, cells_x);
    const z1 = @min(z0 + baked.heightfield_tile_axis, cells_z);
    var needed: usize = 0;
    var z = z0;
    while (z < z1) : (z += 1) {
        var x = x0;
        while (x < x1) : (x += 1) {
            if (!(try view.heightCell(@as(usize, z) * cells_x + x)).hole) needed += 2;
        }
    }
    if (needed > output.len) return error.CapacityExceeded;
    var count: usize = 0;
    z = z0;
    while (z < z1) : (z += 1) {
        var x = x0;
        while (x < x1) : (x += 1) {
            const cell_index: usize = @as(usize, z) * cells_x + x;
            const cell = try view.heightCell(cell_index);
            if (cell.hole) continue;
            const p00 = try heightPoint(view, dimensions.width, x, z);
            const p10 = try heightPoint(view, dimensions.width, x + 1, z);
            const p01 = try heightPoint(view, dimensions.width, x, z + 1);
            const p11 = try heightPoint(view, dimensions.width, x + 1, z + 1);
            output[count] = .{ .triangle = .{ .a = p00, .b = p10, .c = p01 }, .cell = @intCast(cell_index), .half = 0, .material_id = cell.material_id };
            output[count + 1] = .{ .triangle = .{ .a = p10, .b = p11, .c = p01 }, .cell = @intCast(cell_index), .half = 1, .material_id = cell.material_id };
            count += 2;
        }
    }
    return output[0..count];
}
/// Converts overlapping Sphere–Triangle primitive contacts into a stable mesh
/// patch. `scratch` is caller-owned and may be larger than four; output is
/// reduced only after every accepted triangle has been evaluated.
pub fn sphereTrianglePatch(sphere: analytic.Sphere, triangles: []const Triangle, triangle_ids: []const u32, scratch: []gjk.ContactPoint, status: *fp.MathStatus) ContactError!gjk.ContactPatch {
    if (triangles.len != triangle_ids.len) return error.CapacityExceeded;
    if (triangles.len > scratch.len) return error.CapacityExceeded;
    var count: usize = 0;
    for (triangles, triangle_ids) |triangle, triangle_id| {
        const result = analytic.sphereTriangle(sphere, .{ .a = triangle.a, .b = triangle.b, .c = triangle.c }, triangle_id, @import("../math/geometry.zig").Vec3.zero, ids.ColliderId.init(0, 0), ids.ColliderId.init(1, 0), status);
        if (result.separation.raw > 0) continue;
        scratch[count] = .{ .point_a = result.witness_a, .point_b = result.witness_b, .separation = result.separation, .feature_a = 0, .feature_b = triangle_id };
        count += 1;
    }
    return gjk.reducePatch(scratch[0..count], status);
}

/// BVH-backed Sphere–Mesh query. Candidate generation is conservative by the
/// sphere AABB; only ordered leaf candidates reach Sphere–Triangle contact.
pub fn sphereMeshPatch(view: runtime_view.View, sphere: analytic.Sphere, workspace: SphereMeshWorkspace, status: *fp.MathStatus) RuntimeError!gjk.ContactPatch {
    return sphereMeshPatchTransformed(view, .{}, sphere, workspace, status);
}

/// Sphere against an immutable mesh carried by a body transform. The mesh is
/// never modified: only caller-owned traversal bounds and local vertices are
/// transformed for this query.
pub fn sphereMeshPatchTransformed(view: runtime_view.View, mesh_transform: @import("../math/geometry.zig").Transform3, sphere: analytic.Sphere, workspace: SphereMeshWorkspace, status: *fp.MathStatus) RuntimeError!gjk.ContactPatch {
    const mesh_bvh = try loadMeshBvh(view, workspace.nodes, workspace.primitives);
    for (workspace.nodes[0..mesh_bvh.nodes.len]) |*node| node.bounds = transformBounds(node.bounds, mesh_transform, status);
    var query_primitives = [_]u32{0};
    const radius = sphere.radius;
    const query_bounds = @import("../math/geometry.zig").Aabb3{ .min = .{ .x = sphere.center.x.sub(radius, status), .y = sphere.center.y.sub(radius, status), .z = sphere.center.z.sub(radius, status) }, .max = .{ .x = sphere.center.x.add(radius, status), .y = sphere.center.y.add(radius, status), .z = sphere.center.z.add(radius, status) } };
    const query_nodes = [_]baked.BvhNode{baked.BvhNode.leaf(query_bounds, 0, 1)};
    const pairs = try traverseBvhPairs(&query_nodes, &query_primitives, mesh_bvh.nodes, mesh_bvh.primitives, workspace.work, workspace.pair_scratch, workspace.pair_output);
    if (pairs.len > workspace.contacts.len) return error.CapacityExceeded;
    var count: usize = 0;
    for (pairs) |pair| {
        const index = pair.b;
        const source = try view.triangle(index);
        const triangle = transformedTriangle(view, mesh_transform, source, status) catch |err| return err;
        const result = analytic.sphereTriangle(sphere, .{ .a = triangle.a, .b = triangle.b, .c = triangle.c }, index, @import("../math/geometry.zig").Vec3.zero, ids.ColliderId.init(0, 0), ids.ColliderId.init(1, 0), status);
        if (result.separation.raw > 0) continue;
        workspace.contacts[count] = .{ .point_a = result.witness_a, .point_b = result.witness_b, .normal = result.normal, .separation = result.separation, .feature_a = 0, .feature_b = index };
        count += 1;
    }
    count = try suppressInternalEdgeContacts(view, workspace.contacts[0..count], status);
    return gjk.reducePatch(workspace.contacts[0..count], status);
}

/// BVH-backed Capsule–Mesh query using the exact Capsule–Triangle primitive.
pub fn capsuleMeshPatch(view: runtime_view.View, capsule: analytic.Capsule, workspace: SphereMeshWorkspace, status: *fp.MathStatus) RuntimeError!gjk.ContactPatch {
    return capsuleMeshPatchTransformed(view, .{}, capsule, workspace, status);
}

/// Capsule counterpart of `sphereMeshPatchTransformed`.
pub fn capsuleMeshPatchTransformed(view: runtime_view.View, mesh_transform: @import("../math/geometry.zig").Transform3, capsule: analytic.Capsule, workspace: SphereMeshWorkspace, status: *fp.MathStatus) RuntimeError!gjk.ContactPatch {
    const mesh_bvh = try loadMeshBvh(view, workspace.nodes, workspace.primitives);
    for (workspace.nodes[0..mesh_bvh.nodes.len]) |*node| node.bounds = transformBounds(node.bounds, mesh_transform, status);
    var query_primitives = [_]u32{0};
    const radius = capsule.radius;
    const min_x = fp.Fp{ .raw = @min(capsule.segment.a.x.raw, capsule.segment.b.x.raw) };
    const min_y = fp.Fp{ .raw = @min(capsule.segment.a.y.raw, capsule.segment.b.y.raw) };
    const min_z = fp.Fp{ .raw = @min(capsule.segment.a.z.raw, capsule.segment.b.z.raw) };
    const max_x = fp.Fp{ .raw = @max(capsule.segment.a.x.raw, capsule.segment.b.x.raw) };
    const max_y = fp.Fp{ .raw = @max(capsule.segment.a.y.raw, capsule.segment.b.y.raw) };
    const max_z = fp.Fp{ .raw = @max(capsule.segment.a.z.raw, capsule.segment.b.z.raw) };
    const query_bounds = @import("../math/geometry.zig").Aabb3{ .min = .{ .x = min_x.sub(radius, status), .y = min_y.sub(radius, status), .z = min_z.sub(radius, status) }, .max = .{ .x = max_x.add(radius, status), .y = max_y.add(radius, status), .z = max_z.add(radius, status) } };
    const query_nodes = [_]baked.BvhNode{baked.BvhNode.leaf(query_bounds, 0, 1)};
    const pairs = try traverseBvhPairs(&query_nodes, &query_primitives, mesh_bvh.nodes, mesh_bvh.primitives, workspace.work, workspace.pair_scratch, workspace.pair_output);
    if (pairs.len > workspace.contacts.len) return error.CapacityExceeded;
    var count: usize = 0;
    for (pairs) |pair| {
        const index = pair.b;
        const source = try view.triangle(index);
        const triangle = transformedTriangle(view, mesh_transform, source, status) catch |err| return err;
        const result = analytic.capsuleTriangle(capsule, .{ .a = triangle.a, .b = triangle.b, .c = triangle.c }, index, @import("../math/geometry.zig").Vec3.zero, ids.ColliderId.init(0, 0), ids.ColliderId.init(1, 0), status);
        if (result.separation.raw > 0) continue;
        workspace.contacts[count] = .{ .point_a = result.witness_a, .point_b = result.witness_b, .normal = result.normal, .separation = result.separation, .feature_a = 0, .feature_b = index };
        count += 1;
    }
    count = try suppressInternalEdgeContacts(view, workspace.contacts[0..count], status);
    return gjk.reducePatch(workspace.contacts[0..count], status);
}

/// Exact runtime convex--mesh dispatch. The broad phase uses the shape's
/// world AABB, while each returned triangle has passed GJK against the actual
/// shape support function. Results are sorted canonical mesh triangle IDs.
pub fn convexMeshIntersections(shape: shapes.Shape, assets: *const store.Store, convex_transform: @import("../math/geometry.zig").Transform3, view: runtime_view.View, mesh_transform: @import("../math/geometry.zig").Transform3, workspace: ConvexMeshWorkspace, status: *fp.MathStatus) ConvexSurfaceError![]const u32 {
    const mesh_bvh = try loadMeshBvh(view, workspace.nodes, workspace.primitives);
    for (workspace.nodes[0..mesh_bvh.nodes.len]) |*node| node.bounds = transformBounds(node.bounds, mesh_transform, status);
    const query_bounds = try shapes.worldAabb(shape, assets, convex_transform, status);
    const query_nodes = [_]baked.BvhNode{baked.BvhNode.leaf(query_bounds, 0, 1)};
    const query_primitives = [_]u32{0};
    const candidates = try traverseBvhPairs(&query_nodes, &query_primitives, mesh_bvh.nodes, mesh_bvh.primitives, workspace.work, workspace.pair_scratch, workspace.pair_output);
    var required: usize = 0;
    for (candidates) |candidate| {
        if (try convexMeshTriangleOverlaps(shape, assets, convex_transform, view, mesh_transform, candidate.b, status)) required += 1;
    }
    if (required > workspace.intersections.len) return error.CapacityExceeded;
    var count: usize = 0;
    for (candidates) |candidate| {
        if (try convexMeshTriangleOverlaps(shape, assets, convex_transform, view, mesh_transform, candidate.b, status)) {
            workspace.intersections[count] = candidate.b;
            count += 1;
        }
    }
    return workspace.intersections[0..count];
}

/// Finds the exact nearest convex--mesh triangle witness by walking the
/// immutable BVH in canonical node order and evaluating every reached leaf.
/// It is intentionally a distance query rather than an AABB approximation;
/// a non-convergent triangle classification is returned unchanged.
pub fn convexMeshDistance(shape: shapes.Shape, assets: *const store.Store, convex_transform: @import("../math/geometry.zig").Transform3, view: runtime_view.View, mesh_transform: @import("../math/geometry.zig").Transform3, workspace: ConvexMeshDistanceWorkspace, status: *fp.MathStatus) ConvexSurfaceError!gjk.Result {
    const mesh_bvh = try loadMeshBvh(view, workspace.nodes, workspace.primitives);
    if (workspace.stack.len == 0) return error.CapacityExceeded;
    var stack_len: usize = 1;
    workspace.stack[0] = 0;
    var best: ?gjk.Result = null;
    while (stack_len != 0) {
        stack_len -= 1;
        const node_index = workspace.stack[stack_len];
        if (node_index >= mesh_bvh.nodes.len) return error.InvalidBvh;
        const node = mesh_bvh.nodes[node_index];
        if ((node.flags & baked.BvhNode.leaf_flag) == 0) {
            if (node.count != 0 or node.first + 1 >= mesh_bvh.nodes.len) return error.InvalidBvh;
            try pushDistanceNode(workspace.stack, &stack_len, node.first + 1);
            try pushDistanceNode(workspace.stack, &stack_len, node.first);
            continue;
        }
        const end = @as(usize, node.first) + node.count;
        if (end > mesh_bvh.primitives.len) return error.InvalidBvh;
        for (mesh_bvh.primitives[node.first..end]) |primitive| {
            const source = try view.triangle(primitive);
            var context: ConvexTriangleContext = .{ .shape = shape, .assets = assets, .transform = convex_transform, .triangle = try transformedTriangle(view, mesh_transform, source, status) };
            var candidate = try convexTriangleDistance(&context, context.triangle.a.sub(convex_transform.position, status), status);
            if (candidate.status == .non_convergent) return candidate;
            candidate.feature_b = primitive;
            if (best == null or distanceResultLess(candidate, best.?)) best = candidate;
        }
    }
    return best orelse error.NoSurface;
}

/// Finds the exact nearest convex--HeightField triangle witness. Hole cells
/// are omitted by `expandHeightTile`, and all returned features use the
/// canonical `(cell << 1) | half` identity.
pub fn convexHeightfieldDistance(shape: shapes.Shape, assets: *const store.Store, convex_transform: @import("../math/geometry.zig").Transform3, view: runtime_view.View, heightfield_transform: @import("../math/geometry.zig").Transform3, workspace: ConvexHeightfieldDistanceWorkspace, status: *fp.MathStatus) ConvexSurfaceError!gjk.Result {
    if (view.header.kind != .height_field or view.heightTileNodeCount() == 0 or workspace.stack.len == 0) return error.InvalidBvh;
    var stack_len: usize = 1;
    workspace.stack[0] = 0;
    var best: ?gjk.Result = null;
    while (stack_len != 0) {
        stack_len -= 1;
        const node_index = workspace.stack[stack_len];
        if (node_index >= view.heightTileNodeCount()) return error.InvalidBvh;
        const node = try view.heightTileNode(node_index);
        if ((node.flags & baked.BvhNode.leaf_flag) == 0) {
            if (node.count != 0 or node.first + 1 >= view.heightTileNodeCount()) return error.InvalidBvh;
            try pushDistanceNode(workspace.stack, &stack_len, node.first + 1);
            try pushDistanceNode(workspace.stack, &stack_len, node.first);
            continue;
        }
        const end = @as(usize, node.first) + node.count;
        for (node.first..end) |tile| {
            const triangles = try expandHeightTile(view, @intCast(tile), workspace.triangles);
            for (triangles) |height_triangle| {
                var context: ConvexTriangleContext = .{ .shape = shape, .assets = assets, .transform = convex_transform, .triangle = transformTriangle(height_triangle.triangle, heightfield_transform, status) };
                var candidate = try convexTriangleDistance(&context, context.triangle.a.sub(convex_transform.position, status), status);
                if (candidate.status == .non_convergent) return candidate;
                candidate.feature_b = (height_triangle.cell << 1) | height_triangle.half;
                if (best == null or distanceResultLess(candidate, best.?)) best = candidate;
            }
        }
    }
    return best orelse error.NoSurface;
}

/// Reduces convex--triangle witnesses into the normal four-point patch
/// representation. Lower-dimensional simplices use the deterministic
/// fallback supplied by `convexTriangleContact`.
pub fn convexMeshPatch(shape: shapes.Shape, assets: *const store.Store, convex_transform: @import("../math/geometry.zig").Transform3, view: runtime_view.View, mesh_transform: @import("../math/geometry.zig").Transform3, workspace: ConvexMeshPatchWorkspace, status: *fp.MathStatus) ConvexContactError!gjk.ContactPatch {
    const hits = try convexMeshIntersections(shape, assets, convex_transform, view, mesh_transform, workspace.query, status);
    if (hits.len > workspace.contacts.len) return error.CapacityExceeded;
    var count: usize = 0;
    for (hits) |triangle_id| {
        const source = try view.triangle(triangle_id);
        const triangle = try transformedTriangle(view, mesh_transform, source, status);
        var context: ConvexTriangleContext = .{ .shape = shape, .assets = assets, .transform = convex_transform, .triangle = triangle };
        const result = try convexTriangleContact(&context, triangle.a.sub(convex_transform.position, status), workspace.triangle, status);
        var contact = result.contact orelse unreachable;
        contact.feature_b = triangle_id;
        workspace.contacts[count] = contact;
        count += 1;
    }
    count = try suppressInternalEdgeContacts(view, workspace.contacts[0..count], status);
    return gjk.reducePatch(workspace.contacts[0..count], status);
}

fn convexMeshTriangleOverlaps(shape: shapes.Shape, assets: *const store.Store, convex_transform: @import("../math/geometry.zig").Transform3, view: runtime_view.View, mesh_transform: @import("../math/geometry.zig").Transform3, triangle_id: u32, status: *fp.MathStatus) ConvexSurfaceError!bool {
    const source = try view.triangle(triangle_id);
    const triangle = try transformedTriangle(view, mesh_transform, source, status);
    var context: ConvexTriangleContext = .{ .shape = shape, .assets = assets, .transform = convex_transform, .triangle = triangle };
    const seed = triangle.a.sub(convex_transform.position, status);
    return (try convexTriangleIntersect(&context, seed, status)).status == .intersecting;
}

fn transformedTriangle(view: runtime_view.View, transform: @import("../math/geometry.zig").Transform3, source: baked.Triangle, status: *fp.MathStatus) runtime_view.Error!Triangle {
    return .{ .a = transform.apply(try view.vertex(source.a), status), .b = transform.apply(try view.vertex(source.b), status), .c = transform.apply(try view.vertex(source.c), status) };
}
fn transformTriangle(source: Triangle, transform: @import("../math/geometry.zig").Transform3, status: *fp.MathStatus) Triangle {
    return .{ .a = transform.apply(source.a, status), .b = transform.apply(source.b, status), .c = transform.apply(source.c, status) };
}
fn pushDistanceNode(stack: []u32, len: *usize, value: u32) Error!void {
    if (len.* == stack.len) return error.CapacityExceeded;
    stack[len.*] = value;
    len.* += 1;
}
fn distanceResultLess(a: gjk.Result, b: gjk.Result) bool {
    if (a.distance.raw != b.distance.raw) return a.distance.raw < b.distance.raw;
    if (a.feature_b != b.feature_b) return a.feature_b < b.feature_b;
    return a.feature_a < b.feature_a;
}

/// Discrete mesh–mesh collision: traverse the two immutable baked BVHs, then
/// run exact triangle SAT on every ordered candidate. The result is committed
/// only after its required capacity is known, so a capacity error leaves
/// `workspace.overlaps` untouched. Body transforms belong to the caller;
/// callers pass views whose vertices are already in the common query frame.
pub fn meshMeshOverlaps(view_a: runtime_view.View, view_b: runtime_view.View, workspace: MeshMeshWorkspace) RuntimeError![]const PrimitivePair {
    var status = fp.MathStatus{};
    return meshMeshOverlapsTransformed(view_a, .{}, view_b, .{}, workspace, &status);
}

/// Dynamic-safe mesh–mesh query. The baked topology remains immutable while
/// each body's transform is applied to BVH bounds and triangle vertices for
/// this query. All scratch storage is supplied by the caller.
pub fn meshMeshOverlapsTransformed(view_a: runtime_view.View, transform_a: @import("../math/geometry.zig").Transform3, view_b: runtime_view.View, transform_b: @import("../math/geometry.zig").Transform3, workspace: MeshMeshWorkspace, status: *fp.MathStatus) RuntimeError![]const PrimitivePair {
    const candidates = try meshMeshCandidatesTransformed(view_a, transform_a, view_b, transform_b, workspace, status);
    var required: usize = 0;
    for (candidates) |candidate| {
        if (try meshTrianglePairOverlaps(view_a, transform_a, view_b, transform_b, candidate, status)) required += 1;
    }
    if (required > workspace.overlaps.len) return error.CapacityExceeded;
    var count: usize = 0;
    for (candidates) |candidate| {
        if (try meshTrianglePairOverlaps(view_a, transform_a, view_b, transform_b, candidate, status)) {
            workspace.overlaps[count] = candidate;
            count += 1;
        }
    }
    return workspace.overlaps[0..count];
}

/// Produces the canonical BVH leaf-pair work list without classifying any
/// triangle pair.  Task 23 uses this serial ownership freeze before fixed-slot
/// parallel classification and stable compaction.
pub fn meshMeshCandidatesTransformed(view_a: runtime_view.View, transform_a: @import("../math/geometry.zig").Transform3, view_b: runtime_view.View, transform_b: @import("../math/geometry.zig").Transform3, workspace: MeshMeshWorkspace, status: *fp.MathStatus) RuntimeError![]const PrimitivePair {
    const mesh_a = try loadMeshBvh(view_a, workspace.nodes_a, workspace.primitives_a);
    const mesh_b = try loadMeshBvh(view_b, workspace.nodes_b, workspace.primitives_b);
    for (workspace.nodes_a[0..mesh_a.nodes.len]) |*node| node.bounds = transformBounds(node.bounds, transform_a, status);
    for (workspace.nodes_b[0..mesh_b.nodes.len]) |*node| node.bounds = transformBounds(node.bounds, transform_b, status);
    return traverseBvhPairs(mesh_a.nodes, mesh_a.primitives, mesh_b.nodes, mesh_b.primitives, workspace.work, workspace.pair_scratch, workspace.pair_output);
}

/// Exact one-candidate classifier for fixed-slot Task 23 jobs.
pub fn meshTrianglePairOverlaps(view_a: runtime_view.View, transform_a: @import("../math/geometry.zig").Transform3, view_b: runtime_view.View, transform_b: @import("../math/geometry.zig").Transform3, pair: PrimitivePair, status: *fp.MathStatus) runtime_view.Error!bool {
    return meshTrianglesOverlap(view_a, transform_a, pair.a, view_b, transform_b, pair.b, status);
}

/// Generates the deterministic contact owned by one already-classified pair.
pub fn meshTrianglePairContact(view_a: runtime_view.View, transform_a: @import("../math/geometry.zig").Transform3, view_b: runtime_view.View, transform_b: @import("../math/geometry.zig").Transform3, pair: PrimitivePair, status: *fp.MathStatus) runtime_view.Error!gjk.ContactPoint {
    const source_a = try view_a.triangle(pair.a);
    const source_b = try view_b.triangle(pair.b);
    const a = try transformedTriangle(view_a, transform_a, source_a, status);
    const b = try transformedTriangle(view_b, transform_b, source_b, status);
    return triangleTriangleContact(a, b, pair.a, pair.b, status) orelse unreachable;
}

/// Dynamic-safe Mesh--Mesh patch generation. It reuses the ordered exact SAT
/// overlap set, then emits one deterministic triangle witness per pair before
/// reducing to the protocol's four contact points.
pub fn meshMeshPatchTransformed(view_a: runtime_view.View, transform_a: @import("../math/geometry.zig").Transform3, view_b: runtime_view.View, transform_b: @import("../math/geometry.zig").Transform3, workspace: MeshMeshPatchWorkspace, status: *fp.MathStatus) RuntimeError!gjk.ContactPatch {
    const overlaps = try meshMeshOverlapsTransformed(view_a, transform_a, view_b, transform_b, workspace.query, status);
    if (overlaps.len > workspace.contacts.len) return error.CapacityExceeded;
    var count: usize = 0;
    for (overlaps) |pair| {
        workspace.contacts[count] = try meshTrianglePairContact(view_a, transform_a, view_b, transform_b, pair, status);
        count += 1;
    }
    return gjk.reducePatch(workspace.contacts[0..count], status);
}

/// Identity-transform convenience entry point for immutable meshes queried in
/// their shared world frame.
pub fn meshMeshPatch(view_a: runtime_view.View, view_b: runtime_view.View, workspace: MeshMeshPatchWorkspace) RuntimeError!gjk.ContactPatch {
    var status = fp.MathStatus{};
    return meshMeshPatchTransformed(view_a, .{}, view_b, .{}, workspace, &status);
}

fn meshTrianglesOverlap(view_a: runtime_view.View, transform_a: @import("../math/geometry.zig").Transform3, triangle_a: u32, view_b: runtime_view.View, transform_b: @import("../math/geometry.zig").Transform3, triangle_b: u32, status: *fp.MathStatus) runtime_view.Error!bool {
    const source_a = try view_a.triangle(triangle_a);
    const source_b = try view_b.triangle(triangle_b);
    return trianglesOverlap(
        .{ .a = transform_a.apply(try view_a.vertex(source_a.a), status), .b = transform_a.apply(try view_a.vertex(source_a.b), status), .c = transform_a.apply(try view_a.vertex(source_a.c), status) },
        .{ .a = transform_b.apply(try view_b.vertex(source_b.a), status), .b = transform_b.apply(try view_b.vertex(source_b.b), status), .c = transform_b.apply(try view_b.vertex(source_b.c), status) },
    );
}

fn transformBounds(bounds: @import("../math/geometry.zig").Aabb3, transform: @import("../math/geometry.zig").Transform3, status: *fp.MathStatus) @import("../math/geometry.zig").Aabb3 {
    var first = true;
    var result: @import("../math/geometry.zig").Aabb3 = undefined;
    inline for ([_]fp.Fp{ bounds.min.x, bounds.max.x }) |x| inline for ([_]fp.Fp{ bounds.min.y, bounds.max.y }) |y| inline for ([_]fp.Fp{ bounds.min.z, bounds.max.z }) |z| {
        const point = transform.apply(.{ .x = x, .y = y, .z = z }, status);
        if (first) {
            result = .{ .min = point, .max = point };
            first = false;
        } else {
            result.min.x = fp.Fp{ .raw = @min(result.min.x.raw, point.x.raw) };
            result.min.y = fp.Fp{ .raw = @min(result.min.y.raw, point.y.raw) };
            result.min.z = fp.Fp{ .raw = @min(result.min.z.raw, point.z.raw) };
            result.max.x = fp.Fp{ .raw = @max(result.max.x.raw, point.x.raw) };
            result.max.y = fp.Fp{ .raw = @max(result.max.y.raw, point.y.raw) };
            result.max.z = fp.Fp{ .raw = @max(result.max.z.raw, point.z.raw) };
        }
    };
    return result;
}

/// Removes only duplicate seam contacts: adjacent welded triangles whose
/// normals differ by at most the fixed smooth-angle threshold and which
/// resolve to the same mesh witness. Sharp edges and distinct contact
/// positions are intentionally retained.
fn suppressInternalEdgeContacts(view: runtime_view.View, contacts: []gjk.ContactPoint, status: *fp.MathStatus) RuntimeError!usize {
    var count: usize = 0;
    for (contacts) |contact| {
        var duplicate = false;
        for (contacts[0..count]) |prior| {
            if (!samePoint(contact.point_b, prior.point_b) or contact.feature_b == prior.feature_b) continue;
            if (try coplanarAdjacent(view, contact.feature_b, prior.feature_b, status)) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            contacts[count] = contact;
            count += 1;
        }
    }
    return count;
}
fn samePoint(a: @import("../math/geometry.zig").Vec3, b: @import("../math/geometry.zig").Vec3) bool {
    return a.x.raw == b.x.raw and a.y.raw == b.y.raw and a.z.raw == b.z.raw;
}
fn coplanarAdjacent(view: runtime_view.View, first_id: u32, second_id: u32, status: *fp.MathStatus) RuntimeError!bool {
    const first = try view.triangle(first_id);
    const second = try view.triangle(second_id);
    const first_indices = [_]u32{ first.a, first.b, first.c };
    const second_indices = [_]u32{ second.a, second.b, second.c };
    var shared: u8 = 0;
    for (first_indices) |left| for (second_indices) |right| {
        if (left == right) shared += 1;
    };
    if (shared != 2) return false;
    const a0 = try view.vertex(first.a);
    const a1 = try view.vertex(first.b);
    const a2 = try view.vertex(first.c);
    const b0 = try view.vertex(second.a);
    const b1 = try view.vertex(second.b);
    const b2 = try view.vertex(second.c);
    const normal_a = a1.sub(a0, status).cross(a2.sub(a0, status), status);
    const normal_b = b1.sub(b0, status).cross(b2.sub(b0, status), status);
    const dot = normal_a.dot(normal_b, status);
    if (dot.raw <= 0) return false;
    // cos^2(theta) >= 15/16: a deterministic ~14.5 degree weld threshold.
    // Squaring avoids any square-root or platform-dependent normalization.
    const left = dot.mul(dot, status);
    const lengths = normal_a.lengthSquared(status).mul(normal_b.lengthSquared(status), status);
    const threshold = lengths.mul(fp.Fp.fromRatio(15, 16, status), status);
    return left.raw >= threshold.raw;
}

/// BVH-backed Sphere–HeightField query. Tile expansion preserves canonical
/// cell/half order and hole suppression before invoking Sphere–Triangle.
pub fn sphereHeightfieldPatch(view: runtime_view.View, sphere: analytic.Sphere, workspace: SphereHeightfieldWorkspace, status: *fp.MathStatus) RuntimeError!gjk.ContactPatch {
    return sphereHeightfieldPatchTransformed(view, .{}, sphere, workspace, status);
}

/// Sphere against a static or kinematic immutable HeightField. Tile bounds and
/// expanded triangles are transformed into the sphere's query frame without
/// modifying the baked asset.
pub fn sphereHeightfieldPatchTransformed(view: runtime_view.View, heightfield_transform: @import("../math/geometry.zig").Transform3, sphere: analytic.Sphere, workspace: SphereHeightfieldWorkspace, status: *fp.MathStatus) RuntimeError!gjk.ContactPatch {
    if (view.header.kind != .height_field or view.heightTileNodeCount() == 0) return error.InvalidBvh;
    const tile_count = view.heightTileNodeCount();
    if (workspace.tile_nodes.len < tile_count) return error.CapacityExceeded;
    for (workspace.tile_nodes[0..tile_count], 0..) |*node, i| node.* = try view.heightTileNode(i);
    for (workspace.tile_nodes[0..tile_count]) |*node| node.bounds = transformBounds(node.bounds, heightfield_transform, status);
    var query_primitives = [_]u32{0};
    const radius = sphere.radius;
    const query_bounds = @import("../math/geometry.zig").Aabb3{ .min = .{ .x = sphere.center.x.sub(radius, status), .y = sphere.center.y.sub(radius, status), .z = sphere.center.z.sub(radius, status) }, .max = .{ .x = sphere.center.x.add(radius, status), .y = sphere.center.y.add(radius, status), .z = sphere.center.z.add(radius, status) } };
    const query_nodes = [_]baked.BvhNode{baked.BvhNode.leaf(query_bounds, 0, 1)};
    const pairs = try traverseHeightTiles(&query_nodes, &query_primitives, workspace.tile_nodes[0..tile_count], workspace.work, workspace.pair_scratch, workspace.pair_output);
    var contact_count: usize = 0;
    for (pairs) |pair| {
        const expanded = try expandHeightTile(view, pair.b, workspace.triangles);
        if (contact_count + expanded.len > workspace.contacts.len) return error.CapacityExceeded;
        for (expanded) |height_triangle| {
            const feature = (height_triangle.cell << 1) | height_triangle.half;
            const triangle = Triangle{ .a = heightfield_transform.apply(height_triangle.triangle.a, status), .b = heightfield_transform.apply(height_triangle.triangle.b, status), .c = heightfield_transform.apply(height_triangle.triangle.c, status) };
            const result = analytic.sphereTriangle(sphere, .{ .a = triangle.a, .b = triangle.b, .c = triangle.c }, feature, @import("../math/geometry.zig").Vec3.zero, ids.ColliderId.init(0, 0), ids.ColliderId.init(1, 0), status);
            if (result.separation.raw > 0) continue;
            workspace.contacts[contact_count] = .{ .point_a = result.witness_a, .point_b = result.witness_b, .normal = result.normal, .separation = result.separation, .feature_a = 0, .feature_b = feature };
            contact_count += 1;
        }
    }
    return gjk.reducePatch(workspace.contacts[0..contact_count], status);
}

/// Exact runtime convex--HeightField dispatch. Hole cells are removed during
/// tile expansion before GJK, and returned IDs retain the cell/half feature
/// encoding used by the contact path.
pub fn convexHeightfieldIntersections(shape: shapes.Shape, assets: *const store.Store, convex_transform: @import("../math/geometry.zig").Transform3, view: runtime_view.View, heightfield_transform: @import("../math/geometry.zig").Transform3, workspace: ConvexHeightfieldWorkspace, status: *fp.MathStatus) ConvexSurfaceError![]const u32 {
    if (view.header.kind != .height_field or view.heightTileNodeCount() == 0) return error.InvalidBvh;
    const tile_count = view.heightTileNodeCount();
    if (workspace.tile_nodes.len < tile_count) return error.CapacityExceeded;
    for (workspace.tile_nodes[0..tile_count], 0..) |*node, i| node.* = try view.heightTileNode(i);
    for (workspace.tile_nodes[0..tile_count]) |*node| node.bounds = transformBounds(node.bounds, heightfield_transform, status);
    const query_bounds = try shapes.worldAabb(shape, assets, convex_transform, status);
    const query_nodes = [_]baked.BvhNode{baked.BvhNode.leaf(query_bounds, 0, 1)};
    const query_primitives = [_]u32{0};
    const tiles = try traverseHeightTiles(&query_nodes, &query_primitives, workspace.tile_nodes[0..tile_count], workspace.work, workspace.pair_scratch, workspace.pair_output);
    var required: usize = 0;
    for (tiles) |tile| {
        const expanded = try expandHeightTile(view, tile.b, workspace.triangles);
        for (expanded) |height_triangle| {
            if (try convexHeightTriangleOverlaps(shape, assets, convex_transform, heightfield_transform, height_triangle.triangle, status)) required += 1;
        }
    }
    if (required > workspace.intersections.len) return error.CapacityExceeded;
    var count: usize = 0;
    for (tiles) |tile| {
        const expanded = try expandHeightTile(view, tile.b, workspace.triangles);
        for (expanded) |height_triangle| {
            if (try convexHeightTriangleOverlaps(shape, assets, convex_transform, heightfield_transform, height_triangle.triangle, status)) {
                workspace.intersections[count] = (height_triangle.cell << 1) | height_triangle.half;
                count += 1;
            }
        }
    }
    return workspace.intersections[0..count];
}

/// EPA-backed convex--HeightField patch. Returned HeightField feature IDs are
/// decoded directly into canonical cell triangles, so the second contact pass
/// does not depend on the incidental order in which BVH tiles were visited.
pub fn convexHeightfieldPatch(shape: shapes.Shape, assets: *const store.Store, convex_transform: @import("../math/geometry.zig").Transform3, view: runtime_view.View, heightfield_transform: @import("../math/geometry.zig").Transform3, workspace: ConvexHeightfieldPatchWorkspace, status: *fp.MathStatus) ConvexContactError!gjk.ContactPatch {
    const hits = try convexHeightfieldIntersections(shape, assets, convex_transform, view, heightfield_transform, workspace.query, status);
    if (hits.len > workspace.contacts.len) return error.CapacityExceeded;
    const dimensions = try view.heightDimensions();
    const cells_x = dimensions.width - 1;
    var count: usize = 0;
    for (hits) |feature| {
        const cell = feature >> 1;
        const half: u1 = @truncate(feature);
        const x = cell % cells_x;
        const z = cell / cells_x;
        const p00 = try heightPoint(view, dimensions.width, x, z);
        const p10 = try heightPoint(view, dimensions.width, x + 1, z);
        const p01 = try heightPoint(view, dimensions.width, x, z + 1);
        const p11 = try heightPoint(view, dimensions.width, x + 1, z + 1);
        const source: Triangle = if (half == 0) .{ .a = p00, .b = p10, .c = p01 } else .{ .a = p10, .b = p11, .c = p01 };
        const triangle = Triangle{ .a = heightfield_transform.apply(source.a, status), .b = heightfield_transform.apply(source.b, status), .c = heightfield_transform.apply(source.c, status) };
        var context: ConvexTriangleContext = .{ .shape = shape, .assets = assets, .transform = convex_transform, .triangle = triangle };
        const result = try convexTriangleContact(&context, triangle.a.sub(convex_transform.position, status), workspace.triangle, status);
        var contact = result.contact orelse unreachable;
        contact.feature_b = feature;
        workspace.contacts[count] = contact;
        count += 1;
    }
    return gjk.reducePatch(workspace.contacts[0..count], status);
}

fn convexHeightTriangleOverlaps(shape: shapes.Shape, assets: *const store.Store, convex_transform: @import("../math/geometry.zig").Transform3, heightfield_transform: @import("../math/geometry.zig").Transform3, source: Triangle, status: *fp.MathStatus) shapes.Error!bool {
    const triangle = Triangle{ .a = heightfield_transform.apply(source.a, status), .b = heightfield_transform.apply(source.b, status), .c = heightfield_transform.apply(source.c, status) };
    var context: ConvexTriangleContext = .{ .shape = shape, .assets = assets, .transform = convex_transform, .triangle = triangle };
    const seed = triangle.a.sub(convex_transform.position, status);
    return (try convexTriangleIntersect(&context, seed, status)).status == .intersecting;
}

/// Traverses canonical Compound leaves and aggregates sphere contacts from
/// mesh/heightfield leaves into one stable patch. Other terminal kinds are a
/// caller error: they belong to the convex dispatcher rather than this
/// surface-only path.
pub fn sphereCompoundSurfacePatch(compound: shapes.Shape, assets: *const store.Store, transform: @import("../math/geometry.zig").Transform3, sphere: analytic.Sphere, workspace: SphereCompoundSurfaceWorkspace, status: *fp.MathStatus) CompoundError!gjk.ContactPatch {
    const leaves = try shapes.collectCompoundLeaves(compound, assets, transform, workspace.leaves, status);
    if (leaves.len > workspace.merged.len / 4) return error.CapacityExceeded;
    var count: usize = 0;
    for (leaves) |leaf| {
        const view = switch (leaf.shape) {
            .triangle_mesh => |asset| try runtime_view.find(assets, asset.source_id),
            .height_field => |asset| try runtime_view.find(assets, asset.source_id),
            else => return error.UnsupportedShape,
        };
        var patch = switch (leaf.shape) {
            .triangle_mesh => try sphereMeshPatchTransformed(view, leaf.transform, sphere, workspace.mesh, status),
            .height_field => try sphereHeightfieldPatchTransformed(view, leaf.transform, sphere, workspace.heightfield, status),
            else => unreachable,
        };
        for (patch.points[0..patch.len]) |*point| {
            point.path_b = leaf.path;
            workspace.merged[count] = point.*;
            count += 1;
        }
    }
    return gjk.reducePatch(workspace.merged[0..count], status);
}

/// Aggregates convex contacts across immutable mesh and HeightField Compound
/// leaves, preserving every canonical child path before the final four-point
/// reduction. Leaf traversal and all temporary buffers remain caller-owned.
pub fn convexCompoundSurfacePatch(compound: shapes.Shape, assets: *const store.Store, transform: @import("../math/geometry.zig").Transform3, shape: shapes.Shape, shape_transform: @import("../math/geometry.zig").Transform3, workspace: ConvexCompoundSurfaceWorkspace, status: *fp.MathStatus) ConvexCompoundError!gjk.ContactPatch {
    const leaves = try shapes.collectCompoundLeaves(compound, assets, transform, workspace.leaves, status);
    if (leaves.len > workspace.merged.len / 4) return error.CapacityExceeded;
    var count: usize = 0;
    for (leaves) |leaf| {
        const view = switch (leaf.shape) {
            .triangle_mesh => |asset| try runtime_view.find(assets, asset.source_id),
            .height_field => |asset| try runtime_view.find(assets, asset.source_id),
            else => return error.UnsupportedShape,
        };
        var patch = switch (leaf.shape) {
            .triangle_mesh => try convexMeshPatch(shape, assets, shape_transform, view, leaf.transform, workspace.mesh, status),
            .height_field => try convexHeightfieldPatch(shape, assets, shape_transform, view, leaf.transform, workspace.heightfield, status),
            else => unreachable,
        };
        for (patch.points[0..patch.len]) |*point| {
            point.path_b = leaf.path;
            workspace.merged[count] = point.*;
            count += 1;
        }
    }
    return gjk.reducePatch(workspace.merged[0..count], status);
}
/// Height BVH leaves store tile index ranges rather than a primitive table.
fn traverseHeightTiles(query_nodes: []const baked.BvhNode, query_primitives: []const u32, tiles: []const baked.BvhNode, work: []NodePair, scratch: []PrimitivePair, output: []PrimitivePair) Error![]const PrimitivePair {
    // Materialize the tiny canonical tile primitive table into scratch's pair
    // storage would couple types, so use a fixed local pass over leaf ranges.
    if (query_nodes.len != 1 or query_primitives.len != 1 or tiles.len == 0 or work.len == 0) return error.InvalidBvh;
    var work_len: usize = 0;
    try pushOrdered(work, &work_len, .{ .a = 0, .b = 0 });
    var count: usize = 0;
    while (work_len != 0) {
        const pair = work[0];
        std.mem.copyForwards(NodePair, work[0 .. work_len - 1], work[1..work_len]);
        work_len -= 1;
        if (pair.a >= query_nodes.len or pair.b >= tiles.len) return error.InvalidBvh;
        const a = query_nodes[pair.a];
        const b = tiles[pair.b];
        if (!a.bounds.overlaps(b.bounds)) continue;
        const leaf = (b.flags & baked.BvhNode.leaf_flag) != 0;
        if (leaf) {
            const end = @as(usize, b.first) + b.count;
            if (end > std.math.maxInt(u32)) return error.InvalidBvh;
            var tile = b.first;
            while (tile < end) : (tile += 1) {
                if (count == scratch.len) return error.CapacityExceeded;
                scratch[count] = .{ .a = 0, .b = tile };
                count += 1;
            }
        } else {
            if (b.first + 1 >= tiles.len) return error.InvalidBvh;
            try pushOrdered(work, &work_len, .{ .a = 0, .b = b.first });
            try pushOrdered(work, &work_len, .{ .a = 0, .b = b.first + 1 });
        }
    }
    insertionSortPairs(scratch[0..count]);
    if (count > output.len) return error.CapacityExceeded;
    @memcpy(output[0..count], scratch[0..count]);
    return output[0..count];
}
fn heightPoint(view: runtime_view.View, width: u32, x: u32, z: u32) runtime_view.Error!@import("../math/geometry.zig").Vec3 {
    return .{ .x = @import("../math/fp.zig").Fp.fromInt(@intCast(x)), .y = try view.heightSample(@as(usize, z) * width + x), .z = @import("../math/fp.zig").Fp.fromInt(@intCast(z)) };
}

/// Traverses two baked BVHs in a fixed `(node_a,node_b)` priority order. The
/// caller supplies work and scratch buffers; `output` is untouched on failure.
pub fn traverseBvhPairs(nodes_a: []const baked.BvhNode, primitives_a: []const u32, nodes_b: []const baked.BvhNode, primitives_b: []const u32, work: []NodePair, scratch: []PrimitivePair, output: []PrimitivePair) Error![]const PrimitivePair {
    if (nodes_a.len == 0 or nodes_b.len == 0 or work.len == 0) return error.InvalidBvh;
    var work_len: usize = 0;
    try pushOrdered(work, &work_len, .{ .a = 0, .b = 0 });
    var result_len: usize = 0;
    while (work_len != 0) {
        const pair = work[0];
        std.mem.copyForwards(NodePair, work[0 .. work_len - 1], work[1..work_len]);
        work_len -= 1;
        if (pair.a >= nodes_a.len or pair.b >= nodes_b.len) return error.InvalidBvh;
        const a = nodes_a[pair.a];
        const b = nodes_b[pair.b];
        if (!a.bounds.overlaps(b.bounds)) continue;
        const a_leaf = (a.flags & baked.BvhNode.leaf_flag) != 0;
        const b_leaf = (b.flags & baked.BvhNode.leaf_flag) != 0;
        if (a_leaf and b_leaf) {
            const a_end = @as(usize, a.first) + a.count;
            const b_end = @as(usize, b.first) + b.count;
            if (a_end > primitives_a.len or b_end > primitives_b.len) return error.InvalidBvh;
            for (primitives_a[a.first..a_end]) |primitive_a| for (primitives_b[b.first..b_end]) |primitive_b| {
                if (result_len == scratch.len) return error.CapacityExceeded;
                scratch[result_len] = .{ .a = primitive_a, .b = primitive_b };
                result_len += 1;
            };
        } else if (a_leaf) {
            try pushChildrenA(work, &work_len, pair.a, b, false);
        } else if (b_leaf) {
            try pushChildrenA(work, &work_len, pair.b, a, true);
        } else {
            if (a.first + 1 >= nodes_a.len or b.first + 1 >= nodes_b.len) return error.InvalidBvh;
            try pushOrdered(work, &work_len, .{ .a = a.first, .b = b.first });
            try pushOrdered(work, &work_len, .{ .a = a.first, .b = b.first + 1 });
            try pushOrdered(work, &work_len, .{ .a = a.first + 1, .b = b.first });
            try pushOrdered(work, &work_len, .{ .a = a.first + 1, .b = b.first + 1 });
        }
    }
    insertionSortPairs(scratch[0..result_len]);
    if (result_len > output.len) return error.CapacityExceeded;
    @memcpy(output[0..result_len], scratch[0..result_len]);
    return output[0..result_len];
}
fn pushChildrenA(work: []NodePair, len: *usize, leaf_node: u32, inner: baked.BvhNode, swap: bool) Error!void {
    if (inner.count != 0) return error.InvalidBvh;
    const first = inner.first;
    try pushOrdered(work, len, if (swap) .{ .a = first, .b = leaf_node } else .{ .a = leaf_node, .b = first });
    try pushOrdered(work, len, if (swap) .{ .a = first + 1, .b = leaf_node } else .{ .a = leaf_node, .b = first + 1 });
}
fn pushOrdered(work: []NodePair, len: *usize, value: NodePair) Error!void {
    if (len.* == work.len) return error.CapacityExceeded;
    var at = len.*;
    while (at > 0 and pairLess(value, work[at - 1])) : (at -= 1) work[at] = work[at - 1];
    work[at] = value;
    len.* += 1;
}
fn pairLess(a: NodePair, b: NodePair) bool {
    return a.a < b.a or (a.a == b.a and a.b < b.b);
}
fn insertionSortPairs(values: []PrimitivePair) void {
    var i: usize = 1;
    while (i < values.len) : (i += 1) {
        const value = values[i];
        var at = i;
        while (at > 0 and primitiveLess(value, values[at - 1])) : (at -= 1) values[at] = values[at - 1];
        values[at] = value;
    }
}
fn primitiveLess(a: PrimitivePair, b: PrimitivePair) bool {
    return a.a < b.a or (a.a == b.a and a.b < b.b);
}
