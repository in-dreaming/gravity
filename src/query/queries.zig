//! Read-only deterministic query primitives and stable hit publication.
const fp = @import("../math/fp.zig");
const geometry = @import("../math/geometry.zig");
const ids = @import("../core/ids.zig");
const shapes = @import("../collision/shapes.zig");
const gjk = @import("../collision/gjk.zig");
const analytic = @import("../collision/analytic.zig");
const store = @import("../assets/store.zig");
const runtime_view = @import("../assets/runtime_view.zig");
const baked = @import("../geometry/baked.zig");
const mesh = @import("../collision/mesh.zig");

pub const Error = error{ CapacityExceeded, InvalidQuery, UnsupportedShape };
pub const Mode = enum { any, closest, all };
pub const Ray = struct {
    origin: geometry.Vec3,
    /// Segment displacement over fraction [0, 1], not a normalized direction.
    delta: geometry.Vec3,
};
pub const Filter = struct { category: u32 = 1, mask: u32 = ~@as(u32, 0), group: i32 = 0 };
pub const RayHit = struct { fraction: fp.Fp, point: geometry.Vec3, normal: geometry.Vec3 };
pub const MeshRayHit = struct { fraction: fp.Fp, point: geometry.Vec3, normal: geometry.Vec3, primitive: u32 };
pub const ShapeCastStatus = enum { hit, miss, non_convergent };
pub const ShapeCastHit = struct {
    status: ShapeCastStatus,
    fraction: fp.Fp = .zero,
    point: geometry.Vec3 = .zero,
    normal: geometry.Vec3 = .unit_x,
    feature: u32 = 0,
    iterations: u8 = 0,
};
pub const Hit = struct {
    fraction: fp.Fp,
    collider: ids.ColliderId,
    child_path: shapes.ChildPath = .{},
    primitive: u32 = 0,
    feature: u32 = 0,
    point: geometry.Vec3,
    normal: geometry.Vec3,
    pub fn lessThan(a: Hit, b: Hit) bool {
        if (a.fraction.raw != b.fraction.raw) return a.fraction.raw < b.fraction.raw;
        if (a.collider.index() != b.collider.index()) return a.collider.index() < b.collider.index();
        if (a.collider.generation() != b.collider.generation()) return a.collider.generation() < b.collider.generation();
        if (a.child_path.len != b.child_path.len) return a.child_path.len < b.child_path.len;
        for (a.child_path.values[0..a.child_path.len], b.child_path.values[0..b.child_path.len]) |left, right| if (left != right) return left < right;
        if (a.primitive != b.primitive) return a.primitive < b.primitive;
        return a.feature < b.feature;
    }
};
pub const Publication = struct { hits: []const Hit, required: usize };
pub const Item = struct { id: ids.ColliderId, collider: *const shapes.Collider, transform: geometry.Transform3 };
/// Reusable caller-owned traversal storage for asset-backed ray queries.
/// It is scratch only and never mutates the immutable asset store or World.
pub const RayWorkspace = struct { bvh_nodes: []u32, mesh_hits: []MeshRayHit, height_triangles: []mesh.HeightTriangle, compound_leaves: []shapes.CompoundLeaf };
/// Caller-owned scratch for overlap queries whose convex operand may meet an
/// immutable surface asset. It deliberately reuses Task 11's exact triangle
/// traversal workspaces instead of treating an asset AABB as geometry.
pub const SurfaceOverlapWorkspace = struct {
    compound_leaves: []shapes.CompoundLeaf,
    mesh: mesh.ConvexMeshWorkspace,
    heightfield: mesh.ConvexHeightfieldWorkspace,
};
/// Storage used solely to validate that a Compound caster consists entirely
/// of convex terminal leaves before it enters the GJK support adapter.
pub const ShapeCastWorkspace = struct { compound_leaves: []shapes.CompoundLeaf };
/// Fixed storage for a convex caster against one immutable surface target.
/// It contains only traversal scratch and never changes baked assets.
pub const SurfaceCastWorkspace = struct {
    compound_leaves: []shapes.CompoundLeaf,
    mesh: mesh.ConvexMeshDistanceWorkspace,
    heightfield: mesh.ConvexHeightfieldDistanceWorkspace,
};

/// Point overlap has closed-set semantics: contact on a shape boundary is an
/// overlap.  Asset-backed and surface shapes require their dedicated BVH
/// traversal entry points and are therefore rejected here rather than being
/// approximated by their bounds.
pub fn pointOverlapsPrimitive(point: geometry.Vec3, shape: shapes.Shape, transform: geometry.Transform3, status: *fp.MathStatus) Error!bool {
    const local = transform.inverseApply(point, status);
    return switch (shape) {
        .sphere => |sphere| local.lengthSquared(status).raw <= sphere.radius.mul(sphere.radius, status).raw,
        .box => |box| local.x.abs(status).raw <= box.half_extents.x.raw and local.y.abs(status).raw <= box.half_extents.y.raw and local.z.abs(status).raw <= box.half_extents.z.raw,
        .capsule => |capsule| blk: {
            const y = if (local.y.raw < -capsule.half_height.raw) capsule.half_height.neg(status) else if (local.y.raw > capsule.half_height.raw) capsule.half_height else local.y;
            const offset = local.sub(.{ .y = y }, status);
            break :blk offset.lengthSquared(status).raw <= capsule.radius.mul(capsule.radius, status).raw;
        },
        else => error.UnsupportedShape,
    };
}

/// Closed-set point overlap for every convex query shape.  Compound is
/// resolved into its immutable terminal leaves, so nested child transforms
/// and the stable baked child ordering are preserved rather than tested via
/// the Compound's broadphase bounds.
pub fn pointOverlapsConvex(point: geometry.Vec3, shape: shapes.Shape, transform: geometry.Transform3, assets: *const store.Store, compound_leaves: []shapes.CompoundLeaf, status: *fp.MathStatus) (shapes.Error || Error)!bool {
    if (shape == .compound) {
        const leaves = try shapes.collectCompoundLeaves(shape, assets, transform, compound_leaves, status);
        for (leaves) |leaf| if (try pointOverlapsConvex(point, leaf.shape, leaf.transform, assets, compound_leaves, status)) return true;
        return false;
    }
    if (shape == .convex_hull) {
        const asset_shape = shape.convex_hull;
        const source_id = if (asset_shape.source_id != 0) asset_shape.source_id else asset_shape.asset.index();
        const view = try runtime_view.find(assets, source_id);
        return pointInsideConvexHull(transform.inverseApply(point, status), view, status);
    }
    return pointOverlapsPrimitive(point, shape, transform, status);
}

/// Closed-set point overlap for every immutable runtime shape. Surface assets
/// are tested against exact triangles after a deterministic BVH candidate walk;
/// a mesh's conservative bounds never constitute a hit. `workspace` is scratch
/// only and remains wholly caller-owned.
pub fn pointOverlapsShape(point: geometry.Vec3, shape: shapes.Shape, transform: geometry.Transform3, assets: *const store.Store, workspace: RayWorkspace, status: *fp.MathStatus) (shapes.Error || mesh.Error || Error)!bool {
    return switch (shape) {
        .compound => blk: {
            const leaves = try shapes.collectCompoundLeaves(shape, assets, transform, workspace.compound_leaves, status);
            for (leaves) |leaf| if (try pointOverlapsShape(point, leaf.shape, leaf.transform, assets, workspace, status)) break :blk true;
            break :blk false;
        },
        .triangle_mesh => |asset_shape| blk: {
            const source_id = if (asset_shape.source_id != 0) asset_shape.source_id else asset_shape.asset.index();
            break :blk try pointOverlapsMesh(point, try runtime_view.find(assets, source_id), transform, workspace.bvh_nodes, status);
        },
        .height_field => |asset_shape| blk: {
            const source_id = if (asset_shape.source_id != 0) asset_shape.source_id else asset_shape.asset.index();
            break :blk try pointOverlapsHeightfield(point, try runtime_view.find(assets, source_id), transform, workspace.bvh_nodes, workspace.height_triangles, status);
        },
        else => pointOverlapsConvex(point, shape, transform, assets, workspace.compound_leaves, status),
    };
}

fn pointOverlapsMesh(point: geometry.Vec3, view: runtime_view.View, transform: geometry.Transform3, work: []u32, status: *fp.MathStatus) (runtime_view.Error || Error)!bool {
    if (view.header.kind != .triangle_mesh or view.nodeCount() == 0 or work.len == 0) return error.InvalidQuery;
    const local = transform.inverseApply(point, status);
    var work_len: usize = 1;
    work[0] = 0;
    while (work_len != 0) {
        const node_index = work[0];
        for (work[0 .. work_len - 1], work[1..work_len]) |*dst, src| dst.* = src;
        work_len -= 1;
        if (node_index >= view.nodeCount()) return error.InvalidQuery;
        const node = try view.node(node_index);
        if (!pointInAabb(local, node.bounds)) continue;
        if ((node.flags & baked.BvhNode.leaf_flag) == 0) {
            if (node.count != 0 or node.first + 1 >= view.nodeCount()) return error.InvalidQuery;
            try pushNode(work, &work_len, node.first);
            try pushNode(work, &work_len, node.first + 1);
            continue;
        }
        const end = @as(usize, node.first) + node.count;
        if (end > view.primitiveCount()) return error.InvalidQuery;
        for (node.first..end) |primitive_index| {
            const primitive = try view.primitive(primitive_index);
            const triangle = try view.triangle(primitive);
            if (pointOnTriangle(local, .{ .a = try view.vertex(triangle.a), .b = try view.vertex(triangle.b), .c = try view.vertex(triangle.c) }, status)) return true;
        }
    }
    return false;
}

fn pointOverlapsHeightfield(point: geometry.Vec3, view: runtime_view.View, transform: geometry.Transform3, work: []u32, triangle_scratch: []mesh.HeightTriangle, status: *fp.MathStatus) (runtime_view.Error || mesh.Error || Error)!bool {
    if (view.header.kind != .height_field or view.heightTileNodeCount() == 0 or work.len == 0) return error.InvalidQuery;
    const local = transform.inverseApply(point, status);
    var work_len: usize = 1;
    work[0] = 0;
    while (work_len != 0) {
        const node_index = work[0];
        for (work[0 .. work_len - 1], work[1..work_len]) |*dst, src| dst.* = src;
        work_len -= 1;
        if (node_index >= view.heightTileNodeCount()) return error.InvalidQuery;
        const node = try view.heightTileNode(node_index);
        if (!pointInAabb(local, node.bounds)) continue;
        if ((node.flags & baked.BvhNode.leaf_flag) == 0) {
            if (node.count != 0 or node.first + 1 >= view.heightTileNodeCount()) return error.InvalidQuery;
            try pushNode(work, &work_len, node.first);
            try pushNode(work, &work_len, node.first + 1);
            continue;
        }
        const end = @as(usize, node.first) + node.count;
        for (node.first..end) |tile| {
            const triangles = try mesh.expandHeightTile(view, @intCast(tile), triangle_scratch);
            for (triangles) |triangle| if (pointOnTriangle(local, triangle.triangle, status)) return true;
        }
    }
    return false;
}

/// Exact AABB-vs-convex overlap through the same fixed GJK path as shape
/// overlap.  Bounds are represented as a world-space box, so rotated target
/// shapes retain their true geometry instead of using a broadphase AABB.
pub fn aabbOverlapsConvex(bounds: geometry.Aabb3, shape: shapes.Shape, transform: geometry.Transform3, assets: *const store.Store, status: *fp.MathStatus) (shapes.Error || Error)!bool {
    const center = bounds.min.add(bounds.max, status).scale(fp.Fp.fromRatio(1, 2, status), status);
    const half = bounds.max.sub(bounds.min, status).scale(fp.Fp.fromRatio(1, 2, status), status);
    return convexOverlaps(.{ .box = .{ .half_extents = half } }, .{ .position = center }, shape, transform, assets, status);
}

/// Exact closed-set AABB overlap for every supported target shape. The AABB
/// is a convex box caster, not an approximation of the target; mesh and
/// HeightField targets are dispatched through their triangle BVHs.
pub fn aabbOverlapsShape(bounds: geometry.Aabb3, shape: shapes.Shape, transform: geometry.Transform3, assets: *const store.Store, workspace: SurfaceOverlapWorkspace, status: *fp.MathStatus) (shapes.Error || mesh.Error || Error)!bool {
    const center = bounds.min.add(bounds.max, status).scale(fp.Fp.fromRatio(1, 2, status), status);
    const half = bounds.max.sub(bounds.min, status).scale(fp.Fp.fromRatio(1, 2, status), status);
    return convexOverlapsShape(.{ .box = .{ .half_extents = half } }, .{ .position = center }, shape, transform, assets, workspace, status);
}

/// Exact overlap of a convex (or all-convex Compound) query shape with a
/// runtime target. Surface targets use Task 11's per-triangle GJK traversal;
/// Compound targets preserve baked leaf transforms and cannot silently turn a
/// surface child into a support-mapped convex hull.
pub fn convexOverlapsShape(caster: shapes.Shape, caster_transform: geometry.Transform3, target: shapes.Shape, target_transform: geometry.Transform3, assets: *const store.Store, workspace: SurfaceOverlapWorkspace, status: *fp.MathStatus) (shapes.Error || mesh.Error || Error)!bool {
    if (!(try isResolvedConvex(caster, assets, workspace.compound_leaves, status))) return error.UnsupportedShape;
    if (target == .compound) {
        const leaves = try shapes.collectCompoundLeaves(target, assets, target_transform, workspace.compound_leaves, status);
        for (leaves) |leaf| if (try convexOverlapsShape(caster, caster_transform, leaf.shape, leaf.transform, assets, workspace, status)) return true;
        return false;
    }
    return switch (target) {
        .triangle_mesh => |asset_shape| blk: {
            const source_id = if (asset_shape.source_id != 0) asset_shape.source_id else asset_shape.asset.index();
            const intersections = try mesh.convexMeshIntersections(caster, assets, caster_transform, try runtime_view.find(assets, source_id), target_transform, workspace.mesh, status);
            break :blk intersections.len != 0;
        },
        .height_field => |asset_shape| blk: {
            const source_id = if (asset_shape.source_id != 0) asset_shape.source_id else asset_shape.asset.index();
            const intersections = try mesh.convexHeightfieldIntersections(caster, assets, caster_transform, try runtime_view.find(assets, source_id), target_transform, workspace.heightfield, status);
            break :blk intersections.len != 0;
        },
        else => convexOverlapsResolved(caster, caster_transform, target, target_transform, assets, workspace.compound_leaves, status),
    };
}

/// Convex-vs-convex overlap reuses the Task 10 fixed 32-iteration GJK path.
/// A non-convergent result is surfaced explicitly; it is never reported as a
/// successful overlap.
pub fn convexOverlaps(a: shapes.Shape, transform_a: geometry.Transform3, b: shapes.Shape, transform_b: geometry.Transform3, assets: *const store.Store, status: *fp.MathStatus) (shapes.Error || Error)!bool {
    if (!isConvex(a) or !isConvex(b) or a == .compound or b == .compound) return error.UnsupportedShape;
    var pair = gjk.ShapePairContext{ .shape_a = a, .shape_b = b, .assets = assets, .transform_a = transform_a, .transform_b = transform_b };
    const result = try gjk.intersectShapes(&pair, transform_b.position.sub(transform_a.position, status), status);
    return switch (result.status) {
        .intersecting => true,
        .separated => false,
        .non_convergent => error.InvalidQuery,
    };
}

/// Resolves Compound leaves before the exact GJK overlap test.  This is the
/// public form for query callers: a Compound containing mesh or heightfield
/// leaves is rejected instead of accidentally being treated as one convex
/// support hull.
pub fn convexOverlapsResolved(a: shapes.Shape, transform_a: geometry.Transform3, b: shapes.Shape, transform_b: geometry.Transform3, assets: *const store.Store, compound_leaves: []shapes.CompoundLeaf, status: *fp.MathStatus) (shapes.Error || Error)!bool {
    if (!(try isResolvedConvex(a, assets, compound_leaves, status)) or !(try isResolvedConvex(b, assets, compound_leaves, status))) return error.UnsupportedShape;
    var pair = gjk.ShapePairContext{ .shape_a = a, .shape_b = b, .assets = assets, .transform_a = transform_a, .transform_b = transform_b };
    const result = try gjk.intersectShapes(&pair, transform_b.position.sub(transform_a.position, status), status);
    return switch (result.status) {
        .intersecting => true,
        .separated => false,
        .non_convergent => error.InvalidQuery,
    };
}

/// Fixed 32-iteration conservative advancement for a convex caster against a
/// convex target.  The returned status makes non-convergence observable; no
/// approximate final witness is ever reported as a collision.
pub fn convexShapeCast(caster: shapes.Shape, caster_start: geometry.Transform3, delta: geometry.Vec3, target: shapes.Shape, target_transform: geometry.Transform3, assets: *const store.Store, workspace: ShapeCastWorkspace, status: *fp.MathStatus) (shapes.Error || Error)!ShapeCastHit {
    if (!(try isResolvedConvex(caster, assets, workspace.compound_leaves, status)) or !(try isResolvedConvex(target, assets, workspace.compound_leaves, status))) return error.UnsupportedShape;
    var fraction = fp.Fp.zero;
    var iteration: u8 = 0;
    while (iteration < 32) : (iteration += 1) {
        var transform = caster_start;
        transform.position = transform.position.add(delta.scale(fraction, status), status);
        var pair = gjk.ShapePairContext{ .shape_a = caster, .shape_b = target, .assets = assets, .transform_a = transform, .transform_b = target_transform };
        const result = try gjk.distanceShapes(&pair, .{ .direction = target_transform.position.sub(transform.position, status) }, status);
        if (result.status == .non_convergent) return .{ .status = .non_convergent, .iterations = iteration + 1 };
        var separating = result.witness_b.sub(result.witness_a, status);
        const unit = separating.normalize(status);
        if (result.status == .intersecting or result.distance.raw <= 1) return .{ .status = .hit, .fraction = fraction, .point = result.witness_a, .normal = if (unit.valid) unit.value else geometry.Vec3.unit_x, .feature = result.feature_b, .iterations = iteration + 1 };
        if (!unit.valid) return .{ .status = .non_convergent, .iterations = iteration + 1 };
        const closing_speed = delta.dot(unit.value, status);
        if (closing_speed.raw <= 0) return .{ .status = .miss, .iterations = iteration + 1 };
        const advance = result.distance.div(closing_speed, status);
        if (advance.raw <= 0) return .{ .status = .non_convergent, .iterations = iteration + 1 };
        fraction = fraction.add(advance, status);
        if (fraction.raw > fp.Fp.one.raw) return .{ .status = .miss, .iterations = iteration + 1 };
    }
    return .{ .status = .non_convergent, .iterations = 32 };
}

/// Fixed 32-iteration conservative advancement for a convex caster against
/// any target shape. Surface leaves use exact triangle GJK distance and
/// Compound targets resolve their stable terminal-child ordering instead of
/// collapsing to a broadphase AABB.
pub fn convexShapeCastSurface(caster: shapes.Shape, caster_start: geometry.Transform3, delta: geometry.Vec3, target: shapes.Shape, target_transform: geometry.Transform3, assets: *const store.Store, workspace: SurfaceCastWorkspace, status: *fp.MathStatus) (shapes.Error || mesh.Error || Error)!ShapeCastHit {
    if (!(try isResolvedConvex(caster, assets, workspace.compound_leaves, status))) return error.UnsupportedShape;
    var fraction = fp.Fp.zero;
    var iteration: u8 = 0;
    while (iteration < 32) : (iteration += 1) {
        var transform = caster_start;
        transform.position = transform.position.add(delta.scale(fraction, status), status);
        const result = (try surfaceDistance(caster, transform, target, target_transform, assets, workspace, status)) orelse return .{ .status = .miss, .iterations = iteration + 1 };
        if (result.status == .non_convergent) return .{ .status = .non_convergent, .iterations = iteration + 1 };
        const separating = result.witness_b.sub(result.witness_a, status);
        // An intersecting GJK simplex commonly has coincident witnesses. Do
        // not normalize that zero vector: zero normalization is an expected
        // geometric classification here, not a negative-square-root math
        // fault. Use the stable transform-center direction as a fallback.
        if (result.status == .intersecting or result.distance.raw <= 1) {
            const normal = castNormal(separating, target_transform.position.sub(transform.position, status), status);
            return .{ .status = .hit, .fraction = fraction, .point = result.witness_a, .normal = normal, .feature = result.feature_b, .iterations = iteration + 1 };
        }
        const unit = separating.normalize(status);
        if (!unit.valid) return .{ .status = .non_convergent, .iterations = iteration + 1 };
        const closing_speed = delta.dot(unit.value, status);
        if (closing_speed.raw <= 0) return .{ .status = .miss, .iterations = iteration + 1 };
        const advance = result.distance.div(closing_speed, status);
        if (advance.raw <= 0) return .{ .status = .non_convergent, .iterations = iteration + 1 };
        fraction = fraction.add(advance, status);
        if (fraction.raw > fp.Fp.one.raw) return .{ .status = .miss, .iterations = iteration + 1 };
    }
    return .{ .status = .non_convergent, .iterations = 32 };
}
fn castNormal(primary: geometry.Vec3, fallback: geometry.Vec3, status: *fp.MathStatus) geometry.Vec3 {
    if (primary.lengthSquared(status).raw > 0) return primary.normalize(status).value;
    if (fallback.lengthSquared(status).raw > 0) return fallback.normalize(status).value;
    return geometry.Vec3.unit_x;
}
fn surfaceDistance(caster: shapes.Shape, caster_transform: geometry.Transform3, target: shapes.Shape, target_transform: geometry.Transform3, assets: *const store.Store, workspace: SurfaceCastWorkspace, status: *fp.MathStatus) (shapes.Error || mesh.Error || Error)!?gjk.Result {
    return switch (target) {
        .compound => blk: {
            const leaves = try shapes.collectCompoundLeaves(target, assets, target_transform, workspace.compound_leaves, status);
            var best: ?gjk.Result = null;
            for (leaves) |leaf| {
                const candidate = (try surfaceDistance(caster, caster_transform, leaf.shape, leaf.transform, assets, workspace, status)) orelse continue;
                if (candidate.status == .non_convergent) break :blk candidate;
                if (best == null or surfaceResultLess(candidate, best.?)) best = candidate;
            }
            break :blk best;
        },
        .triangle_mesh => |asset_shape| blk: {
            const source_id = if (asset_shape.source_id != 0) asset_shape.source_id else asset_shape.asset.index();
            break :blk mesh.convexMeshDistance(caster, assets, caster_transform, try runtime_view.find(assets, source_id), target_transform, workspace.mesh, status) catch |err| switch (err) {
                error.NoSurface => null,
                else => return err,
            };
        },
        .height_field => |asset_shape| blk: {
            const source_id = if (asset_shape.source_id != 0) asset_shape.source_id else asset_shape.asset.index();
            break :blk mesh.convexHeightfieldDistance(caster, assets, caster_transform, try runtime_view.find(assets, source_id), target_transform, workspace.heightfield, status) catch |err| switch (err) {
                error.NoSurface => null,
                else => return err,
            };
        },
        else => blk: {
            if (!(try isResolvedConvex(target, assets, workspace.compound_leaves, status))) return error.UnsupportedShape;
            var pair = gjk.ShapePairContext{ .shape_a = caster, .shape_b = target, .assets = assets, .transform_a = caster_transform, .transform_b = target_transform };
            break :blk try gjk.distanceShapes(&pair, .{ .direction = target_transform.position.sub(caster_transform.position, status) }, status);
        },
    };
}
fn surfaceResultLess(a: gjk.Result, b: gjk.Result) bool {
    if (a.distance.raw != b.distance.raw) return a.distance.raw < b.distance.raw;
    if (a.feature_b != b.feature_b) return a.feature_b < b.feature_b;
    return a.feature_a < b.feature_a;
}

/// Uses the same category/mask/group convention as simulation collision
/// filtering, without body-type suppression because queries are read-only.
pub fn passesFilter(filter: Filter, collider: *const shapes.Collider) bool {
    const group_override = filter.group != 0 and filter.group == collider.group;
    if (group_override) return filter.group > 0;
    return (filter.mask & collider.category) != 0 and (collider.mask & filter.category) != 0 and collider.enabled;
}

/// Ray/segment versus a sphere. A zero-length ray reports fraction zero only
/// when its origin is inside or on the sphere; it never divides by zero.
pub fn raySphere(ray: Ray, center: geometry.Vec3, radius: fp.Fp, status: *fp.MathStatus) ?RayHit {
    if (radius.raw <= 0) return null;
    const offset = ray.origin.sub(center, status);
    const a = ray.delta.dot(ray.delta, status);
    const c = offset.dot(offset, status).sub(radius.mul(radius, status), status);
    if (a.raw == 0) {
        if (c.raw > 0) return null;
        const normal = offset.normalize(status);
        return .{ .fraction = .zero, .point = ray.origin, .normal = if (normal.valid) normal.value else geometry.Vec3.unit_x };
    }
    if (c.raw <= 0) {
        const normal = offset.normalize(status);
        return .{ .fraction = .zero, .point = ray.origin, .normal = if (normal.valid) normal.value else geometry.Vec3.unit_x };
    }
    const b = offset.dot(ray.delta, status);
    const discriminant = b.mul(b, status).sub(a.mul(c, status), status);
    if (discriminant.raw < 0) return null;
    const fraction = b.neg(status).sub(discriminant.sqrt(status), status).div(a, status);
    if (fraction.raw < 0 or fraction.raw > fp.Fp.one.raw) return null;
    const point = ray.origin.add(ray.delta.scale(fraction, status), status);
    const normal = point.sub(center, status).normalize(status);
    return .{ .fraction = fraction, .point = point, .normal = if (normal.valid) normal.value else geometry.Vec3.unit_x };
}

/// Slab ray/segment test for conservative world AABBs. Parallel slabs are
/// explicitly classified and boundary contact is retained.
pub fn rayAabb(ray: Ray, bounds: geometry.Aabb3, status: *fp.MathStatus) ?fp.Fp {
    var enter = fp.Fp.zero;
    var exit = fp.Fp.one;
    const axes = [_][4]fp.Fp{ .{ ray.origin.x, ray.delta.x, bounds.min.x, bounds.max.x }, .{ ray.origin.y, ray.delta.y, bounds.min.y, bounds.max.y }, .{ ray.origin.z, ray.delta.z, bounds.min.z, bounds.max.z } };
    for (axes) |axis| {
        const origin = axis[0];
        const delta = axis[1];
        const min = axis[2];
        const max = axis[3];
        if (delta.raw == 0) {
            if (origin.raw < min.raw or origin.raw > max.raw) return null;
            continue;
        }
        var left = min.sub(origin, status).div(delta, status);
        var right = max.sub(origin, status).div(delta, status);
        if (left.raw > right.raw) {
            const swap = left;
            left = right;
            right = swap;
        }
        if (left.raw > enter.raw) enter = left;
        if (right.raw < exit.raw) exit = right;
        if (enter.raw > exit.raw) return null;
    }
    return if (exit.raw < 0 or enter.raw > fp.Fp.one.raw) null else if (enter.raw < 0) fp.Fp.zero else enter;
}

/// Traverses a baked triangle BVH using caller-provided node storage.  Every
/// leaf triangle is tested exactly; the BVH is only a conservative reject
/// stage.  `hits` is sorted by `(fraction, primitive)` before return.
pub fn rayMesh(ray: Ray, view: runtime_view.View, transform: geometry.Transform3, work: []u32, hits: []MeshRayHit, status: *fp.MathStatus) (runtime_view.Error || Error)![]const MeshRayHit {
    if (view.header.kind != .triangle_mesh or view.nodeCount() == 0 or work.len == 0) return error.InvalidQuery;
    const local_ray = Ray{ .origin = transform.inverseApply(ray.origin, status), .delta = transform.orientation.inverseRotate(ray.delta, status) };
    var work_len: usize = 1;
    work[0] = 0;
    var hit_len: usize = 0;
    while (work_len != 0) {
        const node_index = work[0];
        for (work[0 .. work_len - 1], work[1..work_len]) |*dst, src| dst.* = src;
        work_len -= 1;
        if (node_index >= view.nodeCount()) return error.InvalidQuery;
        const node = try view.node(node_index);
        if (rayAabb(local_ray, node.bounds, status) == null) continue;
        if ((node.flags & baked.BvhNode.leaf_flag) == 0) {
            if (node.count != 0 or node.first + 1 >= view.nodeCount()) return error.InvalidQuery;
            try pushNode(work, &work_len, node.first);
            try pushNode(work, &work_len, node.first + 1);
            continue;
        }
        const end = @as(usize, node.first) + node.count;
        if (end > view.primitiveCount()) return error.InvalidQuery;
        for (node.first..end) |primitive_index| {
            const primitive = try view.primitive(primitive_index);
            const triangle = try view.triangle(primitive);
            const hit = rayTriangle(local_ray, try view.vertex(triangle.a), try view.vertex(triangle.b), try view.vertex(triangle.c), status) orelse continue;
            if (hit_len == hits.len) return error.CapacityExceeded;
            hits[hit_len] = .{ .fraction = hit.fraction, .point = transform.apply(hit.point, status), .normal = transform.orientation.rotate(hit.normal, status), .primitive = primitive };
            hit_len += 1;
        }
    }
    sortMeshHits(hits[0..hit_len]);
    return hits[0..hit_len];
}

/// Counts exact mesh intersections using the identical deterministic BVH walk
/// as `rayMesh`, but deliberately never publishes a partial hit set.  Query
/// orchestration uses this preflight to report the required output capacity.
fn countRayMesh(ray: Ray, view: runtime_view.View, transform: geometry.Transform3, work: []u32, status: *fp.MathStatus) (runtime_view.Error || Error)!usize {
    if (view.header.kind != .triangle_mesh or view.nodeCount() == 0 or work.len == 0) return error.InvalidQuery;
    const local_ray = Ray{ .origin = transform.inverseApply(ray.origin, status), .delta = transform.orientation.inverseRotate(ray.delta, status) };
    var work_len: usize = 1;
    work[0] = 0;
    var count: usize = 0;
    while (work_len != 0) {
        const node_index = work[0];
        for (work[0 .. work_len - 1], work[1..work_len]) |*dst, src| dst.* = src;
        work_len -= 1;
        if (node_index >= view.nodeCount()) return error.InvalidQuery;
        const node = try view.node(node_index);
        if (rayAabb(local_ray, node.bounds, status) == null) continue;
        if ((node.flags & baked.BvhNode.leaf_flag) == 0) {
            if (node.count != 0 or node.first + 1 >= view.nodeCount()) return error.InvalidQuery;
            try pushNode(work, &work_len, node.first);
            try pushNode(work, &work_len, node.first + 1);
            continue;
        }
        const end = @as(usize, node.first) + node.count;
        if (end > view.primitiveCount()) return error.InvalidQuery;
        for (node.first..end) |primitive_index| {
            const primitive = try view.primitive(primitive_index);
            const triangle = try view.triangle(primitive);
            if (rayTriangle(local_ray, try view.vertex(triangle.a), try view.vertex(triangle.b), try view.vertex(triangle.c), status) != null) count += 1;
        }
    }
    return count;
}

/// HeightField ray traversal mirrors mesh traversal but expands only the
/// selected baked tiles.  Hole cells are excluded by `expandHeightTile`; its
/// canonical feature ID is `(cell << 1) | half`.
pub fn rayHeightfield(ray: Ray, view: runtime_view.View, transform: geometry.Transform3, work: []u32, triangle_scratch: []mesh.HeightTriangle, hits: []MeshRayHit, status: *fp.MathStatus) (runtime_view.Error || mesh.Error || Error)![]const MeshRayHit {
    if (view.header.kind != .height_field or view.heightTileNodeCount() == 0 or work.len == 0) return error.InvalidQuery;
    const local_ray = Ray{ .origin = transform.inverseApply(ray.origin, status), .delta = transform.orientation.inverseRotate(ray.delta, status) };
    var work_len: usize = 1;
    work[0] = 0;
    var hit_len: usize = 0;
    while (work_len != 0) {
        const node_index = work[0];
        for (work[0 .. work_len - 1], work[1..work_len]) |*dst, src| dst.* = src;
        work_len -= 1;
        if (node_index >= view.heightTileNodeCount()) return error.InvalidQuery;
        const node = try view.heightTileNode(node_index);
        if (rayAabb(local_ray, node.bounds, status) == null) continue;
        if ((node.flags & baked.BvhNode.leaf_flag) == 0) {
            if (node.count != 0 or node.first + 1 >= view.heightTileNodeCount()) return error.InvalidQuery;
            try pushNode(work, &work_len, node.first);
            try pushNode(work, &work_len, node.first + 1);
            continue;
        }
        const end = @as(usize, node.first) + node.count;
        for (node.first..end) |tile| {
            const triangles = try mesh.expandHeightTile(view, @intCast(tile), triangle_scratch);
            for (triangles) |triangle| {
                const hit = rayTriangle(local_ray, triangle.triangle.a, triangle.triangle.b, triangle.triangle.c, status) orelse continue;
                if (hit_len == hits.len) return error.CapacityExceeded;
                const primitive = (triangle.cell << 1) | triangle.half;
                hits[hit_len] = .{ .fraction = hit.fraction, .point = transform.apply(hit.point, status), .normal = transform.orientation.rotate(hit.normal, status), .primitive = primitive };
                hit_len += 1;
            }
        }
    }
    sortMeshHits(hits[0..hit_len]);
    return hits[0..hit_len];
}

/// Exact HeightField hit count with the same tile BVH and hole expansion as
/// `rayHeightfield`.  It is a preflight only; `triangle_scratch` remains
/// caller-owned and a too-small scratch buffer is still an explicit fault.
fn countRayHeightfield(ray: Ray, view: runtime_view.View, transform: geometry.Transform3, work: []u32, triangle_scratch: []mesh.HeightTriangle, status: *fp.MathStatus) (runtime_view.Error || mesh.Error || Error)!usize {
    if (view.header.kind != .height_field or view.heightTileNodeCount() == 0 or work.len == 0) return error.InvalidQuery;
    const local_ray = Ray{ .origin = transform.inverseApply(ray.origin, status), .delta = transform.orientation.inverseRotate(ray.delta, status) };
    var work_len: usize = 1;
    work[0] = 0;
    var count: usize = 0;
    while (work_len != 0) {
        const node_index = work[0];
        for (work[0 .. work_len - 1], work[1..work_len]) |*dst, src| dst.* = src;
        work_len -= 1;
        if (node_index >= view.heightTileNodeCount()) return error.InvalidQuery;
        const node = try view.heightTileNode(node_index);
        if (rayAabb(local_ray, node.bounds, status) == null) continue;
        if ((node.flags & baked.BvhNode.leaf_flag) == 0) {
            if (node.count != 0 or node.first + 1 >= view.heightTileNodeCount()) return error.InvalidQuery;
            try pushNode(work, &work_len, node.first);
            try pushNode(work, &work_len, node.first + 1);
            continue;
        }
        const end = @as(usize, node.first) + node.count;
        for (node.first..end) |tile| {
            const triangles = try mesh.expandHeightTile(view, @intCast(tile), triangle_scratch);
            for (triangles) |triangle| {
                if (rayTriangle(local_ray, triangle.triangle.a, triangle.triangle.b, triangle.triangle.c, status) != null) count += 1;
            }
        }
    }
    return count;
}

/// Exact closed-convex-hull ray query.  The origin-inside case is published
/// at fraction zero; otherwise every baked surface triangle is tested and
/// the lowest `(fraction, triangle)` result is selected deterministically.
pub fn rayConvexHull(ray: Ray, view: runtime_view.View, transform: geometry.Transform3, status: *fp.MathStatus) (runtime_view.Error || Error)!?MeshRayHit {
    if (view.header.kind != .convex_hull or view.triangleCount() == 0) return error.InvalidQuery;
    const local_ray = Ray{ .origin = transform.inverseApply(ray.origin, status), .delta = transform.orientation.inverseRotate(ray.delta, status) };
    if (try pointInsideConvexHull(local_ray.origin, view, status)) return .{ .fraction = .zero, .point = ray.origin, .normal = geometry.Vec3.unit_x, .primitive = 0 };
    var best: ?MeshRayHit = null;
    for (0..view.triangleCount()) |index| {
        const triangle = try view.triangle(index);
        const hit = rayTriangle(local_ray, try view.vertex(triangle.a), try view.vertex(triangle.b), try view.vertex(triangle.c), status) orelse continue;
        const candidate = MeshRayHit{ .fraction = hit.fraction, .point = transform.apply(hit.point, status), .normal = transform.orientation.rotate(hit.normal, status), .primitive = @intCast(index) };
        if (best == null or candidate.fraction.raw < best.?.fraction.raw or (candidate.fraction.raw == best.?.fraction.raw and candidate.primitive < best.?.primitive)) best = candidate;
    }
    return best;
}

/// Moller--Trumbore evaluated entirely in fixed point.  It treats triangle
/// edges as hits and accepts either winding, while returning a normal opposed
/// to the ray displacement for a stable query-facing convention.
pub fn rayTriangle(ray: Ray, a: geometry.Vec3, b: geometry.Vec3, c: geometry.Vec3, status: *fp.MathStatus) ?RayHit {
    const edge_ab = b.sub(a, status);
    const edge_ac = c.sub(a, status);
    const p = ray.delta.cross(edge_ac, status);
    const determinant = edge_ab.dot(p, status);
    if (determinant.raw == 0) return null;
    const origin_offset = ray.origin.sub(a, status);
    const u = origin_offset.dot(p, status).div(determinant, status);
    if (u.raw < 0 or u.raw > fp.Fp.one.raw) return null;
    const q = origin_offset.cross(edge_ab, status);
    const v = ray.delta.dot(q, status).div(determinant, status);
    if (v.raw < 0 or u.add(v, status).raw > fp.Fp.one.raw) return null;
    const fraction = edge_ac.dot(q, status).div(determinant, status);
    if (fraction.raw < 0 or fraction.raw > fp.Fp.one.raw) return null;
    const outward = edge_ab.cross(edge_ac, status).normalize(status);
    if (!outward.valid) return null;
    const normal = if (outward.value.dot(ray.delta, status).raw > 0) negate(outward.value, status) else outward.value;
    return .{ .fraction = fraction, .point = ray.origin.add(ray.delta.scale(fraction, status), status), .normal = normal };
}

/// Exact segment ray against an oriented box. The ray is transformed into the
/// box's local space; the returned normal is transformed back to world space.
pub fn rayBox(ray: Ray, transform: geometry.Transform3, half_extents: geometry.Vec3, status: *fp.MathStatus) ?RayHit {
    if (half_extents.x.raw <= 0 or half_extents.y.raw <= 0 or half_extents.z.raw <= 0) return null;
    const local = Ray{ .origin = transform.orientation.inverseRotate(ray.origin.sub(transform.position, status), status), .delta = transform.orientation.inverseRotate(ray.delta, status) };
    const fraction = rayAabb(local, .{ .min = negate(half_extents, status), .max = half_extents }, status) orelse return null;
    const point = local.origin.add(local.delta.scale(fraction, status), status);
    const normal = boxNormal(point, half_extents, status);
    return .{ .fraction = fraction, .point = transform.apply(point, status), .normal = transform.orientation.rotate(normal, status) };
}

/// Exact segment ray against a capsule aligned with its local Y axis. The
/// side quadratic and both spherical caps are evaluated, then tied by the
/// same minimum-fraction rule used by all query modes.
pub fn rayCapsule(ray: Ray, transform: geometry.Transform3, radius: fp.Fp, half_height: fp.Fp, status: *fp.MathStatus) ?RayHit {
    if (radius.raw <= 0 or half_height.raw < 0) return null;
    const local = Ray{ .origin = transform.orientation.inverseRotate(ray.origin.sub(transform.position, status), status), .delta = transform.orientation.inverseRotate(ray.delta, status) };
    const closest_y = if (local.origin.y.raw < -half_height.raw) half_height.neg(status) else if (local.origin.y.raw > half_height.raw) half_height else local.origin.y;
    const inside = local.origin.sub(.{ .y = closest_y }, status).lengthSquared(status).raw <= radius.mul(radius, status).raw;
    if (inside) {
        const normal = local.origin.sub(.{ .y = closest_y }, status).normalize(status);
        return .{ .fraction = .zero, .point = ray.origin, .normal = transform.orientation.rotate(if (normal.valid) normal.value else geometry.Vec3.unit_x, status) };
    }
    var best: ?RayHit = null;
    const a = local.delta.x.mul(local.delta.x, status).add(local.delta.z.mul(local.delta.z, status), status);
    const b = local.origin.x.mul(local.delta.x, status).add(local.origin.z.mul(local.delta.z, status), status);
    const c = local.origin.x.mul(local.origin.x, status).add(local.origin.z.mul(local.origin.z, status), status).sub(radius.mul(radius, status), status);
    if (a.raw != 0) {
        const discriminant = b.mul(b, status).sub(a.mul(c, status), status);
        if (discriminant.raw >= 0) {
            const fraction = b.neg(status).sub(discriminant.sqrt(status), status).div(a, status);
            const y = local.origin.y.add(local.delta.y.mul(fraction, status), status);
            if (fraction.raw >= 0 and fraction.raw <= fp.Fp.one.raw and y.raw >= -half_height.raw and y.raw <= half_height.raw) {
                const point = local.origin.add(local.delta.scale(fraction, status), status);
                const normal = (geometry.Vec3{ .x = point.x, .z = point.z }).normalize(status);
                if (normal.valid) best = .{ .fraction = fraction, .point = point, .normal = normal.value };
            }
        }
    }
    for ([_]fp.Fp{ half_height.neg(status), half_height }) |y| {
        const hit = raySphere(local, .{ .y = y }, radius, status) orelse continue;
        if (best == null or hit.fraction.raw < best.?.fraction.raw) best = hit;
    }
    const value = best orelse return null;
    return .{ .fraction = value.fraction, .point = transform.apply(value.point, status), .normal = transform.orientation.rotate(value.normal, status) };
}

/// Sorts candidate hits by the frozen query key, then publishes Any/Closest
/// as the same first element. All reports required count on insufficient
/// output and leaves caller memory untouched in that case.
pub fn publish(mode: Mode, candidates: []Hit, output: []Hit) Error!Publication {
    sortHits(candidates);
    const wanted: usize = switch (mode) {
        .any, .closest => if (candidates.len == 0) 0 else 1,
        .all => candidates.len,
    };
    if (output.len < wanted) return .{ .hits = output[0..0], .required = wanted };
    if (wanted > 0) @memcpy(output[0..wanted], candidates[0..wanted]);
    return .{ .hits = output[0..wanted], .required = wanted };
}
/// Intersects analytic primitives with a count pass before publication.  Thus
/// both caller buffers can be smaller than the full result without partial
/// publication: `required` remains available to the caller.
pub fn rayPrimitives(ray: Ray, filter: Filter, items: []const Item, candidates: []Hit, output: []Hit, mode: Mode, status: *fp.MathStatus) Error!Publication {
    var required: usize = 0;
    for (items) |item| {
        if (!passesFilter(filter, item.collider)) continue;
        if ((try rayShape(ray, item.collider.shape, item.transform, status)) != null) required += 1;
    }
    const wanted: usize = switch (mode) {
        .all => required,
        .any, .closest => if (required == 0) 0 else 1,
    };
    if (output.len < wanted or (mode == .all and candidates.len < required)) return .{ .hits = output[0..0], .required = wanted };
    if (wanted == 0) return .{ .hits = output[0..0], .required = 0 };
    if (mode != .all) {
        var best: ?Hit = null;
        for (items) |item| {
            if (!passesFilter(filter, item.collider)) continue;
            const hit = try rayShape(ray, item.collider.shape, item.transform, status) orelse continue;
            const candidate = Hit{ .fraction = hit.fraction, .collider = item.id, .primitive = 0, .feature = hit.feature, .point = hit.point, .normal = hit.normal };
            if (best == null or candidate.lessThan(best.?)) best = candidate;
        }
        output[0] = best.?;
        return .{ .hits = output[0..1], .required = 1 };
    }
    var count: usize = 0;
    for (items) |item| {
        if (!passesFilter(filter, item.collider)) continue;
        const hit = try rayShape(ray, item.collider.shape, item.transform, status) orelse continue;
        if (count == candidates.len) return error.CapacityExceeded;
        candidates[count] = .{ .fraction = hit.fraction, .collider = item.id, .primitive = 0, .feature = hit.feature, .point = hit.point, .normal = hit.normal };
        count += 1;
    }
    return publish(mode, candidates[0..count], output);
}

/// Full ray query for analytic primitives, immutable assets and Compound
/// leaves.  Mesh hits retain their baked triangle ID; Compound hits retain
/// the complete child path.  Surface tests are exact; BVHs only reject
/// impossible mesh and heightfield leaves.
pub fn rayShapes(ray: Ray, filter: Filter, items: []const Item, assets: *const store.Store, workspace: RayWorkspace, candidates: []Hit, output: []Hit, mode: Mode, status: *fp.MathStatus) (shapes.Error || mesh.Error || Error)!Publication {
    var required: usize = 0;
    for (items) |item| {
        if (!passesFilter(filter, item.collider)) continue;
        if (item.collider.shape == .compound) {
            const leaves = try shapes.collectCompoundLeaves(item.collider.shape, assets, item.transform, workspace.compound_leaves, status);
            for (leaves) |leaf| required += try countRayShape(ray, leaf.shape, leaf.transform, assets, workspace, status);
        } else required += try countRayShape(ray, item.collider.shape, item.transform, assets, workspace, status);
    }
    const wanted: usize = switch (mode) {
        .all => required,
        .any, .closest => if (required == 0) 0 else 1,
    };
    // `candidates` is caller-owned too.  Refuse before the publication pass,
    // so neither it nor `output` receives a truncated result.
    if (output.len < wanted or candidates.len < required) return .{ .hits = output[0..0], .required = wanted };
    if (wanted == 0) return .{ .hits = output[0..0], .required = 0 };
    var count: usize = 0;
    for (items) |item| {
        if (!passesFilter(filter, item.collider)) continue;
        if (item.collider.shape == .compound) {
            const leaves = try shapes.collectCompoundLeaves(item.collider.shape, assets, item.transform, workspace.compound_leaves, status);
            for (leaves) |leaf| try appendRayShape(ray, item.id, leaf.path, leaf.shape, leaf.transform, assets, workspace, candidates, &count, status);
        } else try appendRayShape(ray, item.id, .{}, item.collider.shape, item.transform, assets, workspace, candidates, &count, status);
    }
    return publish(mode, candidates[0..count], output);
}
const ShapeRayHit = struct { fraction: fp.Fp, point: geometry.Vec3, normal: geometry.Vec3, feature: u32 };
fn countRayShape(ray: Ray, shape: shapes.Shape, transform: geometry.Transform3, assets: *const store.Store, workspace: RayWorkspace, status: *fp.MathStatus) (shapes.Error || mesh.Error || Error)!usize {
    return switch (shape) {
        .convex_hull => |asset_shape| blk: {
            const source_id = if (asset_shape.source_id != 0) asset_shape.source_id else asset_shape.asset.index();
            const view = try runtime_view.find(assets, source_id);
            break :blk if ((try rayConvexHull(ray, view, transform, status)) == null) 0 else 1;
        },
        .triangle_mesh => |asset_shape| blk: {
            const source_id = if (asset_shape.source_id != 0) asset_shape.source_id else asset_shape.asset.index();
            break :blk try countRayMesh(ray, try runtime_view.find(assets, source_id), transform, workspace.bvh_nodes, status);
        },
        .height_field => |asset_shape| blk: {
            const source_id = if (asset_shape.source_id != 0) asset_shape.source_id else asset_shape.asset.index();
            break :blk try countRayHeightfield(ray, try runtime_view.find(assets, source_id), transform, workspace.bvh_nodes, workspace.height_triangles, status);
        },
        .compound => error.InvalidQuery,
        else => if ((try rayShape(ray, shape, transform, status)) == null) 0 else 1,
    };
}
fn appendRayShape(ray: Ray, id: ids.ColliderId, path: shapes.ChildPath, shape: shapes.Shape, transform: geometry.Transform3, assets: *const store.Store, workspace: RayWorkspace, candidates: []Hit, count: *usize, status: *fp.MathStatus) (shapes.Error || mesh.Error || Error)!void {
    switch (shape) {
        .convex_hull => |asset_shape| {
            const source_id = if (asset_shape.source_id != 0) asset_shape.source_id else asset_shape.asset.index();
            const view = try runtime_view.find(assets, source_id);
            const hull_hit = try rayConvexHull(ray, view, transform, status) orelse return;
            if (count.* == candidates.len) return error.CapacityExceeded;
            candidates[count.*] = .{ .fraction = hull_hit.fraction, .collider = id, .child_path = path, .primitive = hull_hit.primitive, .feature = hull_hit.primitive, .point = hull_hit.point, .normal = hull_hit.normal };
            count.* += 1;
        },
        .triangle_mesh => |asset_shape| {
            const source_id = if (asset_shape.source_id != 0) asset_shape.source_id else asset_shape.asset.index();
            const view = try runtime_view.find(assets, source_id);
            const mesh_hits = try rayMesh(ray, view, transform, workspace.bvh_nodes, workspace.mesh_hits, status);
            for (mesh_hits) |mesh_hit| {
                if (count.* == candidates.len) return error.CapacityExceeded;
                candidates[count.*] = .{ .fraction = mesh_hit.fraction, .collider = id, .child_path = path, .primitive = mesh_hit.primitive, .feature = mesh_hit.primitive, .point = mesh_hit.point, .normal = mesh_hit.normal };
                count.* += 1;
            }
        },
        .height_field => |asset_shape| {
            const source_id = if (asset_shape.source_id != 0) asset_shape.source_id else asset_shape.asset.index();
            const view = try runtime_view.find(assets, source_id);
            const height_hits = try rayHeightfield(ray, view, transform, workspace.bvh_nodes, workspace.height_triangles, workspace.mesh_hits, status);
            for (height_hits) |height_hit| {
                if (count.* == candidates.len) return error.CapacityExceeded;
                candidates[count.*] = .{ .fraction = height_hit.fraction, .collider = id, .child_path = path, .primitive = height_hit.primitive, .feature = height_hit.primitive, .point = height_hit.point, .normal = height_hit.normal };
                count.* += 1;
            }
        },
        .compound => return error.InvalidQuery,
        else => {
            const hit = try rayShape(ray, shape, transform, status) orelse return;
            if (count.* == candidates.len) return error.CapacityExceeded;
            candidates[count.*] = .{ .fraction = hit.fraction, .collider = id, .child_path = path, .primitive = 0, .feature = hit.feature, .point = hit.point, .normal = hit.normal };
            count.* += 1;
        },
    }
}
fn rayShape(ray: Ray, shape: shapes.Shape, transform: geometry.Transform3, status: *fp.MathStatus) Error!?ShapeRayHit {
    return switch (shape) {
        .sphere => |sphere| blk: {
            const hit = raySphere(ray, transform.position, sphere.radius, status) orelse break :blk null;
            break :blk .{ .fraction = hit.fraction, .point = hit.point, .normal = hit.normal, .feature = 0 };
        },
        .box => |box| blk: {
            const hit = rayBox(ray, transform, box.half_extents, status) orelse break :blk null;
            break :blk .{ .fraction = hit.fraction, .point = hit.point, .normal = hit.normal, .feature = 0 };
        },
        .capsule => |capsule| blk: {
            const hit = rayCapsule(ray, transform, capsule.radius, capsule.half_height, status) orelse break :blk null;
            break :blk .{ .fraction = hit.fraction, .point = hit.point, .normal = hit.normal, .feature = 0 };
        },
        else => error.UnsupportedShape,
    };
}
fn sortHits(items: []Hit) void {
    var i: usize = 1;
    while (i < items.len) : (i += 1) {
        const value = items[i];
        var j = i;
        while (j > 0 and value.lessThan(items[j - 1])) : (j -= 1) items[j] = items[j - 1];
        items[j] = value;
    }
}
fn pushNode(work: []u32, len: *usize, value: u32) Error!void {
    if (len.* == work.len) return error.CapacityExceeded;
    var at = len.*;
    while (at > 0 and value < work[at - 1]) : (at -= 1) work[at] = work[at - 1];
    work[at] = value;
    len.* += 1;
}
fn pointInAabb(point: geometry.Vec3, bounds: geometry.Aabb3) bool {
    return point.x.raw >= bounds.min.x.raw and point.x.raw <= bounds.max.x.raw and point.y.raw >= bounds.min.y.raw and point.y.raw <= bounds.max.y.raw and point.z.raw >= bounds.min.z.raw and point.z.raw <= bounds.max.z.raw;
}
/// Fixed-point closest-point classification has exact closed-set semantics:
/// only a zero squared separation is a surface hit, with no tolerance that
/// could cause a near miss to become platform- or mode-dependent.
fn pointOnTriangle(point: geometry.Vec3, triangle: mesh.Triangle, status: *fp.MathStatus) bool {
    const closest = analytic.closestPointTriangle(point, .{ .a = triangle.a, .b = triangle.b, .c = triangle.c }, status);
    return point.sub(closest, status).lengthSquared(status).raw == 0;
}
fn sortMeshHits(items: []MeshRayHit) void {
    var i: usize = 1;
    while (i < items.len) : (i += 1) {
        const value = items[i];
        var j = i;
        while (j > 0 and (value.fraction.raw < items[j - 1].fraction.raw or (value.fraction.raw == items[j - 1].fraction.raw and value.primitive < items[j - 1].primitive))) : (j -= 1) items[j] = items[j - 1];
        items[j] = value;
    }
}
fn pointInsideConvexHull(point: geometry.Vec3, view: runtime_view.View, status: *fp.MathStatus) runtime_view.Error!bool {
    if (view.vertexCount() == 0 or view.faceCount() == 0) return false;
    var center = geometry.Vec3.zero;
    for (0..view.vertexCount()) |index| center = center.add(try view.vertex(index), status);
    center = center.scale(fp.Fp.one.div(fp.Fp.fromInt(@intCast(view.vertexCount())), status), status);
    for (0..view.faceCount()) |face_index| {
        const face = try view.face(face_index);
        if (face.half_edge_count < 3) return false;
        const edge_a = try view.halfEdge(face.first_half_edge);
        const edge_b = try view.halfEdge(edge_a.next);
        const edge_c = try view.halfEdge(edge_b.next);
        const a = try view.vertex(edge_a.origin);
        const b = try view.vertex(edge_b.origin);
        const c = try view.vertex(edge_c.origin);
        var normal = b.sub(a, status).cross(c.sub(a, status), status);
        if (normal.dot(center.sub(a, status), status).raw > 0) normal = negate(normal, status);
        if (normal.dot(point.sub(a, status), status).raw > 0) return false;
    }
    return true;
}
fn negate(value: geometry.Vec3, status: *fp.MathStatus) geometry.Vec3 {
    return .{ .x = value.x.neg(status), .y = value.y.neg(status), .z = value.z.neg(status) };
}
fn boxNormal(point: geometry.Vec3, half_extents: geometry.Vec3, status: *fp.MathStatus) geometry.Vec3 {
    const dx = half_extents.x.sub(point.x.abs(status), status);
    const dy = half_extents.y.sub(point.y.abs(status), status);
    const dz = half_extents.z.sub(point.z.abs(status), status);
    if (dx.raw <= dy.raw and dx.raw <= dz.raw) return .{ .x = if (point.x.raw >= 0) .one else fp.Fp.one.neg(status) };
    if (dy.raw <= dz.raw) return .{ .y = if (point.y.raw >= 0) .one else fp.Fp.one.neg(status) };
    return .{ .z = if (point.z.raw >= 0) .one else fp.Fp.one.neg(status) };
}
fn isConvex(shape: shapes.Shape) bool {
    return switch (shape) {
        .sphere, .box, .capsule, .convex_hull, .compound => true,
        else => false,
    };
}
fn isResolvedConvex(shape: shapes.Shape, assets: *const store.Store, leaves: []shapes.CompoundLeaf, status: *fp.MathStatus) shapes.Error!bool {
    if (shape != .compound) return isConvex(shape) and shape != .compound;
    const resolved = try shapes.collectCompoundLeaves(shape, assets, .{}, leaves, status);
    for (resolved) |leaf| if (!isConvex(leaf.shape) or leaf.shape == .compound) return false;
    return true;
}
