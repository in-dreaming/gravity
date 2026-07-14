//! Fixed-iteration Minkowski support and GJK intersection foundation.
const std = @import("std");
const geometry = @import("../math/geometry.zig");
const fp = @import("../math/fp.zig");
const shapes = @import("shapes.zig");
const store = @import("../assets/store.zig");
const analytic = @import("analytic.zig");
const ids = @import("../core/ids.zig");

pub const SupportVertex = struct { point: geometry.Vec3, witness_a: geometry.Vec3, witness_b: geometry.Vec3, feature_a: u32, feature_b: u32 };
pub const SupportContext = struct { ptr: *const anyopaque, call: *const fn (*const anyopaque, geometry.Vec3, *fp.MathStatus) SupportVertex };
/// Runtime adapter for Task 07 shapes.  Failure is captured during a support
/// call and re-emitted by `intersectShapes`; the GJK callback itself stays
/// allocation-free and does not erase malformed-asset errors.
pub const ShapePairContext = struct {
    shape_a: shapes.Shape,
    shape_b: shapes.Shape,
    assets: *const store.Store,
    transform_a: geometry.Transform3 = .{},
    transform_b: geometry.Transform3 = .{},
    failure: ?shapes.Error = null,
};
pub fn shapeSupportContext(pair: *ShapePairContext) SupportContext {
    return .{ .ptr = pair, .call = shapePairSupport };
}
pub fn intersectShapes(pair: *ShapePairContext, initial_direction: geometry.Vec3, status: *fp.MathStatus) shapes.Error!Result {
    pair.failure = null;
    const result = intersect(shapeSupportContext(pair), initial_direction, status);
    if (pair.failure) |err| return err;
    return result;
}
pub fn distanceShapes(pair: *ShapePairContext, seed: Seed, status: *fp.MathStatus) shapes.Error!Result {
    pair.failure = null;
    const result = distance(shapeSupportContext(pair), seed.direction, status);
    if (pair.failure) |err| return err;
    return result;
}
fn shapePairSupport(raw: *const anyopaque, direction: geometry.Vec3, status: *fp.MathStatus) SupportVertex {
    const pair: *ShapePairContext = @ptrCast(@alignCast(@constCast(raw)));
    const local_a = pair.transform_a.orientation.inverseRotate(direction, status);
    const reverse = negate(direction, status);
    const local_b = pair.transform_b.orientation.inverseRotate(reverse, status);
    const support_a = shapes.support(pair.shape_a, pair.assets, local_a, status) catch |err| {
        pair.failure = err;
        return .{ .point = geometry.Vec3.zero, .witness_a = geometry.Vec3.zero, .witness_b = geometry.Vec3.zero, .feature_a = 0, .feature_b = 0 };
    };
    const support_b = shapes.support(pair.shape_b, pair.assets, local_b, status) catch |err| {
        pair.failure = err;
        return .{ .point = geometry.Vec3.zero, .witness_a = geometry.Vec3.zero, .witness_b = geometry.Vec3.zero, .feature_a = 0, .feature_b = 0 };
    };
    const witness_a = pair.transform_a.apply(support_a.point, status);
    const witness_b = pair.transform_b.apply(support_b.point, status);
    return .{ .point = witness_a.sub(witness_b, status), .witness_a = witness_a, .witness_b = witness_b, .feature_a = shapeFeatureKey(support_a.feature), .feature_b = shapeFeatureKey(support_b.feature) };
}
fn shapeFeatureKey(feature: shapes.Feature) u32 {
    return switch (feature) {
        .vertex => |value| value,
        .edge => |value| value | 0x4000_0000,
        .face => |value| value | 0x8000_0000,
        .primitive => |value| value | 0xc000_0000,
    };
}
pub const Status = enum { separated, intersecting, non_convergent };
/// A cacheable starting direction. It is derived only from deterministic query
/// output and remains valid only while the caller's shape cache keys match.
pub const Seed = struct { direction: geometry.Vec3 = geometry.Vec3.unit_x };
pub const Result = struct {
    status: Status,
    iterations: u8,
    simplex: [4]SupportVertex,
    simplex_len: u8,
    direction: geometry.Vec3,
    distance: fp.Fp = fp.Fp.zero,
    witness_a: geometry.Vec3 = geometry.Vec3.zero,
    witness_b: geometry.Vec3 = geometry.Vec3.zero,
    feature_a: u32 = 0,
    feature_b: u32 = 0,
};
pub fn seedFromResult(result: Result) Seed {
    const direction = result.direction;
    return .{ .direction = if (direction.x.raw == 0 and direction.y.raw == 0 and direction.z.raw == 0) geometry.Vec3.unit_x else direction };
}
pub const EpaFace = struct { vertices: [3]u16, normal: geometry.Vec3, distance: fp.Fp, active: bool = true };
pub const EpaStatus = enum { converged, non_convergent, capacity_exceeded, invalid_simplex };
pub const EpaResult = struct {
    status: EpaStatus,
    iterations: u8,
    normal: geometry.Vec3 = geometry.Vec3.unit_x,
    depth: fp.Fp = fp.Fp.zero,
    witness_a: geometry.Vec3 = geometry.Vec3.zero,
    witness_b: geometry.Vec3 = geometry.Vec3.zero,
    feature_a: u32 = 0,
    feature_b: u32 = 0,
};
/// All EPA storage is supplied by the caller, keeping collision queries
/// allocation-free and making capacity failure observable to the caller.
pub const EpaWorkspace = struct {
    vertices: []SupportVertex,
    faces: []EpaFace,
    visible: []bool,
    horizon: []HorizonEdge,
};
pub const EpaPool = struct {
    faces: []EpaFace,
    len: usize = 0,
    pub const Error = error{CapacityExceeded};
    pub fn append(self: *EpaPool, face: EpaFace) Error!void {
        for (self.faces[0..self.len], 0..) |prior, i| if (!prior.active) {
            self.faces[i] = face;
            return;
        };
        if (self.len == self.faces.len or self.len >= 256) return error.CapacityExceeded;
        self.faces[self.len] = face;
        self.len += 1;
    }
    pub fn active(self: *const EpaPool) []const EpaFace {
        return self.faces[0..self.len];
    }
    pub fn closest(self: *const EpaPool) ?usize {
        return closestEpaFace(self.active());
    }
    pub fn inactiveCount(self: *const EpaPool) usize {
        var count: usize = 0;
        for (self.faces[0..self.len]) |face| {
            if (!face.active) count += 1;
        }
        return count;
    }
};
pub const HorizonEdge = struct { from: u16, to: u16 };
pub const ContactPoint = struct {
    point_a: geometry.Vec3,
    point_b: geometry.Vec3,
    /// Stable A-to-B normal for this primitive witness. Patch reduction keeps
    /// the selected witness intact so a surface manifold need not infer a
    /// direction from coincident points.
    normal: geometry.Vec3 = geometry.Vec3.unit_x,
    separation: fp.Fp,
    feature_a: u32,
    feature_b: u32,
    path_a: shapes.ChildPath = .{},
    path_b: shapes.ChildPath = .{},
};
pub const ContactPatch = struct {
    points: [4]ContactPoint = undefined,
    len: u8 = 0,
    pub const Error = error{CapacityExceeded};
    pub fn append(self: *ContactPatch, point: ContactPoint) Error!void {
        if (@as(usize, self.len) == self.points.len) return error.CapacityExceeded;
        self.points[self.len] = point;
        self.len += 1;
    }
};
pub const ClipVertex = struct { point: geometry.Vec3, feature: u32 };
pub const ClipError = error{CapacityExceeded};
pub const FaceError = shapes.Error || ClipError || error{InvalidFace};
pub const Face = struct { normal: geometry.Vec3, offset: fp.Fp, feature: u32, vertices: []const ClipVertex };
pub const ManifoldWorkspace = struct {
    epa: EpaWorkspace,
    reference: []ClipVertex,
    incident: []ClipVertex,
    scratch_a: []ClipVertex,
    scratch_b: []ClipVertex,
    contacts: []ContactPoint,
};
pub const ConvexResult = struct {
    gjk: Result,
    epa: ?EpaResult = null,
    patch: ContactPatch = .{},
    path_a: shapes.ChildPath = .{},
    path_b: shapes.ChildPath = .{},
};

/// Sutherland-Hodgman clipping against the inside half-space
/// `dot(normal, point) <= offset`.  Boundary points are retained and every
/// generated edge point gets the lesser endpoint feature as its stable key.
pub fn clipPolygonAgainstPlane(input: []const ClipVertex, normal: geometry.Vec3, offset: fp.Fp, output: []ClipVertex, status: *fp.MathStatus) ClipError![]const ClipVertex {
    if (input.len == 0) return output[0..0];
    var count: usize = 0;
    var previous = input[input.len - 1];
    var previous_distance = normal.dot(previous.point, status).sub(offset, status);
    for (input) |current| {
        const current_distance = normal.dot(current.point, status).sub(offset, status);
        const previous_inside = previous_distance.raw <= 0;
        const current_inside = current_distance.raw <= 0;
        if (previous_inside != current_inside) {
            if (count == output.len) return error.CapacityExceeded;
            const denominator = previous_distance.sub(current_distance, status);
            const t = previous_distance.div(denominator, status);
            const delta = current.point.sub(previous.point, status);
            output[count] = .{ .point = previous.point.add(delta.scale(t, status), status), .feature = @min(previous.feature, current.feature) };
            count += 1;
        }
        if (current_inside) {
            if (count == output.len) return error.CapacityExceeded;
            output[count] = current;
            count += 1;
        }
        previous = current;
        previous_distance = current_distance;
    }
    return output[0..count];
}

/// Turns an already side-clipped incident polygon into contacts on reference
/// shape A.  `separation <= 0` is penetration/touching in the common normal
/// convention, and the reference witness is the orthogonal face projection.
pub fn contactsFromIncident(reference_normal: geometry.Vec3, reference_offset: fp.Fp, reference_feature: u32, incident: []const ClipVertex, output: []ContactPoint, status: *fp.MathStatus) ClipError![]const ContactPoint {
    var count: usize = 0;
    for (incident) |vertex| {
        const separation = reference_normal.dot(vertex.point, status).sub(reference_offset, status);
        if (separation.raw > 0) continue;
        if (count == output.len) return error.CapacityExceeded;
        output[count] = .{ .point_a = vertex.point.sub(reference_normal.scale(separation, status), status), .point_b = vertex.point, .separation = separation, .feature_a = reference_feature, .feature_b = vertex.feature };
        count += 1;
    }
    return output[0..count];
}

/// Extracts the face most aligned with a world-space direction.  Box faces
/// are procedural; hull faces follow their baked, outward half-edge winding.
pub fn referenceFace(shape: shapes.Shape, assets: *const store.Store, transform: geometry.Transform3, direction: geometry.Vec3, output: []ClipVertex, status: *fp.MathStatus) FaceError!Face {
    return switch (shape) {
        .box => |box| boxReferenceFace(box, transform, direction, output, status),
        .convex_hull => hullReferenceFace(shape, assets, transform, direction, output, status),
        else => error.InvalidFace,
    };
}
fn boxReferenceFace(box: shapes.Box, transform: geometry.Transform3, direction: geometry.Vec3, output: []ClipVertex, status: *fp.MathStatus) FaceError!Face {
    if (output.len < 4) return error.CapacityExceeded;
    const local = transform.orientation.inverseRotate(direction, status);
    const ax = @abs(local.x.raw);
    const ay = @abs(local.y.raw);
    const az = @abs(local.z.raw);
    const axis: u2 = if (ax >= ay and ax >= az) 0 else if (ay >= az) 1 else 2;
    const sign = switch (axis) {
        0 => local.x.raw >= 0,
        1 => local.y.raw >= 0,
        else => local.z.raw >= 0,
    };
    const sx = if (sign) box.half_extents.x else box.half_extents.x.neg(status);
    const sy = if (sign) box.half_extents.y else box.half_extents.y.neg(status);
    const sz = if (sign) box.half_extents.z else box.half_extents.z.neg(status);
    const x0 = box.half_extents.x.neg(status);
    const y0 = box.half_extents.y.neg(status);
    const z0 = box.half_extents.z.neg(status);
    const vertices = switch (axis) {
        0 => [_]geometry.Vec3{ .{ .x = sx, .y = y0, .z = z0 }, .{ .x = sx, .y = box.half_extents.y, .z = z0 }, .{ .x = sx, .y = box.half_extents.y, .z = box.half_extents.z }, .{ .x = sx, .y = y0, .z = box.half_extents.z } },
        1 => [_]geometry.Vec3{ .{ .x = x0, .y = sy, .z = z0 }, .{ .x = x0, .y = sy, .z = box.half_extents.z }, .{ .x = box.half_extents.x, .y = sy, .z = box.half_extents.z }, .{ .x = box.half_extents.x, .y = sy, .z = z0 } },
        else => [_]geometry.Vec3{ .{ .x = x0, .y = y0, .z = sz }, .{ .x = box.half_extents.x, .y = y0, .z = sz }, .{ .x = box.half_extents.x, .y = box.half_extents.y, .z = sz }, .{ .x = x0, .y = box.half_extents.y, .z = sz } },
    };
    const local_normal = switch (axis) {
        0 => geometry.Vec3{ .x = if (sign) fp.Fp.one else fp.Fp.one.neg(status) },
        1 => geometry.Vec3{ .y = if (sign) fp.Fp.one else fp.Fp.one.neg(status) },
        else => geometry.Vec3{ .z = if (sign) fp.Fp.one else fp.Fp.one.neg(status) },
    };
    const normal = transform.orientation.rotate(local_normal, status);
    const feature = shapeFeatureKey(.{ .face = @as(u32, axis) + (if (sign) @as(u32, 0) else 3) });
    for (vertices, 0..) |vertex, i| output[i] = .{ .point = transform.apply(vertex, status), .feature = feature };
    return .{ .normal = normal, .offset = normal.dot(output[0].point, status), .feature = feature, .vertices = output[0..4] };
}
fn hullReferenceFace(shape: shapes.Shape, assets: *const store.Store, transform: geometry.Transform3, direction: geometry.Vec3, output: []ClipVertex, status: *fp.MathStatus) FaceError!Face {
    var best_index: ?u32 = null;
    var best_normal = geometry.Vec3.zero;
    var best_score = fp.Fp.min;
    var index: u32 = 0;
    while (true) : (index += 1) {
        const hull_face = shapes.hullFace(shape, assets, index) catch |err| switch (err) {
            error.EndOfInput => break,
            else => return err,
        };
        if (hull_face.half_edge_count < 3) return error.InvalidFace;
        const normal = try hullFaceNormal(shape, assets, transform, hull_face, status);
        const score = normal.dot(direction, status);
        if (best_index == null or score.raw > best_score.raw) {
            best_index = index;
            best_normal = normal;
            best_score = score;
        }
    }
    const selected = best_index orelse return error.InvalidFace;
    const hull_face = try shapes.hullFace(shape, assets, selected);
    if (hull_face.half_edge_count > output.len) return error.CapacityExceeded;
    var edge_index = hull_face.first_half_edge;
    var count: usize = 0;
    while (count < hull_face.half_edge_count) : (count += 1) {
        const edge = try shapes.hullHalfEdge(shape, assets, edge_index);
        output[count] = .{ .point = transform.apply(try shapes.vertex(shape, assets, edge.origin, status), status), .feature = shapeFeatureKey(.{ .vertex = edge.origin }) };
        edge_index = edge.next;
    }
    return .{ .normal = best_normal, .offset = best_normal.dot(output[0].point, status), .feature = shapeFeatureKey(.{ .face = selected }), .vertices = output[0..count] };
}
fn hullFaceNormal(shape: shapes.Shape, assets: *const store.Store, transform: geometry.Transform3, face: @import("../geometry/baked.zig").HullFace, status: *fp.MathStatus) FaceError!geometry.Vec3 {
    const first = try shapes.hullHalfEdge(shape, assets, face.first_half_edge);
    const second = try shapes.hullHalfEdge(shape, assets, first.next);
    const third = try shapes.hullHalfEdge(shape, assets, second.next);
    const a = transform.apply(try shapes.vertex(shape, assets, first.origin, status), status);
    const b = transform.apply(try shapes.vertex(shape, assets, second.origin, status), status);
    const c = transform.apply(try shapes.vertex(shape, assets, third.origin, status), status);
    const normal = b.sub(a, status).cross(c.sub(a, status), status).normalize(status);
    if (!normal.valid) return error.InvalidFace;
    return normal.value;
}

/// Clips an incident face against every reference side plane, emits contacts,
/// and reduces them to the stable four-point patch.
pub fn clipFacePair(reference: Face, incident: Face, scratch_a: []ClipVertex, scratch_b: []ClipVertex, contacts: []ContactPoint, status: *fp.MathStatus) ClipError!ContactPatch {
    if (incident.vertices.len > scratch_a.len) return error.CapacityExceeded;
    @memcpy(scratch_a[0..incident.vertices.len], incident.vertices);
    var current: []const ClipVertex = scratch_a[0..incident.vertices.len];
    var alternate = scratch_b;
    for (reference.vertices, 0..) |edge_start, i| {
        const edge_end = reference.vertices[(i + 1) % reference.vertices.len];
        const side_normal = edge_end.point.sub(edge_start.point, status).cross(reference.normal, status);
        const clipped = try clipPolygonAgainstPlane(current, side_normal, side_normal.dot(edge_start.point, status), alternate, status);
        current = clipped;
        alternate = if (alternate.ptr == scratch_a.ptr) scratch_b else scratch_a;
        if (current.len == 0) return .{};
    }
    const candidates = try contactsFromIncident(reference.normal, reference.offset, reference.feature, current, contacts, status);
    return reducePatch(candidates, status);
}

/// Full runtime convex query. Face-bearing shapes use reference/incident
/// clipping; smooth shapes retain the EPA witness as a single contact. EPA
/// failures remain explicit in the result rather than publishing an estimate.
pub fn collideShapes(pair: *ShapePairContext, initial_direction: geometry.Vec3, workspace: ManifoldWorkspace, status: *fp.MathStatus) FaceError!ConvexResult {
    return collideShapesAt(pair, initial_direction, workspace, status, 0);
}
fn collideShapesAt(pair: *ShapePairContext, initial_direction: geometry.Vec3, workspace: ManifoldWorkspace, status: *fp.MathStatus, depth: u8) FaceError!ConvexResult {
    if (depth >= 8) return error.CompoundDepth;
    if (pair.shape_a == .compound or pair.shape_b == .compound) return collideCompound(pair, initial_direction, workspace, status, depth);
    if (smoothAnalytic(pair, status)) |result| return result;
    var intersection = try intersectShapes(pair, initial_direction, status);
    if (intersection.status == .separated) intersection = try distanceShapes(pair, .{ .direction = initial_direction }, status);
    var result = ConvexResult{ .gjk = intersection };
    if (intersection.status != .intersecting or intersection.simplex_len != 4) return result;
    pair.failure = null;
    const penetration = epa(shapeSupportContext(pair), intersection.simplex[0..intersection.simplex_len], workspace.epa, status);
    if (pair.failure) |err| return err;
    result.epa = penetration;
    if (penetration.status != .converged) return result;
    const reference = referenceFace(pair.shape_a, pair.assets, pair.transform_a, penetration.normal, workspace.reference, status) catch |err| switch (err) {
        error.InvalidFace => {
            try result.patch.append(.{ .point_a = penetration.witness_a, .point_b = penetration.witness_b, .separation = penetration.depth.neg(status), .feature_a = penetration.feature_a, .feature_b = penetration.feature_b });
            return result;
        },
        else => return err,
    };
    const incident = referenceFace(pair.shape_b, pair.assets, pair.transform_b, negate(penetration.normal, status), workspace.incident, status) catch |err| switch (err) {
        error.InvalidFace => {
            try result.patch.append(.{ .point_a = penetration.witness_a, .point_b = penetration.witness_b, .separation = penetration.depth.neg(status), .feature_a = penetration.feature_a, .feature_b = penetration.feature_b });
            return result;
        },
        else => return err,
    };
    result.patch = try clipFacePair(reference, incident, workspace.scratch_a, workspace.scratch_b, workspace.contacts, status);
    if (result.patch.len == 0) try result.patch.append(.{ .point_a = penetration.witness_a, .point_b = penetration.witness_b, .separation = penetration.depth.neg(status), .feature_a = penetration.feature_a, .feature_b = penetration.feature_b });
    return result;
}
fn smoothAnalytic(pair: *ShapePairContext, status: *fp.MathStatus) ?ConvexResult {
    if (pair.shape_a == .box and pair.shape_b == .box) return null;
    const a = smoothQueryShape(pair.shape_a, pair.transform_a, status) orelse return null;
    const b = smoothQueryShape(pair.shape_b, pair.transform_b, status) orelse return null;
    const value = analytic.collide(a, b, geometry.Vec3.zero, ids.ColliderId.init(0, 0), ids.ColliderId.init(1, 0), status) orelse return null;
    const separation = value.separation;
    const state: Status = if (separation.raw > 0) .separated else .intersecting;
    var simplex: [4]SupportVertex = undefined;
    simplex[0] = .{ .point = value.witness_a.sub(value.witness_b, status), .witness_a = value.witness_a, .witness_b = value.witness_b, .feature_a = 0, .feature_b = 0 };
    var result = ConvexResult{ .gjk = .{ .status = state, .iterations = 0, .simplex = simplex, .simplex_len = 1, .direction = value.normal, .distance = separation.abs(status), .witness_a = value.witness_a, .witness_b = value.witness_b, .feature_a = 0, .feature_b = 0 } };
    if (state == .intersecting) result.patch.append(.{ .point_a = value.witness_a, .point_b = value.witness_b, .separation = separation, .feature_a = 0, .feature_b = 0 }) catch unreachable;
    return result;
}
fn smoothQueryShape(shape: shapes.Shape, transform: geometry.Transform3, status: *fp.MathStatus) ?analytic.QueryShape {
    return switch (shape) {
        .sphere => |value| .{ .sphere = .{ .center = transform.position, .radius = value.radius } },
        .capsule => |value| .{ .capsule = .{ .segment = .{ .a = transform.apply(.{ .y = value.half_height.neg(status) }, status), .b = transform.apply(.{ .y = value.half_height }, status) }, .radius = value.radius } },
        .box => |value| .{ .box = .{ .center = transform.position, .half_extents = value.half_extents, .orientation = transform.orientation } },
        else => null,
    };
}
fn collideCompound(pair: *ShapePairContext, initial_direction: geometry.Vec3, workspace: ManifoldWorkspace, status: *fp.MathStatus, depth: u8) FaceError!ConvexResult {
    const a_count = if (pair.shape_a == .compound) try shapes.compoundChildCount(pair.shape_a, pair.assets) else 1;
    const b_count = if (pair.shape_b == .compound) try shapes.compoundChildCount(pair.shape_b, pair.assets) else 1;
    var best_separated: ?ConvexResult = null;
    var merged: ?ConvexResult = null;
    for (0..a_count) |ai| for (0..b_count) |bi| {
        var child_pair = pair.*;
        var ordinal_a: ?u32 = null;
        var ordinal_b: ?u32 = null;
        if (pair.shape_a == .compound) {
            const child = try shapes.compoundChild(pair.shape_a, pair.assets, ai);
            child_pair.shape_a = child.shape;
            child_pair.transform_a = composeTransform(pair.transform_a, child.transform, status);
            ordinal_a = child.ordinal;
        }
        if (pair.shape_b == .compound) {
            const child = try shapes.compoundChild(pair.shape_b, pair.assets, bi);
            child_pair.shape_b = child.shape;
            child_pair.transform_b = composeTransform(pair.transform_b, child.transform, status);
            ordinal_b = child.ordinal;
        }
        var candidate = try collideShapesAt(&child_pair, initial_direction, workspace, status, depth + 1);
        if (ordinal_a) |ordinal| prependResultPath(&candidate, .a, ordinal) catch return error.CompoundDepth;
        if (ordinal_b) |ordinal| prependResultPath(&candidate, .b, ordinal) catch return error.CompoundDepth;
        if (candidate.gjk.status == .intersecting) {
            if (candidate.epa) |penetration| if (penetration.status != .converged) return candidate;
            if (merged) |*prior| {
                prior.patch = mergePatches(prior.patch, candidate.patch, status);
                if (compoundResultBetter(candidate, prior.*, status)) {
                    const patch = prior.patch;
                    prior.* = candidate;
                    prior.patch = patch;
                }
            } else merged = candidate;
        } else if (best_separated == null or compoundResultBetter(candidate, best_separated.?, status)) best_separated = candidate;
    };
    return merged orelse best_separated orelse error.CompoundDepth;
}
fn composeTransform(parent: geometry.Transform3, child: geometry.Transform3, status: *fp.MathStatus) geometry.Transform3 {
    return .{ .position = parent.apply(child.position, status), .orientation = parent.orientation.mul(child.orientation, status) };
}
const Side = enum { a, b };
fn prependResultPath(result: *ConvexResult, side: Side, ordinal: u32) error{CompoundDepth}!void {
    const path = switch (side) {
        .a => &result.path_a,
        .b => &result.path_b,
    };
    try prependPath(path, ordinal);
    for (result.patch.points[0..result.patch.len]) |*point| try prependPath(switch (side) {
        .a => &point.path_a,
        .b => &point.path_b,
    }, ordinal);
}
fn prependPath(path: *shapes.ChildPath, ordinal: u32) error{CompoundDepth}!void {
    if (path.len == path.values.len) return error.CompoundDepth;
    var i: usize = path.len;
    while (i > 0) : (i -= 1) path.values[i] = path.values[i - 1];
    path.values[0] = ordinal;
    path.len += 1;
}
fn mergePatches(a: ContactPatch, b: ContactPatch, status: *fp.MathStatus) ContactPatch {
    var candidates: [8]ContactPoint = undefined;
    const a_len: usize = a.len;
    const b_len: usize = b.len;
    @memcpy(candidates[0..a_len], a.points[0..a_len]);
    @memcpy(candidates[a_len .. a_len + b_len], b.points[0..b_len]);
    return reducePatch(candidates[0 .. a_len + b_len], status);
}
fn compoundResultBetter(a: ConvexResult, b: ConvexResult, status: *fp.MathStatus) bool {
    const a_intersects = a.gjk.status == .intersecting;
    const b_intersects = b.gjk.status == .intersecting;
    if (a_intersects != b_intersects) return a_intersects;
    if (a_intersects) {
        const a_depth = if (a.epa) |value| value.depth else fp.Fp.zero;
        const b_depth = if (b.epa) |value| value.depth else fp.Fp.zero;
        if (a_depth.raw != b_depth.raw) return a_depth.raw > b_depth.raw;
    } else {
        const aw = a.gjk.witness_a.sub(a.gjk.witness_b, status).lengthSquared(status);
        const bw = b.gjk.witness_a.sub(b.gjk.witness_b, status).lengthSquared(status);
        if (aw.raw != bw.raw) return aw.raw < bw.raw;
    }
    return pathLess(a.path_a, b.path_a) or (!pathLess(b.path_a, a.path_a) and pathLess(a.path_b, b.path_b));
}
fn pathLess(a: shapes.ChildPath, b: shapes.ChildPath) bool {
    const common = @min(a.len, b.len);
    for (a.values[0..common], b.values[0..common]) |left, right| if (left != right) return left < right;
    return a.len < b.len;
}

/// Reduces an arbitrary deterministic clipping candidate list to the protocol
/// maximum of four points: deepest, farthest from it, largest triangle, then
/// largest remaining coverage.  Feature keys resolve every geometric tie.
pub fn reducePatch(candidates: []const ContactPoint, status: *fp.MathStatus) ContactPatch {
    var result = ContactPatch{};
    if (candidates.len == 0) return result;
    var chosen: [4]usize = undefined;
    var chosen_len: usize = 0;
    chosen[chosen_len] = bestCandidate(candidates, &.{}, .deepest, status);
    chosen_len += 1;
    while (chosen_len < 4 and chosen_len < candidates.len) : (chosen_len += 1) {
        const mode: PatchPick = switch (chosen_len) {
            1 => .farthest,
            2 => .area,
            else => .coverage,
        };
        chosen[chosen_len] = bestCandidate(candidates, chosen[0..chosen_len], mode, status);
    }
    for (chosen[0..chosen_len]) |index| result.append(candidates[index]) catch unreachable;
    return result;
}
const PatchPick = enum { deepest, farthest, area, coverage };
fn bestCandidate(candidates: []const ContactPoint, selected: []const usize, mode: PatchPick, status: *fp.MathStatus) usize {
    var best: ?usize = null;
    for (candidates, 0..) |candidate, i| {
        if (containsIndex(selected, i)) continue;
        if (best == null or patchCandidateBetter(candidate, candidates[best.?], selected, candidates, mode, status)) best = i;
    }
    return best.?;
}
fn containsIndex(indices: []const usize, needle: usize) bool {
    for (indices) |index| if (index == needle) return true;
    return false;
}
fn patchCandidateBetter(a: ContactPoint, b: ContactPoint, selected: []const usize, candidates: []const ContactPoint, mode: PatchPick, status: *fp.MathStatus) bool {
    const score_a = patchScore(a, selected, candidates, mode, status);
    const score_b = patchScore(b, selected, candidates, mode, status);
    if (score_a.raw != score_b.raw) return switch (mode) {
        .deepest => score_a.raw < score_b.raw,
        else => score_a.raw > score_b.raw,
    };
    return contactLess(a, b);
}
fn patchScore(candidate: ContactPoint, selected: []const usize, candidates: []const ContactPoint, mode: PatchPick, status: *fp.MathStatus) fp.Fp {
    if (mode == .deepest) return candidate.separation;
    const point = contactMidpoint(candidate, status);
    const first = contactMidpoint(candidates[selected[0]], status);
    if (mode == .farthest) return point.sub(first, status).lengthSquared(status);
    const second = contactMidpoint(candidates[selected[1]], status);
    if (mode == .area) return point.sub(first, status).cross(second.sub(first, status), status).lengthSquared(status);
    var minimum = point.sub(first, status).lengthSquared(status);
    for (selected[1..]) |index| {
        const squared_distance = point.sub(contactMidpoint(candidates[index], status), status).lengthSquared(status);
        if (squared_distance.raw < minimum.raw) minimum = squared_distance;
    }
    return minimum;
}
fn contactMidpoint(point: ContactPoint, status: *fp.MathStatus) geometry.Vec3 {
    return point.point_a.add(point.point_b, status).scale(fp.Fp.fromRatio(1, 2, status), status);
}
fn contactLess(a: ContactPoint, b: ContactPoint) bool {
    if (a.feature_a != b.feature_a) return a.feature_a < b.feature_a;
    if (a.feature_b != b.feature_b) return a.feature_b < b.feature_b;
    if (a.point_a.x.raw != b.point_a.x.raw) return a.point_a.x.raw < b.point_a.x.raw;
    if (a.point_a.y.raw != b.point_a.y.raw) return a.point_a.y.raw < b.point_a.y.raw;
    return a.point_a.z.raw < b.point_a.z.raw;
}
pub fn makeEpaFace(vertices: []const SupportVertex, edge: HorizonEdge, apex: u16, status: *fp.MathStatus) ?EpaFace {
    if (edge.from >= vertices.len or edge.to >= vertices.len or apex >= vertices.len) return null;
    const a = vertices[edge.from].point;
    const b = vertices[edge.to].point;
    const c = vertices[apex].point;
    const normal = b.sub(a, status).cross(c.sub(a, status), status).normalize(status);
    if (!normal.valid) return null;
    var value = normal.value;
    var indices = [3]u16{ edge.from, edge.to, apex };
    var face_distance = value.dot(a, status);
    if (face_distance.raw < 0) {
        value = negate(value, status);
        face_distance = face_distance.neg(status);
        std.mem.swap(u16, &indices[0], &indices[1]);
    }
    return .{ .vertices = indices, .normal = value, .distance = face_distance };
}

/// Expands a tetrahedral GJK simplex into an EPA polytope.  It has exactly 64
/// expansion attempts and never returns a best-effort result on capacity or
/// iteration failure.
pub fn epa(context: SupportContext, simplex: []const SupportVertex, workspace: EpaWorkspace, status: *fp.MathStatus) EpaResult {
    if (simplex.len != 4 or workspace.vertices.len < 4 or workspace.faces.len < 4 or workspace.visible.len < workspace.faces.len) return .{ .status = .invalid_simplex, .iterations = 0 };
    workspace.vertices[0..4].* = simplex[0..4].*;
    var vertex_len: usize = 4;
    var pool = EpaPool{ .faces = workspace.faces };
    const initial = [_][4]u16{ .{ 0, 1, 2, 3 }, .{ 0, 3, 1, 2 }, .{ 0, 2, 3, 1 }, .{ 1, 3, 2, 0 } };
    for (initial) |indices| {
        const face = makeInitialEpaFace(workspace.vertices[0..vertex_len], indices[0], indices[1], indices[2], indices[3], status) orelse return .{ .status = .invalid_simplex, .iterations = 0 };
        pool.append(face) catch return .{ .status = .capacity_exceeded, .iterations = 0 };
    }
    var iteration: u8 = 0;
    while (iteration < 64) : (iteration += 1) {
        const index = pool.closest() orelse return .{ .status = .invalid_simplex, .iterations = iteration };
        const face = pool.faces[index];
        const support = context.call(context.ptr, face.normal, status);
        const advance = support.point.dot(face.normal, status).sub(face.distance, status);
        if (advance.raw <= 1 or containsPoint(workspace.vertices[0..vertex_len], support.point)) return epaFaceResult(.converged, iteration + 1, face, workspace.vertices, status);
        if (vertex_len == workspace.vertices.len or vertex_len >= 130) return .{ .status = .capacity_exceeded, .iterations = iteration + 1 };
        var visible_count: usize = 0;
        for (pool.faces[0..pool.len], 0..) |candidate, i| {
            const offset = support.point.sub(workspace.vertices[candidate.vertices[0]].point, status);
            const visible = candidate.active and candidate.normal.dot(offset, status).raw > 0;
            workspace.visible[i] = visible;
            if (visible) visible_count += 1;
        }
        if (visible_count == 0) return epaFaceResult(.converged, iteration + 1, face, workspace.vertices, status);
        const horizon = collectHorizon(pool.faces[0..pool.len], workspace.visible[0..pool.len], workspace.horizon) catch return .{ .status = .capacity_exceeded, .iterations = iteration + 1 };
        const reusable = pool.inactiveCount() + visible_count;
        const append_capacity = (pool.faces.len - pool.len) + reusable;
        if (horizon.len > append_capacity or pool.len > 256) return .{ .status = .capacity_exceeded, .iterations = iteration + 1 };
        const apex: u16 = @intCast(vertex_len);
        workspace.vertices[vertex_len] = support;
        vertex_len += 1;
        for (workspace.visible[0..pool.len], 0..) |visible, i| {
            if (visible) pool.faces[i].active = false;
        }
        for (horizon) |edge| {
            const next = makeEpaFace(workspace.vertices[0..vertex_len], edge, apex, status) orelse return .{ .status = .invalid_simplex, .iterations = iteration + 1 };
            pool.append(next) catch return .{ .status = .capacity_exceeded, .iterations = iteration + 1 };
        }
    }
    return .{ .status = .non_convergent, .iterations = 64 };
}

/// The first EPA faces must be oriented from the tetrahedron's opposite
/// vertex, not merely by distance from the origin.  This matters when a GJK
/// simplex has an origin-coplanar temporary face despite the full Minkowski
/// polytope having positive penetration in that direction.
fn makeInitialEpaFace(vertices: []const SupportVertex, first: u16, second: u16, third: u16, opposite: u16, status: *fp.MathStatus) ?EpaFace {
    if (first >= vertices.len or second >= vertices.len or third >= vertices.len or opposite >= vertices.len) return null;
    var indices = [3]u16{ first, second, third };
    const a = vertices[first].point;
    const b = vertices[second].point;
    const c = vertices[third].point;
    const other = vertices[opposite].point;
    var normal = b.sub(a, status).cross(c.sub(a, status), status).normalize(status);
    if (!normal.valid) return null;
    if (normal.value.dot(other.sub(a, status), status).raw > 0) {
        normal.value = negate(normal.value, status);
        std.mem.swap(u16, &indices[0], &indices[1]);
    }
    const face_distance = normal.value.dot(a, status);
    // A valid enclosing tetrahedron has non-negative distances; a negative
    // value proves the simplex did not contain the origin as claimed.
    if (face_distance.raw < 0) return null;
    return .{ .vertices = indices, .normal = normal.value, .distance = face_distance };
}

fn containsPoint(vertices: []const SupportVertex, point: geometry.Vec3) bool {
    for (vertices) |vertex| if (vertex.point.x.raw == point.x.raw and vertex.point.y.raw == point.y.raw and vertex.point.z.raw == point.z.raw) return true;
    return false;
}
fn epaFaceResult(result_status: EpaStatus, iterations: u8, face: EpaFace, vertices: []const SupportVertex, status: *fp.MathStatus) EpaResult {
    const a = vertices[face.vertices[0]];
    const b = vertices[face.vertices[1]];
    const c = vertices[face.vertices[2]];
    const projected = face.normal.scale(face.distance, status);
    const weights = triangleWeights(projected, a.point, b.point, c.point, status);
    const witness_a = weightedPoint(a.witness_a, b.witness_a, c.witness_a, weights, status);
    const witness_b = weightedPoint(a.witness_b, b.witness_b, c.witness_b, weights, status);
    const best = if (weights[0].raw >= weights[1].raw and weights[0].raw >= weights[2].raw) a else if (weights[1].raw >= weights[2].raw) b else c;
    return .{ .status = result_status, .iterations = iterations, .normal = face.normal, .depth = face.distance, .witness_a = witness_a, .witness_b = witness_b, .feature_a = best.feature_a, .feature_b = best.feature_b };
}
fn triangleWeights(point: geometry.Vec3, a: geometry.Vec3, b: geometry.Vec3, c: geometry.Vec3, status: *fp.MathStatus) [3]fp.Fp {
    const v0 = b.sub(a, status);
    const v1 = c.sub(a, status);
    const v2 = point.sub(a, status);
    const d00 = v0.dot(v0, status);
    const d01 = v0.dot(v1, status);
    const d11 = v1.dot(v1, status);
    const d20 = v2.dot(v0, status);
    const d21 = v2.dot(v1, status);
    const denominator = d00.mul(d11, status).sub(d01.mul(d01, status), status);
    if (denominator.raw == 0) return .{ fp.Fp.one, fp.Fp.zero, fp.Fp.zero };
    const v = d11.mul(d20, status).sub(d01.mul(d21, status), status).div(denominator, status);
    const w = d00.mul(d21, status).sub(d01.mul(d20, status), status).div(denominator, status);
    return .{ fp.Fp.one.sub(v, status).sub(w, status), v, w };
}
fn weightedPoint(a: geometry.Vec3, b: geometry.Vec3, c: geometry.Vec3, weights: [3]fp.Fp, status: *fp.MathStatus) geometry.Vec3 {
    return a.scale(weights[0], status).add(b.scale(weights[1], status), status).add(c.scale(weights[2], status), status);
}
/// Collects boundary edges from visible faces. An edge with its reverse is an
/// internal horizon edge and is removed; output is insertion-sorted by key.
pub fn collectHorizon(faces: []const EpaFace, visible: []const bool, output: []HorizonEdge) EpaPool.Error![]const HorizonEdge {
    var count: usize = 0;
    for (faces, 0..) |face, i| {
        if (!face.active or !visible[i]) continue;
        const edges = [_]HorizonEdge{ .{ .from = face.vertices[0], .to = face.vertices[1] }, .{ .from = face.vertices[1], .to = face.vertices[2] }, .{ .from = face.vertices[2], .to = face.vertices[0] } };
        for (edges) |edge| {
            var reverse: ?usize = null;
            for (output[0..count], 0..) |prior, j| if (prior.from == edge.to and prior.to == edge.from) {
                reverse = j;
                break;
            };
            if (reverse) |at| {
                std.mem.copyForwards(HorizonEdge, output[at .. count - 1], output[at + 1 .. count]);
                count -= 1;
                continue;
            }
            if (count == output.len) return error.CapacityExceeded;
            output[count] = edge;
            count += 1;
        }
    }
    var i: usize = 1;
    while (i < count) : (i += 1) {
        const value = output[i];
        var j = i;
        while (j > 0 and edgeLess(value, output[j - 1])) : (j -= 1) output[j] = output[j - 1];
        output[j] = value;
    }
    return output[0..count];
}
fn edgeLess(a: HorizonEdge, b: HorizonEdge) bool {
    return a.from < b.from or (a.from == b.from and a.to < b.to);
}

/// Selects the closest active face using the protocol key
/// `(distance, normal components, sorted vertex ids)`. EPA expansion is kept
/// separate so callers can reserve fixed vertex/face buffers before mutation.
pub fn closestEpaFace(faces: []const EpaFace) ?usize {
    var best: ?usize = null;
    for (faces, 0..) |face, i| {
        if (!face.active) continue;
        if (best == null or epaFaceLess(face, faces[best.?])) best = i;
    }
    return best;
}
fn epaFaceLess(a: EpaFace, b: EpaFace) bool {
    if (a.distance.raw != b.distance.raw) return a.distance.raw < b.distance.raw;
    if (a.normal.x.raw != b.normal.x.raw) return a.normal.x.raw < b.normal.x.raw;
    if (a.normal.y.raw != b.normal.y.raw) return a.normal.y.raw < b.normal.y.raw;
    if (a.normal.z.raw != b.normal.z.raw) return a.normal.z.raw < b.normal.z.raw;
    const av = sortedVertices(a.vertices);
    const bv = sortedVertices(b.vertices);
    inline for (av, bv) |left, right| if (left != right) return left < right;
    return false;
}
fn sortedVertices(value: [3]u16) [3]u16 {
    var result = value;
    if (result[0] > result[1]) std.mem.swap(u16, &result[0], &result[1]);
    if (result[1] > result[2]) std.mem.swap(u16, &result[1], &result[2]);
    if (result[0] > result[1]) std.mem.swap(u16, &result[0], &result[1]);
    return result;
}

pub fn intersect(context: SupportContext, initial_direction: geometry.Vec3, status: *fp.MathStatus) Result {
    var simplex: [4]SupportVertex = undefined;
    var len: u8 = 0;
    var direction = initial_direction;
    if (direction.lengthSquared(status).raw == 0) direction = geometry.Vec3.unit_x;
    var iteration: u8 = 0;
    while (iteration < 32) : (iteration += 1) {
        const point = context.call(context.ptr, direction, status);
        if (point.point.dot(direction, status).raw < 0) return .{ .status = .separated, .iterations = iteration + 1, .simplex = simplex, .simplex_len = len, .direction = direction };
        for (simplex[0..len]) |prior| if (prior.point.x.raw == point.point.x.raw and prior.point.y.raw == point.point.y.raw and prior.point.z.raw == point.point.z.raw) return .{ .status = .intersecting, .iterations = iteration + 1, .simplex = simplex, .simplex_len = len, .direction = direction };
        simplex[len] = point;
        len += 1;
        if (reduce(&simplex, &len, &direction, status)) return .{ .status = .intersecting, .iterations = iteration + 1, .simplex = simplex, .simplex_len = len, .direction = direction };
    }
    return .{ .status = .non_convergent, .iterations = 32, .simplex = simplex, .simplex_len = len, .direction = direction };
}

/// Fixed-iteration GJK distance query. The returned witnesses are the
/// barycentric combination of the reduced Minkowski simplex. `intersect`
/// remains the minimal classification path; callers that need a separation
/// magnitude and cache seed should use this entry point.
pub fn distance(context: SupportContext, initial_direction: geometry.Vec3, status: *fp.MathStatus) Result {
    var simplex: [4]SupportVertex = undefined;
    var direction = initial_direction;
    if (direction.lengthSquared(status).raw == 0) direction = geometry.Vec3.unit_x;
    simplex[0] = context.call(context.ptr, direction, status);
    var len: u8 = 1;
    var iteration: u8 = 1;
    var closest = simplexWitness(simplex[0..len], status);
    direction = negate(closest.point, status);
    while (iteration < 32) : (iteration += 1) {
        if (direction.lengthSquared(status).raw <= 1) return distanceResult(.intersecting, iteration, simplex, len, direction, closest, status);
        const candidate = context.call(context.ptr, direction, status);
        const progress = candidate.point.dot(direction, status).sub(closest.point.dot(direction, status), status);
        if (progress.raw <= 1 or containsPoint(simplex[0..len], candidate.point)) return distanceResult(.separated, iteration + 1, simplex, len, direction, closest, status);
        simplex[len] = candidate;
        len += 1;
        if (reduce(&simplex, &len, &direction, status)) return distanceResult(.intersecting, iteration + 1, simplex, len, direction, .{ .point = geometry.Vec3.zero, .witness_a = geometry.Vec3.zero, .witness_b = geometry.Vec3.zero, .feature_a = 0, .feature_b = 0 }, status);
        closest = simplexWitness(simplex[0..len], status);
        direction = negate(closest.point, status);
    }
    return distanceResult(.non_convergent, 32, simplex, len, direction, closest, status);
}
const SimplexWitness = struct { point: geometry.Vec3, witness_a: geometry.Vec3, witness_b: geometry.Vec3, feature_a: u32, feature_b: u32 };
fn simplexWitness(simplex: []const SupportVertex, status: *fp.MathStatus) SimplexWitness {
    if (simplex.len == 1) return .{ .point = simplex[0].point, .witness_a = simplex[0].witness_a, .witness_b = simplex[0].witness_b, .feature_a = simplex[0].feature_a, .feature_b = simplex[0].feature_b };
    var weights: [3]fp.Fp = .{ fp.Fp.zero, fp.Fp.zero, fp.Fp.zero };
    if (simplex.len == 2) {
        const axis = simplex[1].point.sub(simplex[0].point, status);
        const length2 = axis.lengthSquared(status);
        const numerator = negate(simplex[0].point, status).dot(axis, status);
        weights[1] = if (length2.raw == 0) fp.Fp.zero else clamp01(numerator.div(length2, status));
        weights[0] = fp.Fp.one.sub(weights[1], status);
    } else {
        weights = triangleWeights(geometry.Vec3.zero, simplex[0].point, simplex[1].point, simplex[2].point, status);
    }
    var point = simplex[0].point.scale(weights[0], status);
    var witness_a = simplex[0].witness_a.scale(weights[0], status);
    var witness_b = simplex[0].witness_b.scale(weights[0], status);
    var feature = simplex[0];
    var best_weight = weights[0];
    var i: usize = 1;
    while (i < simplex.len) : (i += 1) {
        point = point.add(simplex[i].point.scale(weights[i], status), status);
        witness_a = witness_a.add(simplex[i].witness_a.scale(weights[i], status), status);
        witness_b = witness_b.add(simplex[i].witness_b.scale(weights[i], status), status);
        if (weights[i].raw > best_weight.raw) {
            feature = simplex[i];
            best_weight = weights[i];
        }
    }
    return .{ .point = point, .witness_a = witness_a, .witness_b = witness_b, .feature_a = feature.feature_a, .feature_b = feature.feature_b };
}
fn distanceResult(result_status: Status, iterations: u8, simplex: [4]SupportVertex, len: u8, direction: geometry.Vec3, witness: SimplexWitness, status: *fp.MathStatus) Result {
    return .{ .status = result_status, .iterations = iterations, .simplex = simplex, .simplex_len = len, .direction = direction, .distance = witness.point.lengthSquared(status).sqrt(status), .witness_a = witness.witness_a, .witness_b = witness.witness_b, .feature_a = witness.feature_a, .feature_b = witness.feature_b };
}
fn clamp01(value: fp.Fp) fp.Fp {
    if (value.raw < 0) return fp.Fp.zero;
    if (value.raw > fp.Fp.one.raw) return fp.Fp.one;
    return value;
}
fn reduce(simplex: *[4]SupportVertex, len: *u8, direction: *geometry.Vec3, status: *fp.MathStatus) bool {
    const a = simplex[len.* - 1].point;
    const ao = negate(a, status);
    if (len.* == 1) {
        direction.* = ao;
        return false;
    }
    const b = simplex[len.* - 2].point;
    const ab = b.sub(a, status);
    if (len.* == 2) {
        direction.* = ab.cross(ao, status).cross(ab, status);
        if (direction.lengthSquared(status).raw == 0) direction.* = perpendicular(ab, status);
        return false;
    }
    const c = simplex[len.* - 3].point;
    const ac = c.sub(a, status);
    const abc = ab.cross(ac, status);
    if (len.* == 3) {
        const ac_perp = abc.cross(ac, status);
        if (ac_perp.dot(ao, status).raw > 0) {
            simplex[0] = simplex[len.* - 3];
            simplex[1] = simplex[len.* - 1];
            len.* = 2;
            direction.* = ac.cross(ao, status).cross(ac, status);
            return false;
        }
        const ab_perp = ab.cross(abc, status);
        if (ab_perp.dot(ao, status).raw > 0) {
            simplex[0] = simplex[len.* - 2];
            simplex[1] = simplex[len.* - 1];
            len.* = 2;
            direction.* = ab.cross(ao, status).cross(ab, status);
            return false;
        }
        direction.* = if (abc.dot(ao, status).raw > 0) abc else negate(abc, status);
        return false;
    }
    const d = simplex[len.* - 4].point;
    const ad = d.sub(a, status);
    const faces = [_]geometry.Vec3{ ab.cross(ac, status), ac.cross(ad, status), ad.cross(ab, status) };
    const opposite = [_]geometry.Vec3{ ad, ab, ac };
    for (faces, 0..) |face_value, i| {
        var face = face_value;
        if (face.dot(opposite[i], status).raw > 0) face = negate(face, status);
        if (face.dot(ao, status).raw <= 0) continue;
        const keep = switch (i) {
            0 => [_]u8{ len.* - 3, len.* - 2, len.* - 1 },
            1 => [_]u8{ len.* - 4, len.* - 3, len.* - 1 },
            else => [_]u8{ len.* - 2, len.* - 4, len.* - 1 },
        };
        const next = [_]SupportVertex{ simplex[keep[0]], simplex[keep[1]], simplex[keep[2]] };
        simplex[0] = next[0];
        simplex[1] = next[1];
        simplex[2] = next[2];
        len.* = 3;
        direction.* = face;
        return false;
    }
    return true;
}
fn negate(value: geometry.Vec3, status: *fp.MathStatus) geometry.Vec3 {
    return .{ .x = value.x.neg(status), .y = value.y.neg(status), .z = value.z.neg(status) };
}
fn perpendicular(value: geometry.Vec3, status: *fp.MathStatus) geometry.Vec3 {
    const basis = if (@abs(value.x.raw) <= @abs(value.y.raw) and @abs(value.x.raw) <= @abs(value.z.raw)) geometry.Vec3.unit_x else if (@abs(value.y.raw) <= @abs(value.z.raw)) geometry.Vec3.unit_y else geometry.Vec3.unit_z;
    return value.cross(basis, status);
}
