//! Deterministic analytic narrow-phase primitives and the common result shape.
const geometry = @import("../math/geometry.zig");
const fp = @import("../math/fp.zig");
const ids = @import("../core/ids.zig");

pub const Feature = union(enum) { sphere: void, capsule_side: void, capsule_end: u1, box_face: u3, box_edge: u4, box_vertex: u3, primitive: u32 };
pub const NarrowResult = struct {
    normal: geometry.Vec3, // A -> B
    separation: fp.Fp, // positive separate, zero touching, negative overlap
    witness_a: geometry.Vec3,
    witness_b: geometry.Vec3,
    feature_a: Feature,
    feature_b: Feature,
};
pub const Segment = struct { a: geometry.Vec3, b: geometry.Vec3 };
pub const Triangle = struct { a: geometry.Vec3, b: geometry.Vec3, c: geometry.Vec3 };
pub const Sphere = struct { center: geometry.Vec3, radius: fp.Fp };
pub const Capsule = struct { segment: Segment, radius: fp.Fp };
pub const Obb = struct { center: geometry.Vec3, half_extents: geometry.Vec3, orientation: geometry.Quat };
pub const QueryShape = union(enum) { sphere: Sphere, capsule: Capsule, box: Obb };
pub const Closest = struct { a: geometry.Vec3, b: geometry.Vec3, ta: fp.Fp, tb: fp.Fp };
pub const SegmentTriangleClosest = struct { segment: geometry.Vec3, triangle: geometry.Vec3, t: fp.Fp };

pub fn closestPointSegment(point: geometry.Vec3, segment: Segment, status: *fp.MathStatus) struct { point: geometry.Vec3, t: fp.Fp } {
    const axis = segment.b.sub(segment.a, status);
    const length2 = axis.dot(axis, status);
    if (length2.raw <= 0) return .{ .point = segment.a, .t = fp.Fp.zero };
    const t = clamp01(point.sub(segment.a, status).dot(axis, status).div(length2, status));
    return .{ .point = segment.a.add(axis.scale(t, status), status), .t = t };
}
/// Closest point on a triangle with Ericson region tests in fixed branch order.
pub fn closestPointTriangle(point: geometry.Vec3, triangle: Triangle, status: *fp.MathStatus) geometry.Vec3 {
    const ab = triangle.b.sub(triangle.a, status);
    const ac = triangle.c.sub(triangle.a, status);
    const ap = point.sub(triangle.a, status);
    const d1 = ab.dot(ap, status);
    const d2 = ac.dot(ap, status);
    if (d1.raw <= 0 and d2.raw <= 0) return triangle.a;
    const bp = point.sub(triangle.b, status);
    const d3 = ab.dot(bp, status);
    const d4 = ac.dot(bp, status);
    if (d3.raw >= 0 and d4.raw <= d3.raw) return triangle.b;
    const vc = d1.mul(d4, status).sub(d3.mul(d2, status), status);
    if (vc.raw <= 0 and d1.raw >= 0 and d3.raw <= 0) return triangle.a.add(ab.scale(d1.div(d1.sub(d3, status), status), status), status);
    const cp = point.sub(triangle.c, status);
    const d5 = ab.dot(cp, status);
    const d6 = ac.dot(cp, status);
    if (d6.raw >= 0 and d5.raw <= d6.raw) return triangle.c;
    const vb = d5.mul(d2, status).sub(d1.mul(d6, status), status);
    if (vb.raw <= 0 and d2.raw >= 0 and d6.raw <= 0) return triangle.a.add(ac.scale(d2.div(d2.sub(d6, status), status), status), status);
    const va = d3.mul(d6, status).sub(d5.mul(d4, status), status);
    if (va.raw <= 0 and d4.sub(d3, status).raw >= 0 and d5.sub(d6, status).raw >= 0) {
        const bc = triangle.c.sub(triangle.b, status);
        const numerator = d4.sub(d3, status);
        const denominator = numerator.add(d5.sub(d6, status), status);
        return triangle.b.add(bc.scale(numerator.div(denominator, status), status), status);
    }
    const sum = va.add(vb, status).add(vc, status);
    if (sum.raw == 0) return triangle.a;
    return triangle.a.add(ab.scale(vb.div(sum, status), status), status).add(ac.scale(vc.div(sum, status), status), status);
}
/// Ericson's segment-segment closest points with fixed branch ordering.
pub fn closestSegments(a: Segment, b: Segment, status: *fp.MathStatus) Closest {
    const u = a.b.sub(a.a, status);
    const v = b.b.sub(b.a, status);
    const w = a.a.sub(b.a, status);
    const aa = u.dot(u, status);
    const bb = u.dot(v, status);
    const cc = v.dot(v, status);
    const dd = u.dot(w, status);
    const ee = v.dot(w, status);
    const denom = aa.mul(cc, status).sub(bb.mul(bb, status), status);
    var s = fp.Fp.zero;
    var t = fp.Fp.zero;
    if (aa.raw <= 0 and cc.raw <= 0) {} else if (aa.raw <= 0) {
        t = clamp01(ee.div(cc, status));
    } else if (cc.raw <= 0) {
        s = clamp01(dd.neg(status).div(aa, status));
    } else {
        s = if (denom.raw <= 0) fp.Fp.zero else clamp01(bb.mul(ee, status).sub(cc.mul(dd, status), status).div(denom, status));
        t = bb.mul(s, status).add(ee, status).div(cc, status);
        if (t.raw < 0) {
            t = fp.Fp.zero;
            s = clamp01(dd.neg(status).div(aa, status));
        } else if (t.raw > fp.Fp.one.raw) {
            t = fp.Fp.one;
            s = clamp01(bb.sub(dd, status).div(aa, status));
        }
    }
    return .{ .a = a.a.add(u.scale(s, status), status), .b = b.a.add(v.scale(t, status), status), .ta = s, .tb = t };
}
pub fn sphereSphere(a: Sphere, b: Sphere, relative_velocity: geometry.Vec3, a_id: ids.ColliderId, b_id: ids.ColliderId, status: *fp.MathStatus) NarrowResult {
    const delta = b.center.sub(a.center, status);
    const distance = delta.lengthSquared(status).sqrt(status);
    const normal = fallbackNormal(delta, relative_velocity, a_id, b_id, status);
    const radii = a.radius.add(b.radius, status);
    return .{ .normal = normal, .separation = distance.sub(radii, status), .witness_a = a.center.add(normal.scale(a.radius, status), status), .witness_b = b.center.sub(normal.scale(b.radius, status), status), .feature_a = .{ .sphere = {} }, .feature_b = .{ .sphere = {} } };
}
pub fn sphereCapsule(a: Sphere, b: Capsule, relative_velocity: geometry.Vec3, a_id: ids.ColliderId, b_id: ids.ColliderId, status: *fp.MathStatus) NarrowResult {
    const closest = closestPointSegment(a.center, b.segment, status);
    const delta = closest.point.sub(a.center, status);
    const distance = delta.lengthSquared(status).sqrt(status);
    const normal = fallbackNormal(delta, relative_velocity, a_id, b_id, status);
    return .{ .normal = normal, .separation = distance.sub(a.radius.add(b.radius, status), status), .witness_a = a.center.add(normal.scale(a.radius, status), status), .witness_b = closest.point.sub(normal.scale(b.radius, status), status), .feature_a = .{ .sphere = {} }, .feature_b = capsuleFeature(closest.t) };
}
pub fn capsuleCapsule(a: Capsule, b: Capsule, relative_velocity: geometry.Vec3, a_id: ids.ColliderId, b_id: ids.ColliderId, status: *fp.MathStatus) NarrowResult {
    const closest = closestSegments(a.segment, b.segment, status);
    const delta = closest.b.sub(closest.a, status);
    const distance = delta.lengthSquared(status).sqrt(status);
    const normal = fallbackNormal(delta, relative_velocity, a_id, b_id, status);
    return .{ .normal = normal, .separation = distance.sub(a.radius.add(b.radius, status), status), .witness_a = closest.a.add(normal.scale(a.radius, status), status), .witness_b = closest.b.sub(normal.scale(b.radius, status), status), .feature_a = capsuleFeature(closest.ta), .feature_b = capsuleFeature(closest.tb) };
}
pub fn sphereBox(a: Sphere, b: Obb, relative_velocity: geometry.Vec3, a_id: ids.ColliderId, b_id: ids.ColliderId, status: *fp.MathStatus) NarrowResult {
    const local = b.orientation.inverseRotate(a.center.sub(b.center, status), status);
    var clamped = geometry.Vec3{ .x = clampAxis(local.x, b.half_extents.x), .y = clampAxis(local.y, b.half_extents.y), .z = clampAxis(local.z, b.half_extents.z) };
    if (clamped.x.raw == local.x.raw and clamped.y.raw == local.y.raw and clamped.z.raw == local.z.raw) {
        const dx = b.half_extents.x.sub(local.x.abs(status), status);
        const dy = b.half_extents.y.sub(local.y.abs(status), status);
        const dz = b.half_extents.z.sub(local.z.abs(status), status);
        if (dx.raw <= dy.raw and dx.raw <= dz.raw) clamped.x = if (local.x.raw >= 0) b.half_extents.x else b.half_extents.x.neg(status) else if (dy.raw <= dz.raw) clamped.y = if (local.y.raw >= 0) b.half_extents.y else b.half_extents.y.neg(status) else clamped.z = if (local.z.raw >= 0) b.half_extents.z else b.half_extents.z.neg(status);
    }
    const closest = b.orientation.rotate(clamped, status).add(b.center, status);
    const delta = closest.sub(a.center, status);
    const normal = fallbackNormal(delta, relative_velocity, a_id, b_id, status);
    return .{ .normal = normal, .separation = delta.lengthSquared(status).sqrt(status).sub(a.radius, status), .witness_a = a.center.add(normal.scale(a.radius, status), status), .witness_b = closest, .feature_a = .{ .sphere = {} }, .feature_b = .{ .box_face = faceFeature(b.orientation.inverseRotate(normal, status)) } };
}
/// Exact closest-point Sphere–Triangle query for Task 11 primitive contacts.
/// The triangle stays a one-sided feature only at the manifold layer; this
/// primitive itself is deliberately two-sided and deterministic.
pub fn sphereTriangle(a: Sphere, b: Triangle, triangle_id: u32, relative_velocity: geometry.Vec3, a_id: ids.ColliderId, b_id: ids.ColliderId, status: *fp.MathStatus) NarrowResult {
    const closest = closestPointTriangle(a.center, b, status);
    const delta = closest.sub(a.center, status);
    var normal = fallbackNormal(delta, relative_velocity, a_id, b_id, status);
    if (delta.lengthSquared(status).raw == 0) {
        const face = b.b.sub(b.a, status).cross(b.c.sub(b.a, status), status).normalize(status);
        if (face.valid) normal = face.value;
    }
    return .{ .normal = normal, .separation = delta.lengthSquared(status).sqrt(status).sub(a.radius, status), .witness_a = a.center.add(normal.scale(a.radius, status), status), .witness_b = closest, .feature_a = .{ .sphere = {} }, .feature_b = .{ .primitive = triangle_id } };
}
/// Fixed-order closest query between a segment and a triangle. A segment that
/// pierces the triangle plane returns the exact shared point before the six
/// endpoint/edge candidates are considered.
pub fn closestSegmentTriangle(segment: Segment, triangle: Triangle, status: *fp.MathStatus) SegmentTriangleClosest {
    const normal = triangle.b.sub(triangle.a, status).cross(triangle.c.sub(triangle.a, status), status);
    const direction = segment.b.sub(segment.a, status);
    const denominator = normal.dot(direction, status);
    if (denominator.raw != 0) {
        const t = normal.dot(triangle.a.sub(segment.a, status), status).div(denominator, status);
        if (t.raw >= 0 and t.raw <= fp.Fp.one.raw) {
            const point = segment.a.add(direction.scale(t, status), status);
            if (pointInsideTrianglePlane(point, triangle, normal, status)) return .{ .segment = point, .triangle = point, .t = t };
        }
    }
    var best = SegmentTriangleClosest{ .segment = segment.a, .triangle = closestPointTriangle(segment.a, triangle, status), .t = fp.Fp.zero };
    considerSegmentTriangle(&best, .{ .segment = segment.b, .triangle = closestPointTriangle(segment.b, triangle, status), .t = fp.Fp.one }, status);
    const edges = [_]Segment{ .{ .a = triangle.a, .b = triangle.b }, .{ .a = triangle.b, .b = triangle.c }, .{ .a = triangle.c, .b = triangle.a } };
    for (edges) |edge| {
        const closest = closestSegments(segment, edge, status);
        considerSegmentTriangle(&best, .{ .segment = closest.a, .triangle = closest.b, .t = closest.ta }, status);
    }
    return best;
}
fn pointInsideTrianglePlane(point: geometry.Vec3, triangle: Triangle, normal: geometry.Vec3, status: *fp.MathStatus) bool {
    const values = [_]fp.Fp{ triangle.b.sub(triangle.a, status).cross(point.sub(triangle.a, status), status).dot(normal, status), triangle.c.sub(triangle.b, status).cross(point.sub(triangle.b, status), status).dot(normal, status), triangle.a.sub(triangle.c, status).cross(point.sub(triangle.c, status), status).dot(normal, status) };
    var positive = true;
    var negative = true;
    for (values) |value| {
        positive = positive and value.raw >= 0;
        negative = negative and value.raw <= 0;
    }
    return positive or negative;
}
fn considerSegmentTriangle(best: *SegmentTriangleClosest, candidate: SegmentTriangleClosest, status: *fp.MathStatus) void {
    const current_distance = best.segment.sub(best.triangle, status).lengthSquared(status);
    const next_distance = candidate.segment.sub(candidate.triangle, status).lengthSquared(status);
    if (next_distance.raw < current_distance.raw) best.* = candidate;
}
pub fn capsuleTriangle(a: Capsule, b: Triangle, triangle_id: u32, relative_velocity: geometry.Vec3, a_id: ids.ColliderId, b_id: ids.ColliderId, status: *fp.MathStatus) NarrowResult {
    const closest = closestSegmentTriangle(a.segment, b, status);
    const delta = closest.triangle.sub(closest.segment, status);
    var normal = fallbackNormal(delta, relative_velocity, a_id, b_id, status);
    if (delta.lengthSquared(status).raw == 0) {
        const face = b.b.sub(b.a, status).cross(b.c.sub(b.a, status), status).normalize(status);
        if (face.valid) normal = face.value;
    }
    return .{ .normal = normal, .separation = delta.lengthSquared(status).sqrt(status).sub(a.radius, status), .witness_a = closest.segment.add(normal.scale(a.radius, status), status), .witness_b = closest.triangle, .feature_a = capsuleFeature(closest.t), .feature_b = .{ .primitive = triangle_id } };
}
pub fn capsuleBox(a: Capsule, b: Obb, relative_velocity: geometry.Vec3, a_id: ids.ColliderId, b_id: ids.ColliderId, status: *fp.MathStatus) NarrowResult {
    const local = Segment{ .a = b.orientation.inverseRotate(a.segment.a.sub(b.center, status), status), .b = b.orientation.inverseRotate(a.segment.b.sub(b.center, status), status) };
    const closest = closestSegmentBox(local, b.half_extents, status);
    const point_a = b.orientation.rotate(closest.a, status).add(b.center, status);
    const point_b = b.orientation.rotate(closest.b, status).add(b.center, status);
    const delta = point_b.sub(point_a, status);
    const normal = fallbackNormal(delta, relative_velocity, a_id, b_id, status);
    return .{ .normal = normal, .separation = delta.lengthSquared(status).sqrt(status).sub(a.radius, status), .witness_a = point_a.add(normal.scale(a.radius, status), status), .witness_b = point_b, .feature_a = capsuleFeature(closest.t), .feature_b = .{ .box_face = 0 } };
}
pub fn boxBox(a: Obb, b: Obb, relative_velocity: geometry.Vec3, a_id: ids.ColliderId, b_id: ids.ColliderId, status: *fp.MathStatus) NarrowResult {
    const axes_a = obbAxes(a.orientation, status);
    const axes_b = obbAxes(b.orientation, status);
    const delta = b.center.sub(a.center, status);
    var best_sep = fp.Fp.min;
    var best_axis = fallbackNormal(delta, relative_velocity, a_id, b_id, status);
    var found = false;
    for (axes_a) |axis| testSatAxis(a, b, delta, axis, &best_sep, &best_axis, &found, status);
    for (axes_b) |axis| testSatAxis(a, b, delta, axis, &best_sep, &best_axis, &found, status);
    for (axes_a) |left| for (axes_b) |right| {
        const cross = left.cross(right, status);
        const normalized = cross.normalize(status);
        if (normalized.valid) testSatAxis(a, b, delta, normalized.value, &best_sep, &best_axis, &found, status);
    };
    const normal = if (found) best_axis else fallbackNormal(delta, relative_velocity, a_id, b_id, status);
    return .{ .normal = normal, .separation = best_sep, .witness_a = obbSupport(a, normal, status), .witness_b = obbSupport(b, negate(normal, status), status), .feature_a = .{ .box_face = faceFeature(a.orientation.inverseRotate(normal, status)) }, .feature_b = .{ .box_face = faceFeature(b.orientation.inverseRotate(negate(normal, status), status)) } };
}
/// Dispatches supported analytic pairs while preserving the caller's A/B
/// ordering. Unsupported pairs are deliberately left to Task 10 GJK.
pub fn collide(a: QueryShape, b: QueryShape, relative_velocity: geometry.Vec3, a_id: ids.ColliderId, b_id: ids.ColliderId, status: *fp.MathStatus) ?NarrowResult {
    return switch (a) {
        .sphere => |av| switch (b) {
            .sphere => |bv| sphereSphere(av, bv, relative_velocity, a_id, b_id, status),
            .capsule => |bv| sphereCapsule(av, bv, relative_velocity, a_id, b_id, status),
            .box => |bv| sphereBox(av, bv, relative_velocity, a_id, b_id, status),
        },
        .capsule => |av| switch (b) {
            .sphere => |bv| swapResult(sphereCapsule(bv, av, negate(relative_velocity, status), b_id, a_id, status), status),
            .capsule => |bv| capsuleCapsule(av, bv, relative_velocity, a_id, b_id, status),
            .box => |bv| capsuleBox(av, bv, relative_velocity, a_id, b_id, status),
        },
        .box => |av| switch (b) {
            .sphere => |bv| swapResult(sphereBox(bv, av, negate(relative_velocity, status), b_id, a_id, status), status),
            .capsule => |bv| swapResult(capsuleBox(bv, av, negate(relative_velocity, status), b_id, a_id, status), status),
            .box => |bv| boxBox(av, bv, relative_velocity, a_id, b_id, status),
        },
    };
}
pub fn swapResult(value: NarrowResult, status: *fp.MathStatus) NarrowResult {
    return .{ .normal = negate(value.normal, status), .separation = value.separation, .witness_a = value.witness_b, .witness_b = value.witness_a, .feature_a = value.feature_b, .feature_b = value.feature_a };
}
fn testSatAxis(a: Obb, b: Obb, delta: geometry.Vec3, axis: geometry.Vec3, best_sep: *fp.Fp, best_axis: *geometry.Vec3, found: *bool, status: *fp.MathStatus) void {
    const projection = delta.dot(axis, status);
    const distance = projection.abs(status);
    const radius = obbRadius(a, axis, status).add(obbRadius(b, axis, status), status);
    const separation = distance.sub(radius, status);
    if (!found.* or separation.raw > best_sep.raw) {
        best_sep.* = separation;
        best_axis.* = if (projection.raw >= 0) axis else negate(axis, status);
        found.* = true;
    }
}
fn obbAxes(q: geometry.Quat, status: *fp.MathStatus) [3]geometry.Vec3 {
    return .{ q.rotate(geometry.Vec3.unit_x, status), q.rotate(geometry.Vec3.unit_y, status), q.rotate(geometry.Vec3.unit_z, status) };
}
fn obbRadius(box: Obb, axis: geometry.Vec3, status: *fp.MathStatus) fp.Fp {
    const axes = obbAxes(box.orientation, status);
    return axes[0].dot(axis, status).abs(status).mul(box.half_extents.x, status).add(axes[1].dot(axis, status).abs(status).mul(box.half_extents.y, status), status).add(axes[2].dot(axis, status).abs(status).mul(box.half_extents.z, status), status);
}
fn obbSupport(box: Obb, direction: geometry.Vec3, status: *fp.MathStatus) geometry.Vec3 {
    const local = box.orientation.inverseRotate(direction, status);
    const point = geometry.Vec3{ .x = if (local.x.raw >= 0) box.half_extents.x else box.half_extents.x.neg(status), .y = if (local.y.raw >= 0) box.half_extents.y else box.half_extents.y.neg(status), .z = if (local.z.raw >= 0) box.half_extents.z else box.half_extents.z.neg(status) };
    return box.orientation.rotate(point, status).add(box.center, status);
}
fn faceFeature(normal: geometry.Vec3) u3 {
    const ax = @abs(normal.x.raw);
    const ay = @abs(normal.y.raw);
    const az = @abs(normal.z.raw);
    const axis: u3 = if (ay > ax and ay >= az) 1 else if (az > ax and az > ay) 2 else 0;
    const component = switch (axis) {
        0 => normal.x.raw,
        1 => normal.y.raw,
        else => normal.z.raw,
    };
    return axis * 2 + @intFromBool(component >= 0);
}
fn negate(value: geometry.Vec3, status: *fp.MathStatus) geometry.Vec3 {
    return .{ .x = value.x.neg(status), .y = value.y.neg(status), .z = value.z.neg(status) };
}
const SegmentBoxClosest = struct { a: geometry.Vec3, b: geometry.Vec3, t: fp.Fp };
fn closestSegmentBox(segment: Segment, half: geometry.Vec3, status: *fp.MathStatus) SegmentBoxClosest {
    var best = SegmentBoxClosest{ .a = segment.a, .b = clampToHalf(segment.a, half), .t = fp.Fp.zero };
    var best_d2 = best.a.sub(best.b, status).lengthSquared(status);
    const end_b = SegmentBoxClosest{ .a = segment.b, .b = clampToHalf(segment.b, half), .t = fp.Fp.one };
    const end_d2 = end_b.a.sub(end_b.b, status).lengthSquared(status);
    if (end_d2.raw < best_d2.raw) {
        best = end_b;
        best_d2 = end_d2;
    }
    const corners = boxCorners(half, status);
    const edges = [_][2]u4{ .{ 0, 1 }, .{ 0, 2 }, .{ 0, 4 }, .{ 1, 3 }, .{ 1, 5 }, .{ 2, 3 }, .{ 2, 6 }, .{ 3, 7 }, .{ 4, 5 }, .{ 4, 6 }, .{ 5, 7 }, .{ 6, 7 } };
    for (edges) |edge| {
        const candidate = closestSegments(segment, .{ .a = corners[edge[0]], .b = corners[edge[1]] }, status);
        const d2 = candidate.a.sub(candidate.b, status).lengthSquared(status);
        if (d2.raw < best_d2.raw) {
            best = .{ .a = candidate.a, .b = candidate.b, .t = candidate.ta };
            best_d2 = d2;
        }
    }
    return best;
}
fn clampToHalf(point: geometry.Vec3, half: geometry.Vec3) geometry.Vec3 {
    return .{ .x = clampAxis(point.x, half.x), .y = clampAxis(point.y, half.y), .z = clampAxis(point.z, half.z) };
}
fn boxCorners(half: geometry.Vec3, status: *fp.MathStatus) [8]geometry.Vec3 {
    const nx = half.x.neg(status);
    const ny = half.y.neg(status);
    const nz = half.z.neg(status);
    return .{ .{ .x = nx, .y = ny, .z = nz }, .{ .x = half.x, .y = ny, .z = nz }, .{ .x = nx, .y = half.y, .z = nz }, .{ .x = half.x, .y = half.y, .z = nz }, .{ .x = nx, .y = ny, .z = half.z }, .{ .x = half.x, .y = ny, .z = half.z }, .{ .x = nx, .y = half.y, .z = half.z }, .{ .x = half.x, .y = half.y, .z = half.z } };
}
fn clampAxis(value: fp.Fp, half: fp.Fp) fp.Fp {
    return if (value.raw < -half.raw) .{ .raw = -half.raw } else if (value.raw > half.raw) half else value;
}
fn clamp01(value: fp.Fp) fp.Fp {
    return if (value.raw < 0) fp.Fp.zero else if (value.raw > fp.Fp.one.raw) fp.Fp.one else value;
}
fn capsuleFeature(t: fp.Fp) Feature {
    return if (t.raw == 0) .{ .capsule_end = 0 } else if (t.raw == fp.Fp.one.raw) .{ .capsule_end = 1 } else .{ .capsule_side = {} };
}
fn fallbackNormal(delta: geometry.Vec3, velocity: geometry.Vec3, a: ids.ColliderId, b: ids.ColliderId, status: *fp.MathStatus) geometry.Vec3 {
    const delta_n = delta.normalize(status);
    if (delta_n.valid) return delta_n.value;
    const velocity_n = velocity.normalize(status);
    if (velocity_n.valid) return velocity_n.value;
    return if (a.value < b.value) geometry.Vec3.unit_x else geometry.Vec3{ .x = fp.Fp.one.neg(status) };
}
