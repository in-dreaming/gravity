//! Runtime primitive shape views and deterministic collision filtering.
const geometry = @import("../math/geometry.zig");
const fp = @import("../math/fp.zig");
const ids = @import("../core/ids.zig");
const runtime_view = @import("../assets/runtime_view.zig");
const store = @import("../assets/store.zig");
const baked = @import("../geometry/baked.zig");

pub const BodyType = enum { static, dynamic, kinematic };
pub const ShapeKind = enum { sphere, box, capsule, convex_hull, compound, triangle_mesh, height_field };
pub const Feature = union(enum) { vertex: u32, edge: u32, face: u32, primitive: u32 };
pub const ChildPath = struct { values: [8]u32 = [_]u32{0} ** 8, len: u8 = 0 };
pub const Material = struct { friction: fp.Fp = fp.Fp.one, restitution: fp.Fp = fp.Fp.zero };
pub const Sphere = struct { radius: fp.Fp };
pub const Box = struct { half_extents: geometry.Vec3 };
pub const Capsule = struct { radius: fp.Fp, half_height: fp.Fp };
pub const AssetShape = struct {
    /// Stable Task 05 source id. Zero keeps compatibility with the temporary
    /// pre-World AssetId-index lookup used by the first runtime callers.
    source_id: u64 = 0,
    asset: ids.AssetId = ids.AssetId.invalid,
    revision: u32 = 1,
};
pub const Shape = union(ShapeKind) { sphere: Sphere, box: Box, capsule: Capsule, convex_hull: AssetShape, compound: AssetShape, triangle_mesh: AssetShape, height_field: AssetShape };
pub const Collider = struct { body: ids.BodyId, local: geometry.Transform3 = .{}, shape: Shape, material: Material = .{}, category: u32 = 1, mask: u32 = std.math.maxInt(u32), group: i32 = 0, sensor: bool = false, enabled: bool = true, revision: u32 = 1 };
const std = @import("std");
pub const FilterResult = enum { ignore, overlap, contact };
pub const Error = runtime_view.Error || error{ InvalidShape, InvalidBodyShape, UnsupportedShape, CompoundDepth, CapacityExceeded, MissingMass, InvalidMass };
pub const SupportPoint = struct { point: geometry.Vec3, feature: Feature, child_path: ChildPath = .{} };
/// A resolved immutable Compound child. Its transform is relative to the
/// Compound's local frame and can be composed with any caller world transform.
pub const CompoundChildShape = struct { ordinal: u32, shape: Shape, transform: geometry.Transform3 };
/// A terminal child resolved from a possibly nested Compound. `transform` is
/// in the caller's requested frame and `path` is root-to-leaf ordinal order.
pub const CompoundLeaf = struct { shape: Shape, transform: geometry.Transform3, path: ChildPath };
pub const MassProperties = struct {
    mass: fp.Fp,
    center: geometry.Vec3,
    inertia: geometry.SymmetricMat3,
};
pub const MassOverride = MassProperties;
/// Cache identity for derived AABB/support/manifold data. Future mutable shape
/// providers must advance `revision`; immutable baked assets start at one.
pub const ShapeCacheKey = struct { kind: ShapeKind, source_id: u64, revision: u32 };

pub fn validatePrimitive(shape: Shape) error{InvalidShape}!void {
    switch (shape) {
        .sphere => |v| if (v.radius.raw <= 0) return error.InvalidShape,
        .box => |v| if (v.half_extents.x.raw <= 0 or v.half_extents.y.raw <= 0 or v.half_extents.z.raw <= 0) return error.InvalidShape,
        .capsule => |v| if (v.radius.raw <= 0 or v.half_height.raw < 0) return error.InvalidShape,
        else => {},
    }
}
pub fn cacheKey(shape: Shape) ShapeCacheKey {
    return switch (shape) {
        .sphere => .{ .kind = .sphere, .source_id = 0, .revision = 1 },
        .box => .{ .kind = .box, .source_id = 0, .revision = 1 },
        .capsule => .{ .kind = .capsule, .source_id = 0, .revision = 1 },
        inline else => |value, tag| .{ .kind = tag, .source_id = if (value.source_id != 0) value.source_id else value.asset.index(), .revision = value.revision },
    };
}
pub fn cacheValid(prior: ShapeCacheKey, shape: Shape) bool {
    const current = cacheKey(shape);
    return prior.kind == current.kind and prior.source_id == current.source_id and prior.revision == current.revision;
}

/// Validates the frozen body-kind boundary before a collider is published.
/// Dynamic height fields are forbidden; every other immutable asset shape can
/// move rigidly, subject to its independently validated mass properties.
pub fn validateBodyShape(shape: Shape, body_type: BodyType) Error!void {
    try validatePrimitive(shape);
    if (body_type == .dynamic and shape == .height_field) return error.InvalidBodyShape;
}

/// Returns the local-space conservative bounds of any immutable runtime shape.
pub fn localAabb(shape: Shape, assets: *const store.Store, status: *fp.MathStatus) Error!geometry.Aabb3 {
    return localAabbAt(shape, assets, status, 0);
}
fn localAabbAt(shape: Shape, assets: *const store.Store, status: *fp.MathStatus, depth: u8) Error!geometry.Aabb3 {
    try validatePrimitive(shape);
    return switch (shape) {
        .sphere => |value| symmetricBounds(value.radius),
        .box => |value| .{ .min = negate(value.half_extents, status), .max = value.half_extents },
        .capsule => |value| capsuleBounds(value, status),
        .convex_hull => |value| assetVertexBounds(try viewFor(value, assets, .convex_hull)),
        .triangle_mesh => |value| assetVertexBounds(try viewFor(value, assets, .triangle_mesh)),
        .height_field => |value| (try (try viewFor(value, assets, .height_field)).heightTileNode(0)).bounds,
        .compound => |value| compoundBounds(try viewFor(value, assets, .compound), assets, status, depth),
    };
}

/// Applies an arbitrary rigid transform by transforming all eight local AABB
/// corners. This is conservative and exact for the stored fixed-point corners.
pub fn worldAabb(shape: Shape, assets: *const store.Store, transform: geometry.Transform3, status: *fp.MathStatus) Error!geometry.Aabb3 {
    const local = try localAabb(shape, assets, status);
    var first = true;
    var result: geometry.Aabb3 = undefined;
    inline for ([_]fp.Fp{ local.min.x, local.max.x }) |x| inline for ([_]fp.Fp{ local.min.y, local.max.y }) |y| inline for ([_]fp.Fp{ local.min.z, local.max.z }) |z| {
        const point = transform.apply(.{ .x = x, .y = y, .z = z }, status);
        if (first) {
            result = .{ .min = point, .max = point };
            first = false;
        } else extend(&result, point);
    };
    return result;
}

/// Finds the lexicographically stable extreme feature. For asset shapes the
/// canonical baked vertex index is the tie breaker, never source byte order.
pub fn support(shape: Shape, assets: *const store.Store, direction: geometry.Vec3, status: *fp.MathStatus) Error!SupportPoint {
    return supportAt(shape, assets, direction, status, 0);
}
fn supportAt(shape: Shape, assets: *const store.Store, direction: geometry.Vec3, status: *fp.MathStatus, depth: u8) Error!SupportPoint {
    try validatePrimitive(shape);
    return switch (shape) {
        .sphere => |value| blk: {
            const n = direction.normalize(status);
            const unit = if (n.valid) n.value else geometry.Vec3.unit_x;
            break :blk .{ .point = unit.scale(value.radius, status), .feature = .{ .vertex = 0 } };
        },
        .box => |value| .{ .point = .{ .x = if (direction.x.raw < 0) value.half_extents.x.neg(status) else value.half_extents.x, .y = if (direction.y.raw < 0) value.half_extents.y.neg(status) else value.half_extents.y, .z = if (direction.z.raw < 0) value.half_extents.z.neg(status) else value.half_extents.z }, .feature = .{ .vertex = boxVertex(direction) } },
        .capsule => |value| capsuleSupport(value, direction, status),
        .convex_hull => |value| vertexSupport(try viewFor(value, assets, .convex_hull), direction, status),
        .triangle_mesh => |value| vertexSupport(try viewFor(value, assets, .triangle_mesh), direction, status),
        .height_field => |value| heightSupport(try viewFor(value, assets, .height_field), direction, status),
        .compound => |value| compoundSupport(try viewFor(value, assets, .compound), assets, direction, status, depth),
    };
}

fn assetView(shape: AssetShape, assets: *const store.Store) Error!runtime_view.View {
    // Task 05 stores assets by canonical source id. The index fallback keeps
    // the pre-World handle ABI usable until the asset slot table is introduced.
    return runtime_view.find(assets, if (shape.source_id != 0) shape.source_id else shape.asset.index());
}
fn viewFor(shape: AssetShape, assets: *const store.Store, expected: baked.Kind) Error!runtime_view.View {
    const view = try assetView(shape, assets);
    if (view.header.kind != expected) return error.InvalidShape;
    return view;
}
fn symmetricBounds(radius: fp.Fp) geometry.Aabb3 {
    return .{ .min = .{ .x = .{ .raw = -radius.raw }, .y = .{ .raw = -radius.raw }, .z = .{ .raw = -radius.raw } }, .max = .{ .x = radius, .y = radius, .z = radius } };
}
fn negate(value: geometry.Vec3, status: *fp.MathStatus) geometry.Vec3 {
    return .{ .x = value.x.neg(status), .y = value.y.neg(status), .z = value.z.neg(status) };
}
fn capsuleBounds(value: Capsule, status: *fp.MathStatus) geometry.Aabb3 {
    const y = value.half_height.add(value.radius, status);
    return .{ .min = .{ .x = value.radius.neg(status), .y = y.neg(status), .z = value.radius.neg(status) }, .max = .{ .x = value.radius, .y = y, .z = value.radius } };
}
fn extend(bounds: *geometry.Aabb3, point: geometry.Vec3) void {
    bounds.min.x.raw = @min(bounds.min.x.raw, point.x.raw);
    bounds.min.y.raw = @min(bounds.min.y.raw, point.y.raw);
    bounds.min.z.raw = @min(bounds.min.z.raw, point.z.raw);
    bounds.max.x.raw = @max(bounds.max.x.raw, point.x.raw);
    bounds.max.y.raw = @max(bounds.max.y.raw, point.y.raw);
    bounds.max.z.raw = @max(bounds.max.z.raw, point.z.raw);
}
fn assetVertexBounds(view: runtime_view.View) Error!geometry.Aabb3 {
    if (view.vertexCount() == 0) return error.InvalidShape;
    var result = geometry.Aabb3{ .min = try view.vertex(0), .max = try view.vertex(0) };
    var i: usize = 1;
    while (i < view.vertexCount()) : (i += 1) extend(&result, try view.vertex(i));
    return result;
}
fn boxVertex(direction: geometry.Vec3) u32 {
    return @as(u32, @intFromBool(direction.x.raw >= 0)) | (@as(u32, @intFromBool(direction.y.raw >= 0)) << 1) | (@as(u32, @intFromBool(direction.z.raw >= 0)) << 2);
}
fn capsuleSupport(value: Capsule, direction: geometry.Vec3, status: *fp.MathStatus) SupportPoint {
    const n = direction.normalize(status);
    const unit = if (n.valid) n.value else geometry.Vec3.unit_x;
    const y = if (direction.y.raw < 0) value.half_height.neg(status) else value.half_height;
    return .{ .point = unit.scale(value.radius, status).add(.{ .y = y }, status), .feature = .{ .vertex = if (direction.y.raw < 0) 0 else 1 } };
}
fn vertexSupport(view: runtime_view.View, direction: geometry.Vec3, status: *fp.MathStatus) Error!SupportPoint {
    if (view.vertexCount() == 0) return error.InvalidShape;
    var best = try view.vertex(0);
    var best_dot = best.dot(direction, status);
    var best_id: usize = 0;
    var i: usize = 1;
    while (i < view.vertexCount()) : (i += 1) {
        const point = try view.vertex(i);
        const dot = point.dot(direction, status);
        if (dot.raw > best_dot.raw) {
            best = point;
            best_dot = dot;
            best_id = i;
        }
    }
    return .{ .point = best, .feature = .{ .vertex = @intCast(best_id) } };
}
fn heightSupport(view: runtime_view.View, direction: geometry.Vec3, status: *fp.MathStatus) Error!SupportPoint {
    const dims = try view.heightDimensions();
    const total = @as(usize, dims.width) * dims.height;
    if (total == 0) return error.InvalidShape;
    var best = geometry.Vec3{ .x = fp.Fp.zero, .y = try view.heightSample(0), .z = fp.Fp.zero };
    var best_dot = best.dot(direction, status);
    var best_id: usize = 0;
    var i: usize = 1;
    while (i < total) : (i += 1) {
        const point = geometry.Vec3{ .x = fp.Fp.fromInt(@intCast(i % dims.width)), .y = try view.heightSample(i), .z = fp.Fp.fromInt(@intCast(i / dims.width)) };
        const dot = point.dot(direction, status);
        if (dot.raw > best_dot.raw) {
            best = point;
            best_dot = dot;
            best_id = i;
        }
    }
    return .{ .point = best, .feature = .{ .vertex = @intCast(best_id) } };
}
fn compoundBounds(view: runtime_view.View, assets: *const store.Store, status: *fp.MathStatus, depth: u8) Error!geometry.Aabb3 {
    if (depth >= 8 or view.childCount() == 0) return error.CompoundDepth;
    var result: ?geometry.Aabb3 = null;
    var i: usize = 0;
    while (i < view.childCount()) : (i += 1) {
        const child = try view.child(i);
        const asset = assets.findByHash(&child.content_hash) orelse return error.MissingAsset;
        const child_view = try runtime_view.View.init(asset);
        const child_shape = assetShapeForView(child_view);
        const local = try localAabbAt(child_shape, assets, status, depth + 1);
        const transformed = try transformBounds(local, .{ .position = child.translation, .orientation = child.rotation }, status);
        if (result) |*bounds| {
            extend(bounds, transformed.min);
            extend(bounds, transformed.max);
        } else result = transformed;
    }
    return result orelse error.InvalidShape;
}
fn compoundSupport(view: runtime_view.View, assets: *const store.Store, direction: geometry.Vec3, status: *fp.MathStatus, depth: u8) Error!SupportPoint {
    if (depth >= 8 or view.childCount() == 0) return error.CompoundDepth;
    var result: ?SupportPoint = null;
    var best_dot: fp.Fp = fp.Fp.min;
    var i: usize = 0;
    while (i < view.childCount()) : (i += 1) {
        const child = try view.child(i);
        const asset = assets.findByHash(&child.content_hash) orelse return error.MissingAsset;
        const shape = assetShapeForView(try runtime_view.View.init(asset));
        const local_direction = child.rotation.inverseRotate(direction, status);
        var point = try supportAt(shape, assets, local_direction, status, depth + 1);
        point.point = child.rotation.rotate(point.point, status).add(child.translation, status);
        point.child_path = try prependChild(@intCast(i), point.child_path);
        const dot = point.point.dot(direction, status);
        if (result == null or dot.raw > best_dot.raw) {
            result = point;
            best_dot = dot;
        }
    }
    return result.?;
}
pub fn compoundChildCount(shape: Shape, assets: *const store.Store) Error!usize {
    const value = switch (shape) {
        .compound => |compound| compound,
        else => return error.UnsupportedShape,
    };
    return (try viewFor(value, assets, .compound)).childCount();
}
pub fn compoundChild(shape: Shape, assets: *const store.Store, index: usize) Error!CompoundChildShape {
    const value = switch (shape) {
        .compound => |compound| compound,
        else => return error.UnsupportedShape,
    };
    const child = try (try viewFor(value, assets, .compound)).child(index);
    const asset = assets.findByHash(&child.content_hash) orelse return error.MissingAsset;
    return .{ .ordinal = child.ordinal, .shape = assetShapeForView(try runtime_view.View.init(asset)), .transform = .{ .position = child.translation, .orientation = child.rotation } };
}

/// Enumerates terminal Compound leaves in canonical child-index order. The
/// caller supplies all storage; a capacity error publishes no partial result.
pub fn collectCompoundLeaves(shape: Shape, assets: *const store.Store, root_transform: geometry.Transform3, output: []CompoundLeaf, status: *fp.MathStatus) Error![]const CompoundLeaf {
    var scratch_count: usize = 0;
    try collectLeavesAt(shape, assets, root_transform, .{}, output, &scratch_count, status, 0);
    return output[0..scratch_count];
}

fn collectLeavesAt(shape: Shape, assets: *const store.Store, transform: geometry.Transform3, path: ChildPath, output: []CompoundLeaf, count: *usize, status: *fp.MathStatus, depth: u8) Error!void {
    if (depth >= 8) return error.CompoundDepth;
    if (shape != .compound) {
        if (count.* == output.len) return error.CapacityExceeded;
        output[count.*] = .{ .shape = shape, .transform = transform, .path = path };
        count.* += 1;
        return;
    }
    const children = try compoundChildCount(shape, assets);
    // Validate required output capacity before publishing any leaf. This
    // conservative pass is intentionally recursive but allocation-free.
    var required: usize = 0;
    for (0..children) |index| {
        const child = try compoundChild(shape, assets, index);
        required += try leafCount(child.shape, assets, depth + 1);
    }
    if (required > output.len - count.*) return error.CapacityExceeded;
    for (0..children) |index| {
        const child = try compoundChild(shape, assets, index);
        const child_path = try prependChild(child.ordinal, path);
        try collectLeavesAt(child.shape, assets, composeTransform(transform, child.transform, status), child_path, output, count, status, depth + 1);
    }
}

fn leafCount(shape: Shape, assets: *const store.Store, depth: u8) Error!usize {
    if (depth >= 8) return error.CompoundDepth;
    if (shape != .compound) return 1;
    var result: usize = 0;
    const children = try compoundChildCount(shape, assets);
    for (0..children) |index| result += try leafCount((try compoundChild(shape, assets, index)).shape, assets, depth + 1);
    return result;
}

fn composeTransform(parent: geometry.Transform3, child: geometry.Transform3, status: *fp.MathStatus) geometry.Transform3 {
    return .{ .position = parent.apply(child.position, status), .orientation = parent.orientation.mul(child.orientation, status) };
}
fn prependChild(child: u32, path: ChildPath) Error!ChildPath {
    if (path.len >= path.values.len) return error.CompoundDepth;
    var result = ChildPath{};
    result.values[0] = child;
    for (path.values[0..path.len], 0..) |value, i| result.values[i + 1] = value;
    result.len = path.len + 1;
    return result;
}
fn transformBounds(bounds: geometry.Aabb3, transform: geometry.Transform3, status: *fp.MathStatus) Error!geometry.Aabb3 {
    var first = true;
    var result: geometry.Aabb3 = undefined;
    inline for ([_]fp.Fp{ bounds.min.x, bounds.max.x }) |x| inline for ([_]fp.Fp{ bounds.min.y, bounds.max.y }) |y| inline for ([_]fp.Fp{ bounds.min.z, bounds.max.z }) |z| {
        const point = transform.apply(.{ .x = x, .y = y, .z = z }, status);
        if (first) {
            result = .{ .min = point, .max = point };
            first = false;
        } else extend(&result, point);
    };
    return result;
}
fn assetShapeForView(view: runtime_view.View) Shape {
    const asset: AssetShape = .{ .source_id = view.header.source_id };
    return switch (view.header.kind) {
        .convex_hull => .{ .convex_hull = asset },
        .triangle_mesh => .{ .triangle_mesh = asset },
        .height_field => .{ .height_field = asset },
        .compound => .{ .compound = asset },
    };
}

/// Computes central mass properties using density in mass per local-volume.
/// Meshes without baked volume/inertia must supply an override; this preserves
/// the Task 06 watertight/override boundary instead of guessing from an AABB.
pub fn massProperties(shape: Shape, assets: *const store.Store, density: fp.Fp, override: ?MassOverride, status: *fp.MathStatus) Error!MassProperties {
    if (override) |value| {
        try validateMass(value);
        return value;
    }
    if (density.raw <= 0) return error.InvalidMass;
    return massAt(shape, assets, density, status, 0);
}
/// Retrieves a canonical local vertex for feature clipping and manifold keys.
/// Curved support-only primitives intentionally expose no fabricated vertices.
pub fn vertex(shape: Shape, assets: *const store.Store, index: u32, status: *fp.MathStatus) Error!geometry.Vec3 {
    return switch (shape) {
        .box => |value| if (index < 8) .{ .x = if ((index & 1) == 0) value.half_extents.x.neg(status) else value.half_extents.x, .y = if ((index & 2) == 0) value.half_extents.y.neg(status) else value.half_extents.y, .z = if ((index & 4) == 0) value.half_extents.z.neg(status) else value.half_extents.z } else error.InvalidIndex,
        .capsule => |value| if (index < 2) .{ .y = if (index == 0) value.half_height.neg(status) else value.half_height } else error.InvalidIndex,
        .convex_hull => |value| (try viewFor(value, assets, .convex_hull)).vertex(index),
        .triangle_mesh => |value| (try viewFor(value, assets, .triangle_mesh)).vertex(index),
        .height_field => |value| heightVertex(try viewFor(value, assets, .height_field), index),
        else => error.UnsupportedShape,
    };
}
/// Mesh primitive IDs are canonical triangle IDs; HeightField triangle
/// expansion is deliberately deferred to Task 11's cell traversal.
pub fn primitive(shape: Shape, assets: *const store.Store, index: u32) Error!baked.Triangle {
    return switch (shape) {
        .triangle_mesh => |value| (try viewFor(value, assets, .triangle_mesh)).triangle(index),
        .convex_hull => |value| (try viewFor(value, assets, .convex_hull)).triangle(index),
        else => error.UnsupportedShape,
    };
}
pub fn hullFace(shape: Shape, assets: *const store.Store, index: u32) Error!baked.HullFace {
    const view = switch (shape) {
        .convex_hull => |value| try viewFor(value, assets, .convex_hull),
        else => return error.UnsupportedShape,
    };
    return view.face(index);
}
pub fn hullHalfEdge(shape: Shape, assets: *const store.Store, index: u32) Error!baked.HalfEdge {
    const view = switch (shape) {
        .convex_hull => |value| try viewFor(value, assets, .convex_hull),
        else => return error.UnsupportedShape,
    };
    return view.halfEdge(index);
}
fn heightVertex(view: runtime_view.View, index: u32) Error!geometry.Vec3 {
    const dimensions = try view.heightDimensions();
    const total = @as(u64, dimensions.width) * dimensions.height;
    if (@as(u64, index) >= total) return error.InvalidIndex;
    return .{ .x = fp.Fp.fromInt(@intCast(index % dimensions.width)), .y = try view.heightSample(index), .z = fp.Fp.fromInt(@intCast(index / dimensions.width)) };
}
fn massAt(shape: Shape, assets: *const store.Store, density: fp.Fp, status: *fp.MathStatus, depth: u8) Error!MassProperties {
    try validatePrimitive(shape);
    return switch (shape) {
        .sphere => |value| sphereMass(value, density, status),
        .box => |value| boxMass(value, density, status),
        .capsule => |value| capsuleMass(value, density, status),
        .convex_hull => |value| hullMass(try viewFor(value, assets, .convex_hull), density, status),
        .triangle_mesh => return error.MissingMass,
        .height_field => return error.InvalidBodyShape,
        .compound => |value| compoundMass(try viewFor(value, assets, .compound), assets, density, status, depth),
    };
}
fn sphereMass(value: Sphere, density: fp.Fp, status: *fp.MathStatus) MassProperties {
    const r2 = value.radius.mul(value.radius, status);
    const volume = geometry.pi.mul(value.radius.mul(r2, status), status).mul(fp.Fp.fromRatio(4, 3, status), status);
    const mass = volume.mul(density, status);
    const i = mass.mul(r2, status).mul(fp.Fp.fromRatio(2, 5, status), status);
    return .{ .mass = mass, .center = geometry.Vec3.zero, .inertia = diagonal(i, i, i) };
}
fn boxMass(value: Box, density: fp.Fp, status: *fp.MathStatus) MassProperties {
    const mass = value.half_extents.x.mul(value.half_extents.y, status).mul(value.half_extents.z, status).mul(fp.Fp.fromInt(8), status).mul(density, status);
    const x2 = value.half_extents.x.mul(value.half_extents.x, status);
    const y2 = value.half_extents.y.mul(value.half_extents.y, status);
    const z2 = value.half_extents.z.mul(value.half_extents.z, status);
    const third = fp.Fp.fromRatio(1, 3, status);
    return .{ .mass = mass, .center = geometry.Vec3.zero, .inertia = diagonal(mass.mul(y2.add(z2, status), status).mul(third, status), mass.mul(x2.add(z2, status), status).mul(third, status), mass.mul(x2.add(y2, status), status).mul(third, status)) };
}
fn capsuleMass(value: Capsule, density: fp.Fp, status: *fp.MathStatus) MassProperties {
    const r2 = value.radius.mul(value.radius, status);
    const h2 = value.half_height.mul(value.half_height, status);
    const cylinder_mass = geometry.pi.mul(r2, status).mul(value.half_height, status).mul(fp.Fp.fromInt(2), status).mul(density, status);
    const sphere = sphereMass(.{ .radius = value.radius }, density, status);
    const mass = cylinder_mass.add(sphere.mass, status);
    const iy = cylinder_mass.mul(r2, status).mul(fp.Fp.fromRatio(1, 2, status), status).add(sphere.inertia.yy, status);
    const cylinder_ix = cylinder_mass.mul(r2.mul(fp.Fp.fromInt(3), status).add(h2.mul(fp.Fp.fromInt(4), status), status), status).mul(fp.Fp.fromRatio(1, 12, status), status);
    const sphere_ix = sphere.mass.mul(r2.mul(fp.Fp.fromRatio(2, 5, status), status).add(value.half_height.mul(value.radius, status).mul(fp.Fp.fromRatio(3, 4, status), status), status).add(h2, status), status);
    const ix = cylinder_ix.add(sphere_ix, status);
    return .{ .mass = mass, .center = geometry.Vec3.zero, .inertia = diagonal(ix, iy, ix) };
}
fn hullMass(view: runtime_view.View, density: fp.Fp, status: *fp.MathStatus) Error!MassProperties {
    const baked_mass = try view.mass();
    const mass = baked_mass.volume.mul(density, status);
    return .{ .mass = mass, .center = baked_mass.center, .inertia = scaleInertia(baked_mass.inertia, density, status) };
}
fn compoundMass(view: runtime_view.View, assets: *const store.Store, density: fp.Fp, status: *fp.MathStatus, depth: u8) Error!MassProperties {
    if (depth >= 8 or view.childCount() == 0) return error.CompoundDepth;
    var total = fp.Fp.zero;
    var weighted_center = geometry.Vec3.zero;
    var i: usize = 0;
    while (i < view.childCount()) : (i += 1) {
        const child = try view.child(i);
        const asset = assets.findByHash(&child.content_hash) orelse return error.MissingAsset;
        const child_mass = try massAt(assetShapeForView(try runtime_view.View.init(asset)), assets, density, status, depth + 1);
        const center = child.rotation.rotate(child_mass.center, status).add(child.translation, status);
        total = total.add(child_mass.mass, status);
        weighted_center = weighted_center.add(center.scale(child_mass.mass, status), status);
    }
    if (total.raw <= 0) return error.InvalidMass;
    const center = weighted_center.scale(fp.Fp.one.div(total, status), status);
    var inertia = zeroInertia();
    i = 0;
    while (i < view.childCount()) : (i += 1) {
        const child = try view.child(i);
        const asset = assets.findByHash(&child.content_hash) orelse return error.MissingAsset;
        const child_mass = try massAt(assetShapeForView(try runtime_view.View.init(asset)), assets, density, status, depth + 1);
        const rotated = child_mass.inertia.rotate(child.rotation, status);
        const offset = child.rotation.rotate(child_mass.center, status).add(child.translation, status).sub(center, status);
        inertia = addInertia(inertia, addInertia(rotated, parallelAxis(child_mass.mass, offset, status), status), status);
    }
    return .{ .mass = total, .center = center, .inertia = inertia };
}
fn diagonal(x: fp.Fp, y: fp.Fp, z: fp.Fp) geometry.SymmetricMat3 {
    return .{ .xx = x, .yy = y, .zz = z, .xy = fp.Fp.zero, .xz = fp.Fp.zero, .yz = fp.Fp.zero };
}
fn zeroInertia() geometry.SymmetricMat3 {
    return diagonal(fp.Fp.zero, fp.Fp.zero, fp.Fp.zero);
}
fn scaleInertia(value: geometry.SymmetricMat3, scale: fp.Fp, status: *fp.MathStatus) geometry.SymmetricMat3 {
    return .{ .xx = value.xx.mul(scale, status), .yy = value.yy.mul(scale, status), .zz = value.zz.mul(scale, status), .xy = value.xy.mul(scale, status), .xz = value.xz.mul(scale, status), .yz = value.yz.mul(scale, status) };
}
fn addInertia(a: geometry.SymmetricMat3, b: geometry.SymmetricMat3, status: *fp.MathStatus) geometry.SymmetricMat3 {
    return .{ .xx = a.xx.add(b.xx, status), .yy = a.yy.add(b.yy, status), .zz = a.zz.add(b.zz, status), .xy = a.xy.add(b.xy, status), .xz = a.xz.add(b.xz, status), .yz = a.yz.add(b.yz, status) };
}
fn parallelAxis(mass: fp.Fp, offset: geometry.Vec3, status: *fp.MathStatus) geometry.SymmetricMat3 {
    const x2 = offset.x.mul(offset.x, status);
    const y2 = offset.y.mul(offset.y, status);
    const z2 = offset.z.mul(offset.z, status);
    return .{ .xx = mass.mul(y2.add(z2, status), status), .yy = mass.mul(x2.add(z2, status), status), .zz = mass.mul(x2.add(y2, status), status), .xy = mass.mul(offset.x.mul(offset.y, status).neg(status), status), .xz = mass.mul(offset.x.mul(offset.z, status).neg(status), status), .yz = mass.mul(offset.y.mul(offset.z, status).neg(status), status) };
}
fn validateMass(value: MassProperties) Error!void {
    if (value.mass.raw <= 0 or value.inertia.xx.raw < 0 or value.inertia.yy.raw < 0 or value.inertia.zz.raw < 0) return error.InvalidMass;
}
pub fn filter(a: *const Collider, a_type: BodyType, b: *const Collider, b_type: BodyType) FilterResult {
    if (!a.enabled or !b.enabled or a.body.value == b.body.value) return .ignore;
    const group_override = a.group != 0 and a.group == b.group;
    if (group_override and a.group < 0) return .ignore;
    if (!group_override and ((a.mask & b.category) == 0 or (b.mask & a.category) == 0)) return .ignore;
    if (a_type != .dynamic and b_type != .dynamic) return .ignore;
    return response(a, b);
}
fn response(a: *const Collider, b: *const Collider) FilterResult {
    return if (a.sensor or b.sensor) .overlap else .contact;
}
