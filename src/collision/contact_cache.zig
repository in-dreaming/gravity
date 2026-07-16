//! Persistent, caller-buffered contact patch cache and ordered contact events.
const std = @import("std");
const fp = @import("../math/fp.zig");
const geometry = @import("../math/geometry.zig");
const ids = @import("../core/ids.zig");
const shapes = @import("shapes.zig");
const hash = @import("../state/hash.zig");
const codec = @import("../state/codec.zig");

pub const Error = error{ CapacityExceeded, InvalidOrder, InvalidPatch };
pub const EventKind = enum(u8) { begin, persist, end, sensor_enter, sensor_stay, sensor_exit };
pub const ManifoldKey = struct {
    collider_a: ids.ColliderId,
    collider_b: ids.ColliderId,
    path_a: shapes.ChildPath = .{},
    path_b: shapes.ChildPath = .{},
    primitive_a: u32 = 0,
    primitive_b: u32 = 0,
    shape_revision_a: u32 = 0,
    shape_revision_b: u32 = 0,
    material_revision_a: u32 = 0,
    material_revision_b: u32 = 0,
};
pub const TangentBasis = struct { normal: geometry.Vec3, first: geometry.Vec3, second: geometry.Vec3 };
pub const CachedPoint = struct { feature_a: u32, feature_b: u32, normal_impulse: fp.Fp = fp.Fp.zero, tangent_first: fp.Fp = fp.Fp.zero, tangent_second: fp.Fp = fp.Fp.zero };
pub const Patch = struct { key: ManifoldKey, normal: geometry.Vec3, points: [4]CachedPoint = undefined, len: u8 = 0, sensor: bool = false };
pub const Event = struct { kind: EventKind, key: ManifoldKey };
pub const Cache = struct {
    patches: []Patch,
    len: usize = 0,
    pub fn active(self: *const Cache) []const Patch {
        return self.patches[0..self.len];
    }
};
pub const MergeWorkspace = struct { next: []Patch, events: []Event };
pub const MergeResult = struct { events: []const Event };
pub const CodecError = codec.Error || Error;

/// Encodes only canonical live cache state. The sizing pass makes output
/// capacity failure observable before any caller-visible result is returned.
pub fn encode(cache: *const Cache, output: []u8) CodecError![]const u8 {
    var sizing = codec.Writer.sizing();
    try writeCache(cache, &sizing);
    if (sizing.written() > output.len) return error.OutOfSpace;
    var writer = codec.Writer.init(output);
    try writeCache(cache, &writer);
    return output[0..writer.written()];
}

/// Decodes into caller scratch first and commits only after byte, capacity and
/// canonical ordering validation succeeds.
pub fn decode(input: []const u8, cache: *Cache, scratch: []Patch) CodecError!void {
    var reader = codec.Reader.init(input);
    const count = try reader.unsigned(u32);
    if (count > scratch.len or count > cache.patches.len) return error.CapacityExceeded;
    for (scratch[0..count]) |*patch| patch.* = try readPatch(&reader);
    try reader.finish();
    if (!patchesOrdered(scratch[0..count])) return error.InvalidOrder;
    @memcpy(cache.patches[0..count], scratch[0..count]);
    cache.len = count;
}
fn writeCache(cache: *const Cache, writer: *codec.Writer) codec.Error!void {
    try writer.unsigned(u32, @intCast(cache.len));
    for (cache.active()) |patch| {
        try writeKey(patch.key, writer);
        try writer.vec3(patch.normal);
        try writer.boolean(patch.sensor);
        try writer.byte(patch.len);
        for (patch.points[0..patch.len]) |point| {
            try writer.unsigned(u32, point.feature_a);
            try writer.unsigned(u32, point.feature_b);
            try writer.fpValue(point.normal_impulse);
            try writer.fpValue(point.tangent_first);
            try writer.fpValue(point.tangent_second);
        }
    }
}
fn writeKey(key: ManifoldKey, writer: *codec.Writer) codec.Error!void {
    try writer.id(key.collider_a);
    try writer.id(key.collider_b);
    try writePath(key.path_a, writer);
    try writePath(key.path_b, writer);
    try writer.unsigned(u32, key.primitive_a);
    try writer.unsigned(u32, key.primitive_b);
    try writer.unsigned(u32, key.shape_revision_a);
    try writer.unsigned(u32, key.shape_revision_b);
    try writer.unsigned(u32, key.material_revision_a);
    try writer.unsigned(u32, key.material_revision_b);
}
fn writePath(path: shapes.ChildPath, writer: *codec.Writer) codec.Error!void {
    try writer.byte(path.len);
    for (path.values[0..path.len]) |value| try writer.unsigned(u32, value);
}
fn readPatch(reader: *codec.Reader) CodecError!Patch {
    var patch = Patch{ .key = try readKey(reader), .normal = try reader.vec3(), .sensor = try reader.boolean() };
    patch.len = try reader.byte();
    if (patch.len > 4) return error.InvalidPatch;
    for (patch.points[0..patch.len]) |*point| point.* = .{ .feature_a = try reader.unsigned(u32), .feature_b = try reader.unsigned(u32), .normal_impulse = try reader.fpValue(), .tangent_first = try reader.fpValue(), .tangent_second = try reader.fpValue() };
    return try normalized(patch);
}
fn readKey(reader: *codec.Reader) CodecError!ManifoldKey {
    return .{ .collider_a = try reader.id(), .collider_b = try reader.id(), .path_a = try readPath(reader), .path_b = try readPath(reader), .primitive_a = try reader.unsigned(u32), .primitive_b = try reader.unsigned(u32), .shape_revision_a = try reader.unsigned(u32), .shape_revision_b = try reader.unsigned(u32), .material_revision_a = try reader.unsigned(u32), .material_revision_b = try reader.unsigned(u32) };
}
fn readPath(reader: *codec.Reader) CodecError!shapes.ChildPath {
    var path = shapes.ChildPath{};
    path.len = try reader.byte();
    if (path.len > path.values.len) return error.InvalidPatch;
    for (path.values[0..path.len]) |*value| value.* = try reader.unsigned(u32);
    return path;
}

/// Visits every future-relevant cache field in its frozen sorted order. The
/// visitor must provide `writeU8`, `writeU32`, `writeU64`, and `writeI64`; no padding or unused
/// point slots participate in canonical state.
pub fn visitCanonical(cache: *const Cache, visitor: anytype) void {
    visitor.writeU64(cache.len);
    for (cache.active()) |patch| {
        visitKey(patch.key, visitor);
        visitor.writeI64(patch.normal.x.raw);
        visitor.writeI64(patch.normal.y.raw);
        visitor.writeI64(patch.normal.z.raw);
        visitor.writeU8(@intFromBool(patch.sensor));
        visitor.writeU8(patch.len);
        for (patch.points[0..patch.len]) |point| {
            visitor.writeU32(point.feature_a);
            visitor.writeU32(point.feature_b);
            visitor.writeI64(point.normal_impulse.raw);
            visitor.writeI64(point.tangent_first.raw);
            visitor.writeI64(point.tangent_second.raw);
        }
    }
}

/// Visits the published Tick event sequence. Events are not restored by a
/// snapshot, but their ordered hash is exposed for replay diagnostics.
pub fn visitEventsCanonical(events: []const Event, visitor: anytype) void {
    visitor.writeU64(events.len);
    for (events) |event| {
        visitor.writeU8(@intFromEnum(event.kind));
        visitKey(event.key, visitor);
    }
}

/// Hashes the canonical persistent cache state with the engine state domain.
pub fn canonicalHash(cache: *const Cache) hash.Hash256 {
    var writer = HashVisitor{};
    visitCanonical(cache, &writer);
    return writer.sink.final256();
}
const HashVisitor = struct {
    sink: hash.Sink = hash.Sink.init(.state),
    pub fn writeU8(self: *@This(), value: u8) void {
        self.sink.update(&[_]u8{value});
    }
    pub fn writeU32(self: *@This(), value: u32) void {
        var bytes: [4]u8 = undefined;
        std.mem.writeInt(u32, &bytes, value, .little);
        self.sink.update(&bytes);
    }
    pub fn writeU64(self: *@This(), value: u64) void {
        var bytes: [8]u8 = undefined;
        std.mem.writeInt(u64, &bytes, value, .little);
        self.sink.update(&bytes);
    }
    pub fn writeI64(self: *@This(), value: i64) void {
        self.writeU64(@bitCast(value));
    }
};
fn visitKey(key: ManifoldKey, visitor: anytype) void {
    visitor.writeU64(key.collider_a.value);
    visitor.writeU64(key.collider_b.value);
    visitPath(key.path_a, visitor);
    visitPath(key.path_b, visitor);
    visitor.writeU32(key.primitive_a);
    visitor.writeU32(key.primitive_b);
    visitor.writeU32(key.shape_revision_a);
    visitor.writeU32(key.shape_revision_b);
    visitor.writeU32(key.material_revision_a);
    visitor.writeU32(key.material_revision_b);
}
fn visitPath(path: shapes.ChildPath, visitor: anytype) void {
    visitor.writeU8(path.len);
    for (path.values[0..path.len]) |value| visitor.writeU32(value);
}

/// Canonical sorted merge. `incoming` is immutable narrow-phase output; its
/// impulses are ignored and reconstructed from matching old points. On any
/// fault neither cache nor event buffer is published.
pub fn merge(cache: *Cache, incoming: []const Patch, workspace: MergeWorkspace, warm_normal_cos_min: fp.Fp, status: *fp.MathStatus) Error!MergeResult {
    if (!patchesOrdered(cache.active()) or !patchesOrdered(incoming)) return error.InvalidOrder;
    const worst: usize = cache.len + incoming.len;
    if (worst > workspace.next.len or worst > cache.patches.len or worst > workspace.events.len) return error.CapacityExceeded;
    var old_i: usize = 0;
    var new_i: usize = 0;
    var count: usize = 0;
    var event_count: usize = 0;
    while (old_i < cache.len or new_i < incoming.len) {
        const take_old = new_i == incoming.len or (old_i < cache.len and keyLess(cache.patches[old_i].key, incoming[new_i].key));
        const take_new = old_i == cache.len or (new_i < incoming.len and keyLess(incoming[new_i].key, cache.patches[old_i].key));
        if (take_old) {
            try appendEvent(workspace.events, &event_count, if (cache.patches[old_i].sensor) .sensor_exit else .end, cache.patches[old_i].key);
            old_i += 1;
        } else if (take_new) {
            var patch = try normalized(incoming[new_i]);
            clearImpulses(&patch);
            workspace.next[count] = patch;
            count += 1;
            try appendEvent(workspace.events, &event_count, if (patch.sensor) .sensor_enter else .begin, patch.key);
            new_i += 1;
        } else {
            const old = cache.patches[old_i];
            var patch = try normalized(incoming[new_i]);
            if (old.sensor != patch.sensor) {
                try appendEvent(workspace.events, &event_count, if (old.sensor) .sensor_exit else .end, old.key);
                clearImpulses(&patch);
                try appendEvent(workspace.events, &event_count, if (patch.sensor) .sensor_enter else .begin, patch.key);
            } else {
                inherit(&patch, old, warm_normal_cos_min, status);
                try appendEvent(workspace.events, &event_count, if (patch.sensor) .sensor_stay else .persist, patch.key);
            }
            workspace.next[count] = patch;
            count += 1;
            old_i += 1;
            new_i += 1;
        }
    }
    if (count > cache.patches.len) return error.CapacityExceeded;
    sortEvents(workspace.events[0..event_count]);
    @memcpy(cache.patches[0..count], workspace.next[0..count]);
    cache.len = count;
    return .{ .events = workspace.events[0..event_count] };
}

fn normalized(value: Patch) Error!Patch {
    if (value.len > 4) return error.InvalidPatch;
    var result = value;
    var i: usize = 1;
    while (i < result.len) : (i += 1) {
        const point = result.points[i];
        var at = i;
        while (at > 0 and pointLess(point, result.points[at - 1])) : (at -= 1) result.points[at] = result.points[at - 1];
        result.points[at] = point;
    }
    return result;
}
fn inherit(next: *Patch, old: Patch, cos_min: fp.Fp, status: *fp.MathStatus) void {
    if (next.sensor) {
        clearImpulses(next);
        return;
    }
    if (revisionsDiffer(next.key, old.key) or next.normal.dot(old.normal, status).raw < cos_min.raw) {
        clearImpulses(next);
        return;
    }
    const old_basis = tangentBasis(old.normal, status);
    const next_basis = tangentBasis(next.normal, status);
    for (next.points[0..next.len]) |*point| for (old.points[0..old.len]) |prior| if (point.feature_a == prior.feature_a and point.feature_b == prior.feature_b) {
        point.normal_impulse = prior.normal_impulse;
        const world = old_basis.first.scale(prior.tangent_first, status).add(old_basis.second.scale(prior.tangent_second, status), status);
        point.tangent_first = world.dot(next_basis.first, status);
        point.tangent_second = world.dot(next_basis.second, status);
        break;
    };
}
pub fn tangentBasis(normal: geometry.Vec3, status: *fp.MathStatus) TangentBasis {
    const axis = if (@abs(normal.x.raw) <= @abs(normal.y.raw) and @abs(normal.x.raw) <= @abs(normal.z.raw)) geometry.Vec3.unit_x else if (@abs(normal.y.raw) <= @abs(normal.z.raw)) geometry.Vec3.unit_y else geometry.Vec3.unit_z;
    const first_n = axis.cross(normal, status).normalize(status);
    const first = if (first_n.valid) first_n.value else geometry.Vec3.unit_x;
    return .{ .normal = normal, .first = first, .second = normal.cross(first, status) };
}
fn clearImpulses(patch: *Patch) void {
    for (patch.points[0..patch.len]) |*point| point.normal_impulse = fp.Fp.zero;
    for (patch.points[0..patch.len]) |*point| {
        point.tangent_first = fp.Fp.zero;
        point.tangent_second = fp.Fp.zero;
    }
}
fn appendEvent(events: []Event, count: *usize, kind: EventKind, key: ManifoldKey) Error!void {
    if (count.* == events.len) return error.CapacityExceeded;
    events[count.*] = .{ .kind = kind, .key = key };
    count.* += 1;
}
fn sortEvents(events: []Event) void {
    var i: usize = 1;
    while (i < events.len) : (i += 1) {
        const value = events[i];
        var at = i;
        while (at > 0 and eventLess(value, events[at - 1])) : (at -= 1) events[at] = events[at - 1];
        events[at] = value;
    }
}
fn eventLess(a: Event, b: Event) bool {
    if (@intFromEnum(a.kind) != @intFromEnum(b.kind)) return @intFromEnum(a.kind) < @intFromEnum(b.kind);
    return keyLess(a.key, b.key);
}
fn revisionsDiffer(a: ManifoldKey, b: ManifoldKey) bool {
    return a.shape_revision_a != b.shape_revision_a or a.shape_revision_b != b.shape_revision_b or a.material_revision_a != b.material_revision_a or a.material_revision_b != b.material_revision_b;
}
fn pointLess(a: CachedPoint, b: CachedPoint) bool {
    return a.feature_a < b.feature_a or (a.feature_a == b.feature_a and a.feature_b < b.feature_b);
}
fn patchesOrdered(values: []const Patch) bool {
    if (values.len < 2) return true;
    for (values[1..], values[0..values.len -| 1]) |right, left| if (!keyLess(left.key, right.key)) return false;
    return true;
}
pub fn keyLess(a: ManifoldKey, b: ManifoldKey) bool {
    if (a.collider_a.value != b.collider_a.value) return a.collider_a.value < b.collider_a.value;
    if (a.collider_b.value != b.collider_b.value) return a.collider_b.value < b.collider_b.value;
    if (pathLess(a.path_a, b.path_a)) return true;
    if (pathLess(b.path_a, a.path_a)) return false;
    if (pathLess(a.path_b, b.path_b)) return true;
    if (pathLess(b.path_b, a.path_b)) return false;
    if (a.primitive_a != b.primitive_a) return a.primitive_a < b.primitive_a;
    return a.primitive_b < b.primitive_b;
}
fn pathLess(a: shapes.ChildPath, b: shapes.ChildPath) bool {
    for (a.values[0..@min(a.len, b.len)], b.values[0..@min(a.len, b.len)]) |x, y| if (x != y) return x < y;
    return a.len < b.len;
}
