//! Deterministic island discovery and fixed-capacity 6D constraint rows.
const std = @import("std");
const fp = @import("../math/fp.zig");
const geometry = @import("../math/geometry.zig");
const ids = @import("../core/ids.zig");
const shapes = @import("../collision/shapes.zig");
const body_world = @import("world.zig");

pub const Error = error{ CapacityExceeded, InvalidBody, InvalidConstraint };
pub const EdgeKind = enum(u8) { contact, joint, dof };
pub const Edge = struct { kind: EdgeKind, body_a: ids.BodyId, body_b: ids.BodyId = ids.BodyId.invalid, owner: u64 = 0 };
pub const Island = struct { id: ids.BodyId, first_member: u32, member_count: u32 };
pub const RowKind = enum(u8) { joint, contact, lock_translation, lock_rotation };
pub const RowKey = struct {
    kind: RowKind,
    min_body: ids.BodyId,
    max_body: ids.BodyId,
    owner: u64,
    row_index: u16,
    pub fn lessThan(a: RowKey, b: RowKey) bool {
        if (@intFromEnum(a.kind) != @intFromEnum(b.kind)) return @intFromEnum(a.kind) < @intFromEnum(b.kind);
        if (a.min_body.value != b.min_body.value) return idLess(a.min_body, b.min_body);
        if (a.max_body.value != b.max_body.value) return idLess(a.max_body, b.max_body);
        if (a.owner != b.owner) return a.owner < b.owner;
        return a.row_index < b.row_index;
    }
};
/// A scalar row over two 6D body velocities. Bounds are impulse bounds.
pub const ConstraintRow = struct {
    key: RowKey,
    ja_linear: geometry.Vec3 = .{},
    ja_angular: geometry.Vec3 = .{},
    jb_linear: geometry.Vec3 = .{},
    jb_angular: geometry.Vec3 = .{},
    effective_mass: fp.Fp = .zero,
    /// Positive CFM term for implicit spring rows. It is part of the scalar
    /// effective-mass denominator, never a post-solve velocity tweak.
    softness: fp.Fp = .zero,
    bias: fp.Fp = .zero,
    lower: fp.Fp = .min,
    upper: fp.Fp = .max,
    accumulated_impulse: fp.Fp = .zero,
};
/// Authoring input shared by contact and future joint producers. The builder
/// canonicalizes body order and swaps the complete 6D Jacobian when needed.
pub const RowSpec = struct {
    kind: RowKind,
    body_a: ids.BodyId,
    body_b: ids.BodyId,
    owner: u64,
    row_index: u16,
    ja_linear: geometry.Vec3 = .{},
    ja_angular: geometry.Vec3 = .{},
    jb_linear: geometry.Vec3 = .{},
    jb_angular: geometry.Vec3 = .{},
    bias: fp.Fp = .zero,
    softness: fp.Fp = .zero,
    lower: fp.Fp = .min,
    upper: fp.Fp = .max,
    accumulated_impulse: fp.Fp = .zero,
};
pub const BuildResult = struct { islands: []const Island, members: []const ids.BodyId, rows: []const ConstraintRow };

/// The frozen 3D representation of the 2D preset: Z translation and X/Y
/// rotation locks. No Vec2 state is introduced.
pub fn planarLocks() body_world.DofLock {
    return .{ .linear_z = true, .angular_x = true, .angular_y = true };
}

/// Finalizes a contact or joint row after its Jacobians, bounds and bias were
/// authored by the owning subsystem. A zero effective mass is never silently
/// converted to an unconstrained row.
pub fn finalizeRow(world: *const body_world.World, row: *ConstraintRow, status: *fp.MathStatus) Error!void {
    row.effective_mass = try effectiveMass(world, row, status);
}

/// Converts authored contact/joint rows without heap allocation. `scratch`
/// isolates all intermediate writes, so invalid input or K=0 leaves `output`
/// untouched. Lock rows use the dedicated body-state builder below.
pub fn buildAuthoredRows(world: *const body_world.World, specs: []const RowSpec, output: []ConstraintRow, scratch: []ConstraintRow, status: *fp.MathStatus) Error![]const ConstraintRow {
    if (specs.len > output.len or specs.len > scratch.len) return error.CapacityExceeded;
    for (specs, 0..) |spec, i| {
        if (spec.kind != .contact and spec.kind != .joint) return error.InvalidConstraint;
        _ = world.bodyIndex(spec.body_a) orelse return error.InvalidBody;
        _ = world.bodyIndex(spec.body_b) orelse return error.InvalidBody;
        if (spec.lower.raw > spec.upper.raw or spec.softness.raw < 0) return error.InvalidConstraint;
        var row = ConstraintRow{ .key = .{ .kind = spec.kind, .min_body = spec.body_a, .max_body = spec.body_b, .owner = spec.owner, .row_index = spec.row_index }, .ja_linear = spec.ja_linear, .ja_angular = spec.ja_angular, .jb_linear = spec.jb_linear, .jb_angular = spec.jb_angular, .bias = spec.bias, .softness = spec.softness, .lower = spec.lower, .upper = spec.upper, .accumulated_impulse = spec.accumulated_impulse };
        if (idLess(row.key.max_body, row.key.min_body)) {
            const body = row.key.min_body;
            row.key.min_body = row.key.max_body;
            row.key.max_body = body;
            const linear = row.ja_linear;
            row.ja_linear = row.jb_linear;
            row.jb_linear = linear;
            const angular = row.ja_angular;
            row.ja_angular = row.jb_angular;
            row.jb_angular = angular;
        }
        try finalizeRow(world, &row, status);
        scratch[i] = row;
    }
    sortRows(scratch[0..specs.len]);
    @memcpy(output[0..specs.len], scratch[0..specs.len]);
    return output[0..specs.len];
}

/// Produces the single PGS row order: joint rows, contact rows, then DOF lock
/// rows according to their complete keys. Inputs are immutable producer
/// buffers; capacity failure occurs before the destination is touched.
pub fn mergeRows(authored: []const ConstraintRow, locks: []const ConstraintRow, output: []ConstraintRow) Error![]const ConstraintRow {
    if (authored.len + locks.len > output.len) return error.CapacityExceeded;
    @memcpy(output[0..authored.len], authored);
    @memcpy(output[authored.len .. authored.len + locks.len], locks);
    sortRows(output[0 .. authored.len + locks.len]);
    return output[0 .. authored.len + locks.len];
}

/// Builds connected components over awake dynamic bodies only. Static and
/// kinematic bodies may occur in edges but never bridge two dynamic islands.
pub fn build(world: *const body_world.World, edges: []const Edge, edge_scratch: []Edge, islands: []Island, members: []ids.BodyId, rows: []ConstraintRow, status: *fp.MathStatus) Error!BuildResult {
    if (edges.len > edge_scratch.len) return error.CapacityExceeded;
    var dynamic_count: usize = 0;
    var row_required: usize = 0;
    for (world.storage.alive, 0..) |alive, i| {
        if (!alive or world.storage.body_type[i] != .dynamic) continue;
        dynamic_count += 1;
        const locks = world.storage.locks[i];
        row_required += @as(usize, @intFromBool(locks.linear_x)) + @as(usize, @intFromBool(locks.linear_y)) + @as(usize, @intFromBool(locks.linear_z)) + @as(usize, @intFromBool(locks.angular_x)) + @as(usize, @intFromBool(locks.angular_y)) + @as(usize, @intFromBool(locks.angular_z));
    }
    // Each dynamic body may form one island, so these checks guarantee all
    // caller-visible result buffers remain untouched on capacity failure.
    if (members.len < dynamic_count or islands.len < dynamic_count or rows.len < row_required) return error.CapacityExceeded;
    for (edges, 0..) |edge, i| {
        try validateEdge(world, edge);
        edge_scratch[i] = edge;
    }
    sortEdges(edge_scratch[0..edges.len]);
    var member_count: usize = 0;
    var island_count: usize = 0;
    var visited: usize = 0;
    while (visited < world.storage.alive.len) : (visited += 1) {
        const start = world.bodyIdAt(visited) orelse continue;
        if (world.storage.body_type[visited] != .dynamic or marked(members[0..member_count], start)) continue;
        if (island_count == islands.len) return error.CapacityExceeded;
        const first = member_count;
        try appendMember(members, &member_count, start);
        var cursor = first;
        while (cursor < member_count) : (cursor += 1) {
            const current = members[cursor];
            for (edge_scratch[0..edges.len]) |edge| {
                const other = neighbor(edge, current) orelse continue;
                const other_index = world.bodyIndex(other) orelse return error.InvalidBody;
                if (world.storage.body_type[other_index] != .dynamic or marked(members[first..member_count], other)) continue;
                try appendMember(members, &member_count, other);
            }
        }
        sortIds(members[first..member_count]);
        islands[island_count] = .{ .id = members[first], .first_member = @intCast(first), .member_count = @intCast(member_count - first) };
        island_count += 1;
    }
    sortIslands(islands[0..island_count]);
    const row_count = try buildLockRows(world, rows, status);
    return .{ .islands = islands[0..island_count], .members = members[0..member_count], .rows = rows[0..row_count] };
}

/// Union-find production path with caller-owned parent storage. It emits the
/// same ascending island/member order as `build` without rescanning every edge
/// for every discovered member.
pub fn buildWithParents(world: *const body_world.World, edges: []const Edge, edge_scratch: []Edge, islands: []Island, members: []ids.BodyId, rows: []ConstraintRow, parents: []u32, status: *fp.MathStatus) Error!BuildResult {
    if (edges.len > edge_scratch.len or parents.len < world.storage.alive.len) return error.CapacityExceeded;
    var dynamic_count: usize = 0;
    var row_required: usize = 0;
    const sentinel = std.math.maxInt(u32);
    for (world.storage.alive, 0..) |alive, index| {
        if (!alive or world.storage.body_type[index] != .dynamic) {
            parents[index] = sentinel;
            continue;
        }
        parents[index] = @intCast(index);
        dynamic_count += 1;
        const locks = world.storage.locks[index];
        row_required += @as(usize, @intFromBool(locks.linear_x)) + @as(usize, @intFromBool(locks.linear_y)) + @as(usize, @intFromBool(locks.linear_z)) + @as(usize, @intFromBool(locks.angular_x)) + @as(usize, @intFromBool(locks.angular_y)) + @as(usize, @intFromBool(locks.angular_z));
    }
    if (members.len < dynamic_count or islands.len < dynamic_count or rows.len < row_required) return error.CapacityExceeded;
    for (edges, 0..) |edge, index| {
        try validateEdge(world, edge);
        edge_scratch[index] = edge;
        const a = world.bodyIndex(edge.body_a).?;
        if (world.storage.body_type[a] != .dynamic or !edge.body_b.isValid()) continue;
        const b = world.bodyIndex(edge.body_b).?;
        if (world.storage.body_type[b] != .dynamic) continue;
        const root_a = findRoot(parents, @intCast(a));
        const root_b = findRoot(parents, @intCast(b));
        if (root_a == root_b) continue;
        const lower = @min(root_a, root_b);
        const upper = @max(root_a, root_b);
        parents[upper] = lower;
    }
    // Compress once before canonical output grouping.
    for (parents[0..world.storage.alive.len], 0..) |parent, index| {
        if (parent != sentinel) parents[index] = findRoot(parents, @intCast(index));
    }
    var member_count: usize = 0;
    var island_count: usize = 0;
    for (parents[0..world.storage.alive.len], 0..) |parent, root| {
        if (parent == sentinel or parent != root) continue;
        const first = member_count;
        for (parents[0..world.storage.alive.len], 0..) |candidate_root, candidate| {
            if (candidate_root != root) continue;
            try appendMember(members, &member_count, world.bodyIdAt(candidate) orelse return error.InvalidBody);
        }
        islands[island_count] = .{ .id = members[first], .first_member = @intCast(first), .member_count = @intCast(member_count - first) };
        island_count += 1;
    }
    const row_count = try buildLockRows(world, rows, status);
    return .{ .islands = islands[0..island_count], .members = members[0..member_count], .rows = rows[0..row_count] };
}

fn findRoot(parents: []const u32, start: u32) u32 {
    var current = start;
    while (parents[current] != current) current = parents[current];
    return current;
}

fn buildLockRows(world: *const body_world.World, output: []ConstraintRow, status: *fp.MathStatus) Error!usize {
    var count: usize = 0;
    for (world.storage.alive, 0..) |alive, index| {
        if (!alive or world.storage.body_type[index] != .dynamic) continue;
        const id = world.bodyIdAt(index).?;
        const locks = world.storage.locks[index];
        for ([_]struct { enabled: bool, kind: RowKind, axis: geometry.Vec3 }{
            .{ .enabled = locks.linear_x, .kind = .lock_translation, .axis = .unit_x }, .{ .enabled = locks.linear_y, .kind = .lock_translation, .axis = .unit_y }, .{ .enabled = locks.linear_z, .kind = .lock_translation, .axis = .unit_z },
            .{ .enabled = locks.angular_x, .kind = .lock_rotation, .axis = .unit_x },   .{ .enabled = locks.angular_y, .kind = .lock_rotation, .axis = .unit_y },   .{ .enabled = locks.angular_z, .kind = .lock_rotation, .axis = .unit_z },
        }) |entry| {
            if (!entry.enabled) continue;
            if (count == output.len) return error.CapacityExceeded;
            var row = ConstraintRow{ .key = .{ .kind = entry.kind, .min_body = id, .max_body = id, .owner = id.value, .row_index = @intCast(count) }, .lower = .min, .upper = .max };
            if (entry.kind == .lock_translation) row.ja_linear = entry.axis else row.ja_angular = entry.axis;
            try finalizeRow(world, &row, status);
            output[count] = row;
            count += 1;
        }
    }
    sortRows(output[0..count]);
    return count;
}
fn effectiveMass(world: *const body_world.World, row: *const ConstraintRow, status: *fp.MathStatus) Error!fp.Fp {
    const a = world.bodyIndex(row.key.min_body) orelse return error.InvalidBody;
    var k = row.ja_linear.dot(row.ja_linear, status).mul(world.storage.inverse_mass[a], status);
    const inv_inertia = world.storage.inverse_inertia_local[a].rotate(world.storage.orientation[a], status).toMat3();
    k = k.add(row.ja_angular.dot(inv_inertia.mulVec(row.ja_angular, status), status), status);
    if (row.key.max_body.value != row.key.min_body.value) {
        const b = world.bodyIndex(row.key.max_body) orelse return error.InvalidBody;
        k = k.add(row.jb_linear.dot(row.jb_linear, status).mul(world.storage.inverse_mass[b], status), status);
        const inv_b = world.storage.inverse_inertia_local[b].rotate(world.storage.orientation[b], status).toMat3();
        k = k.add(row.jb_angular.dot(inv_b.mulVec(row.jb_angular, status), status), status);
    }
    k = k.add(row.softness, status);
    if (k.raw <= 0) return error.InvalidConstraint;
    return fp.Fp.one.div(k, status);
}
fn validateEdge(world: *const body_world.World, edge: Edge) Error!void {
    _ = world.bodyIndex(edge.body_a) orelse return error.InvalidBody;
    if (edge.body_b.isValid()) _ = world.bodyIndex(edge.body_b) orelse return error.InvalidBody;
}
fn neighbor(edge: Edge, body: ids.BodyId) ?ids.BodyId {
    if (edge.body_a.value == body.value) return if (edge.body_b.isValid()) edge.body_b else null;
    if (edge.body_b.value == body.value) return edge.body_a;
    return null;
}
fn marked(items: []const ids.BodyId, id: ids.BodyId) bool {
    for (items) |item| if (item.value == id.value) return true;
    return false;
}
fn appendMember(output: []ids.BodyId, count: *usize, id: ids.BodyId) Error!void {
    if (count.* == output.len) return error.CapacityExceeded;
    output[count.*] = id;
    count.* += 1;
}
fn idLess(a: ids.BodyId, b: ids.BodyId) bool {
    return if (a.index() != b.index()) a.index() < b.index() else a.generation() < b.generation();
}
fn idMin(a: ids.BodyId, b: ids.BodyId) ids.BodyId {
    return if (idLess(a, b)) a else b;
}
fn idMax(a: ids.BodyId, b: ids.BodyId) ids.BodyId {
    return if (idLess(a, b)) b else a;
}
fn sortIds(items: []ids.BodyId) void {
    var i: usize = 1;
    while (i < items.len) : (i += 1) {
        const value = items[i];
        var j = i;
        while (j > 0 and idLess(value, items[j - 1])) : (j -= 1) items[j] = items[j - 1];
        items[j] = value;
    }
}
fn edgeLess(a: Edge, b: Edge) bool {
    const amin = idMin(a.body_a, a.body_b);
    const amax = idMax(a.body_a, a.body_b);
    const bmin = idMin(b.body_a, b.body_b);
    const bmax = idMax(b.body_a, b.body_b);
    return if (@intFromEnum(a.kind) != @intFromEnum(b.kind)) @intFromEnum(a.kind) < @intFromEnum(b.kind) else if (amin.value != bmin.value) idLess(amin, bmin) else if (amax.value != bmax.value) idLess(amax, bmax) else a.owner < b.owner;
}
fn sortEdges(items: []Edge) void {
    var i: usize = 1;
    while (i < items.len) : (i += 1) {
        const value = items[i];
        var j = i;
        while (j > 0 and edgeLess(value, items[j - 1])) : (j -= 1) items[j] = items[j - 1];
        items[j] = value;
    }
}
fn sortIslands(items: []Island) void {
    var i: usize = 1;
    while (i < items.len) : (i += 1) {
        const value = items[i];
        var j = i;
        while (j > 0 and idLess(value.id, items[j - 1].id)) : (j -= 1) items[j] = items[j - 1];
        items[j] = value;
    }
}
fn sortRows(items: []ConstraintRow) void {
    std.sort.block(ConstraintRow, items, {}, rowLess);
}
fn rowLess(_: void, a: ConstraintRow, b: ConstraintRow) bool {
    return a.key.lessThan(b.key);
}
