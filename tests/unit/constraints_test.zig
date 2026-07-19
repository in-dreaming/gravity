const std = @import("std");
const gravity = @import("gravity");
const fp = gravity.math.fp;
const g = gravity.math.geometry;
const shapes = gravity.collision.shapes;
const world = gravity.dynamics.world;
const constraints = gravity.dynamics.constraints;

const Fixture = struct {
    types: [4]shapes.BodyType = undefined,
    position: [4]g.Vec3 = undefined,
    orientation: [4]g.Quat = undefined,
    linear: [4]g.Vec3 = undefined,
    angular: [4]g.Vec3 = undefined,
    mass: [4]fp.Fp = undefined,
    inertia: [4]g.SymmetricMat3 = undefined,
    force: [4]g.Vec3 = undefined,
    torque: [4]g.Vec3 = undefined,
    locks: [4]world.DofLock = undefined,
    generation: [4]u32 = undefined,
    alive: [4]bool = undefined,
    retired: [4]bool = undefined,
    target: [4]bool = undefined,
    target_position: [4]g.Vec3 = undefined,
    target_orientation: [4]g.Quat = undefined,
    fn init(self: *Fixture) !world.World {
        return world.World.init(.{ .body_type = &self.types, .position = &self.position, .orientation = &self.orientation, .linear_velocity = &self.linear, .angular_velocity = &self.angular, .inverse_mass = &self.mass, .inverse_inertia_local = &self.inertia, .force = &self.force, .torque = &self.torque, .locks = &self.locks, .generation = &self.generation, .alive = &self.alive, .retired = &self.retired, .has_target = &self.target, .target_position = &self.target_position, .target_orientation = &self.target_orientation });
    }
};
fn inertia() g.SymmetricMat3 {
    return .{ .xx = .one, .yy = .one, .zz = .one, .xy = .zero, .xz = .zero, .yz = .zero };
}

test "ordered island BFS does not bridge through static bodies" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const b = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const wall = try state.create(.{ .body_type = .static, .inverse_inertia_local = inertia() }, &status);
    const c = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const input = [_]constraints.Edge{ .{ .kind = .contact, .body_a = wall, .body_b = c }, .{ .kind = .contact, .body_a = b, .body_b = wall }, .{ .kind = .joint, .body_a = b, .body_b = a } };
    var edge_scratch: [3]constraints.Edge = undefined;
    var islands: [4]constraints.Island = undefined;
    var members: [4]gravity.core.ids.BodyId = undefined;
    var rows: [8]constraints.ConstraintRow = undefined;
    const result = try constraints.build(&state, &input, &edge_scratch, &islands, &members, &rows, &status);
    try std.testing.expectEqual(@as(usize, 2), result.islands.len);
    try std.testing.expectEqual(@as(u32, 2), result.islands[0].member_count);
    try std.testing.expectEqual(@as(u32, 1), result.islands[1].member_count);
    try std.testing.expectEqual(a.value, result.islands[0].id.value);
    try std.testing.expectEqual(c.value, result.islands[1].id.value);
    var union_edges: [3]constraints.Edge = undefined;
    var union_islands: [4]constraints.Island = undefined;
    var union_members: [4]gravity.core.ids.BodyId = undefined;
    var union_rows: [8]constraints.ConstraintRow = undefined;
    var parents: [4]u32 = undefined;
    const union_result = try constraints.buildWithParents(&state, &input, &union_edges, &union_islands, &union_members, &union_rows, &parents, &status);
    try std.testing.expectEqualSlices(u8, std.mem.sliceAsBytes(result.islands), std.mem.sliceAsBytes(union_result.islands));
    try std.testing.expectEqualSlices(u8, std.mem.sliceAsBytes(result.members), std.mem.sliceAsBytes(union_result.members));
}

test "DOF rows use 6D Jacobians stable keys and reject zero K" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const body = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia(), .locks = constraints.planarLocks() }, &status);
    var edge_scratch: [0]constraints.Edge = .{};
    var islands: [4]constraints.Island = undefined;
    var members: [4]gravity.core.ids.BodyId = undefined;
    var rows_a: [4]constraints.ConstraintRow = undefined;
    var rows_b: [4]constraints.ConstraintRow = undefined;
    const first = try constraints.build(&state, &.{}, &edge_scratch, &islands, &members, &rows_a, &status);
    const second = try constraints.build(&state, &.{}, &edge_scratch, &islands, &members, &rows_b, &status);
    try std.testing.expectEqual(@as(usize, 3), first.rows.len);
    try std.testing.expectEqual(constraints.RowKind.lock_translation, first.rows[0].key.kind);
    try std.testing.expectEqual(constraints.RowKind.lock_rotation, first.rows[1].key.kind);
    try std.testing.expectEqual(constraints.RowKind.lock_rotation, first.rows[2].key.kind);
    try std.testing.expectEqualSlices(u8, std.mem.sliceAsBytes(first.rows), std.mem.sliceAsBytes(second.rows));
    const sentinel = constraints.ConstraintRow{ .key = .{ .kind = .contact, .min_body = body, .max_body = body, .owner = 77, .row_index = 0 } };
    var too_small = [_]constraints.ConstraintRow{ sentinel, sentinel };
    try std.testing.expectError(error.CapacityExceeded, constraints.build(&state, &.{}, &edge_scratch, &islands, &members, &too_small, &status));
    try std.testing.expectEqualDeep(sentinel, too_small[0]);
    var invalid = constraints.ConstraintRow{ .key = .{ .kind = .joint, .min_body = body, .max_body = body, .owner = 1, .row_index = 0 } };
    try std.testing.expectError(error.InvalidConstraint, constraints.finalizeRow(&state, &invalid, &status));
}

test "authored contact and joint rows canonicalize bodies transactionally" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const b = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const specs = [_]constraints.RowSpec{
        .{ .kind = .joint, .body_a = b, .body_b = a, .owner = 9, .row_index = 1, .ja_linear = .{ .x = fp.Fp.fromInt(-1) }, .jb_linear = .{ .x = .one } },
        .{ .kind = .contact, .body_a = a, .body_b = b, .owner = 3, .row_index = 0, .ja_linear = .{ .y = .one }, .jb_linear = .{ .y = fp.Fp.fromInt(-1) }, .lower = .zero },
    };
    var output: [2]constraints.ConstraintRow = undefined;
    var scratch: [2]constraints.ConstraintRow = undefined;
    const rows = try constraints.buildAuthoredRows(&state, &specs, &output, &scratch, &status);
    try std.testing.expectEqual(constraints.RowKind.joint, rows[0].key.kind);
    try std.testing.expectEqual(constraints.RowKind.contact, rows[1].key.kind);
    try std.testing.expectEqual(a.value, rows[0].key.min_body.value);
    try std.testing.expectEqual(b.value, rows[0].key.max_body.value);
    try std.testing.expectEqual(fp.Fp.fromRatio(1, 2, &status).raw, rows[0].effective_mass.raw);
    const sentinel = output[0];
    const bad = [_]constraints.RowSpec{.{ .kind = .contact, .body_a = a, .body_b = b, .owner = 1, .row_index = 0, .lower = .one, .upper = .zero }};
    try std.testing.expectError(error.InvalidConstraint, constraints.buildAuthoredRows(&state, &bad, &output, &scratch, &status));
    try std.testing.expectEqualDeep(sentinel, output[0]);
    const lock = constraints.ConstraintRow{ .key = .{ .kind = .lock_translation, .min_body = a, .max_body = a, .owner = a.value, .row_index = 0 } };
    var merged: [3]constraints.ConstraintRow = undefined;
    const all = try constraints.mergeRows(rows, &.{lock}, &merged);
    try std.testing.expectEqual(constraints.RowKind.joint, all[0].key.kind);
    try std.testing.expectEqual(constraints.RowKind.contact, all[1].key.kind);
    try std.testing.expectEqual(constraints.RowKind.lock_translation, all[2].key.kind);
    const merged_sentinel = merged[0];
    try std.testing.expectError(error.CapacityExceeded, constraints.mergeRows(rows, &.{lock}, merged[0..2]));
    try std.testing.expectEqualDeep(merged_sentinel, merged[0]);
}
