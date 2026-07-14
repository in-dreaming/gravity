const std = @import("std");
const gravity = @import("gravity");
const fp = gravity.math.fp;
const g = gravity.math.geometry;
const shapes = gravity.collision.shapes;
const world = gravity.dynamics.world;
const joints = gravity.dynamics.joints;
const constraints = gravity.dynamics.constraints;
const Fixture = struct {
    types: [2]shapes.BodyType = undefined,
    position: [2]g.Vec3 = undefined,
    orientation: [2]g.Quat = undefined,
    linear: [2]g.Vec3 = undefined,
    angular: [2]g.Vec3 = undefined,
    mass: [2]fp.Fp = undefined,
    inertia: [2]g.SymmetricMat3 = undefined,
    force: [2]g.Vec3 = undefined,
    torque: [2]g.Vec3 = undefined,
    locks: [2]world.DofLock = undefined,
    generation: [2]u32 = undefined,
    alive: [2]bool = undefined,
    retired: [2]bool = undefined,
    target: [2]bool = undefined,
    target_position: [2]g.Vec3 = undefined,
    target_orientation: [2]g.Quat = undefined,
    fn init(self: *Fixture) !world.World {
        return world.World.init(.{ .body_type = &self.types, .position = &self.position, .orientation = &self.orientation, .linear_velocity = &self.linear, .angular_velocity = &self.angular, .inverse_mass = &self.mass, .inverse_inertia_local = &self.inertia, .force = &self.force, .torque = &self.torque, .locks = &self.locks, .generation = &self.generation, .alive = &self.alive, .retired = &self.retired, .has_target = &self.target, .target_position = &self.target_position, .target_orientation = &self.target_orientation });
    }
};
fn inertia() g.SymmetricMat3 {
    return .{ .xx = .one, .yy = .one, .zz = .one, .xy = .zero, .xz = .zero, .yz = .zero };
}
const VisitDigest = struct {
    value: u64 = 0xcbf29ce484222325,
    fn mix(self: *VisitDigest, input: u64) void {
        self.value = (self.value ^ input) *% 0x100000001b3;
    }
    pub fn writeU8(self: *VisitDigest, value: u8) void {
        self.mix(value);
    }
    pub fn writeU32(self: *VisitDigest, value: u32) void {
        self.mix(value);
    }
    pub fn writeU64(self: *VisitDigest, value: u64) void {
        self.mix(value);
    }
    pub fn writeI64(self: *VisitDigest, value: i64) void {
        self.mix(@bitCast(value));
    }
};
test "joint pool canonicalizes parallel frames and cascades body destruction" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const b = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    var values: [2]joints.Joint = undefined;
    var generations: [2]u32 = undefined;
    var alive: [2]bool = undefined;
    var retired: [2]bool = undefined;
    var pool = try joints.Pool.init(.{ .values = &values, .generation = &generations, .alive = &alive, .retired = &retired });
    const id = try pool.create(&state, .{ .kind = .hinge, .body_a = a, .body_b = b, .frame_a = .{ .axis = .unit_y, .secondary = .unit_y } }, &status);
    try std.testing.expect(@abs(values[id.index()].frame_a.axis.dot(values[id.index()].frame_a.secondary, &status).raw) <= 1);
    try std.testing.expectEqual(g.Quat.identity.w.raw, values[id.index()].reference_orientation.w.raw);
    try joints.destroyBody(&state, &pool, a);
    try std.testing.expect(!alive[id.index()]);
    try std.testing.expect(state.bodyIndex(a) == null);
    try std.testing.expectError(error.InvalidJoint, pool.destroy(id));
}

test "joint canonical visitor includes generations, limits, and warm impulses" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const b = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    var values: [1]joints.Joint = undefined;
    var generations: [1]u32 = undefined;
    var alive: [1]bool = undefined;
    var retired: [1]bool = undefined;
    var pool = try joints.Pool.init(.{ .values = &values, .generation = &generations, .alive = &alive, .retired = &retired });
    const id = try pool.create(&state, .{ .kind = .cone_twist, .body_a = a, .body_b = b, .cone_twist = .{ .enabled = true, .swing_max = .one } }, &status);
    var first = VisitDigest{};
    joints.visitCanonical(&pool, &first);
    values[id.index()].impulses[7] = .one;
    var second = VisitDigest{};
    joints.visitCanonical(&pool, &second);
    try std.testing.expect(first.value != second.value);
    try pool.destroy(id);
    var third = VisitDigest{};
    joints.visitCanonical(&pool, &third);
    try std.testing.expect(second.value != third.value);
}

test "joint command batches sort, cascade destroy bodies, and remain transactional" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const b = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    var values: [2]joints.Joint = undefined;
    var generations: [2]u32 = undefined;
    var alive: [2]bool = undefined;
    var retired: [2]bool = undefined;
    var pool = try joints.Pool.init(.{ .values = &values, .generation = &generations, .alive = &alive, .retired = &retired });
    var sorted: [3]joints.Command = undefined;
    var shadow_alive: [2]bool = undefined;
    var shadow_generation: [2]u32 = undefined;
    var shadow_retired: [2]bool = undefined;
    var shadow_a: [2]gravity.core.ids.BodyId = undefined;
    var shadow_b: [2]gravity.core.ids.BodyId = undefined;
    const scratch = joints.CommandScratch{ .commands = &sorted, .alive = &shadow_alive, .generation = &shadow_generation, .retired = &shadow_retired, .body_a = &shadow_a, .body_b = &shadow_b };
    var receipts: [3]joints.CommandReceipt = undefined;
    const create = joints.Command{ .key = .{ .phase_priority = 0, .issuer = 1, .sequence = 0 }, .op = .{ .create = .{ .kind = .hinge, .body_a = a, .body_b = b } } };
    const invalid = joints.Command{ .key = .{ .phase_priority = 1, .issuer = 0, .sequence = 0 }, .op = .{ .destroy = gravity.core.ids.JointId.init(1, 0) } };
    try std.testing.expectError(error.InvalidJoint, joints.executeCommands(&pool, &state, &.{ create, invalid }, scratch, &receipts, &status));
    try std.testing.expect(!alive[0] and !alive[1]);

    const second = joints.Command{ .key = .{ .phase_priority = 0, .issuer = 0, .sequence = 0 }, .op = .{ .create = .{ .kind = .slider, .body_a = a, .body_b = b } } };
    const cascade = joints.Command{ .key = .{ .phase_priority = 1, .issuer = 0, .sequence = 0 }, .op = .{ .destroy_body = a } };
    const result = try joints.executeCommands(&pool, &state, &.{ create, second, cascade }, scratch, &receipts, &status);
    try std.testing.expect(result[0].created != null and result[1].created != null);
    try std.testing.expect(!alive[0] and !alive[1]);
}

test "joint DOF contracts cover all six kinds" {
    try std.testing.expectEqual(@as(u8, 1), joints.equalityRowCount(.distance));
    try std.testing.expectEqual(@as(u8, 3), joints.equalityRowCount(.ball_socket));
    try std.testing.expectEqual(@as(u8, 5), joints.equalityRowCount(.hinge));
    try std.testing.expectEqual(@as(u8, 5), joints.equalityRowCount(.slider));
    try std.testing.expectEqual(@as(u8, 6), joints.equalityRowCount(.fixed));
    try std.testing.expectEqual(@as(u8, 3), joints.equalityRowCount(.cone_twist));
    try std.testing.expectEqual(@as(u8, 0), joints.releasedDofCount(.fixed));
}

test "motor clamp never exceeds its deterministic impulse cap" {
    var status = fp.MathStatus{};
    const motor = joints.Motor{ .enabled = true, .max_force = fp.Fp.fromInt(120) };
    const tick = fp.Fp.fromRatio(1, 60, &status);
    const cap = joints.motorImpulseCap(motor, tick, &status);
    try std.testing.expectEqual(cap.raw, joints.clampMotorImpulse(fp.Fp.fromInt(7), motor, tick, &status).raw);
    try std.testing.expectEqual(-cap.raw, joints.clampMotorImpulse(fp.Fp.fromInt(-7), motor, tick, &status).raw);
}

test "distance control prioritizes hard limits and clears changed-side warm impulse" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const b = try state.create(.{ .transform = .{ .position = .{ .x = fp.Fp.fromInt(2) } }, .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    var joint = joints.Joint{ .kind = .distance, .body_a = a, .body_b = b, .frame_a = .{}, .frame_b = .{}, .reference = .one, .limit = .{ .enabled = true, .min = .one, .max = fp.Fp.fromRatio(3, 2, &status) }, .motor = .{ .enabled = true, .target_velocity = .one, .max_force = fp.Fp.fromInt(60) }, .spring = .{}, .limit_state = .lower, .impulses = [_]fp.Fp{.zero} ** 8 };
    joint.impulses[6] = .one;
    const owner = gravity.core.ids.JointId.init(0, 0);
    var output: [1]constraints.ConstraintRow = undefined;
    var scratch: [1]constraints.ConstraintRow = undefined;
    const rows = try joints.buildDistanceControlRow(&state, &joint, owner, fp.Fp.fromRatio(1, 60, &status), &output, &scratch, &status);
    try std.testing.expectEqual(@as(usize, 1), rows.len);
    try std.testing.expectEqual(joints.LimitState.upper, joint.limit_state);
    try std.testing.expectEqual(fp.Fp.zero.raw, rows[0].accumulated_impulse.raw);
    try std.testing.expectEqual(fp.Fp.zero.raw, rows[0].lower.raw);
    try std.testing.expect(rows[0].bias.raw < 0);
    try joints.solveRows(&state, output[0..rows.len], .{}, &status);
    try std.testing.expect(state.storage.linear_velocity[b.index()].x.raw < 0);
}

test "distance spring uses positive implicit softness" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const b = try state.create(.{ .transform = .{ .position = .{ .x = fp.Fp.fromInt(2) } }, .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    var joint = joints.Joint{ .kind = .distance, .body_a = a, .body_b = b, .frame_a = .{}, .frame_b = .{}, .reference = .one, .limit = .{}, .motor = .{}, .spring = .{ .enabled = true, .frequency = .one, .damping_ratio = .one } };
    var output: [1]constraints.ConstraintRow = undefined;
    var scratch: [1]constraints.ConstraintRow = undefined;
    const rows = try joints.buildDistanceControlRow(&state, &joint, gravity.core.ids.JointId.init(0, 0), fp.Fp.fromRatio(1, 60, &status), &output, &scratch, &status);
    try std.testing.expect(rows[0].softness.raw > 0);
    try std.testing.expect(rows[0].bias.raw < 0);
}

test "distance motor solves and writes its bounded control impulse back" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const b = try state.create(.{ .transform = .{ .position = .{ .x = .one } }, .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    var values: [1]joints.Joint = undefined;
    var generations: [1]u32 = undefined;
    var alive: [1]bool = undefined;
    var retired: [1]bool = undefined;
    var pool = try joints.Pool.init(.{ .values = &values, .generation = &generations, .alive = &alive, .retired = &retired });
    const id = try pool.create(&state, .{ .kind = .distance, .body_a = a, .body_b = b, .motor = .{ .enabled = true, .target_velocity = .one, .max_force = fp.Fp.fromInt(60) } }, &status);
    var output: [1]constraints.ConstraintRow = undefined;
    var scratch: [1]constraints.ConstraintRow = undefined;
    const tick = fp.Fp.fromRatio(1, 60, &status);
    const rows = try joints.buildDistanceControlRow(&state, &values[id.index()], id, tick, &output, &scratch, &status);
    try joints.solveRows(&state, output[0..rows.len], .{}, &status);
    try joints.writeBackImpulses(&pool, rows);
    const cap = joints.motorImpulseCap(values[id.index()].motor, tick, &status);
    try std.testing.expect(state.storage.linear_velocity[b.index()].x.raw > 0);
    try std.testing.expect(@abs(values[id.index()].impulses[6].raw) <= cap.raw);
}

test "slider control constrains its signed released translation" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const b = try state.create(.{ .transform = .{ .position = .{ .x = fp.Fp.fromInt(2) } }, .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    var joint = joints.Joint{ .kind = .slider, .body_a = a, .body_b = b, .frame_a = .{}, .frame_b = .{}, .reference = .zero, .limit = .{ .enabled = true, .min = .zero, .max = .one }, .motor = .{}, .spring = .{} };
    var output: [1]constraints.ConstraintRow = undefined;
    var scratch: [1]constraints.ConstraintRow = undefined;
    const rows = try joints.buildSliderControlRow(&state, &joint, gravity.core.ids.JointId.init(0, 0), fp.Fp.fromRatio(1, 60, &status), &output, &scratch, &status);
    try std.testing.expectEqual(joints.LimitState.upper, joint.limit_state);
    try joints.solveRows(&state, output[0..rows.len], .{}, &status);
    try std.testing.expect(state.storage.linear_velocity[b.index()].x.raw < 0);
}

test "hinge control obtains a signed canonical angle through 180 boundaries" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const half_angle = g.pi.div(fp.Fp.fromInt(4), &status);
    const trig = g.cordic(half_angle);
    const b = try state.create(.{ .transform = .{ .orientation = .{ .x = trig.sin, .w = trig.cos } }, .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    var joint = joints.Joint{ .kind = .hinge, .body_a = a, .body_b = b, .frame_a = .{}, .frame_b = .{}, .reference = .zero, .limit = .{ .enabled = true, .min = g.pi.neg(&status), .max = g.pi.div(fp.Fp.fromInt(4), &status) }, .motor = .{}, .spring = .{} };
    var output: [1]constraints.ConstraintRow = undefined;
    var scratch: [1]constraints.ConstraintRow = undefined;
    const rows = try joints.buildHingeControlRow(&state, &joint, gravity.core.ids.JointId.init(0, 0), fp.Fp.fromRatio(1, 60, &status), &output, &scratch, &status);
    try std.testing.expectEqual(joints.LimitState.upper, joint.limit_state);
    try std.testing.expect(rows[0].bias.raw < 0);
    try joints.solveRows(&state, output[0..rows.len], .{}, &status);
    try std.testing.expect(state.storage.angular_velocity[b.index()].x.raw < 0);
    try std.testing.expectEqual(g.pi.raw, g.atan2(.zero, fp.Fp.fromInt(-1), &status).raw);
    try std.testing.expectEqual(-g.pi.raw, g.atan2(.zero, fp.Fp.fromInt(-1), &status).neg(&status).raw);
}

test "cone twist keeps anchors while its swing cone publishes an angular limit" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const half_angle = g.pi.div(fp.Fp.fromInt(4), &status);
    const trig = g.cordic(half_angle);
    const b = try state.create(.{ .transform = .{ .orientation = .{ .z = trig.sin, .w = trig.cos } }, .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    var joint = joints.Joint{ .kind = .cone_twist, .body_a = a, .body_b = b, .frame_a = .{}, .frame_b = .{}, .limit = .{}, .motor = .{}, .spring = .{}, .cone_twist = .{ .enabled = true, .swing_max = g.pi.div(fp.Fp.fromInt(4), &status) } };
    var rows: [3]constraints.ConstraintRow = undefined;
    var scratch: [3]constraints.ConstraintRow = undefined;
    const baseline = try joints.buildConeTwistRows(&state, &joint, gravity.core.ids.JointId.init(0, 0), &rows, &scratch, &status);
    try std.testing.expect(baseline[0].ja_linear.x.raw != 0);
    var control: [2]constraints.ConstraintRow = undefined;
    var control_scratch: [2]constraints.ConstraintRow = undefined;
    const limited = try joints.buildConeTwistControlRows(&state, &joint, gravity.core.ids.JointId.init(0, 0), fp.Fp.fromRatio(1, 60, &status), &control, &control_scratch, &status);
    try std.testing.expectEqual(@as(usize, 1), limited.len);
    try std.testing.expectEqual(joints.LimitState.upper, joint.cone_states[0]);
    try std.testing.expect(limited[0].ja_angular.z.raw > 0);
}

test "cone twist motor uses its bounded twist row when no hard limit is active" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const b = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    var joint = joints.Joint{ .kind = .cone_twist, .body_a = a, .body_b = b, .frame_a = .{}, .frame_b = .{}, .limit = .{}, .motor = .{ .enabled = true, .target_velocity = .one, .max_force = fp.Fp.fromInt(60) }, .spring = .{} };
    var output: [2]constraints.ConstraintRow = undefined;
    var scratch: [2]constraints.ConstraintRow = undefined;
    const rows = try joints.buildConeTwistControlRows(&state, &joint, gravity.core.ids.JointId.init(0, 0), fp.Fp.fromRatio(1, 60, &status), &output, &scratch, &status);
    try std.testing.expectEqual(@as(usize, 1), rows.len);
    try std.testing.expectEqual(@as(u16, 7), rows[0].key.row_index);
    try joints.solveRows(&state, output[0..rows.len], .{}, &status);
    try std.testing.expect(state.storage.angular_velocity[b.index()].x.raw > 0);
}

test "cone twist spring publishes both swing and twist implicit rows" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const half_angle = g.pi.div(fp.Fp.fromInt(4), &status);
    const trig = g.cordic(half_angle);
    const b = try state.create(.{ .transform = .{ .orientation = .{ .z = trig.sin, .w = trig.cos } }, .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    var joint = joints.Joint{ .kind = .cone_twist, .body_a = a, .body_b = b, .frame_a = .{}, .frame_b = .{}, .reference = .zero, .swing_reference = .zero, .limit = .{}, .motor = .{}, .spring = .{ .enabled = true, .frequency = .one, .damping_ratio = .one } };
    var output: [2]constraints.ConstraintRow = undefined;
    var scratch: [2]constraints.ConstraintRow = undefined;
    const rows = try joints.buildConeTwistControlRows(&state, &joint, gravity.core.ids.JointId.init(0, 0), fp.Fp.fromRatio(1, 60, &status), &output, &scratch, &status);
    try std.testing.expectEqual(@as(usize, 2), rows.len);
    try std.testing.expect(rows[0].softness.raw > 0 and rows[1].softness.raw > 0);
    try std.testing.expect(rows[0].ja_angular.z.raw > 0);
}

test "complete joint row builder merges baseline and active controls canonically" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const b = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    var joint = joints.Joint{ .kind = .cone_twist, .body_a = a, .body_b = b, .frame_a = .{}, .frame_b = .{}, .reference = .zero, .swing_reference = .zero, .limit = .{}, .motor = .{}, .spring = .{ .enabled = true, .frequency = .one, .damping_ratio = .one } };
    var output: [5]constraints.ConstraintRow = undefined;
    var scratch: [10]constraints.ConstraintRow = undefined;
    const rows = try joints.buildRows(&state, &joint, gravity.core.ids.JointId.init(0, 0), fp.Fp.fromRatio(1, 60, &status), &output, &scratch, &status);
    try std.testing.expectEqual(@as(usize, 5), rows.len);
    try std.testing.expectEqual(@as(u16, 0), rows[0].key.row_index);
    try std.testing.expectEqual(@as(u16, 6), rows[3].key.row_index);
    try std.testing.expectEqual(@as(u16, 7), rows[4].key.row_index);
}

test "pool orchestration assembles solves and writes back globally sorted joint rows" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const b = try state.create(.{ .transform = .{ .position = .{ .x = .one } }, .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    var values: [2]joints.Joint = undefined;
    var generations: [2]u32 = undefined;
    var alive: [2]bool = undefined;
    var retired: [2]bool = undefined;
    var pool = try joints.Pool.init(.{ .values = &values, .generation = &generations, .alive = &alive, .retired = &retired });
    const distance = try pool.create(&state, .{ .kind = .distance, .body_a = a, .body_b = b, .motor = .{ .enabled = true, .target_velocity = .one, .max_force = fp.Fp.fromInt(60) } }, &status);
    const hinge = try pool.create(&state, .{ .kind = .hinge, .body_a = a, .body_b = b, .motor = .{ .enabled = true, .target_velocity = .one, .max_force = fp.Fp.fromInt(60) } }, &status);
    var rows: [8]constraints.ConstraintRow = undefined;
    var authored: [8]constraints.ConstraintRow = undefined;
    var build: [12]constraints.ConstraintRow = undefined;
    var states: [2]joints.MutableState = undefined;
    const solved = try joints.solvePool(&state, &pool, fp.Fp.fromRatio(1, 60, &status), .{}, &rows, .{ .authored = &authored, .build = &build, .states = &states }, &status);
    try std.testing.expectEqual(@as(usize, 8), solved.len);
    for (solved[1..], 1..) |row, i| try std.testing.expect(!row.key.lessThan(solved[i - 1].key));
    try std.testing.expect(@abs(values[distance.index()].impulses[6].raw) > 0);
    try std.testing.expect(@abs(values[hinge.index()].impulses[6].raw) > 0);
}

test "all six joint builders emit finalized deterministic Jacobian rows" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const b = try state.create(.{ .transform = .{ .position = .{ .x = .one } }, .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    var values: [6]joints.Joint = undefined;
    var generations: [6]u32 = undefined;
    var alive: [6]bool = undefined;
    var retired: [6]bool = undefined;
    var pool = try joints.Pool.init(.{ .values = &values, .generation = &generations, .alive = &alive, .retired = &retired });
    var output: [6]constraints.ConstraintRow = undefined;
    var scratch: [6]constraints.ConstraintRow = undefined;

    const distance = try pool.create(&state, .{ .kind = .distance, .body_a = a, .body_b = b }, &status);
    try std.testing.expectEqual(@as(usize, 1), (try joints.buildDistanceRow(&state, &values[distance.index()], distance, &output, &scratch, &status)).len);
    const ball = try pool.create(&state, .{ .kind = .ball_socket, .body_a = a, .body_b = b }, &status);
    try std.testing.expectEqual(@as(usize, 3), (try joints.buildBallSocketRows(&state, &values[ball.index()], ball, &output, &scratch, &status)).len);
    const hinge = try pool.create(&state, .{ .kind = .hinge, .body_a = a, .body_b = b }, &status);
    try std.testing.expectEqual(@as(usize, 5), (try joints.buildHingeRows(&state, &values[hinge.index()], hinge, &output, &scratch, &status)).len);
    const slider = try pool.create(&state, .{ .kind = .slider, .body_a = a, .body_b = b }, &status);
    try std.testing.expectEqual(@as(usize, 5), (try joints.buildSliderRows(&state, &values[slider.index()], slider, &output, &scratch, &status)).len);
    const fixed = try pool.create(&state, .{ .kind = .fixed, .body_a = a, .body_b = b }, &status);
    try std.testing.expectEqual(@as(usize, 6), (try joints.buildFixedRows(&state, &values[fixed.index()], fixed, &output, &scratch, &status)).len);
    const cone = try pool.create(&state, .{ .kind = .cone_twist, .body_a = a, .body_b = b }, &status);
    const rows = try joints.buildConeTwistRows(&state, &values[cone.index()], cone, &output, &scratch, &status);
    try std.testing.expectEqual(@as(usize, 3), rows.len);
    for (rows) |row| try std.testing.expect(row.effective_mass.raw > 0);
}

test "joint builders preserve caller output on capacity failure" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const b = try state.create(.{ .transform = .{ .position = .{ .x = .one } }, .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const joint = joints.Joint{ .kind = .ball_socket, .body_a = a, .body_b = b, .frame_a = .{}, .frame_b = .{}, .limit = .{}, .motor = .{}, .spring = .{} };
    const owner = gravity.core.ids.JointId.init(0, 0);
    const sentinel = constraints.ConstraintRow{ .key = .{ .kind = .joint, .min_body = a, .max_body = b, .owner = owner.value, .row_index = 99 } };
    var output = [_]constraints.ConstraintRow{ sentinel, sentinel };
    var scratch: [3]constraints.ConstraintRow = undefined;
    try std.testing.expectError(error.CapacityExceeded, joints.buildBallSocketRows(&state, &joint, owner, &output, &scratch, &status));
    try std.testing.expectEqualDeep(sentinel, output[0]);
}
