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
fn tick(status: *fp.MathStatus) fp.Fp {
    return fp.Fp.fromRatio(1, 60, status);
}

test "pendulum ball socket suppresses anchor velocity against a static pivot" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const pivot = try state.create(.{ .body_type = .static, .inverse_inertia_local = inertia() }, &status);
    const bob = try state.create(.{ .transform = .{ .position = .{ .x = .one } }, .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    state.storage.linear_velocity[bob.index()] = .{ .x = .one };
    const joint = joints.Joint{ .kind = .ball_socket, .body_a = pivot, .body_b = bob, .frame_a = .{}, .frame_b = .{ .anchor = .{ .x = fp.Fp.fromInt(-1) } }, .limit = .{}, .motor = .{}, .spring = .{} };
    var rows: [3]constraints.ConstraintRow = undefined;
    var scratch: [3]constraints.ConstraintRow = undefined;
    const built = try joints.buildBallSocketRows(&state, &joint, gravity.core.ids.JointId.init(0, 0), &rows, &scratch, &status);
    try joints.solveRows(&state, rows[0..built.len], .{}, &status);
    try std.testing.expect(@abs(state.storage.linear_velocity[bob.index()].x.raw) < fp.Fp.one.raw);
}

test "slider and robot arm motors move only their released axes" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const b = try state.create(.{ .transform = .{ .position = .{ .x = .one } }, .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    var slider = joints.Joint{ .kind = .slider, .body_a = a, .body_b = b, .frame_a = .{}, .frame_b = .{}, .reference = .zero, .limit = .{}, .motor = .{ .enabled = true, .target_velocity = .one, .max_force = fp.Fp.fromInt(60) }, .spring = .{} };
    var row: [1]constraints.ConstraintRow = undefined;
    var scratch: [1]constraints.ConstraintRow = undefined;
    const slider_rows = try joints.buildSliderControlRow(&state, &slider, gravity.core.ids.JointId.init(0, 0), tick(&status), &row, &scratch, &status);
    try joints.solveRows(&state, row[0..slider_rows.len], .{}, &status);
    try std.testing.expect(state.storage.linear_velocity[b.index()].x.raw > 0);

    state.storage.angular_velocity[a.index()] = .{};
    state.storage.angular_velocity[b.index()] = .{};
    var hinge = joints.Joint{ .kind = .hinge, .body_a = a, .body_b = b, .frame_a = .{}, .frame_b = .{}, .reference = .zero, .limit = .{}, .motor = .{ .enabled = true, .target_velocity = .one, .max_force = fp.Fp.fromInt(60) }, .spring = .{} };
    const hinge_rows = try joints.buildHingeControlRow(&state, &hinge, gravity.core.ids.JointId.init(1, 0), tick(&status), &row, &scratch, &status);
    try joints.solveRows(&state, row[0..hinge_rows.len], .{}, &status);
    try std.testing.expect(state.storage.angular_velocity[b.index()].x.raw > 0);
}

test "ragdoll cone twist keeps an anchor and has bounded twist actuation" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const torso = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const limb = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    var joint = joints.Joint{ .kind = .cone_twist, .body_a = torso, .body_b = limb, .frame_a = .{}, .frame_b = .{}, .limit = .{}, .motor = .{ .enabled = true, .target_velocity = .one, .max_force = fp.Fp.fromInt(60) }, .spring = .{}, .cone_twist = .{ .enabled = true, .swing_max = g.pi.div(fp.Fp.fromInt(2), &status) } };
    var anchors: [3]constraints.ConstraintRow = undefined;
    var anchor_scratch: [3]constraints.ConstraintRow = undefined;
    const anchor_rows = try joints.buildConeTwistRows(&state, &joint, gravity.core.ids.JointId.init(0, 0), &anchors, &anchor_scratch, &status);
    try std.testing.expect(anchor_rows[0].ja_linear.x.raw != 0);
    var controls: [2]constraints.ConstraintRow = undefined;
    var control_scratch: [2]constraints.ConstraintRow = undefined;
    const rows = try joints.buildConeTwistControlRows(&state, &joint, gravity.core.ids.JointId.init(0, 0), tick(&status), &controls, &control_scratch, &status);
    try joints.solveRows(&state, controls[0..rows.len], .{}, &status);
    const cap = joints.motorImpulseCap(joint.motor, tick(&status), &status);
    try std.testing.expect(@abs(controls[0].accumulated_impulse.raw) <= cap.raw);
    try std.testing.expect(state.storage.angular_velocity[limb.index()].x.raw > 0);
}
