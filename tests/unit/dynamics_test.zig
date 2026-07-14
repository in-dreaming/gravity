const std = @import("std");
const gravity = @import("gravity");
const fp = gravity.math.fp;
const g = gravity.math.geometry;
const shapes = gravity.collision.shapes;
const world = gravity.dynamics.world;

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
    targets: [4]bool = undefined,
    target_position: [4]g.Vec3 = undefined,
    target_orientation: [4]g.Quat = undefined,
    fn init(self: *Fixture) !world.World {
        return world.World.init(.{ .body_type = &self.types, .position = &self.position, .orientation = &self.orientation, .linear_velocity = &self.linear, .angular_velocity = &self.angular, .inverse_mass = &self.mass, .inverse_inertia_local = &self.inertia, .force = &self.force, .torque = &self.torque, .locks = &self.locks, .generation = &self.generation, .alive = &self.alive, .retired = &self.retired, .has_target = &self.targets, .target_position = &self.target_position, .target_orientation = &self.target_orientation });
    }
};
fn unitInertia() g.SymmetricMat3 {
    return .{ .xx = fp.Fp.one, .yy = fp.Fp.one, .zz = fp.Fp.one, .xy = .zero, .xz = .zero, .yz = .zero };
}

test "force impulse locks and generation lifecycle are deterministic" {
    var fixture: Fixture = .{};
    var state = try fixture.init();
    var status = fp.MathStatus{};
    const body = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia(), .locks = .{ .linear_z = true } }, &status);
    try state.applyImpulseAtPoint(body, .{ .x = .one }, .{ .y = .one }, &status);
    try std.testing.expectEqual(fp.Fp.one.raw, fixture.linear[body.index()].x.raw);
    try std.testing.expectEqual(-fp.Fp.one.raw, fixture.angular[body.index()].z.raw);
    try state.applyForce(body, .{ .z = .one }, &status);
    state.step(.one, &status);
    try std.testing.expectEqual(@as(i64, 0), fixture.position[body.index()].z.raw);
    try state.destroy(body);
    try std.testing.expectError(error.InvalidBody, state.applyForce(body, .{}, &status));
    const replacement = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia() }, &status);
    try std.testing.expect(replacement.generation() != body.generation());
}

test "command ordering is canonical and invalid batches are transactional" {
    var left_fixture: Fixture = .{};
    var right_fixture: Fixture = .{};
    var left = try left_fixture.init();
    var right = try right_fixture.init();
    var status = fp.MathStatus{};
    const a = try left.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia() }, &status);
    const b = try right.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia() }, &status);
    var scratch: [3]world.Command = undefined;
    const first = [_]world.Command{ .{ .key = .{ .phase_priority = 1, .issuer = 1, .sequence = 2, .discriminant = 0 }, .op = .{ .force = .{ .body = a, .value = .{ .x = .one } } } }, .{ .key = .{ .phase_priority = 1, .issuer = 1, .sequence = 1, .discriminant = 0 }, .op = .{ .force = .{ .body = a, .value = .{ .y = .one } } } } };
    const reverse = [_]world.Command{ first[1], first[0] };
    try left.execute(&first, &scratch, .one, &status);
    try right.execute(&reverse, &scratch, .one, &status);
    try std.testing.expectEqualDeep(left_fixture.force[a.index()], right_fixture.force[b.index()]);
    const invalid = [_]world.Command{ first[0], .{ .key = .{ .phase_priority = 2, .issuer = 1, .sequence = 1, .discriminant = 0 }, .op = .{ .force = .{ .body = gravity.core.ids.BodyId.invalid, .value = .{ .x = .one } } } } };
    const before = left_fixture.force[a.index()];
    try std.testing.expectError(error.InvalidBody, left.execute(&invalid, &scratch, .one, &status));
    try std.testing.expectEqualDeep(before, left_fixture.force[a.index()]);
}

test "kinematic target snaps exactly and preserves canonical quaternion" {
    var fixture: Fixture = .{};
    var state = try fixture.init();
    var status = fp.MathStatus{};
    const kinematic = try state.create(.{ .body_type = .kinematic, .inverse_inertia_local = unitInertia() }, &status);
    var scratch: [1]world.Command = undefined;
    const target = g.Transform3{ .position = .{ .x = fp.Fp.fromInt(3) }, .orientation = .{ .w = fp.Fp.fromInt(-1) } };
    try state.execute(&.{.{ .key = .{ .phase_priority = 0, .issuer = 0, .sequence = 0, .discriminant = 0 }, .op = .{ .kinematic_target = .{ .body = kinematic, .target = target } } }}, &scratch, .one, &status);
    state.step(.one, &status);
    try std.testing.expectEqual(target.position.x.raw, fixture.position[kinematic.index()].x.raw);
    try std.testing.expect(fixture.orientation[kinematic.index()].w.raw > 0);
}

test "free non-spherical body receives the gyroscopic term" {
    var fixture: Fixture = .{};
    var state = try fixture.init();
    var status = fp.MathStatus{};
    const anisotropic: g.SymmetricMat3 = .{ .xx = fp.Fp.one, .yy = fp.Fp.fromRatio(1, 2, &status), .zz = fp.Fp.fromRatio(1, 3, &status), .xy = .zero, .xz = .zero, .yz = .zero };
    const body = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = anisotropic }, &status);
    var scratch: [1]world.Command = undefined;
    try state.execute(&.{.{ .key = .{ .phase_priority = 0, .issuer = 0, .sequence = 0, .discriminant = 0 }, .op = .{ .velocity = .{ .body = body, .linear = .{}, .angular = .{ .x = .one, .y = .one } } } }}, &scratch, fp.Fp.fromRatio(1, 60, &status), &status);
    const dt = fp.Fp.fromRatio(1, 60, &status);
    state.step(dt, &status);
    const expected_z = fp.Fp.fromRatio(-1, 3, &status).mul(dt, &status);
    try std.testing.expectEqual(expected_z.raw, fixture.angular[body.index()].z.raw);
    try std.testing.expect(fixture.orientation[body.index()].w.raw > 0);
}

test "static kinematic and 2D DOF boundaries remain explicit" {
    var fixture: Fixture = .{};
    var state = try fixture.init();
    var status = fp.MathStatus{};
    try std.testing.expectError(error.InvalidMass, state.create(.{ .inverse_mass = .one, .inverse_inertia_local = .{ .xx = .one, .yy = .zero, .zz = .one, .xy = .zero, .xz = .zero, .yz = .zero } }, &status));
    try std.testing.expect(!fixture.alive[0]);
    const static_body = try state.create(.{ .body_type = .static, .inverse_inertia_local = unitInertia() }, &status);
    const kinematic = try state.create(.{ .body_type = .kinematic, .inverse_inertia_local = unitInertia() }, &status);
    const planar = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia(), .locks = .{ .linear_z = true, .angular_x = true, .angular_y = true } }, &status);
    try std.testing.expectError(error.InvalidBody, state.applyForce(static_body, .{ .x = .one }, &status));
    try std.testing.expectError(error.InvalidBody, state.applyForce(kinematic, .{ .x = .one }, &status));
    var scratch: [1]world.Command = undefined;
    try state.execute(&.{.{ .key = .{ .phase_priority = 0, .issuer = 0, .sequence = 0, .discriminant = 0 }, .op = .{ .velocity = .{ .body = planar, .linear = .{ .z = .one }, .angular = .{ .x = .one, .y = .one, .z = .one } } } }}, &scratch, .one, &status);
    try std.testing.expectEqual(@as(i64, 0), fixture.linear[planar.index()].z.raw);
    try std.testing.expectEqual(@as(i64, 0), fixture.angular[planar.index()].x.raw);
    try std.testing.expectEqual(@as(i64, 0), fixture.angular[planar.index()].y.raw);
    try std.testing.expectEqual(fp.Fp.one.raw, fixture.angular[planar.index()].z.raw);
}

test "collider SoA validates ownership and follows body destroy cascade" {
    var fixture: Fixture = .{};
    var status = fp.MathStatus{};
    var body: [1]gravity.core.ids.BodyId = undefined;
    var local: [1]g.Transform3 = undefined;
    var shape: [1]shapes.Shape = undefined;
    var material: [1]shapes.Material = undefined;
    var category: [1]u32 = undefined;
    var mask: [1]u32 = undefined;
    var group: [1]i32 = undefined;
    var sensor: [1]bool = undefined;
    var enabled: [1]bool = undefined;
    var revision: [1]u32 = undefined;
    var generation: [1]u32 = undefined;
    var alive: [1]bool = undefined;
    var retired: [1]bool = undefined;
    var state = try world.World.initWithColliders(.{ .body_type = &fixture.types, .position = &fixture.position, .orientation = &fixture.orientation, .linear_velocity = &fixture.linear, .angular_velocity = &fixture.angular, .inverse_mass = &fixture.mass, .inverse_inertia_local = &fixture.inertia, .force = &fixture.force, .torque = &fixture.torque, .locks = &fixture.locks, .generation = &fixture.generation, .alive = &fixture.alive, .retired = &fixture.retired, .has_target = &fixture.targets, .target_position = &fixture.target_position, .target_orientation = &fixture.target_orientation }, .{ .body = &body, .local = &local, .shape = &shape, .material = &material, .category = &category, .mask = &mask, .group = &group, .sensor = &sensor, .enabled = &enabled, .revision = &revision, .generation = &generation, .alive = &alive, .retired = &retired });
    const owner = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = unitInertia() }, &status);
    try std.testing.expectError(error.InvalidBodyShape, state.createCollider(.{ .body = owner, .shape = .{ .height_field = .{ .source_id = 91 } } }));
    try std.testing.expect(!alive[0]);
    const collider = try state.createCollider(.{ .body = owner, .shape = .{ .sphere = .{ .radius = .one } } });
    try std.testing.expectEqual(owner.value, body[0].value);
    try state.destroy(owner);
    try std.testing.expect(!alive[0]);
    try std.testing.expectError(error.InvalidCollider, state.destroyCollider(collider));
}
