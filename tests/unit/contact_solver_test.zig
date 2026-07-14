const std = @import("std");
const gravity = @import("gravity");
const fp = gravity.math.fp;
const g = gravity.math.geometry;
const shapes = gravity.collision.shapes;
const world = gravity.dynamics.world;
const solver = gravity.dynamics.contact_solver;
const cache = gravity.collision.contact_cache;

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

test "normal friction angular impulse and split impulse are separated" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const b = try state.create(.{ .body_type = .static, .inverse_inertia_local = inertia() }, &status);
    f.linear[a.index()] = .{ .x = .one, .y = fp.Fp.fromInt(2) };
    var patch = cache.Patch{ .key = .{ .collider_a = .init(0, 0), .collider_b = .init(1, 0) }, .normal = .unit_x, .len = 1 };
    const points = [_]solver.Point{.{ .world_point = .{ .y = .one }, .penetration = fp.Fp.fromInt(1) }};
    var pseudo_linear: [4]g.Vec3 = undefined;
    var pseudo_angular: [4]g.Vec3 = undefined;
    var restitution: [1]fp.Fp = undefined;
    const real_before = f.linear[a.index()];
    try solver.solve(&state, &.{.{ .body_a = a, .body_b = b, .friction_a = .one, .friction_b = .one, .restitution_a = .zero, .restitution_b = .zero, .points = &points, .restitution_bias = &restitution, .patch = &patch }}, .{ .linear = &pseudo_linear, .angular = &pseudo_angular }, .{}, &status);
    try std.testing.expect(patch.points[0].normal_impulse.raw >= 0);
    const tangential_length = patch.points[0].tangent_first.mul(patch.points[0].tangent_first, &status).add(patch.points[0].tangent_second.mul(patch.points[0].tangent_second, &status), &status).sqrt(&status);
    try std.testing.expect(tangential_length.raw <= patch.points[0].normal_impulse.raw);
    try std.testing.expect(f.angular[a.index()].z.raw != 0);
    try std.testing.expect(pseudo_linear[a.index()].x.raw != 0);
    try std.testing.expect(f.linear[a.index()].x.raw != real_before.x.raw);
}

test "warm start is cached and invalid contact does not mutate velocities" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const b = try state.create(.{ .body_type = .static, .inverse_inertia_local = inertia() }, &status);
    var patch = cache.Patch{ .key = .{ .collider_a = .init(0, 0), .collider_b = .init(1, 0) }, .normal = .unit_x, .len = 1 };
    patch.points[0].normal_impulse = .one;
    const points = [_]solver.Point{.{ .world_point = .{} }};
    var pseudo_linear: [4]g.Vec3 = undefined;
    var pseudo_angular: [4]g.Vec3 = undefined;
    var restitution: [1]fp.Fp = undefined;
    try solver.solve(&state, &.{.{ .body_a = a, .body_b = b, .friction_a = .zero, .friction_b = .zero, .restitution_a = .zero, .restitution_b = .zero, .points = &points, .restitution_bias = &restitution, .patch = &patch }}, .{ .linear = &pseudo_linear, .angular = &pseudo_angular }, .{ .velocity_iterations = 0, .position_iterations = 0 }, &status);
    try std.testing.expect(f.linear[a.index()].x.raw < 0);
    const before = f.linear[a.index()];
    patch.sensor = true;
    try std.testing.expectError(error.InvalidContact, solver.solve(&state, &.{.{ .body_a = a, .body_b = b, .friction_a = .zero, .friction_b = .zero, .restitution_a = .zero, .restitution_b = .zero, .points = &points, .restitution_bias = &restitution, .patch = &patch }}, .{ .linear = &pseudo_linear, .angular = &pseudo_angular }, .{}, &status));
    try std.testing.expectEqualDeep(before, f.linear[a.index()]);
}

test "restitution and 100 to 1 mass ratio remain bounded" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const light = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const heavy_inverse = fp.Fp.fromRatio(1, 100, &status);
    const heavy = try state.create(.{ .inverse_mass = heavy_inverse, .inverse_inertia_local = .{ .xx = heavy_inverse, .yy = heavy_inverse, .zz = heavy_inverse, .xy = .zero, .xz = .zero, .yz = .zero } }, &status);
    f.linear[light.index()] = .{ .x = .one };
    var patch = cache.Patch{ .key = .{ .collider_a = .init(0, 0), .collider_b = .init(1, 0) }, .normal = .unit_x, .len = 1 };
    const point = [_]solver.Point{.{ .world_point = .{} }};
    var pseudo_linear: [4]g.Vec3 = undefined;
    var pseudo_angular: [4]g.Vec3 = undefined;
    var restitution: [1]fp.Fp = undefined;
    try solver.solve(&state, &.{.{ .body_a = light, .body_b = heavy, .friction_a = .zero, .friction_b = .zero, .restitution_a = .one, .restitution_b = .one, .points = &point, .restitution_bias = &restitution, .patch = &patch }}, .{ .linear = &pseudo_linear, .angular = &pseudo_angular }, .{ .restitution_threshold = .zero }, &status);
    try std.testing.expect(patch.points[0].normal_impulse.raw >= 0);
    try std.testing.expect(f.linear[light.index()].x.raw != fp.Fp.one.raw);
    try std.testing.expect(f.linear[heavy.index()].x.raw != 0);
    try std.testing.expect(f.linear[heavy.index()].x.raw < fp.Fp.fromInt(1).raw);
}

test "two-point inclined patch is deterministic and keeps each friction disk" {
    var first_f: Fixture = .{};
    var second_f: Fixture = .{};
    var first = try first_f.init();
    var second = try second_f.init();
    var status = fp.MathStatus{};
    const a = try first.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const wall = try first.create(.{ .body_type = .static, .inverse_inertia_local = inertia() }, &status);
    const b = try second.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &status);
    const other_wall = try second.create(.{ .body_type = .static, .inverse_inertia_local = inertia() }, &status);
    first_f.linear[a.index()] = .{ .x = .one, .y = .one, .z = fp.Fp.fromInt(2) };
    second_f.linear[b.index()] = first_f.linear[a.index()];
    const normal = (g.Vec3{ .x = .one, .y = .one }).normalize(&status).value;
    var left_patch = cache.Patch{ .key = .{ .collider_a = .init(0, 0), .collider_b = .init(1, 0) }, .normal = normal, .len = 2 };
    var right_patch = left_patch;
    const points = [_]solver.Point{ .{ .world_point = .{ .y = .one } }, .{ .world_point = .{ .z = .one } } };
    var pseudo_a_linear: [4]g.Vec3 = undefined;
    var pseudo_a_angular: [4]g.Vec3 = undefined;
    var pseudo_b_linear: [4]g.Vec3 = undefined;
    var pseudo_b_angular: [4]g.Vec3 = undefined;
    var bias_a: [2]fp.Fp = undefined;
    var bias_b: [2]fp.Fp = undefined;
    try solver.solve(&first, &.{.{ .body_a = a, .body_b = wall, .friction_a = .one, .friction_b = .one, .restitution_a = .zero, .restitution_b = .zero, .points = &points, .restitution_bias = &bias_a, .patch = &left_patch }}, .{ .linear = &pseudo_a_linear, .angular = &pseudo_a_angular }, .{}, &status);
    try solver.solve(&second, &.{.{ .body_a = b, .body_b = other_wall, .friction_a = .one, .friction_b = .one, .restitution_a = .zero, .restitution_b = .zero, .points = &points, .restitution_bias = &bias_b, .patch = &right_patch }}, .{ .linear = &pseudo_b_linear, .angular = &pseudo_b_angular }, .{}, &status);
    try std.testing.expectEqualDeep(first_f.linear[a.index()], second_f.linear[b.index()]);
    try std.testing.expectEqual(left_patch.len, right_patch.len);
    for (left_patch.points[0..left_patch.len], right_patch.points[0..right_patch.len]) |left, right| try std.testing.expectEqualDeep(left, right);
    for (left_patch.points[0..left_patch.len]) |item| {
        const length = item.tangent_first.mul(item.tangent_first, &status).add(item.tangent_second.mul(item.tangent_second, &status), &status).sqrt(&status);
        try std.testing.expect(length.raw <= item.normal_impulse.raw + 1);
    }
}

test "two body resting stack keeps bounded canonical contact impulses" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const ground = try state.create(.{ .body_type = .static, .inverse_inertia_local = inertia() }, &status);
    const lower = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia(), .transform = .{ .position = .{ .y = .one } } }, &status);
    const upper = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia(), .transform = .{ .position = .{ .y = fp.Fp.fromInt(2) } } }, &status);
    f.linear[lower.index()].y = fp.Fp.fromInt(-1);
    f.linear[upper.index()].y = fp.Fp.fromInt(-1);
    var ground_patch = cache.Patch{ .key = .{ .collider_a = .init(1, 0), .collider_b = .init(0, 0) }, .normal = .{ .y = fp.Fp.fromInt(-1) }, .len = 1 };
    var upper_patch = cache.Patch{ .key = .{ .collider_a = .init(2, 0), .collider_b = .init(1, 0) }, .normal = .{ .y = fp.Fp.fromInt(-1) }, .len = 1 };
    const ground_point = [_]solver.Point{.{ .world_point = .{} }};
    const upper_point = [_]solver.Point{.{ .world_point = .{ .y = fp.Fp.fromInt(3) } }};
    var pseudo_linear: [4]g.Vec3 = undefined;
    var pseudo_angular: [4]g.Vec3 = undefined;
    var ground_bias: [1]fp.Fp = undefined;
    var upper_bias: [1]fp.Fp = undefined;
    var tick: usize = 0;
    while (tick < 30) : (tick += 1) try solver.solve(&state, &.{ .{ .body_a = lower, .body_b = ground, .friction_a = .one, .friction_b = .one, .restitution_a = .zero, .restitution_b = .zero, .points = &ground_point, .restitution_bias = &ground_bias, .patch = &ground_patch }, .{ .body_a = upper, .body_b = lower, .friction_a = .one, .friction_b = .one, .restitution_a = .zero, .restitution_b = .zero, .points = &upper_point, .restitution_bias = &upper_bias, .patch = &upper_patch } }, .{ .linear = &pseudo_linear, .angular = &pseudo_angular }, .{}, &status);
    try std.testing.expect(ground_patch.points[0].normal_impulse.raw >= 0 and upper_patch.points[0].normal_impulse.raw >= 0);
    try std.testing.expect(f.linear[lower.index()].y.raw >= -2 and f.linear[upper.index()].y.raw >= -2);
}
