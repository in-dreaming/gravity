const std = @import("std");
const gravity = @import("gravity");
const fp = gravity.math.fp;
const g = gravity.math.geometry;
const shapes = gravity.collision.shapes;
const world = gravity.dynamics.world;
const constraints = gravity.dynamics.constraints;
const sleeping = gravity.dynamics.sleeping;
const solver = gravity.dynamics.contact_solver;
const cache = gravity.collision.contact_cache;
const joints = gravity.dynamics.joints;

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
    awake: [4]bool = undefined,
    counter: [4]u32 = undefined,
    reason: [4]sleeping.WakeReason = undefined,
    fn init(self: *Fixture) !world.World {
        try sleeping.init(.{ .awake = &self.awake, .counter = &self.counter, .reason = &self.reason });
        return world.World.init(.{ .body_type = &self.types, .position = &self.position, .orientation = &self.orientation, .linear_velocity = &self.linear, .angular_velocity = &self.angular, .inverse_mass = &self.mass, .inverse_inertia_local = &self.inertia, .force = &self.force, .torque = &self.torque, .locks = &self.locks, .generation = &self.generation, .alive = &self.alive, .retired = &self.retired, .has_target = &self.target, .target_position = &self.target_position, .target_orientation = &self.target_orientation });
    }
    fn sleep(self: *Fixture) sleeping.Storage {
        return .{ .awake = &self.awake, .counter = &self.counter, .reason = &self.reason };
    }
};
fn inertia() g.SymmetricMat3 {
    return .{ .xx = .one, .yy = .one, .zz = .one, .xy = .zero, .xz = .zero, .yz = .zero };
}
fn dynamic(state: *world.World, status: *fp.MathStatus) !gravity.core.ids.BodyId {
    return state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, status);
}

test "island sleeps on exact threshold and clears dynamics" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try dynamic(&state, &status);
    const b = try dynamic(&state, &status);
    const islands = [_]constraints.Island{.{ .id = a, .first_member = 0, .member_count = 2 }};
    const members = [_]gravity.core.ids.BodyId{ a, b };
    var events: [2]sleeping.Event = undefined;
    state.storage.force[0].x = .one;
    state.storage.torque[1].y = .one;
    try std.testing.expectEqual(@as(usize, 0), (try sleeping.step(&state, &islands, &members, f.sleep(), .one, .one, 3, &events, &status)).len);
    _ = try sleeping.step(&state, &islands, &members, f.sleep(), .one, .one, 3, &events, &status);
    const slept = try sleeping.step(&state, &islands, &members, f.sleep(), .one, .one, 3, &events, &status);
    try std.testing.expectEqual(@as(usize, 2), slept.len);
    try std.testing.expect(!f.awake[0]);
    try std.testing.expectEqual(@as(i64, 0), state.storage.force[0].x.raw);
    try std.testing.expectEqual(@as(i64, 0), state.storage.torque[1].y.raw);
}

test "movement resets every island counter" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try dynamic(&state, &status);
    const b = try dynamic(&state, &status);
    const islands = [_]constraints.Island{.{ .id = a, .first_member = 0, .member_count = 2 }};
    const members = [_]gravity.core.ids.BodyId{ a, b };
    var events: [2]sleeping.Event = undefined;
    _ = try sleeping.step(&state, &islands, &members, f.sleep(), .one, .one, 3, &events, &status);
    state.storage.linear_velocity[1].x = fp.Fp.fromInt(2);
    _ = try sleeping.step(&state, &islands, &members, f.sleep(), .one, .one, 3, &events, &status);
    try std.testing.expectEqual(@as(u32, 0), f.counter[0]);
    try std.testing.expectEqual(@as(u32, 0), f.counter[1]);
}

test "island merge synchronizes the lowest counter and split preserves it" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try dynamic(&state, &status);
    const b = try dynamic(&state, &status);
    const separate = [_]constraints.Island{ .{ .id = a, .first_member = 0, .member_count = 1 }, .{ .id = b, .first_member = 1, .member_count = 1 } };
    const merged = [_]constraints.Island{.{ .id = a, .first_member = 0, .member_count = 2 }};
    const members = [_]gravity.core.ids.BodyId{ a, b };
    var events: [2]sleeping.Event = undefined;
    _ = try sleeping.step(&state, &separate, &members, f.sleep(), .one, .one, 30, &events, &status);
    _ = try sleeping.step(&state, &separate, &members, f.sleep(), .one, .one, 30, &events, &status);
    state.storage.linear_velocity[1].x = fp.Fp.fromInt(2);
    _ = try sleeping.step(&state, &separate, &members, f.sleep(), .one, .one, 30, &events, &status);
    state.storage.linear_velocity[1] = .{};
    _ = try sleeping.step(&state, &merged, &members, f.sleep(), .one, .one, 30, &events, &status);
    try std.testing.expectEqual(@as(u32, 1), f.counter[0]);
    try std.testing.expectEqual(@as(u32, 1), f.counter[1]);
    _ = try sleeping.step(&state, &separate, &members, f.sleep(), .one, .one, 30, &events, &status);
    try std.testing.expectEqual(@as(u32, 2), f.counter[0]);
    try std.testing.expectEqual(@as(u32, 2), f.counter[1]);
}

test "wake graph wakes complete dynamic component once with first reason" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try dynamic(&state, &status);
    const b = try dynamic(&state, &status);
    const c = try dynamic(&state, &status);
    f.awake[0] = false;
    f.awake[1] = false;
    f.awake[2] = false;
    const edges = [_]constraints.Edge{ .{ .kind = .joint, .body_a = a, .body_b = b }, .{ .kind = .contact, .body_a = b, .body_b = c } };
    const requests = [_]sleeping.Request{ .{ .body = c, .reason = .contact }, .{ .body = a, .reason = .command } };
    var scratch: [4]gravity.core.ids.BodyId = undefined;
    var events: [4]sleeping.Event = undefined;
    const woke = try sleeping.wakeGraph(&state, f.sleep(), &edges, &requests, &scratch, &events);
    try std.testing.expectEqual(@as(usize, 3), woke.len);
    for (woke, 0..) |event, index| {
        try std.testing.expectEqual(sleeping.WakeReason.contact, event.reason);
        try std.testing.expectEqual(@as(u32, @intCast(index)), event.body.index());
    }
}

test "sleep capacity failure is transactional and visitor is stable" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try dynamic(&state, &status);
    const b = try dynamic(&state, &status);
    const islands = [_]constraints.Island{.{ .id = a, .first_member = 0, .member_count = 2 }};
    const members = [_]gravity.core.ids.BodyId{ a, b };
    var no_events: [0]sleeping.Event = .{};
    try std.testing.expectError(error.CapacityExceeded, sleeping.step(&state, &islands, &members, f.sleep(), .one, .one, 1, &no_events, &status));
    try std.testing.expect(f.awake[0] and f.awake[1]);
    try std.testing.expectEqual(@as(u32, 0), f.counter[0]);
    const Visitor = struct {
        count: usize = 0,
        pub fn field(self: *@This(), _: []const u8, _: anytype) void {
            self.count += 1;
        }
    };
    var first = Visitor{};
    var second = Visitor{};
    sleeping.visitCanonical(f.sleep(), &first);
    sleeping.visitCanonical(f.sleep(), &second);
    try std.testing.expectEqual(first.count, second.count);
}

test "active island selection excludes a sleeping component" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try dynamic(&state, &status);
    const b = try dynamic(&state, &status);
    const c = try dynamic(&state, &status);
    f.awake[0] = false;
    f.awake[1] = false;
    const islands = [_]constraints.Island{ .{ .id = a, .first_member = 0, .member_count = 2 }, .{ .id = c, .first_member = 2, .member_count = 1 } };
    const members = [_]gravity.core.ids.BodyId{ a, b, c };
    var out_islands: [2]constraints.Island = undefined;
    var out_members: [3]gravity.core.ids.BodyId = undefined;
    const active = try sleeping.selectActive(&state, &islands, &members, f.sleep(), &out_islands, &out_members);
    try std.testing.expectEqual(@as(usize, 1), active.islands.len);
    try std.testing.expectEqual(c.value, active.islands[0].id.value);
    try std.testing.expectEqual(@as(usize, 1), active.members.len);
}

test "nonzero command wakes before force commit" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const body = try dynamic(&state, &status);
    f.awake[0] = false;
    const commands = [_]world.Command{.{ .key = .{ .phase_priority = 0, .issuer = 1, .sequence = 1, .discriminant = 0 }, .op = .{ .force = .{ .body = body, .value = .{ .x = .one } } } }};
    var command_scratch: [1]world.Command = undefined;
    var requests: [1]sleeping.Request = undefined;
    var graph_scratch: [4]gravity.core.ids.BodyId = undefined;
    var events: [1]sleeping.Event = undefined;
    const woke = try sleeping.executeCommands(&state, f.sleep(), &.{}, &commands, &command_scratch, &requests, &graph_scratch, &events, fp.Fp.one, &status);
    try std.testing.expectEqual(@as(usize, 1), woke.len);
    try std.testing.expectEqual(sleeping.WakeReason.command, woke[0].reason);
    try std.testing.expect(f.awake[0]);
    try std.testing.expectEqual(fp.Fp.one.raw, state.storage.force[0].x.raw);
}

test "awake mask excludes sleeping bodies from integration work" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    _ = try dynamic(&state, &status);
    state.storage.linear_velocity[0].x = .one;
    f.awake[0] = false;
    try state.stepAwake(&f.awake, fp.Fp.one, &status);
    try std.testing.expectEqual(@as(i64, 0), state.storage.position[0].x.raw);
    f.awake[0] = true;
    try state.stepAwake(&f.awake, fp.Fp.one, &status);
    try std.testing.expectEqual(fp.Fp.one.raw, state.storage.position[0].x.raw);
}

test "disabled sleeping keeps the all-awake oracle state" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const body = try dynamic(&state, &status);
    f.awake[0] = false;
    f.counter[0] = 29;
    const islands = [_]constraints.Island{.{ .id = body, .first_member = 0, .member_count = 1 }};
    const members = [_]gravity.core.ids.BodyId{body};
    var simulation = gravity.core.config.SimulationConfig.default;
    simulation.features.sleeping = false;
    var events: [1]sleeping.Event = undefined;
    const result = try sleeping.stepConfigured(&state, &islands, &members, f.sleep(), simulation, &events, &status);
    try std.testing.expectEqual(@as(usize, 0), result.len);
    try std.testing.expect(f.awake[0]);
    try std.testing.expectEqual(@as(u32, 0), f.counter[0]);
}

test "response contacts wake while sensor contacts cannot wake" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try dynamic(&state, &status);
    const b = try dynamic(&state, &status);
    f.awake[0] = false;
    f.awake[1] = false;
    var patch = cache.Patch{ .key = .{ .collider_a = .init(0, 0), .collider_b = .init(1, 0) }, .normal = .unit_x, .len = 1 };
    const points = [_]solver.Point{.{ .world_point = .{} }};
    var restitution: [1]fp.Fp = undefined;
    var pseudo_linear: [4]g.Vec3 = undefined;
    var pseudo_angular: [4]g.Vec3 = undefined;
    const contact = solver.Contact{ .body_a = a, .body_b = b, .friction_a = .zero, .friction_b = .zero, .restitution_a = .zero, .restitution_b = .zero, .points = &points, .restitution_bias = &restitution, .patch = &patch };
    var requests: [2]sleeping.Request = undefined;
    var graph_scratch: [4]gravity.core.ids.BodyId = undefined;
    var events: [2]sleeping.Event = undefined;
    const woke = try sleeping.solveContacts(&state, f.sleep(), &.{.{ .kind = .contact, .body_a = a, .body_b = b }}, &.{contact}, .{ .linear = &pseudo_linear, .angular = &pseudo_angular }, .{ .velocity_iterations = 0, .position_iterations = 0 }, &requests, &graph_scratch, &events, &status);
    try std.testing.expectEqual(@as(usize, 2), woke.len);
    try std.testing.expect(f.awake[0] and f.awake[1]);
    f.awake[0] = false;
    f.awake[1] = false;
    patch.sensor = true;
    try std.testing.expectError(error.InvalidContact, sleeping.solveContacts(&state, f.sleep(), &.{.{ .kind = .contact, .body_a = a, .body_b = b }}, &.{contact}, .{ .linear = &pseudo_linear, .angular = &pseudo_angular }, .{}, &requests, &graph_scratch, &events, &status));
    try std.testing.expect(!f.awake[0] and !f.awake[1]);
}

test "active joint motor wakes its sleeping component" {
    var f: Fixture = .{};
    var state = try f.init();
    var status = fp.MathStatus{};
    const a = try dynamic(&state, &status);
    const b = try dynamic(&state, &status);
    f.awake[0] = false;
    f.awake[1] = false;
    var values: [1]joints.Joint = undefined;
    var generation: [1]u32 = undefined;
    var alive: [1]bool = undefined;
    var retired: [1]bool = undefined;
    var pool = try joints.Pool.init(.{ .values = &values, .generation = &generation, .alive = &alive, .retired = &retired });
    pool.storage.values[0] = .{ .kind = .distance, .body_a = a, .body_b = b, .frame_a = .{}, .frame_b = .{}, .motor = .{ .enabled = true, .max_force = .one }, .limit = .{}, .spring = .{} };
    pool.storage.alive[0] = true;
    var requests: [2]sleeping.Request = undefined;
    var graph_scratch: [4]gravity.core.ids.BodyId = undefined;
    var events: [2]sleeping.Event = undefined;
    const woke = try sleeping.wakeActiveJoints(&state, f.sleep(), &.{.{ .kind = .joint, .body_a = a, .body_b = b }}, &pool, &requests, &graph_scratch, &events);
    try std.testing.expectEqual(@as(usize, 2), woke.len);
    for (woke) |event| try std.testing.expectEqual(sleeping.WakeReason.joint, event.reason);
}
