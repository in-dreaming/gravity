//! Fixed-iteration 3D sequential-impulse contact solver.
const fp = @import("../math/fp.zig");
const geometry = @import("../math/geometry.zig");
const ids = @import("../core/ids.zig");
const cache = @import("../collision/contact_cache.zig");
const body_world = @import("world.zig");
const joints = @import("joints.zig");
const constraints = @import("constraints.zig");

pub const Error = error{ CapacityExceeded, InvalidBody, InvalidContact };
pub const Settings = struct { velocity_iterations: u8 = 10, position_iterations: u8 = 4, restitution_threshold: fp.Fp = fp.Fp.one, max_position_correction: fp.Fp = .{ .raw = 858_993_459 }, baumgarte: fp.Fp = .{ .raw = 858_993_459 } };
pub const Point = struct { world_point: geometry.Vec3, penetration: fp.Fp = .zero };
/// A contact references its persistent Task 12 patch. Point count and feature
/// ordering are inherited from that cache and must already be canonical.
pub const Contact = struct { body_a: ids.BodyId, body_b: ids.BodyId, friction_a: fp.Fp, friction_b: fp.Fp, restitution_a: fp.Fp, restitution_b: fp.Fp, points: []const Point, restitution_bias: []fp.Fp, patch: *cache.Patch };
pub const PseudoVelocities = struct { linear: []geometry.Vec3, angular: []geometry.Vec3 };

pub fn solve(world: *body_world.World, contacts: []const Contact, pseudo: PseudoVelocities, settings: Settings, status: *fp.MathStatus) Error!void {
    try validateInputs(world, contacts, pseudo);
    @memset(pseudo.linear, geometry.Vec3.zero);
    @memset(pseudo.angular, geometry.Vec3.zero);
    for (contacts) |contact| warmStart(world, contact, status);
    for (contacts) |contact| prepareRestitution(world, contact, settings, status);
    var i: u8 = 0;
    while (i < settings.velocity_iterations) : (i += 1) for (contacts) |contact| solveVelocityContact(world, contact, status);
    i = 0;
    while (i < settings.position_iterations) : (i += 1) for (contacts) |contact| solveSplitContact(world, contact, pseudo, settings, status);
}
/// Solves canonical joint rows before contact rows on every fixed velocity
/// iteration. This is the World pipeline's single PGS ordering; using two
/// independently iterated solvers would change the released joint/contact
/// coupling and violate the frozen row order.
pub fn solveWithJointRows(world: *body_world.World, joint_rows: []constraints.ConstraintRow, contacts: []const Contact, pseudo: PseudoVelocities, settings: Settings, status: *fp.MathStatus) (Error || joints.Error || constraints.Error)!void {
    try validateInputs(world, contacts, pseudo);
    try joints.validateRows(world, joint_rows);
    @memset(pseudo.linear, geometry.Vec3.zero);
    @memset(pseudo.angular, geometry.Vec3.zero);
    try joints.warmStartRows(world, joint_rows, status);
    for (contacts) |contact| warmStart(world, contact, status);
    for (contacts) |contact| prepareRestitution(world, contact, settings, status);
    var i: u8 = 0;
    while (i < settings.velocity_iterations) : (i += 1) {
        try joints.solveRowsIteration(world, joint_rows, status);
        for (contacts) |contact| solveVelocityContact(world, contact, status);
    }
    i = 0;
    while (i < settings.position_iterations) : (i += 1) for (contacts) |contact| solveSplitContact(world, contact, pseudo, settings, status);
}
/// Solves a contact introduced after the ordinary substep warm-start (for
/// example a CCD TOI). Joint rows retain their accumulated impulses and are
/// therefore deliberately not warm-started a second time.  The same fixed
/// joint-before-contact PGS order is still used for every velocity iteration.
pub fn solveAdditionalContactWithJointRows(world: *body_world.World, joint_rows: []constraints.ConstraintRow, contacts: []const Contact, pseudo: PseudoVelocities, settings: Settings, status: *fp.MathStatus) (Error || joints.Error || constraints.Error)!void {
    try validateInputs(world, contacts, pseudo);
    try joints.validateRows(world, joint_rows);
    @memset(pseudo.linear, geometry.Vec3.zero);
    @memset(pseudo.angular, geometry.Vec3.zero);
    for (contacts) |contact| warmStart(world, contact, status);
    for (contacts) |contact| prepareRestitution(world, contact, settings, status);
    var i: u8 = 0;
    while (i < settings.velocity_iterations) : (i += 1) {
        try joints.solveRowsIteration(world, joint_rows, status);
        for (contacts) |contact| solveVelocityContact(world, contact, status);
    }
    i = 0;
    while (i < settings.position_iterations) : (i += 1) for (contacts) |contact| solveSplitContact(world, contact, pseudo, settings, status);
}
/// Validates all solver input without mutating velocities or cached impulses.
/// Pipeline phases use this before reserving deterministic wake events.
pub fn validateInputs(world: *const body_world.World, contacts: []const Contact, pseudo: PseudoVelocities) Error!void {
    if (pseudo.linear.len != world.storage.alive.len or pseudo.angular.len != world.storage.alive.len) return error.CapacityExceeded;
    for (contacts) |contact| try validate(world, contact);
}
fn validate(world: *const body_world.World, contact: Contact) Error!void {
    if (contact.patch.len != contact.points.len or contact.restitution_bias.len != contact.points.len or contact.patch.len > 4 or contact.patch.sensor) return error.InvalidContact;
    _ = world.bodyIndex(contact.body_a) orelse return error.InvalidBody;
    _ = world.bodyIndex(contact.body_b) orelse return error.InvalidBody;
    if (contact.friction_a.raw < 0 or contact.friction_b.raw < 0 or contact.restitution_a.raw < 0 or contact.restitution_b.raw < 0) return error.InvalidContact;
}
fn warmStart(world: *body_world.World, contact: Contact, status: *fp.MathStatus) void {
    const basis = cache.tangentBasis(contact.patch.normal, status);
    for (contact.points, 0..) |point, i| {
        const p = contact.patch.points[i];
        const impulse = basis.normal.scale(p.normal_impulse, status).add(basis.first.scale(p.tangent_first, status), status).add(basis.second.scale(p.tangent_second, status), status);
        apply(world, contact, point.world_point, impulse, status);
    }
}
fn prepareRestitution(world: *const body_world.World, contact: Contact, settings: Settings, status: *fp.MathStatus) void {
    const restitution = if (contact.restitution_a.raw > contact.restitution_b.raw) contact.restitution_a else contact.restitution_b;
    for (contact.points, 0..) |point, i| {
        const velocity = relativeVelocity(world, contact, point.world_point, contact.patch.normal, status);
        contact.restitution_bias[i] = if (velocity.raw < settings.restitution_threshold.neg(status).raw) velocity.mul(restitution, status) else .zero;
    }
}
fn solveVelocityContact(world: *body_world.World, contact: Contact, status: *fp.MathStatus) void {
    const basis = cache.tangentBasis(contact.patch.normal, status);
    const friction = contact.friction_a.mul(contact.friction_b, status).sqrt(status);
    for (contact.points, 0..) |point, i| {
        var saved = &contact.patch.points[i];
        const normal_velocity = relativeVelocity(world, contact, point.world_point, basis.normal, status);
        const normal_delta = normal_velocity.add(contact.restitution_bias[i], status).neg(status).mul(effective(world, contact, point.world_point, basis.normal, status), status);
        const old_normal = saved.normal_impulse;
        saved.normal_impulse = maxZero(old_normal.add(normal_delta, status));
        apply(world, contact, point.world_point, basis.normal.scale(saved.normal_impulse.sub(old_normal, status), status), status);
        const old_first = saved.tangent_first;
        const old_second = saved.tangent_second;
        saved.tangent_first = saved.tangent_first.add(relativeVelocity(world, contact, point.world_point, basis.first, status).neg(status).mul(effective(world, contact, point.world_point, basis.first, status), status), status);
        saved.tangent_second = saved.tangent_second.add(relativeVelocity(world, contact, point.world_point, basis.second, status).neg(status).mul(effective(world, contact, point.world_point, basis.second, status), status), status);
        const limit = friction.mul(saved.normal_impulse, status);
        const length = saved.tangent_first.mul(saved.tangent_first, status).add(saved.tangent_second.mul(saved.tangent_second, status), status).sqrt(status);
        if (length.raw > limit.raw and length.raw > 0) {
            const scale = limit.div(length, status);
            saved.tangent_first = saved.tangent_first.mul(scale, status);
            saved.tangent_second = saved.tangent_second.mul(scale, status);
        }
        apply(world, contact, point.world_point, basis.first.scale(saved.tangent_first.sub(old_first, status), status).add(basis.second.scale(saved.tangent_second.sub(old_second, status), status), status), status);
    }
}
fn solveSplitContact(world: *const body_world.World, contact: Contact, pseudo: PseudoVelocities, settings: Settings, status: *fp.MathStatus) void {
    const n = contact.patch.normal;
    for (contact.points) |point| {
        if (point.penetration.raw <= 0) continue;
        const a = world.bodyIndex(contact.body_a).?;
        const b = world.bodyIndex(contact.body_b).?;
        const correction = point.penetration.mul(settings.baumgarte, status);
        const target = if (correction.raw > settings.max_position_correction.raw) settings.max_position_correction else correction;
        const velocity = pointVelocity(world, b, point.world_point, pseudo.linear[b], pseudo.angular[b], status).sub(pointVelocity(world, a, point.world_point, pseudo.linear[a], pseudo.angular[a], status), status).dot(n, status);
        const delta = target.sub(velocity, status).mul(effective(world, contact, point.world_point, n, status), status);
        applyPseudo(world, contact, pseudo, point.world_point, n.scale(maxZero(delta), status), status);
    }
}
fn effective(world: *const body_world.World, contact: Contact, point: geometry.Vec3, axis: geometry.Vec3, status: *fp.MathStatus) fp.Fp {
    const a = world.bodyIndex(contact.body_a).?;
    const b = world.bodyIndex(contact.body_b).?;
    const ra = point.sub(world.storage.position[a], status);
    const rb = point.sub(world.storage.position[b], status);
    const aa = ra.cross(axis, status);
    const bb = rb.cross(axis, status);
    const ia = world.storage.inverse_inertia_local[a].rotate(world.storage.orientation[a], status).toMat3();
    const ib = world.storage.inverse_inertia_local[b].rotate(world.storage.orientation[b], status).toMat3();
    var k = fp.Fp.zero;
    if (world.storage.body_type[a] == .dynamic) k = k.add(world.storage.inverse_mass[a], status).add(aa.dot(ia.mulVec(aa, status), status), status);
    if (world.storage.body_type[b] == .dynamic) k = k.add(world.storage.inverse_mass[b], status).add(bb.dot(ib.mulVec(bb, status), status), status);
    return if (k.raw <= 0) fp.Fp.zero else fp.Fp.one.div(k, status);
}
fn relativeVelocity(world: *const body_world.World, contact: Contact, point: geometry.Vec3, axis: geometry.Vec3, status: *fp.MathStatus) fp.Fp {
    const a = world.bodyIndex(contact.body_a).?;
    const b = world.bodyIndex(contact.body_b).?;
    return pointVelocity(world, b, point, world.storage.linear_velocity[b], world.storage.angular_velocity[b], status).sub(pointVelocity(world, a, point, world.storage.linear_velocity[a], world.storage.angular_velocity[a], status), status).dot(axis, status);
}
fn pointVelocity(world: *const body_world.World, i: usize, point: geometry.Vec3, linear: geometry.Vec3, angular: geometry.Vec3, status: *fp.MathStatus) geometry.Vec3 {
    return linear.add(angular.cross(point.sub(world.storage.position[i], status), status), status);
}
fn apply(world: *body_world.World, contact: Contact, point: geometry.Vec3, impulse: geometry.Vec3, status: *fp.MathStatus) void {
    const a = world.bodyIndex(contact.body_a).?;
    const b = world.bodyIndex(contact.body_b).?;
    applyTo(world, a, point, negate(impulse, status), status);
    applyTo(world, b, point, impulse, status);
}
fn applyPseudo(world: *const body_world.World, contact: Contact, pseudo: PseudoVelocities, point: geometry.Vec3, impulse: geometry.Vec3, status: *fp.MathStatus) void {
    const a = world.bodyIndex(contact.body_a).?;
    const b = world.bodyIndex(contact.body_b).?;
    if (world.storage.body_type[a] == .dynamic) pseudo.linear[a] = pseudo.linear[a].sub(impulse.scale(world.storage.inverse_mass[a], status), status);
    if (world.storage.body_type[b] == .dynamic) pseudo.linear[b] = pseudo.linear[b].add(impulse.scale(world.storage.inverse_mass[b], status), status);
    const ia = world.storage.inverse_inertia_local[a].rotate(world.storage.orientation[a], status).toMat3();
    const ib = world.storage.inverse_inertia_local[b].rotate(world.storage.orientation[b], status).toMat3();
    if (world.storage.body_type[a] == .dynamic) pseudo.angular[a] = pseudo.angular[a].sub(ia.mulVec(point.sub(world.storage.position[a], status).cross(impulse, status), status), status);
    if (world.storage.body_type[b] == .dynamic) pseudo.angular[b] = pseudo.angular[b].add(ib.mulVec(point.sub(world.storage.position[b], status).cross(impulse, status), status), status);
}
fn applyTo(world: *body_world.World, i: usize, point: geometry.Vec3, impulse: geometry.Vec3, status: *fp.MathStatus) void {
    if (world.storage.body_type[i] != .dynamic) return;
    world.storage.linear_velocity[i] = world.storage.linear_velocity[i].add(impulse.scale(world.storage.inverse_mass[i], status), status);
    const inv = world.storage.inverse_inertia_local[i].rotate(world.storage.orientation[i], status).toMat3();
    world.storage.angular_velocity[i] = world.storage.angular_velocity[i].add(inv.mulVec(point.sub(world.storage.position[i], status).cross(impulse, status), status), status);
}
fn maxZero(value: fp.Fp) fp.Fp {
    return if (value.raw < 0) .zero else value;
}
fn negate(value: geometry.Vec3, status: *fp.MathStatus) geometry.Vec3 {
    return .{ .x = value.x.neg(status), .y = value.y.neg(status), .z = value.z.neg(status) };
}
