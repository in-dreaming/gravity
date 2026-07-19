//! Caller-buffered joint lifecycle and canonical local frame validation.
const std = @import("std");
const fp = @import("../math/fp.zig");
const geometry = @import("../math/geometry.zig");
const ids = @import("../core/ids.zig");
const body_world = @import("world.zig");
const constraints = @import("constraints.zig");

pub const Error = error{ CapacityExceeded, InvalidJoint, InvalidBody, InvalidFrame, InvalidLimit, InvalidTime };
pub const Kind = enum { distance, ball_socket, hinge, slider, fixed, cone_twist };
/// Baseline equality rows before an enabled limit or motor adds its own row.
/// These counts encode the released DOF of every frozen joint kind.
pub fn equalityRowCount(kind: Kind) u8 {
    return switch (kind) {
        .distance => 1,
        .ball_socket => 3,
        .hinge => 5,
        .slider => 5,
        .fixed => 6,
        .cone_twist => 3,
    };
}
/// A limit/motor can act only on the one explicitly released scalar axis.
pub fn releasedDofCount(kind: Kind) u8 {
    return switch (kind) {
        .distance, .hinge, .slider => 1,
        .ball_socket => 3,
        .fixed => 0,
        .cone_twist => 3,
    };
}
pub const Limit = struct { enabled: bool = false, min: fp.Fp = .zero, max: fp.Fp = .zero };
pub const Motor = struct { enabled: bool = false, target_velocity: fp.Fp = .zero, max_force: fp.Fp = .zero };
/// Converts the declared force/torque cap to the required per-tick impulse cap.
pub fn motorImpulseCap(motor: Motor, dt: fp.Fp, status: *fp.MathStatus) fp.Fp {
    if (!motor.enabled or dt.raw <= 0) return .zero;
    return motor.max_force.mul(dt, status);
}
/// Clamps a motor's accumulated scalar impulse to force/torque times the tick.
pub fn clampMotorImpulse(value: fp.Fp, motor: Motor, dt: fp.Fp, status: *fp.MathStatus) fp.Fp {
    if (!motor.enabled) return .zero;
    const limit = motorImpulseCap(motor, dt, status).raw;
    return if (value.raw > limit) .{ .raw = limit } else if (value.raw < -limit) .{ .raw = -limit } else value;
}
pub const Spring = struct { enabled: bool = false, frequency: fp.Fp = .zero, damping_ratio: fp.Fp = .zero };
/// Cone-Twist has one non-negative swing cone and one signed twist interval.
pub const ConeTwistLimit = struct { enabled: bool = false, swing_max: fp.Fp = geometry.pi, twist_min: fp.Fp = fp.Fp{ .raw = -geometry.pi.raw }, twist_max: fp.Fp = geometry.pi };
pub const LimitState = enum(u8) { inactive, lower, upper, equal };
pub const SolveSettings = struct { velocity_iterations: u8 = 10 };
/// Local frame axes are persisted in canonical order; `z` is reconstructed
/// from x/y so near-parallel input always picks the same fallback basis.
pub const Frame = struct { anchor: geometry.Vec3 = .{}, axis: geometry.Vec3 = .unit_x, secondary: geometry.Vec3 = .unit_y };
pub const Desc = struct {
    kind: Kind,
    body_a: ids.BodyId,
    body_b: ids.BodyId,
    frame_a: Frame = .{},
    frame_b: Frame = .{},
    /// Distance rest length; omitted values are measured at creation.
    reference: ?fp.Fp = null,
    /// Cone-Twist swing rest angle; omitted values are measured at creation.
    swing_reference: ?fp.Fp = null,
    /// Relative body orientation at creation when no explicit reference is supplied.
    reference_orientation: ?geometry.Quat = null,
    limit: Limit = .{},
    motor: Motor = .{},
    spring: Spring = .{},
    cone_twist: ConeTwistLimit = .{},
};
pub const Joint = struct {
    kind: Kind,
    body_a: ids.BodyId,
    body_b: ids.BodyId,
    frame_a: Frame,
    frame_b: Frame,
    reference: fp.Fp = .zero,
    swing_reference: fp.Fp = .zero,
    reference_orientation: geometry.Quat = .identity,
    limit: Limit,
    motor: Motor,
    spring: Spring,
    cone_twist: ConeTwistLimit = .{},
    limit_state: LimitState = .inactive,
    cone_states: [2]LimitState = [_]LimitState{.inactive} ** 2,
    /// Rows 0..5 are baseline; row 6 is the released-DOF control row.
    impulses: [8]fp.Fp = [_]fp.Fp{.zero} ** 8,
};
pub const Storage = struct { values: []Joint, generation: []u32, alive: []bool, retired: []bool };
pub const CommandKey = struct {
    phase_priority: u8,
    issuer: u32,
    sequence: u32,
    discriminant: u8 = 0,
    pub fn lessThan(a: CommandKey, b: CommandKey) bool {
        return if (a.phase_priority != b.phase_priority) a.phase_priority < b.phase_priority else if (a.issuer != b.issuer) a.issuer < b.issuer else if (a.sequence != b.sequence) a.sequence < b.sequence else a.discriminant < b.discriminant;
    }
};
pub const Command = struct {
    key: CommandKey,
    op: union(enum) { create: Desc, destroy: ids.JointId, destroy_body: ids.BodyId },
    fn canonicalKey(self: Command) CommandKey {
        var key = self.key;
        key.discriminant = @intFromEnum(self.op);
        return key;
    }
};
pub const CommandScratch = struct { commands: []Command, alive: []bool, generation: []u32, retired: []bool, body_a: []ids.BodyId, body_b: []ids.BodyId };
pub const CommandReceipt = struct { key: CommandKey, created: ?ids.JointId = null };
pub const MutableState = struct { limit_state: LimitState, cone_states: [2]LimitState, impulses: [8]fp.Fp };
pub const PoolRowScratch = struct { authored: []constraints.ConstraintRow, build: []constraints.ConstraintRow, states: []MutableState };
pub const Pool = struct {
    storage: Storage,
    pub fn init(storage: Storage) Error!Pool {
        if (storage.values.len != storage.generation.len or storage.values.len != storage.alive.len or storage.values.len != storage.retired.len) return error.CapacityExceeded;
        @memset(storage.generation, 0);
        @memset(storage.alive, false);
        @memset(storage.retired, false);
        return .{ .storage = storage };
    }
    pub fn create(self: *Pool, world: *const body_world.World, desc: Desc, status: *fp.MathStatus) Error!ids.JointId {
        _ = world.bodyIndex(desc.body_a) orelse return error.InvalidBody;
        _ = world.bodyIndex(desc.body_b) orelse return error.InvalidBody;
        if (desc.body_a.value == desc.body_b.value or (desc.limit.enabled and desc.limit.min.raw > desc.limit.max.raw) or desc.motor.max_force.raw < 0 or (desc.spring.enabled and (desc.spring.frequency.raw < 0 or desc.spring.damping_ratio.raw < 0)) or (desc.cone_twist.enabled and (desc.cone_twist.swing_max.raw < 0 or desc.cone_twist.twist_min.raw > desc.cone_twist.twist_max.raw))) return error.InvalidLimit;
        const a = try canonicalFrame(desc.frame_a, status);
        const b = try canonicalFrame(desc.frame_b, status);
        const reference = desc.reference orelse switch (desc.kind) {
            .distance => try distanceCoordinate(world, desc.body_a, desc.body_b, a, b, status),
            .slider => try sliderCoordinate(world, desc.body_a, desc.body_b, a, b, status),
            .hinge => try hingeCoordinate(world, desc.body_a, desc.body_b, a, b, status),
            .cone_twist => try hingeCoordinate(world, desc.body_a, desc.body_b, a, b, status),
            else => fp.Fp.zero,
        };
        const swing_reference = desc.swing_reference orelse if (desc.kind == .cone_twist) try coneSwingCoordinate(world, desc.body_a, desc.body_b, a, b, status) else fp.Fp.zero;
        const reference_orientation = desc.reference_orientation orelse try relativeOrientation(world, desc.body_a, desc.body_b, status);
        for (self.storage.alive, 0..) |alive, i| if (!alive and !self.storage.retired[i]) {
            self.storage.values[i] = .{ .kind = desc.kind, .body_a = desc.body_a, .body_b = desc.body_b, .frame_a = a, .frame_b = b, .reference = reference, .swing_reference = swing_reference, .reference_orientation = reference_orientation, .limit = desc.limit, .motor = desc.motor, .spring = desc.spring, .cone_twist = desc.cone_twist };
            self.storage.alive[i] = true;
            return .init(@intCast(i), self.storage.generation[i]);
        };
        return error.CapacityExceeded;
    }
    pub fn destroy(self: *Pool, id: ids.JointId) Error!void {
        const i: usize = id.index();
        if (i >= self.storage.alive.len or !self.storage.alive[i] or self.storage.generation[i] != id.generation()) return error.InvalidJoint;
        self.storage.alive[i] = false;
        if (self.storage.generation[i] == std.math.maxInt(u32)) self.storage.retired[i] = true else self.storage.generation[i] += 1;
    }
    pub fn destroyBody(self: *Pool, body: ids.BodyId) void {
        for (self.storage.values, 0..) |joint, i| {
            if (!self.storage.alive[i] or (joint.body_a.value != body.value and joint.body_b.value != body.value)) continue;
            self.storage.alive[i] = false;
            if (self.storage.generation[i] == std.math.maxInt(u32)) self.storage.retired[i] = true else self.storage.generation[i] += 1;
        }
    }
};
/// Executes a canonical, caller-buffered joint command transaction. The shadow
/// slot columns prove every command before the live pool is touched; results
/// are emitted in canonical command order after commit.
pub fn executeCommands(pool: *Pool, world: *const body_world.World, commands: []const Command, scratch: CommandScratch, receipts: []CommandReceipt, status: *fp.MathStatus) Error![]const CommandReceipt {
    if (commands.len > scratch.commands.len or commands.len > receipts.len or scratch.alive.len != pool.storage.alive.len or scratch.generation.len != pool.storage.generation.len or scratch.retired.len != pool.storage.retired.len or scratch.body_a.len != pool.storage.values.len or scratch.body_b.len != pool.storage.values.len) return error.CapacityExceeded;
    @memcpy(scratch.commands[0..commands.len], commands);
    sortCommands(scratch.commands[0..commands.len]);
    @memcpy(scratch.alive, pool.storage.alive);
    @memcpy(scratch.generation, pool.storage.generation);
    @memcpy(scratch.retired, pool.storage.retired);
    for (pool.storage.values, 0..) |joint, i| {
        scratch.body_a[i] = joint.body_a;
        scratch.body_b[i] = joint.body_b;
    }
    for (scratch.commands[0..commands.len]) |command| try validateCommand(world, command, scratch.alive, scratch.generation, scratch.retired, scratch.body_a, scratch.body_b, status);
    for (scratch.commands[0..commands.len], 0..) |command, i| {
        receipts[i] = .{ .key = command.canonicalKey() };
        switch (command.op) {
            .create => |desc| receipts[i].created = try pool.create(world, desc, status),
            .destroy => |id| try pool.destroy(id),
            .destroy_body => |body| pool.destroyBody(body),
        }
    }
    return receipts[0..commands.len];
}
/// Destroys every joint referencing `body` before retiring the body slot.
/// This is the required lifecycle entry point for hosts that own separate
/// caller-buffered World and JointPool storage.
pub fn destroyBody(world: *body_world.World, pool: *Pool, body: ids.BodyId) (Error || body_world.Error)!void {
    _ = world.bodyIndex(body) orelse return error.InvalidBody;
    pool.destroyBody(body);
    try world.destroy(body);
}
fn validateCommand(world: *const body_world.World, command: Command, alive: []bool, generation: []u32, retired: []bool, body_a: []ids.BodyId, body_b: []ids.BodyId, status: *fp.MathStatus) Error!void {
    switch (command.op) {
        .create => |desc| {
            try validateDesc(world, desc, status);
            for (alive, 0..) |used, i| if (!used and !retired[i]) {
                alive[i] = true;
                body_a[i] = desc.body_a;
                body_b[i] = desc.body_b;
                return;
            };
            return error.CapacityExceeded;
        },
        .destroy => |id| {
            const i: usize = id.index();
            if (i >= alive.len or !alive[i] or generation[i] != id.generation()) return error.InvalidJoint;
            alive[i] = false;
            if (generation[i] == std.math.maxInt(u32)) retired[i] = true else generation[i] += 1;
        },
        .destroy_body => |body| {
            _ = world.bodyIndex(body) orelse return error.InvalidBody;
            for (alive, 0..) |used, i| {
                if (!used or (body_a[i].value != body.value and body_b[i].value != body.value)) continue;
                alive[i] = false;
                if (generation[i] == std.math.maxInt(u32)) retired[i] = true else generation[i] += 1;
            }
        },
    }
}
fn validateDesc(world: *const body_world.World, desc: Desc, status: *fp.MathStatus) Error!void {
    _ = world.bodyIndex(desc.body_a) orelse return error.InvalidBody;
    _ = world.bodyIndex(desc.body_b) orelse return error.InvalidBody;
    if (desc.body_a.value == desc.body_b.value or (desc.limit.enabled and desc.limit.min.raw > desc.limit.max.raw) or desc.motor.max_force.raw < 0 or (desc.spring.enabled and (desc.spring.frequency.raw < 0 or desc.spring.damping_ratio.raw < 0)) or (desc.cone_twist.enabled and (desc.cone_twist.swing_max.raw < 0 or desc.cone_twist.twist_min.raw > desc.cone_twist.twist_max.raw))) return error.InvalidLimit;
    _ = try canonicalFrame(desc.frame_a, status);
    _ = try canonicalFrame(desc.frame_b, status);
}
fn sortCommands(items: []Command) void {
    var i: usize = 1;
    while (i < items.len) : (i += 1) {
        const value = items[i];
        var j = i;
        while (j > 0 and value.canonicalKey().lessThan(items[j - 1].canonicalKey())) : (j -= 1) items[j] = items[j - 1];
        items[j] = value;
    }
}
/// Visits every future-relevant joint-pool field in slot order. Derived rows
/// are intentionally excluded: they are rebuilt from these canonical values.
pub fn visitCanonical(pool: *const Pool, visitor: anytype) void {
    visitor.writeU64(@intCast(pool.storage.values.len));
    for (pool.storage.values, 0..) |joint, i| {
        visitor.writeU8(@intFromBool(pool.storage.alive[i]));
        visitor.writeU8(@intFromBool(pool.storage.retired[i]));
        visitor.writeU32(pool.storage.generation[i]);
        if (!pool.storage.alive[i]) continue;
        visitor.writeU8(@intFromEnum(joint.kind));
        visitor.writeU64(joint.body_a.value);
        visitor.writeU64(joint.body_b.value);
        visitFrame(joint.frame_a, visitor);
        visitFrame(joint.frame_b, visitor);
        visitor.writeI64(joint.reference.raw);
        visitor.writeI64(joint.swing_reference.raw);
        visitor.writeI64(joint.reference_orientation.x.raw);
        visitor.writeI64(joint.reference_orientation.y.raw);
        visitor.writeI64(joint.reference_orientation.z.raw);
        visitor.writeI64(joint.reference_orientation.w.raw);
        visitor.writeU8(@intFromBool(joint.limit.enabled));
        visitor.writeI64(joint.limit.min.raw);
        visitor.writeI64(joint.limit.max.raw);
        visitor.writeU8(@intFromBool(joint.motor.enabled));
        visitor.writeI64(joint.motor.target_velocity.raw);
        visitor.writeI64(joint.motor.max_force.raw);
        visitor.writeU8(@intFromBool(joint.spring.enabled));
        visitor.writeI64(joint.spring.frequency.raw);
        visitor.writeI64(joint.spring.damping_ratio.raw);
        visitor.writeU8(@intFromBool(joint.cone_twist.enabled));
        visitor.writeI64(joint.cone_twist.swing_max.raw);
        visitor.writeI64(joint.cone_twist.twist_min.raw);
        visitor.writeI64(joint.cone_twist.twist_max.raw);
        visitor.writeU8(@intFromEnum(joint.limit_state));
        for (joint.cone_states) |state| visitor.writeU8(@intFromEnum(state));
        for (joint.impulses) |impulse| visitor.writeI64(impulse.raw);
    }
}
fn visitFrame(frame: Frame, visitor: anytype) void {
    inline for ([_]geometry.Vec3{ frame.anchor, frame.axis, frame.secondary }) |vector| {
        visitor.writeI64(vector.x.raw);
        visitor.writeI64(vector.y.raw);
        visitor.writeI64(vector.z.raw);
    }
}
/// Solves already canonicalized joint rows in fixed PGS order. The same row
/// buffers carry warm impulses in and out; callers can then persist them with
/// `writeBackImpulses` before serializing their joint pool.
pub fn solveRows(world: *body_world.World, rows: []constraints.ConstraintRow, settings: SolveSettings, status: *fp.MathStatus) (Error || constraints.Error)!void {
    try validateRows(world, rows);
    try warmStartRows(world, rows, status);
    var iteration: u8 = 0;
    while (iteration < settings.velocity_iterations) : (iteration += 1) try solveRowsIteration(world, rows, status);
}
/// Validates the canonical joint-row stream without changing world velocity or
/// row impulses. The combined contact solver uses this before its warm start.
pub fn validateRows(world: *const body_world.World, rows: []const constraints.ConstraintRow) (Error || constraints.Error)!void {
    for (rows, 0..) |*row, i| {
        if (i > 0 and row.key.lessThan(rows[i - 1].key)) return error.InvalidJoint;
        _ = world.bodyIndex(row.key.min_body) orelse return error.InvalidBody;
        _ = world.bodyIndex(row.key.max_body) orelse return error.InvalidBody;
        if (row.key.kind != .joint or row.lower.raw > row.upper.raw or row.softness.raw < 0) return error.InvalidJoint;
    }
}
/// Applies persistent joint impulses once before a combined PGS loop.
pub fn warmStartRows(world: *body_world.World, rows: []const constraints.ConstraintRow, status: *fp.MathStatus) (Error || constraints.Error)!void {
    try validateRows(world, rows);
    for (rows) |row| {
        applyRowImpulse(world, row, row.accumulated_impulse, status);
    }
}
/// Solves one canonical joint-row PGS iteration. Callers interleave this
/// immediately before the contact rows on every velocity iteration.
pub fn solveRowsIteration(world: *body_world.World, rows: []constraints.ConstraintRow, status: *fp.MathStatus) (Error || constraints.Error)!void {
    for (rows) |*row| {
        const velocity = rowVelocity(world, row.*, status);
        const correction = velocity.add(row.bias, status).add(row.softness.mul(row.accumulated_impulse, status), status).neg(status).mul(row.effective_mass, status);
        const old = row.accumulated_impulse;
        const candidate = old.add(correction, status);
        row.accumulated_impulse = clamp(candidate, row.lower, row.upper);
        applyRowImpulse(world, row.*, row.accumulated_impulse.sub(old, status), status);
    }
}

/// Applies warm-start impulses for the canonical rows owned by one dynamic
/// island. Static and kinematic endpoints are read-only and therefore do not
/// participate in island membership.
pub fn warmStartRowsForIsland(world: *body_world.World, rows: []const constraints.ConstraintRow, members: []const ids.BodyId, status: *fp.MathStatus) void {
    for (rows) |row| if (rowBelongsToIsland(world, row, members)) applyRowImpulse(world, row, row.accumulated_impulse, status);
}

/// Solves one PGS iteration for one island while retaining the global
/// canonical row order. Independent calls write disjoint dynamic body and row
/// sets, so their execution order cannot affect the result.
pub fn solveRowsIterationForIsland(world: *body_world.World, rows: []constraints.ConstraintRow, members: []const ids.BodyId, status: *fp.MathStatus) void {
    for (rows) |*row| {
        if (!rowBelongsToIsland(world, row.*, members)) continue;
        const velocity = rowVelocity(world, row.*, status);
        const correction = velocity.add(row.bias, status).add(row.softness.mul(row.accumulated_impulse, status), status).neg(status).mul(row.effective_mass, status);
        const old = row.accumulated_impulse;
        row.accumulated_impulse = clamp(old.add(correction, status), row.lower, row.upper);
        applyRowImpulse(world, row.*, row.accumulated_impulse.sub(old, status), status);
    }
}

fn rowBelongsToIsland(world: *const body_world.World, row: constraints.ConstraintRow, members: []const ids.BodyId) bool {
    const a = world.bodyIndex(row.key.min_body) orelse return false;
    if (world.storage.body_type[a] == .dynamic) return containsBody(members, row.key.min_body);
    const b = world.bodyIndex(row.key.max_body) orelse return false;
    return world.storage.body_type[b] == .dynamic and containsBody(members, row.key.max_body);
}

fn containsBody(members: []const ids.BodyId, body: ids.BodyId) bool {
    for (members) |member| if (member.value == body.value) return true;
    return false;
}
/// Builds all baseline and active control rows for one joint. `scratch` is
/// caller-owned temporary storage and must hold twice the documented maximum
/// row count, keeping capacity failure and intermediate producers invisible to
/// the final `output` buffer.
pub fn buildRows(world: *const body_world.World, joint: *Joint, owner: ids.JointId, dt: fp.Fp, output: []constraints.ConstraintRow, scratch: []constraints.ConstraintRow, status: *fp.MathStatus) (Error || constraints.Error)![]const constraints.ConstraintRow {
    const base_count: usize = equalityRowCount(joint.kind);
    const control_max: usize = switch (joint.kind) {
        .distance, .hinge, .slider => 1,
        .cone_twist => 2,
        else => 0,
    };
    const total_max = base_count + control_max;
    if (output.len < total_max or scratch.len < total_max * 2) return error.CapacityExceeded;
    const base_output = scratch[0..base_count];
    const base_work = scratch[base_count .. base_count * 2];
    const control_output = scratch[base_count * 2 .. base_count * 2 + control_max];
    const control_work = scratch[base_count * 2 + control_max .. total_max * 2];
    const base = switch (joint.kind) {
        .distance => try buildDistanceRow(world, joint, owner, base_output, base_work, status),
        .ball_socket => try buildBallSocketRows(world, joint, owner, base_output, base_work, status),
        .hinge => try buildHingeRows(world, joint, owner, base_output, base_work, status),
        .slider => try buildSliderRows(world, joint, owner, base_output, base_work, status),
        .fixed => try buildFixedRows(world, joint, owner, base_output, base_work, status),
        .cone_twist => try buildConeTwistRows(world, joint, owner, base_output, base_work, status),
    };
    const controls = switch (joint.kind) {
        .distance => try buildDistanceControlRow(world, joint, owner, dt, control_output, control_work, status),
        .hinge => try buildHingeControlRow(world, joint, owner, dt, control_output, control_work, status),
        .slider => try buildSliderControlRow(world, joint, owner, dt, control_output, control_work, status),
        .cone_twist => try buildConeTwistControlRows(world, joint, owner, dt, control_output, control_work, status),
        else => control_output[0..0],
    };
    return constraints.mergeRows(base, controls, output);
}
/// Assembles every live joint into one globally sorted row slice. The caller
/// owns all capacity; temporary joint state is restored if any producer fails.
pub fn buildPoolRows(world: *const body_world.World, pool: *Pool, dt: fp.Fp, output: []constraints.ConstraintRow, scratch: PoolRowScratch, status: *fp.MathStatus) (Error || constraints.Error)![]const constraints.ConstraintRow {
    if (scratch.states.len != pool.storage.values.len) return error.CapacityExceeded;
    var required: usize = 0;
    for (pool.storage.alive, 0..) |alive, i| {
        scratch.states[i] = .{ .limit_state = pool.storage.values[i].limit_state, .cone_states = pool.storage.values[i].cone_states, .impulses = pool.storage.values[i].impulses };
        if (!alive) continue;
        required += @as(usize, equalityRowCount(pool.storage.values[i].kind)) + @as(usize, switch (pool.storage.values[i].kind) {
            .distance, .hinge, .slider => 1,
            .cone_twist => 2,
            else => 0,
        });
    }
    if (output.len < required or scratch.authored.len < required or scratch.build.len < 12) return error.CapacityExceeded;
    errdefer restoreMutableStates(pool, scratch.states);
    var count: usize = 0;
    for (pool.storage.values, 0..) |*joint, i| {
        if (!pool.storage.alive[i]) continue;
        const maximum = @as(usize, equalityRowCount(joint.kind)) + @as(usize, switch (joint.kind) {
            .distance, .hinge, .slider => 1,
            .cone_twist => 2,
            else => 0,
        });
        const id = ids.JointId.init(@intCast(i), pool.storage.generation[i]);
        const rows = try buildRows(world, joint, id, dt, scratch.authored[count .. count + maximum], scratch.build, status);
        count += rows.len;
    }
    return constraints.mergeRows(scratch.authored[0..count], &.{}, output);
}
/// Builds, solves, and persists all live joints in one deterministic row order.
pub fn solvePool(world: *body_world.World, pool: *Pool, dt: fp.Fp, settings: SolveSettings, rows: []constraints.ConstraintRow, scratch: PoolRowScratch, status: *fp.MathStatus) (Error || constraints.Error)![]const constraints.ConstraintRow {
    const built = try buildPoolRows(world, pool, dt, rows, scratch, status);
    try solveRows(world, rows[0..built.len], settings, status);
    try writeBackImpulses(pool, rows[0..built.len]);
    return rows[0..built.len];
}
fn restoreMutableStates(pool: *Pool, states: []const MutableState) void {
    for (pool.storage.values, 0..) |*joint, i| {
        joint.limit_state = states[i].limit_state;
        joint.cone_states = states[i].cone_states;
        joint.impulses = states[i].impulses;
    }
}
/// Transfers solved rows to their generation-checked persistent joint slots.
/// Row index zero through five are baseline rows; index six stores the
/// released-DOF control row shared by limit, motor, and spring.
pub fn writeBackImpulses(pool: *Pool, rows: []const constraints.ConstraintRow) Error!void {
    for (rows) |row| {
        if (row.key.kind != .joint) continue;
        const id = ids.JointId{ .value = row.key.owner };
        const index: usize = id.index();
        if (index >= pool.storage.values.len or !pool.storage.alive[index] or pool.storage.generation[index] != id.generation()) return error.InvalidJoint;
        const impulse_index: usize = row.key.row_index;
        if (impulse_index >= pool.storage.values[index].impulses.len) return error.InvalidJoint;
        pool.storage.values[index].impulses[impulse_index] = row.accumulated_impulse;
    }
}
fn rowVelocity(world: *const body_world.World, row: constraints.ConstraintRow, status: *fp.MathStatus) fp.Fp {
    const a = world.bodyIndex(row.key.min_body).?;
    const b = world.bodyIndex(row.key.max_body).?;
    var value = world.storage.linear_velocity[a].dot(row.ja_linear, status).add(world.storage.angular_velocity[a].dot(row.ja_angular, status), status);
    if (a != b) value = value.add(world.storage.linear_velocity[b].dot(row.jb_linear, status), status).add(world.storage.angular_velocity[b].dot(row.jb_angular, status), status);
    return value;
}
fn applyRowImpulse(world: *body_world.World, row: constraints.ConstraintRow, impulse: fp.Fp, status: *fp.MathStatus) void {
    const a = world.bodyIndex(row.key.min_body).?;
    const b = world.bodyIndex(row.key.max_body).?;
    applyRowToBody(world, a, row.ja_linear, row.ja_angular, impulse, status);
    if (a != b) applyRowToBody(world, b, row.jb_linear, row.jb_angular, impulse, status);
}
fn applyRowToBody(world: *body_world.World, index: usize, linear: geometry.Vec3, angular: geometry.Vec3, impulse: fp.Fp, status: *fp.MathStatus) void {
    if (world.storage.body_type[index] != .dynamic) return;
    world.storage.linear_velocity[index] = world.storage.linear_velocity[index].add(linear.scale(impulse.mul(world.storage.inverse_mass[index], status), status), status);
    const inertia = world.storage.inverse_inertia_local[index].rotate(world.storage.orientation[index], status).toMat3();
    world.storage.angular_velocity[index] = world.storage.angular_velocity[index].add(inertia.mulVec(angular.scale(impulse, status), status), status);
}
fn clamp(value: fp.Fp, lower: fp.Fp, upper: fp.Fp) fp.Fp {
    return if (value.raw < lower.raw) lower else if (value.raw > upper.raw) upper else value;
}
/// Emits the scalar Distance Jacobian between the persisted local anchors.
/// Callers provide all row buffers; invalid or degenerate anchors publish no
/// partial row. Other joint kinds are rejected here and use their own builders.
pub fn buildDistanceRow(world: *const body_world.World, joint: *const Joint, owner: ids.JointId, output: []constraints.ConstraintRow, scratch: []constraints.ConstraintRow, status: *fp.MathStatus) (Error || constraints.Error)![]const constraints.ConstraintRow {
    if (joint.kind != .distance) return error.InvalidJoint;
    const a = world.bodyIndex(joint.body_a) orelse return error.InvalidBody;
    const b = world.bodyIndex(joint.body_b) orelse return error.InvalidBody;
    const pa = world.storage.orientation[a].rotate(joint.frame_a.anchor, status).add(world.storage.position[a], status);
    const pb = world.storage.orientation[b].rotate(joint.frame_b.anchor, status).add(world.storage.position[b], status);
    const axis_n = pb.sub(pa, status).normalize(status);
    if (!axis_n.valid) return error.InvalidFrame;
    const axis = axis_n.value;
    const spec = [_]constraints.RowSpec{.{ .kind = .joint, .body_a = joint.body_a, .body_b = joint.body_b, .owner = owner.value, .row_index = 0, .ja_linear = axis, .ja_angular = pa.sub(world.storage.position[a], status).cross(axis, status), .jb_linear = .{ .x = axis.x.neg(status), .y = axis.y.neg(status), .z = axis.z.neg(status) }, .jb_angular = pb.sub(world.storage.position[b], status).cross(.{ .x = axis.x.neg(status), .y = axis.y.neg(status), .z = axis.z.neg(status) }, status) }};
    return constraints.buildAuthoredRows(world, &spec, output, scratch, status);
}
/// Emits the one released Distance DOF only when its hard limit, motor, or
/// implicit spring is active. Hard limits take priority and changing sides
/// clears the related warm impulse before the next solve.
pub fn buildDistanceControlRow(world: *const body_world.World, joint: *Joint, owner: ids.JointId, dt: fp.Fp, output: []constraints.ConstraintRow, scratch: []constraints.ConstraintRow, status: *fp.MathStatus) (Error || constraints.Error)![]const constraints.ConstraintRow {
    if (joint.kind != .distance) return error.InvalidJoint;
    if (dt.raw <= 0) return error.InvalidTime;
    const data = try distanceData(world, joint.body_a, joint.body_b, joint.frame_a, joint.frame_b, status);
    const next_state = classifyLimit(joint.limit, data.coordinate);
    var bias = fp.Fp.zero;
    var softness = fp.Fp.zero;
    var lower = fp.Fp.min;
    var upper = fp.Fp.max;
    if (next_state == .lower) {
        bias = joint.limit.min.sub(data.coordinate, status).mul(.{ .raw = 858_993_459 }, status).div(dt, status);
        upper = .zero;
    } else if (next_state == .upper) {
        bias = joint.limit.max.sub(data.coordinate, status).mul(.{ .raw = 858_993_459 }, status).div(dt, status);
        lower = .zero;
    } else if (next_state == .equal) {
        bias = joint.limit.min.sub(data.coordinate, status).mul(.{ .raw = 858_993_459 }, status).div(dt, status);
    } else if (joint.spring.enabled) {
        const terms = try springTerms(joint.spring, joint.reference.sub(data.coordinate, status), dt, status);
        bias = terms.bias;
        softness = terms.softness;
    } else if (joint.motor.enabled) {
        const cap = motorImpulseCap(joint.motor, dt, status);
        bias = joint.motor.target_velocity;
        lower = .{ .raw = -cap.raw };
        upper = cap;
    } else return output[0..0];
    const warm = if (next_state != joint.limit_state) fp.Fp.zero else joint.impulses[6];
    const spec = [_]constraints.RowSpec{.{ .kind = .joint, .body_a = joint.body_a, .body_b = joint.body_b, .owner = owner.value, .row_index = 6, .ja_linear = data.axis, .ja_angular = data.ra.cross(data.axis, status), .jb_linear = negate(data.axis, status), .jb_angular = data.rb.cross(negate(data.axis, status), status), .bias = bias, .softness = softness, .lower = lower, .upper = upper, .accumulated_impulse = warm }};
    const rows = try constraints.buildAuthoredRows(world, &spec, output, scratch, status);
    joint.limit_state = next_state;
    joint.impulses[6] = warm;
    return rows;
}

const DistanceData = struct { coordinate: fp.Fp, axis: geometry.Vec3, ra: geometry.Vec3, rb: geometry.Vec3 };
fn distanceData(world: *const body_world.World, body_a: ids.BodyId, body_b: ids.BodyId, frame_a: Frame, frame_b: Frame, status: *fp.MathStatus) Error!DistanceData {
    const a = world.bodyIndex(body_a) orelse return error.InvalidBody;
    const b = world.bodyIndex(body_b) orelse return error.InvalidBody;
    const pa = world.storage.orientation[a].rotate(frame_a.anchor, status).add(world.storage.position[a], status);
    const pb = world.storage.orientation[b].rotate(frame_b.anchor, status).add(world.storage.position[b], status);
    const delta = pb.sub(pa, status);
    const axis_n = delta.normalize(status);
    if (!axis_n.valid) return error.InvalidFrame;
    return .{ .coordinate = delta.lengthSquared(status).sqrt(status), .axis = axis_n.value, .ra = pa.sub(world.storage.position[a], status), .rb = pb.sub(world.storage.position[b], status) };
}
fn distanceCoordinate(world: *const body_world.World, body_a: ids.BodyId, body_b: ids.BodyId, frame_a: Frame, frame_b: Frame, status: *fp.MathStatus) Error!fp.Fp {
    return (try distanceData(world, body_a, body_b, frame_a, frame_b, status)).coordinate;
}
const ScalarData = struct { coordinate: fp.Fp, ja_linear: geometry.Vec3 = .{}, ja_angular: geometry.Vec3 = .{}, jb_linear: geometry.Vec3 = .{}, jb_angular: geometry.Vec3 = .{} };
/// Emits Slider's released axis control row. Its scalar coordinate is the
/// signed anchor separation along body A's canonical axis.
pub fn buildSliderControlRow(world: *const body_world.World, joint: *Joint, owner: ids.JointId, dt: fp.Fp, output: []constraints.ConstraintRow, scratch: []constraints.ConstraintRow, status: *fp.MathStatus) (Error || constraints.Error)![]const constraints.ConstraintRow {
    if (joint.kind != .slider) return error.InvalidJoint;
    return buildScalarControlRow(world, joint, owner, dt, try sliderData(world, joint.body_a, joint.body_b, joint.frame_a, joint.frame_b, status), output, scratch, status);
}
/// Emits Hinge's released twist control row. The angle uses a signed atan2 of
/// the persisted secondary axes, so the 180-degree case remains canonical.
pub fn buildHingeControlRow(world: *const body_world.World, joint: *Joint, owner: ids.JointId, dt: fp.Fp, output: []constraints.ConstraintRow, scratch: []constraints.ConstraintRow, status: *fp.MathStatus) (Error || constraints.Error)![]const constraints.ConstraintRow {
    if (joint.kind != .hinge) return error.InvalidJoint;
    return buildScalarControlRow(world, joint, owner, dt, try hingeData(world, joint.body_a, joint.body_b, joint.frame_a, joint.frame_b, status), output, scratch, status);
}
fn buildScalarControlRow(world: *const body_world.World, joint: *Joint, owner: ids.JointId, dt: fp.Fp, data: ScalarData, output: []constraints.ConstraintRow, scratch: []constraints.ConstraintRow, status: *fp.MathStatus) (Error || constraints.Error)![]const constraints.ConstraintRow {
    if (dt.raw <= 0) return error.InvalidTime;
    const next_state = classifyLimit(joint.limit, data.coordinate);
    var bias = fp.Fp.zero;
    var softness = fp.Fp.zero;
    var lower = fp.Fp.min;
    var upper = fp.Fp.max;
    if (next_state == .lower) {
        bias = joint.limit.min.sub(data.coordinate, status).mul(.{ .raw = 858_993_459 }, status).div(dt, status);
        upper = .zero;
    } else if (next_state == .upper) {
        bias = joint.limit.max.sub(data.coordinate, status).mul(.{ .raw = 858_993_459 }, status).div(dt, status);
        lower = .zero;
    } else if (next_state == .equal) {
        bias = joint.limit.min.sub(data.coordinate, status).mul(.{ .raw = 858_993_459 }, status).div(dt, status);
    } else if (joint.spring.enabled) {
        const terms = try springTerms(joint.spring, joint.reference.sub(data.coordinate, status), dt, status);
        bias = terms.bias;
        softness = terms.softness;
    } else if (joint.motor.enabled) {
        const cap = motorImpulseCap(joint.motor, dt, status);
        bias = joint.motor.target_velocity;
        lower = .{ .raw = -cap.raw };
        upper = cap;
    } else return output[0..0];
    const warm = if (next_state != joint.limit_state) fp.Fp.zero else joint.impulses[6];
    const spec = [_]constraints.RowSpec{.{ .kind = .joint, .body_a = joint.body_a, .body_b = joint.body_b, .owner = owner.value, .row_index = 6, .ja_linear = data.ja_linear, .ja_angular = data.ja_angular, .jb_linear = data.jb_linear, .jb_angular = data.jb_angular, .bias = bias, .softness = softness, .lower = lower, .upper = upper, .accumulated_impulse = warm }};
    const rows = try constraints.buildAuthoredRows(world, &spec, output, scratch, status);
    joint.limit_state = next_state;
    joint.impulses[6] = warm;
    return rows;
}
fn sliderData(world: *const body_world.World, body_a: ids.BodyId, body_b: ids.BodyId, frame_a: Frame, frame_b: Frame, status: *fp.MathStatus) Error!ScalarData {
    const a = world.bodyIndex(body_a) orelse return error.InvalidBody;
    const b = world.bodyIndex(body_b) orelse return error.InvalidBody;
    const pa = world.storage.orientation[a].rotate(frame_a.anchor, status).add(world.storage.position[a], status);
    const pb = world.storage.orientation[b].rotate(frame_b.anchor, status).add(world.storage.position[b], status);
    const axis = world.storage.orientation[a].rotate(frame_a.axis, status);
    const ra = pa.sub(world.storage.position[a], status);
    const rb = pb.sub(world.storage.position[b], status);
    const minus = negate(axis, status);
    return .{ .coordinate = pb.sub(pa, status).dot(axis, status), .ja_linear = axis, .ja_angular = ra.cross(axis, status), .jb_linear = minus, .jb_angular = rb.cross(minus, status) };
}
fn sliderCoordinate(world: *const body_world.World, body_a: ids.BodyId, body_b: ids.BodyId, frame_a: Frame, frame_b: Frame, status: *fp.MathStatus) Error!fp.Fp {
    return (try sliderData(world, body_a, body_b, frame_a, frame_b, status)).coordinate;
}
fn hingeData(world: *const body_world.World, body_a: ids.BodyId, body_b: ids.BodyId, frame_a: Frame, frame_b: Frame, status: *fp.MathStatus) Error!ScalarData {
    const a = world.bodyIndex(body_a) orelse return error.InvalidBody;
    const b = world.bodyIndex(body_b) orelse return error.InvalidBody;
    const axis = world.storage.orientation[a].rotate(frame_a.axis, status);
    const secondary_a = world.storage.orientation[a].rotate(frame_a.secondary, status);
    const raw_b = world.storage.orientation[b].rotate(frame_b.secondary, status);
    const secondary_b = try perpendicularUnit(axis, raw_b, status);
    const sin = axis.dot(secondary_a.cross(secondary_b, status), status);
    const cos = secondary_a.dot(secondary_b, status);
    const minus = negate(axis, status);
    return .{ .coordinate = geometry.atan2(sin, cos, status), .ja_angular = axis, .jb_angular = minus };
}
fn hingeCoordinate(world: *const body_world.World, body_a: ids.BodyId, body_b: ids.BodyId, frame_a: Frame, frame_b: Frame, status: *fp.MathStatus) Error!fp.Fp {
    return (try hingeData(world, body_a, body_b, frame_a, frame_b, status)).coordinate;
}
/// Emits up to two Cone-Twist limit rows: a swing-cone normal followed by the
/// signed twist interval. The order is frozen and side transitions clear only
/// the matching warm impulse.
pub fn buildConeTwistControlRows(world: *const body_world.World, joint: *Joint, owner: ids.JointId, dt: fp.Fp, output: []constraints.ConstraintRow, scratch: []constraints.ConstraintRow, status: *fp.MathStatus) (Error || constraints.Error)![]const constraints.ConstraintRow {
    if (joint.kind != .cone_twist) return error.InvalidJoint;
    if (dt.raw <= 0) return error.InvalidTime;
    const data = try coneTwistData(world, joint.body_a, joint.body_b, joint.frame_a, joint.frame_b, status);
    const swing_limit = Limit{ .enabled = joint.cone_twist.enabled, .min = .zero, .max = joint.cone_twist.swing_max };
    const twist_limit = Limit{ .enabled = joint.cone_twist.enabled, .min = joint.cone_twist.twist_min, .max = joint.cone_twist.twist_max };
    const swing = scalarControlTerms(swing_limit, data.swing, joint.swing_reference, .{}, joint.spring, dt, status) catch return error.InvalidLimit;
    const twist = scalarControlTerms(twist_limit, data.twist, joint.reference, joint.motor, joint.spring, dt, status) catch return error.InvalidLimit;
    var specs: [2]constraints.RowSpec = undefined;
    var count: usize = 0;
    if (swing.active) {
        const warm = if (swing.state == joint.cone_states[0]) joint.impulses[6] else fp.Fp.zero;
        specs[count] = .{ .kind = .joint, .body_a = joint.body_a, .body_b = joint.body_b, .owner = owner.value, .row_index = 6, .ja_angular = data.swing_axis, .jb_angular = negate(data.swing_axis, status), .bias = swing.bias, .softness = swing.softness, .lower = swing.lower, .upper = swing.upper, .accumulated_impulse = warm };
        count += 1;
    }
    if (twist.active) {
        const warm = if (twist.state == joint.cone_states[1]) joint.impulses[7] else fp.Fp.zero;
        specs[count] = .{ .kind = .joint, .body_a = joint.body_a, .body_b = joint.body_b, .owner = owner.value, .row_index = 7, .ja_angular = data.twist_axis, .jb_angular = negate(data.twist_axis, status), .bias = twist.bias, .softness = twist.softness, .lower = twist.lower, .upper = twist.upper, .accumulated_impulse = warm };
        count += 1;
    }
    if (count == 0) return output[0..0];
    const rows = try constraints.buildAuthoredRows(world, specs[0..count], output, scratch, status);
    joint.cone_states = .{ swing.state, twist.state };
    if (swing.active) joint.impulses[6] = rows[0].accumulated_impulse;
    if (twist.active) joint.impulses[7] = rows[count - 1].accumulated_impulse;
    return rows;
}
const HardLimitTerms = struct { active: bool, state: LimitState, bias: fp.Fp = .zero, softness: fp.Fp = .zero, lower: fp.Fp = .min, upper: fp.Fp = .max };
fn hardLimitTerms(limit: Limit, coordinate: fp.Fp, dt: fp.Fp, status: *fp.MathStatus) HardLimitTerms {
    const state = classifyLimit(limit, coordinate);
    if (state == .inactive) return .{ .active = false, .state = state };
    const target = switch (state) {
        .lower => limit.min,
        .upper => limit.max,
        .equal => limit.min,
        .inactive => unreachable,
    };
    return .{ .active = true, .state = state, .bias = target.sub(coordinate, status).mul(.{ .raw = 858_993_459 }, status).div(dt, status), .lower = if (state == .upper) .zero else .min, .upper = if (state == .lower) .zero else .max };
}
fn scalarControlTerms(limit: Limit, coordinate: fp.Fp, reference: fp.Fp, motor: Motor, spring: Spring, dt: fp.Fp, status: *fp.MathStatus) Error!HardLimitTerms {
    const hard = hardLimitTerms(limit, coordinate, dt, status);
    if (hard.active) return hard;
    if (spring.enabled) {
        const terms = try springTerms(spring, reference.sub(coordinate, status), dt, status);
        return .{ .active = true, .state = .inactive, .bias = terms.bias, .softness = terms.softness, .lower = .min, .upper = .max };
    }
    if (motor.enabled) {
        const cap = motorImpulseCap(motor, dt, status);
        return .{ .active = true, .state = .inactive, .bias = motor.target_velocity, .lower = .{ .raw = -cap.raw }, .upper = cap };
    }
    return hard;
}
const ConeTwistData = struct { swing: fp.Fp, twist: fp.Fp, swing_axis: geometry.Vec3, twist_axis: geometry.Vec3 };
fn coneTwistData(world: *const body_world.World, body_a: ids.BodyId, body_b: ids.BodyId, frame_a: Frame, frame_b: Frame, status: *fp.MathStatus) Error!ConeTwistData {
    const a = world.bodyIndex(body_a) orelse return error.InvalidBody;
    const b = world.bodyIndex(body_b) orelse return error.InvalidBody;
    const axis_a = world.storage.orientation[a].rotate(frame_a.axis, status);
    const axis_b = world.storage.orientation[b].rotate(frame_b.axis, status);
    const cross = axis_a.cross(axis_b, status);
    // Parallel axes are the zero-swing case, not an invalid normalization.
    // `Vec3.normalize` deliberately records a math fault for a zero vector;
    // using it here made an identity Cone-Twist joint fault before it could
    // select its deterministic perpendicular fallback.
    const cross_length_squared = cross.lengthSquared(status);
    const sine = cross_length_squared.sqrt(status);
    const swing_axis = if (sine.raw == 0)
        try perpendicularUnit(axis_a, world.storage.orientation[a].rotate(frame_a.secondary, status), status)
    else
        cross.scale(fp.Fp.one.div(sine, status), status);
    const cosine = axis_a.dot(axis_b, status);
    const twist = try hingeCoordinate(world, body_a, body_b, frame_a, frame_b, status);
    return .{ .swing = geometry.atan2(sine, cosine, status), .twist = twist, .swing_axis = swing_axis, .twist_axis = axis_a };
}
fn coneSwingCoordinate(world: *const body_world.World, body_a: ids.BodyId, body_b: ids.BodyId, frame_a: Frame, frame_b: Frame, status: *fp.MathStatus) Error!fp.Fp {
    return (try coneTwistData(world, body_a, body_b, frame_a, frame_b, status)).swing;
}
fn relativeOrientation(world: *const body_world.World, body_a: ids.BodyId, body_b: ids.BodyId, status: *fp.MathStatus) Error!geometry.Quat {
    const a = world.bodyIndex(body_a) orelse return error.InvalidBody;
    const b = world.bodyIndex(body_b) orelse return error.InvalidBody;
    return world.storage.orientation[a].conjugate(status).mul(world.storage.orientation[b], status).canonicalize(status);
}
fn perpendicularUnit(axis: geometry.Vec3, candidate: geometry.Vec3, status: *fp.MathStatus) Error!geometry.Vec3 {
    var value = candidate.sub(axis.scale(candidate.dot(axis, status), status), status).normalize(status);
    if (value.valid) return value.value;
    const basis = if (@abs(axis.x.raw) <= @abs(axis.y.raw) and @abs(axis.x.raw) <= @abs(axis.z.raw)) geometry.Vec3.unit_x else if (@abs(axis.y.raw) <= @abs(axis.z.raw)) geometry.Vec3.unit_y else geometry.Vec3.unit_z;
    value = basis.sub(axis.scale(basis.dot(axis, status), status), status).normalize(status);
    if (!value.valid) return error.InvalidFrame;
    return value.value;
}
fn classifyLimit(limit: Limit, coordinate: fp.Fp) LimitState {
    if (!limit.enabled) return .inactive;
    if (limit.min.raw == limit.max.raw) return .equal;
    return if (coordinate.raw < limit.min.raw) .lower else if (coordinate.raw > limit.max.raw) .upper else .inactive;
}
fn springTerms(spring: Spring, displacement: fp.Fp, dt: fp.Fp, status: *fp.MathStatus) Error!struct { bias: fp.Fp, softness: fp.Fp } {
    if (spring.frequency.raw <= 0 or spring.damping_ratio.raw < 0) return error.InvalidLimit;
    const omega = geometry.tau.mul(spring.frequency, status);
    const stiffness = omega.mul(omega, status);
    const damping = fp.Fp.fromInt(2).mul(spring.damping_ratio, status).mul(omega, status);
    const denominator = dt.mul(damping.add(dt.mul(stiffness, status), status), status);
    if (denominator.raw <= 0) return error.InvalidLimit;
    const softness = fp.Fp.one.div(denominator, status);
    return .{ .bias = displacement.mul(dt.mul(stiffness, status).mul(softness, status), status), .softness = softness };
}
fn negate(value: geometry.Vec3, status: *fp.MathStatus) geometry.Vec3 {
    return .{ .x = value.x.neg(status), .y = value.y.neg(status), .z = value.z.neg(status) };
}
/// Emits the three Ball-Socket anchor-coincidence rows in frozen X/Y/Z order.
pub fn buildBallSocketRows(world: *const body_world.World, joint: *const Joint, owner: ids.JointId, output: []constraints.ConstraintRow, scratch: []constraints.ConstraintRow, status: *fp.MathStatus) (Error || constraints.Error)![]const constraints.ConstraintRow {
    if (joint.kind != .ball_socket) return error.InvalidJoint;
    const a = world.bodyIndex(joint.body_a) orelse return error.InvalidBody;
    const b = world.bodyIndex(joint.body_b) orelse return error.InvalidBody;
    const pa = world.storage.orientation[a].rotate(joint.frame_a.anchor, status).add(world.storage.position[a], status);
    const pb = world.storage.orientation[b].rotate(joint.frame_b.anchor, status).add(world.storage.position[b], status);
    const ra = pa.sub(world.storage.position[a], status);
    const rb = pb.sub(world.storage.position[b], status);
    const axes = [_]geometry.Vec3{ .unit_x, .unit_y, .unit_z };
    var specs: [3]constraints.RowSpec = undefined;
    for (axes, 0..) |axis, i| {
        const minus = geometry.Vec3{ .x = axis.x.neg(status), .y = axis.y.neg(status), .z = axis.z.neg(status) };
        specs[i] = .{ .kind = .joint, .body_a = joint.body_a, .body_b = joint.body_b, .owner = owner.value, .row_index = @intCast(i), .ja_linear = axis, .ja_angular = ra.cross(axis, status), .jb_linear = minus, .jb_angular = rb.cross(minus, status) };
    }
    return constraints.buildAuthoredRows(world, &specs, output, scratch, status);
}
/// Emits 3 anchor rows plus 2 angular rows orthogonal to the canonical hinge axis.
pub fn buildHingeRows(world: *const body_world.World, joint: *const Joint, owner: ids.JointId, output: []constraints.ConstraintRow, scratch: []constraints.ConstraintRow, status: *fp.MathStatus) (Error || constraints.Error)![]const constraints.ConstraintRow {
    if (joint.kind != .hinge) return error.InvalidJoint;
    const a = world.bodyIndex(joint.body_a) orelse return error.InvalidBody;
    const b = world.bodyIndex(joint.body_b) orelse return error.InvalidBody;
    const pa = world.storage.orientation[a].rotate(joint.frame_a.anchor, status).add(world.storage.position[a], status);
    const pb = world.storage.orientation[b].rotate(joint.frame_b.anchor, status).add(world.storage.position[b], status);
    const ra = pa.sub(world.storage.position[a], status);
    const rb = pb.sub(world.storage.position[b], status);
    const axis = world.storage.orientation[a].rotate(joint.frame_a.axis, status);
    const secondary = world.storage.orientation[a].rotate(joint.frame_a.secondary, status);
    const tertiary = axis.cross(secondary, status);
    const linear_axes = [_]geometry.Vec3{ .unit_x, .unit_y, .unit_z };
    var specs: [5]constraints.RowSpec = undefined;
    for (linear_axes, 0..) |v, i| {
        const minus = geometry.Vec3{ .x = v.x.neg(status), .y = v.y.neg(status), .z = v.z.neg(status) };
        specs[i] = .{ .kind = .joint, .body_a = joint.body_a, .body_b = joint.body_b, .owner = owner.value, .row_index = @intCast(i), .ja_linear = v, .ja_angular = ra.cross(v, status), .jb_linear = minus, .jb_angular = rb.cross(minus, status) };
    }
    inline for ([_]geometry.Vec3{ secondary, tertiary }, 0..) |v, i| {
        const minus = geometry.Vec3{ .x = v.x.neg(status), .y = v.y.neg(status), .z = v.z.neg(status) };
        specs[3 + i] = .{ .kind = .joint, .body_a = joint.body_a, .body_b = joint.body_b, .owner = owner.value, .row_index = @intCast(3 + i), .ja_angular = v, .jb_angular = minus };
    }
    return constraints.buildAuthoredRows(world, &specs, output, scratch, status);
}
/// Emits the six Fixed rows: three anchor translations followed by three world angular axes.
pub fn buildFixedRows(world: *const body_world.World, joint: *const Joint, owner: ids.JointId, output: []constraints.ConstraintRow, scratch: []constraints.ConstraintRow, status: *fp.MathStatus) (Error || constraints.Error)![]const constraints.ConstraintRow {
    if (joint.kind != .fixed) return error.InvalidJoint;
    const a = world.bodyIndex(joint.body_a) orelse return error.InvalidBody;
    const b = world.bodyIndex(joint.body_b) orelse return error.InvalidBody;
    const pa = world.storage.orientation[a].rotate(joint.frame_a.anchor, status).add(world.storage.position[a], status);
    const pb = world.storage.orientation[b].rotate(joint.frame_b.anchor, status).add(world.storage.position[b], status);
    const ra = pa.sub(world.storage.position[a], status);
    const rb = pb.sub(world.storage.position[b], status);
    const axes = [_]geometry.Vec3{ .unit_x, .unit_y, .unit_z };
    var specs: [6]constraints.RowSpec = undefined;
    for (axes, 0..) |v, i| {
        const minus = geometry.Vec3{ .x = v.x.neg(status), .y = v.y.neg(status), .z = v.z.neg(status) };
        specs[i] = .{ .kind = .joint, .body_a = joint.body_a, .body_b = joint.body_b, .owner = owner.value, .row_index = @intCast(i), .ja_linear = v, .ja_angular = ra.cross(v, status), .jb_linear = minus, .jb_angular = rb.cross(minus, status) };
        specs[3 + i] = .{ .kind = .joint, .body_a = joint.body_a, .body_b = joint.body_b, .owner = owner.value, .row_index = @intCast(3 + i), .ja_angular = v, .jb_angular = minus };
    }
    return constraints.buildAuthoredRows(world, &specs, output, scratch, status);
}
/// Emits Slider's 2 perpendicular translation and 3 angular lock rows.
pub fn buildSliderRows(world: *const body_world.World, joint: *const Joint, owner: ids.JointId, output: []constraints.ConstraintRow, scratch: []constraints.ConstraintRow, status: *fp.MathStatus) (Error || constraints.Error)![]const constraints.ConstraintRow {
    if (joint.kind != .slider) return error.InvalidJoint;
    const a = world.bodyIndex(joint.body_a) orelse return error.InvalidBody;
    const b = world.bodyIndex(joint.body_b) orelse return error.InvalidBody;
    const pa = world.storage.orientation[a].rotate(joint.frame_a.anchor, status).add(world.storage.position[a], status);
    const pb = world.storage.orientation[b].rotate(joint.frame_b.anchor, status).add(world.storage.position[b], status);
    const ra = pa.sub(world.storage.position[a], status);
    const rb = pb.sub(world.storage.position[b], status);
    const axis = world.storage.orientation[a].rotate(joint.frame_a.axis, status);
    const second = world.storage.orientation[a].rotate(joint.frame_a.secondary, status);
    const third = axis.cross(second, status);
    var specs: [5]constraints.RowSpec = undefined;
    for ([_]geometry.Vec3{ second, third }, 0..) |v, i| {
        const minus = geometry.Vec3{ .x = v.x.neg(status), .y = v.y.neg(status), .z = v.z.neg(status) };
        specs[i] = .{ .kind = .joint, .body_a = joint.body_a, .body_b = joint.body_b, .owner = owner.value, .row_index = @intCast(i), .ja_linear = v, .ja_angular = ra.cross(v, status), .jb_linear = minus, .jb_angular = rb.cross(minus, status) };
    }
    for ([_]geometry.Vec3{ axis, second, third }, 0..) |v, i| {
        const minus = geometry.Vec3{ .x = v.x.neg(status), .y = v.y.neg(status), .z = v.z.neg(status) };
        specs[2 + i] = .{ .kind = .joint, .body_a = joint.body_a, .body_b = joint.body_b, .owner = owner.value, .row_index = @intCast(2 + i), .ja_angular = v, .jb_angular = minus };
    }
    return constraints.buildAuthoredRows(world, &specs, output, scratch, status);
}
/// Emits Cone-Twist's three Ball-Socket anchor rows. Swing and twist are
/// deliberately released here and are constrained only by its control rows.
pub fn buildConeTwistRows(world: *const body_world.World, joint: *const Joint, owner: ids.JointId, output: []constraints.ConstraintRow, scratch: []constraints.ConstraintRow, status: *fp.MathStatus) (Error || constraints.Error)![]const constraints.ConstraintRow {
    if (joint.kind != .cone_twist) return error.InvalidJoint;
    const a = world.bodyIndex(joint.body_a) orelse return error.InvalidBody;
    const b = world.bodyIndex(joint.body_b) orelse return error.InvalidBody;
    const pa = world.storage.orientation[a].rotate(joint.frame_a.anchor, status).add(world.storage.position[a], status);
    const pb = world.storage.orientation[b].rotate(joint.frame_b.anchor, status).add(world.storage.position[b], status);
    const ra = pa.sub(world.storage.position[a], status);
    const rb = pb.sub(world.storage.position[b], status);
    var specs: [3]constraints.RowSpec = undefined;
    for ([_]geometry.Vec3{ .unit_x, .unit_y, .unit_z }, 0..) |v, i| {
        const minus = geometry.Vec3{ .x = v.x.neg(status), .y = v.y.neg(status), .z = v.z.neg(status) };
        specs[i] = .{ .kind = .joint, .body_a = joint.body_a, .body_b = joint.body_b, .owner = owner.value, .row_index = @intCast(i), .ja_linear = v, .ja_angular = ra.cross(v, status), .jb_linear = minus, .jb_angular = rb.cross(minus, status) };
    }
    return constraints.buildAuthoredRows(world, &specs, output, scratch, status);
}
fn canonicalFrame(frame: Frame, status: *fp.MathStatus) Error!Frame {
    const axis = frame.axis.normalize(status);
    if (!axis.valid) return error.InvalidFrame;
    var second = frame.secondary.sub(axis.value.scale(frame.secondary.dot(axis.value, status), status), status).normalize(status);
    if (!second.valid) {
        const basis = if (@abs(axis.value.x.raw) <= @abs(axis.value.y.raw) and @abs(axis.value.x.raw) <= @abs(axis.value.z.raw)) geometry.Vec3.unit_x else if (@abs(axis.value.y.raw) <= @abs(axis.value.z.raw)) geometry.Vec3.unit_y else geometry.Vec3.unit_z;
        second = basis.sub(axis.value.scale(basis.dot(axis.value, status), status), status).normalize(status);
        if (!second.valid) return error.InvalidFrame;
    }
    return .{ .anchor = frame.anchor, .axis = axis.value, .secondary = second.value };
}
