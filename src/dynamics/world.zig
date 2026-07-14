//! Deterministic, caller-buffered Body/Collider state and command transaction.
const std = @import("std");
const fp = @import("../math/fp.zig");
const geometry = @import("../math/geometry.zig");
const ids = @import("../core/ids.zig");
const shapes = @import("../collision/shapes.zig");

pub const Error = shapes.Error || error{ CapacityExceeded, InvalidBody, InvalidCollider, InvalidMass, InvalidCommand };
pub const DofLock = packed struct(u8) { linear_x: bool = false, linear_y: bool = false, linear_z: bool = false, angular_x: bool = false, angular_y: bool = false, angular_z: bool = false, _: u2 = 0 };
pub const Desc = struct { body_type: shapes.BodyType = .dynamic, transform: geometry.Transform3 = .{}, inverse_mass: fp.Fp = fp.Fp.zero, inverse_inertia_local: geometry.SymmetricMat3, locks: DofLock = .{} };
pub const ColliderDesc = struct { body: ids.BodyId, collider: shapes.Collider };
pub const CommandKey = struct {
    phase_priority: u8,
    issuer: u32,
    sequence: u32,
    discriminant: u8,
    pub fn lessThan(a: CommandKey, b: CommandKey) bool {
        return if (a.phase_priority != b.phase_priority) a.phase_priority < b.phase_priority else if (a.issuer != b.issuer) a.issuer < b.issuer else if (a.sequence != b.sequence) a.sequence < b.sequence else a.discriminant < b.discriminant;
    }
};
pub const Command = struct {
    key: CommandKey,
    op: union(enum) {
        force: struct { body: ids.BodyId, value: geometry.Vec3 },
        torque: struct { body: ids.BodyId, value: geometry.Vec3 },
        impulse_at_point: struct { body: ids.BodyId, impulse: geometry.Vec3, point: geometry.Vec3 },
        velocity: struct { body: ids.BodyId, linear: geometry.Vec3, angular: geometry.Vec3 },
        kinematic_target: struct { body: ids.BodyId, target: geometry.Transform3 },
        locks: struct { body: ids.BodyId, value: DofLock },
    },
    pub fn canonicalKey(self: Command) CommandKey {
        var key = self.key;
        key.discriminant = @intFromEnum(self.op);
        return key;
    }
};
pub const Storage = struct {
    body_type: []shapes.BodyType,
    position: []geometry.Vec3,
    orientation: []geometry.Quat,
    linear_velocity: []geometry.Vec3,
    angular_velocity: []geometry.Vec3,
    inverse_mass: []fp.Fp,
    inverse_inertia_local: []geometry.SymmetricMat3,
    force: []geometry.Vec3,
    torque: []geometry.Vec3,
    locks: []DofLock,
    generation: []u32,
    alive: []bool,
    retired: []bool,
    has_target: []bool,
    target_position: []geometry.Vec3,
    target_orientation: []geometry.Quat,
};
/// Caller-owned collider columns. Shape payloads are immutable values; every
/// relation and mutable flag remains a separate deterministic SoA column.
pub const ColliderStorage = struct {
    body: []ids.BodyId,
    local: []geometry.Transform3,
    shape: []shapes.Shape,
    material: []shapes.Material,
    category: []u32,
    mask: []u32,
    group: []i32,
    sensor: []bool,
    enabled: []bool,
    revision: []u32,
    generation: []u32,
    alive: []bool,
    retired: []bool,
};
pub const Settings = struct { gravity: geometry.Vec3 = .{}, linear_damping: fp.Fp = fp.Fp.zero, angular_damping: fp.Fp = fp.Fp.zero, max_linear_speed: fp.Fp = fp.Fp.max, max_angular_speed: fp.Fp = fp.Fp.max };
pub const World = struct {
    storage: Storage,
    colliders: ?ColliderStorage = null,
    settings: Settings = .{},
    pub fn init(storage: Storage) Error!World {
        try checkStorage(storage);
        @memset(storage.alive, false);
        @memset(storage.retired, false);
        @memset(storage.generation, 0);
        @memset(storage.has_target, false);
        return .{ .storage = storage };
    }
    pub fn initWithColliders(storage: Storage, colliders: ColliderStorage) Error!World {
        var result = try init(storage);
        try checkColliders(colliders);
        @memset(colliders.alive, false);
        @memset(colliders.retired, false);
        @memset(colliders.generation, 0);
        result.colliders = colliders;
        return result;
    }
    pub fn create(self: *World, desc: Desc, status: *fp.MathStatus) Error!ids.BodyId {
        if ((desc.body_type == .dynamic and (desc.inverse_mass.raw <= 0 or !validDynamicInertia(desc.inverse_inertia_local, status))) or (desc.body_type != .dynamic and desc.inverse_mass.raw != 0)) return error.InvalidMass;
        for (self.storage.alive, 0..) |used, i| if (!used and !self.storage.retired[i]) {
            self.storage.body_type[i] = desc.body_type;
            self.storage.position[i] = desc.transform.position;
            self.storage.orientation[i] = desc.transform.orientation.canonicalize(status);
            self.storage.linear_velocity[i] = .{};
            self.storage.angular_velocity[i] = .{};
            self.storage.inverse_mass[i] = desc.inverse_mass;
            self.storage.inverse_inertia_local[i] = desc.inverse_inertia_local;
            self.storage.force[i] = .{};
            self.storage.torque[i] = .{};
            self.storage.locks[i] = desc.locks;
            self.storage.has_target[i] = false;
            // These fields remain part of the canonical World state even
            // while no kinematic target is pending.  Initialize them for all
            // body types so a newly-created dynamic body never contributes
            // stack garbage to the cross-mode/cross-target state hash.
            self.storage.target_position[i] = desc.transform.position;
            self.storage.target_orientation[i] = self.storage.orientation[i];
            self.storage.alive[i] = true;
            return ids.BodyId.init(@intCast(i), self.storage.generation[i]);
        };
        return error.CapacityExceeded;
    }
    pub fn createCollider(self: *World, collider: shapes.Collider) Error!ids.ColliderId {
        const storage = &(self.colliders orelse return error.InvalidCollider);
        const body = self.index(collider.body) orelse return error.InvalidBody;
        try shapes.validateBodyShape(collider.shape, self.storage.body_type[body]);
        for (storage.alive, 0..) |used, i| if (!used and !storage.retired[i]) {
            storage.body[i] = collider.body;
            storage.local[i] = collider.local;
            storage.shape[i] = collider.shape;
            storage.material[i] = collider.material;
            storage.category[i] = collider.category;
            storage.mask[i] = collider.mask;
            storage.group[i] = collider.group;
            storage.sensor[i] = collider.sensor;
            storage.enabled[i] = collider.enabled;
            storage.revision[i] = collider.revision;
            storage.alive[i] = true;
            return ids.ColliderId.init(@intCast(i), storage.generation[i]);
        };
        return error.CapacityExceeded;
    }
    /// Removes one collider generation without disturbing unrelated rows.
    pub fn destroyCollider(self: *World, id: ids.ColliderId) Error!void {
        const storage = &(self.colliders orelse return error.InvalidCollider);
        const i: usize = id.index();
        if (i >= storage.alive.len or !storage.alive[i] or storage.generation[i] != id.generation()) return error.InvalidCollider;
        storage.alive[i] = false;
        advance(&storage.generation[i], &storage.retired[i]);
    }
    pub fn destroy(self: *World, id: ids.BodyId) Error!void {
        const i = self.index(id) orelse return error.InvalidBody;
        self.storage.alive[i] = false;
        self.storage.force[i] = .{};
        self.storage.torque[i] = .{};
        self.storage.has_target[i] = false;
        advance(&self.storage.generation[i], &self.storage.retired[i]);
        if (self.colliders) |*cs| for (cs.body, 0..) |body, c| if (cs.alive[c] and body.value == id.value) {
            cs.alive[c] = false;
            advance(&cs.generation[c], &cs.retired[c]);
        };
    }
    pub fn execute(self: *World, commands: []const Command, scratch: []Command, dt: fp.Fp, status: *fp.MathStatus) Error!void {
        const ordered = try self.orderedCommands(commands, scratch);
        for (ordered) |command| try self.commit(command, dt, status);
    }
    /// Validates and canonicalizes a batch without changing body state.  This
    /// is used by stateful pipeline phases that must reserve event capacity
    /// before they commit any of their own state.
    pub fn orderedCommands(self: *const World, commands: []const Command, scratch: []Command) Error![]const Command {
        if (commands.len > scratch.len) return error.CapacityExceeded;
        for (commands, 0..) |command, i| scratch[i] = command;
        insertionSort(scratch[0..commands.len]);
        for (scratch[0..commands.len]) |command| try self.validate(command);
        return scratch[0..commands.len];
    }
    pub fn step(self: *World, dt: fp.Fp, status: *fp.MathStatus) void {
        self.stepSubstep(dt, status);
        self.finishTick();
    }
    /// Integrates one pipeline substep. Kinematic targets intentionally remain
    /// pending until `finishTick`, so a target velocity spans every substep.
    pub fn stepSubstep(self: *World, dt: fp.Fp, status: *fp.MathStatus) void {
        self.integrateVelocities(dt, status);
        self.integratePositions(dt, status);
    }
    /// Applies forces and torques to dynamic velocities for one substep.
    /// Constraint/contact solvers run after this stage and before positions.
    pub fn integrateVelocities(self: *World, dt: fp.Fp, status: *fp.MathStatus) void {
        self.integrateLinearVelocity(dt, status, null);
        self.integrateAngularVelocity(dt, status, null);
    }
    /// Applies one velocity stage only to awake dynamic body slots. The mask
    /// is caller-owned persistent sleep state and must match World capacity.
    pub fn integrateVelocitiesAwake(self: *World, awake: []const bool, dt: fp.Fp, status: *fp.MathStatus) Error!void {
        if (awake.len != self.storage.alive.len) return error.CapacityExceeded;
        self.integrateLinearVelocity(dt, status, awake);
        self.integrateAngularVelocity(dt, status, awake);
    }
    /// Advances dynamic positions and orientations from already-solved
    /// velocities for one substep.
    pub fn integratePositions(self: *World, dt: fp.Fp, status: *fp.MathStatus) void {
        self.integratePositionState(dt, status, null);
    }
    /// Advances positions/orientations only for awake dynamic body slots.
    pub fn integratePositionsAwake(self: *World, awake: []const bool, dt: fp.Fp, status: *fp.MathStatus) Error!void {
        if (awake.len != self.storage.alive.len) return error.CapacityExceeded;
        self.integratePositionState(dt, status, awake);
    }
    /// Commits deferred kinematic targets once after all Tick substeps.
    pub fn finishTick(self: *World) void {
        self.snapKinematics();
    }
    /// Integrates only awake dynamic slots.  Sleep storage remains external to
    /// this module so World keeps its Task 13 layout contract; the pipeline
    /// must wake a body before giving it a nonzero force, impulse or velocity.
    pub fn stepAwake(self: *World, awake: []const bool, dt: fp.Fp, status: *fp.MathStatus) Error!void {
        if (awake.len != self.storage.alive.len) return error.CapacityExceeded;
        try self.integrateVelocitiesAwake(awake, dt, status);
        try self.integratePositionsAwake(awake, dt, status);
        self.snapKinematics();
    }
    /// Returns the live slot for a generation-checked body identifier.
    pub fn bodyIndex(self: *const World, id: ids.BodyId) ?usize {
        return self.index(id);
    }
    /// Returns the current generation ID for one live body slot.
    pub fn bodyIdAt(self: *const World, slot: usize) ?ids.BodyId {
        if (slot >= self.storage.alive.len or !self.storage.alive[slot]) return null;
        return ids.BodyId.init(@intCast(slot), self.storage.generation[slot]);
    }
    /// Visits every future-relevant World-owned field in slot order. Derived
    /// broadphase/island state is deliberately excluded; callers combine this
    /// with external cache/joint/sleep visitors for canonical Tick hashing.
    pub fn visitCanonical(self: *const World, visitor: anytype) void {
        // Settings are World-owned simulation inputs. Omitting them would let
        // two worlds with different future integration behavior share a hash.
        visitVec3(self.settings.gravity, visitor);
        visitor.writeI64(self.settings.linear_damping.raw);
        visitor.writeI64(self.settings.angular_damping.raw);
        visitor.writeI64(self.settings.max_linear_speed.raw);
        visitor.writeI64(self.settings.max_angular_speed.raw);
        visitor.writeU64(self.storage.alive.len);
        for (self.storage.alive, 0..) |alive, i| {
            visitor.writeU8(@intFromBool(alive));
            visitor.writeU8(@intFromBool(self.storage.retired[i]));
            visitor.writeU32(self.storage.generation[i]);
            if (!alive) continue;
            visitor.writeU8(@intFromEnum(self.storage.body_type[i]));
            visitVec3(self.storage.position[i], visitor);
            visitQuat(self.storage.orientation[i], visitor);
            visitVec3(self.storage.linear_velocity[i], visitor);
            visitVec3(self.storage.angular_velocity[i], visitor);
            visitor.writeI64(self.storage.inverse_mass[i].raw);
            visitSymmetric(self.storage.inverse_inertia_local[i], visitor);
            visitVec3(self.storage.force[i], visitor);
            visitVec3(self.storage.torque[i], visitor);
            visitor.writeU8(@bitCast(self.storage.locks[i]));
            visitor.writeU8(@intFromBool(self.storage.has_target[i]));
            visitVec3(self.storage.target_position[i], visitor);
            visitQuat(self.storage.target_orientation[i], visitor);
        }
        if (self.colliders) |colliders| {
            visitor.writeU8(1);
            visitor.writeU64(colliders.alive.len);
            for (colliders.alive, 0..) |alive, i| {
                visitor.writeU8(@intFromBool(alive));
                visitor.writeU8(@intFromBool(colliders.retired[i]));
                visitor.writeU32(colliders.generation[i]);
                if (!alive) continue;
                visitor.writeU64(colliders.body[i].value);
                visitTransform(colliders.local[i], visitor);
                visitShape(colliders.shape[i], visitor);
                visitor.writeI64(colliders.material[i].friction.raw);
                visitor.writeI64(colliders.material[i].restitution.raw);
                visitor.writeU32(colliders.category[i]);
                visitor.writeU32(colliders.mask[i]);
                visitor.writeU32(@bitCast(colliders.group[i]));
                visitor.writeU8(@intFromBool(colliders.sensor[i]));
                visitor.writeU8(@intFromBool(colliders.enabled[i]));
                visitor.writeU32(colliders.revision[i]);
            }
        } else visitor.writeU8(0);
    }
    pub fn applyForce(self: *World, id: ids.BodyId, value: geometry.Vec3, status: *fp.MathStatus) Error!void {
        const i = try self.dynamic(id);
        self.storage.force[i] = self.storage.force[i].add(value, status);
    }
    pub fn applyTorque(self: *World, id: ids.BodyId, value: geometry.Vec3, status: *fp.MathStatus) Error!void {
        const i = try self.dynamic(id);
        self.storage.torque[i] = self.storage.torque[i].add(value, status);
    }
    pub fn applyImpulseAtPoint(self: *World, id: ids.BodyId, impulse: geometry.Vec3, point: geometry.Vec3, status: *fp.MathStatus) Error!void {
        const i = try self.dynamic(id);
        self.storage.linear_velocity[i] = self.storage.linear_velocity[i].add(impulse.scale(self.storage.inverse_mass[i], status), status);
        const r = point.sub(self.storage.position[i], status);
        const inverse_world = self.storage.inverse_inertia_local[i].rotate(self.storage.orientation[i], status).toMat3();
        self.storage.angular_velocity[i] = self.storage.angular_velocity[i].add(inverse_world.mulVec(r.cross(impulse, status), status), status);
        applyLocks(&self.storage.linear_velocity[i], &self.storage.angular_velocity[i], self.storage.locks[i]);
    }
    fn validate(self: *const World, command: Command) Error!void {
        const id = switch (command.op) {
            inline else => |v| v.body,
        };
        const i = self.index(id) orelse return error.InvalidBody;
        switch (command.op) {
            .kinematic_target => if (self.storage.body_type[i] != .kinematic) return error.InvalidCommand,
            .locks => if (self.storage.body_type[i] == .static) return error.InvalidCommand,
            else => if (self.storage.body_type[i] != .dynamic) return error.InvalidCommand,
        }
    }
    fn commit(self: *World, command: Command, dt: fp.Fp, status: *fp.MathStatus) Error!void {
        switch (command.op) {
            .force => |v| try self.applyForce(v.body, v.value, status),
            .torque => |v| try self.applyTorque(v.body, v.value, status),
            .impulse_at_point => |v| try self.applyImpulseAtPoint(v.body, v.impulse, v.point, status),
            .velocity => |v| {
                const i = try self.dynamic(v.body);
                self.storage.linear_velocity[i] = v.linear;
                self.storage.angular_velocity[i] = v.angular;
                applyLocks(&self.storage.linear_velocity[i], &self.storage.angular_velocity[i], self.storage.locks[i]);
            },
            .locks => |v| {
                const i = self.index(v.body).?;
                self.storage.locks[i] = v.value;
                applyLocks(&self.storage.linear_velocity[i], &self.storage.angular_velocity[i], v.value);
            },
            .kinematic_target => |v| {
                const i = self.index(v.body).?;
                self.storage.target_position[i] = v.target.position;
                self.storage.target_orientation[i] = v.target.orientation.canonicalize(status);
                self.storage.has_target[i] = true;
                self.storage.linear_velocity[i] = v.target.position.sub(self.storage.position[i], status).scale(fp.Fp.one.div(dt, status), status);
                // The delta quaternion supplies the deterministic one-tick
                // angular velocity seen by collision/constraint code.  The
                // target is still snapped after integration, so this bounded
                // first-order velocity never accumulates orientation drift.
                const delta = self.storage.target_orientation[i].mul(self.storage.orientation[i].conjugate(status), status);
                self.storage.angular_velocity[i] = .{ .x = delta.x.mul(fp.Fp.fromInt(2), status).div(dt, status), .y = delta.y.mul(fp.Fp.fromInt(2), status).div(dt, status), .z = delta.z.mul(fp.Fp.fromInt(2), status).div(dt, status) };
                applyLocks(&self.storage.linear_velocity[i], &self.storage.angular_velocity[i], self.storage.locks[i]);
            },
        }
    }
    fn integrateLinearVelocity(self: *World, dt: fp.Fp, status: *fp.MathStatus, awake: ?[]const bool) void {
        for (self.storage.alive, 0..) |alive, i| {
            if (!alive or self.storage.body_type[i] != .dynamic or (awake != null and !awake.?[i])) continue;
            const a = self.settings.gravity.add(self.storage.force[i].scale(self.storage.inverse_mass[i], status), status);
            self.storage.linear_velocity[i] = self.storage.linear_velocity[i].add(a.scale(dt, status), status).scale(fp.Fp.one.sub(self.settings.linear_damping.mul(dt, status), status), status);
            clamp(&self.storage.linear_velocity[i], self.settings.max_linear_speed, status);
            applyLocks(&self.storage.linear_velocity[i], &self.storage.angular_velocity[i], self.storage.locks[i]);
            self.storage.force[i] = .{};
        }
    }
    fn integrateAngularVelocity(self: *World, dt: fp.Fp, status: *fp.MathStatus, awake: ?[]const bool) void {
        for (self.storage.alive, 0..) |alive, i| {
            if (!alive or self.storage.body_type[i] != .dynamic or (awake != null and !awake.?[i])) continue;
            const inv_world = self.storage.inverse_inertia_local[i].rotate(self.storage.orientation[i], status).toMat3();
            const inertia = inv_world.inverse(status);
            const angular_momentum = if (inertia.valid) inertia.value.mulVec(self.storage.angular_velocity[i], status) else geometry.Vec3.zero;
            const gyro = self.storage.angular_velocity[i].cross(angular_momentum, status);
            const accel = inv_world.mulVec(self.storage.torque[i].sub(gyro, status), status);
            self.storage.angular_velocity[i] = self.storage.angular_velocity[i].add(accel.scale(dt, status), status).scale(fp.Fp.one.sub(self.settings.angular_damping.mul(dt, status), status), status);
            clamp(&self.storage.angular_velocity[i], self.settings.max_angular_speed, status);
            applyLocks(&self.storage.linear_velocity[i], &self.storage.angular_velocity[i], self.storage.locks[i]);
            self.storage.torque[i] = .{};
        }
    }
    fn integratePositionState(self: *World, dt: fp.Fp, status: *fp.MathStatus, awake: ?[]const bool) void {
        for (self.storage.alive, 0..) |alive, i| {
            if (!alive or self.storage.body_type[i] != .dynamic or (awake != null and !awake.?[i])) continue;
            self.storage.position[i] = self.storage.position[i].add(self.storage.linear_velocity[i].scale(dt, status), status);
            self.storage.orientation[i] = self.storage.orientation[i].integrate(self.storage.angular_velocity[i], dt, status);
        }
    }
    fn snapKinematics(self: *World) void {
        for (self.storage.alive, 0..) |alive, i| {
            if (alive and self.storage.body_type[i] == .kinematic and self.storage.has_target[i]) {
                self.storage.position[i] = self.storage.target_position[i];
                self.storage.orientation[i] = self.storage.target_orientation[i];
                self.storage.has_target[i] = false;
            }
        }
    }
    fn index(self: *const World, id: ids.BodyId) ?usize {
        const i: usize = id.index();
        return if (i < self.storage.alive.len and self.storage.alive[i] and self.storage.generation[i] == id.generation()) i else null;
    }
    fn dynamic(self: *World, id: ids.BodyId) Error!usize {
        const i = self.index(id) orelse return error.InvalidBody;
        if (self.storage.body_type[i] != .dynamic) return error.InvalidBody;
        return i;
    }
};
fn checkStorage(s: Storage) Error!void {
    const n = s.alive.len;
    inline for (.{ s.body_type.len, s.position.len, s.orientation.len, s.linear_velocity.len, s.angular_velocity.len, s.inverse_mass.len, s.inverse_inertia_local.len, s.force.len, s.torque.len, s.locks.len, s.generation.len, s.retired.len, s.has_target.len, s.target_position.len, s.target_orientation.len }) |len| if (len != n) return error.CapacityExceeded;
}
fn visitVec3(value: geometry.Vec3, visitor: anytype) void {
    visitor.writeI64(value.x.raw);
    visitor.writeI64(value.y.raw);
    visitor.writeI64(value.z.raw);
}
fn visitQuat(value: geometry.Quat, visitor: anytype) void {
    visitor.writeI64(value.x.raw);
    visitor.writeI64(value.y.raw);
    visitor.writeI64(value.z.raw);
    visitor.writeI64(value.w.raw);
}
fn visitTransform(value: geometry.Transform3, visitor: anytype) void {
    visitVec3(value.position, visitor);
    visitQuat(value.orientation, visitor);
}
fn visitSymmetric(value: geometry.SymmetricMat3, visitor: anytype) void {
    visitor.writeI64(value.xx.raw);
    visitor.writeI64(value.yy.raw);
    visitor.writeI64(value.zz.raw);
    visitor.writeI64(value.xy.raw);
    visitor.writeI64(value.xz.raw);
    visitor.writeI64(value.yz.raw);
}
fn visitShape(value: shapes.Shape, visitor: anytype) void {
    visitor.writeU8(@intFromEnum(value));
    switch (value) {
        .sphere => |sphere| visitor.writeI64(sphere.radius.raw),
        .box => |box| visitVec3(box.half_extents, visitor),
        .capsule => |capsule| {
            visitor.writeI64(capsule.radius.raw);
            visitor.writeI64(capsule.half_height.raw);
        },
        inline else => |asset| {
            visitor.writeU64(asset.source_id);
            visitor.writeU64(asset.asset.value);
            visitor.writeU32(asset.revision);
        },
    }
}
/// Sylvester's criterion on the symmetric inverse inertia prevents singular
/// or non-physical dynamic tensors before a slot is published.
fn validDynamicInertia(value: geometry.SymmetricMat3, status: *fp.MathStatus) bool {
    if (value.xx.raw <= 0 or value.yy.raw <= 0 or value.zz.raw <= 0) return false;
    const first_minor = value.xx.mul(value.yy, status).sub(value.xy.mul(value.xy, status), status);
    if (first_minor.raw <= 0) return false;
    const yyzz = value.yy.mul(value.zz, status).sub(value.yz.mul(value.yz, status), status);
    const xzyz = value.xy.mul(value.zz, status).sub(value.xz.mul(value.yz, status), status);
    const xyyz = value.xy.mul(value.yz, status).sub(value.xz.mul(value.yy, status), status);
    const determinant = value.xx.mul(yyzz, status).sub(value.xy.mul(xzyz, status), status).add(value.xz.mul(xyyz, status), status);
    return determinant.raw > 0;
}
fn checkColliders(s: ColliderStorage) Error!void {
    const n = s.alive.len;
    inline for (.{ s.body.len, s.local.len, s.shape.len, s.material.len, s.category.len, s.mask.len, s.group.len, s.sensor.len, s.enabled.len, s.revision.len, s.generation.len, s.retired.len }) |len| if (len != n) return error.CapacityExceeded;
}
fn advance(generation: *u32, retired: *bool) void {
    if (generation.* == std.math.maxInt(u32)) retired.* = true else generation.* += 1;
}
fn insertionSort(items: []Command) void {
    var i: usize = 1;
    while (i < items.len) : (i += 1) {
        const value = items[i];
        var j = i;
        while (j > 0 and value.canonicalKey().lessThan(items[j - 1].canonicalKey())) : (j -= 1) items[j] = items[j - 1];
        items[j] = value;
    }
}
fn applyLocks(linear: *geometry.Vec3, angular: *geometry.Vec3, locks: DofLock) void {
    if (locks.linear_x) linear.x = .zero;
    if (locks.linear_y) linear.y = .zero;
    if (locks.linear_z) linear.z = .zero;
    if (locks.angular_x) angular.x = .zero;
    if (locks.angular_y) angular.y = .zero;
    if (locks.angular_z) angular.z = .zero;
}
fn clamp(value: *geometry.Vec3, limit: fp.Fp, status: *fp.MathStatus) void {
    // `Fp.max` is the documented unbounded default. Squaring it would create
    // a spurious MathFault on every otherwise-valid integration step.
    if (limit.raw == fp.Fp.max.raw) return;
    const n = value.lengthSquared(status);
    const l2 = limit.mul(limit, status);
    if (n.raw > l2.raw and n.raw > 0) value.* = value.scale(limit.div(n.sqrt(status), status), status);
}
