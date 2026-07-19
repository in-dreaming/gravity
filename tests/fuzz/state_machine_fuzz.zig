const std = @import("std");
const gravity = @import("gravity");

const fp = gravity.math.fp;
const geometry = gravity.math.geometry;
const shapes = gravity.collision.shapes;
const world = gravity.dynamics.world;
const joints = gravity.dynamics.joints;
const rollback = gravity.state.rollback;
const hash = gravity.state.hash;

const capacity = 16;

const Fixture = struct {
    types: [capacity]shapes.BodyType = undefined,
    position: [capacity]geometry.Vec3 = undefined,
    orientation: [capacity]geometry.Quat = undefined,
    linear: [capacity]geometry.Vec3 = undefined,
    angular: [capacity]geometry.Vec3 = undefined,
    mass: [capacity]fp.Fp = undefined,
    inertia: [capacity]geometry.SymmetricMat3 = undefined,
    force: [capacity]geometry.Vec3 = undefined,
    torque: [capacity]geometry.Vec3 = undefined,
    locks: [capacity]world.DofLock = undefined,
    generation: [capacity]u32 = undefined,
    alive: [capacity]bool = undefined,
    retired: [capacity]bool = undefined,
    target: [capacity]bool = undefined,
    target_position: [capacity]geometry.Vec3 = undefined,
    target_orientation: [capacity]geometry.Quat = undefined,

    fn init(self: *Fixture) !world.World {
        return world.World.init(.{
            .body_type = &self.types,
            .position = &self.position,
            .orientation = &self.orientation,
            .linear_velocity = &self.linear,
            .angular_velocity = &self.angular,
            .inverse_mass = &self.mass,
            .inverse_inertia_local = &self.inertia,
            .force = &self.force,
            .torque = &self.torque,
            .locks = &self.locks,
            .generation = &self.generation,
            .alive = &self.alive,
            .retired = &self.retired,
            .has_target = &self.target,
            .target_position = &self.target_position,
            .target_orientation = &self.target_orientation,
        });
    }
};

const Digest = struct {
    value: u64 = 0xcbf29ce484222325,
    fn mix(self: *Digest, input: u64) void {
        self.value = (self.value ^ input) *% 0x100000001b3;
    }
    pub fn writeU8(self: *Digest, value: u8) void {
        self.mix(value);
    }
    pub fn writeU32(self: *Digest, value: u32) void {
        self.mix(value);
    }
    pub fn writeU64(self: *Digest, value: u64) void {
        self.mix(value);
    }
    pub fn writeI64(self: *Digest, value: i64) void {
        self.mix(@bitCast(value));
    }
};

fn inertia() geometry.SymmetricMat3 {
    return .{ .xx = .one, .yy = .one, .zz = .one, .xy = .zero, .xz = .zero, .yz = .zero };
}

fn advance(seed: *u64) u64 {
    seed.* = seed.* *% 6_364_136_223_846_793_005 +% 1_442_695_040_888_963_407;
    return seed.*;
}

test "bounded body joint and rollback state-machine fuzz has a frozen digest" {
    var fixture: Fixture = .{};
    var state = try fixture.init();
    var joint_values: [capacity]joints.Joint = undefined;
    var joint_generations: [capacity]u32 = undefined;
    var joint_alive: [capacity]bool = undefined;
    var joint_retired: [capacity]bool = undefined;
    var pool = try joints.Pool.init(.{
        .values = &joint_values,
        .generation = &joint_generations,
        .alive = &joint_alive,
        .retired = &joint_retired,
    });

    var ticks: [8]u64 = undefined;
    var snapshot_lengths: [8]usize = undefined;
    var input_lengths: [8]usize = undefined;
    var hashes: [8]hash.Hash128 = undefined;
    var snapshots: [8 * 16]u8 = undefined;
    var inputs: [8 * 8]u8 = undefined;
    var valid: [8]bool = undefined;
    var ring = try rollback.Ring.init(&ticks, &snapshot_lengths, &input_lengths, &hashes, &snapshots, &inputs, &valid, 16, 8);

    var status: fp.MathStatus = .{};
    var seed: u64 = 0x25f0_5eed_cafe_babe;
    for (0..10_000) |step| {
        const random = advance(&seed);
        switch (random % 8) {
            0, 1 => {
                const x = fp.Fp.fromInt(@as(i32, @intCast((random >> 8) % 17)) - 8);
                _ = state.create(.{
                    .transform = .{ .position = .{ .x = x } },
                    .inverse_mass = .one,
                    .inverse_inertia_local = inertia(),
                }, &status) catch |err| try std.testing.expectEqual(error.CapacityExceeded, err);
            },
            2 => {
                const slot: usize = @intCast((random >> 16) % capacity);
                if (state.bodyIdAt(slot)) |id| {
                    try joints.destroyBody(&state, &pool, id);
                    try std.testing.expectError(error.InvalidBody, joints.destroyBody(&state, &pool, id));
                }
            },
            3, 4 => {
                const a_slot: usize = @intCast((random >> 16) % capacity);
                const b_slot: usize = @intCast((random >> 24) % capacity);
                if (a_slot != b_slot) if (state.bodyIdAt(a_slot)) |a| if (state.bodyIdAt(b_slot)) |b| {
                    const kinds = [_]joints.Kind{ .distance, .ball_socket, .hinge, .slider, .fixed, .cone_twist };
                    _ = pool.create(&state, .{
                        .kind = kinds[@intCast((random >> 32) % kinds.len)],
                        .body_a = a,
                        .body_b = b,
                        .reference = .zero,
                        .swing_reference = .zero,
                        .reference_orientation = .identity,
                    }, &status) catch |err| try std.testing.expectEqual(error.CapacityExceeded, err);
                };
            },
            5 => {
                const slot: usize = @intCast((random >> 16) % capacity);
                if (joint_alive[slot]) {
                    const id = gravity.core.ids.JointId.init(@intCast(slot), joint_generations[slot]);
                    try pool.destroy(id);
                    try std.testing.expectError(error.InvalidJoint, pool.destroy(id));
                }
            },
            6 => {
                const slot: usize = @intCast((random >> 16) % capacity);
                const invalid_body = gravity.core.ids.BodyId.init(@intCast(slot), fixture.generation[slot] + 1);
                const invalid_joint = gravity.core.ids.JointId.init(@intCast(slot), joint_generations[slot] + 1);
                try std.testing.expectError(error.InvalidBody, state.destroy(invalid_body));
                try std.testing.expectError(error.InvalidJoint, pool.destroy(invalid_joint));
            },
            7 => {
                var snapshot_bytes: [16]u8 = undefined;
                var input_bytes: [8]u8 = undefined;
                std.mem.writeInt(u64, snapshot_bytes[0..8], @intCast(step), .little);
                std.mem.writeInt(u64, snapshot_bytes[8..16], seed, .little);
                std.mem.writeInt(u64, &input_bytes, random, .little);
                const state_hash = hash.oneShot128(.state, &snapshot_bytes);
                try ring.save(@intCast(step), &snapshot_bytes, &input_bytes, state_hash);
                const record = try ring.get(@intCast(step));
                try std.testing.expectEqualSlices(u8, &snapshot_bytes, record.snapshot);
                try std.testing.expectEqualSlices(u8, &input_bytes, record.input);
                try std.testing.expectEqualSlices(u8, &state_hash, &record.state_hash);
            },
            else => unreachable,
        }
    }

    for (joint_values, 0..) |joint, i| if (joint_alive[i]) {
        try std.testing.expect(state.bodyIndex(joint.body_a) != null);
        try std.testing.expect(state.bodyIndex(joint.body_b) != null);
    };

    var digest: Digest = .{};
    state.visitCanonical(&digest);
    joints.visitCanonical(&pool, &digest);
    digest.writeU64(seed);
    try std.testing.expectEqual(@as(u64, 0xf01a119b0ae71319), digest.value);
}
