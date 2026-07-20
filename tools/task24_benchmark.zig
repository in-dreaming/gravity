//! Task 24 serial/Spindle product-corpus benchmark.
//!
//! Each sample restores the same canonical snapshot outside the Tick timer.
//! This freezes authored contacts/joints while excluding snapshot-load cost
//! from pure Tick percentiles. Snapshot and 8-Tick rollback are timed
//! separately. All storage is allocated before the allocator limit is frozen.
const std = @import("std");
const builtin = @import("builtin");
const gravity = @import("gravity");
const corpus = @import("task24_corpus.zig");
const replay_tool = @import("replay.zig");
const contract = @import("gravity_jobs");
const spindle_adapter = @import("gravity_spindle_jobs");
const spindle = @import("spindle_executor");

const fp = gravity.math.fp;
const geometry = gravity.math.geometry;
const shapes = gravity.collision.shapes;
const baked = gravity.geometry.baked;
const asset_store = gravity.assets.store;
const snapshot = gravity.state.snapshot;
const replay = gravity.state.replay;

const default_samples: usize = 64;
const default_warmup: usize = 8;
const mesh_source_id: u64 = 24_024;
const windows_high_priority_class: std.os.windows.DWORD = 0x00000080;
const reference_p_core_affinity: usize = 0x0000ffff;

extern "kernel32" fn SetPriorityClass(process: std.os.windows.HANDLE, priority_class: std.os.windows.DWORD) callconv(.winapi) std.os.windows.BOOL;
extern "kernel32" fn SetProcessAffinityMask(process: std.os.windows.HANDLE, affinity_mask: usize) callconv(.winapi) std.os.windows.BOOL;

const Enforcement = enum { none, fixed_runner, shared_ci };
const Options = struct { scene: corpus.Scene, samples: usize = default_samples, warmup: usize = default_warmup, workers: usize = 0, enforcement: Enforcement = .none };

const Profile = struct {
    current: ?gravity.dynamics.pipeline.Phase = null,
    started: i96 = 0,
    elapsed: [11]u64 = [_]u64{0} ** 11,
    detail_current: ?gravity.dynamics.pipeline.ProfileDetail = null,
    detail_started: i96 = 0,
    detail_elapsed: [6]u64 = [_]u64{0} ** 6,

    fn closeDetail(self: *Profile, now: i96) void {
        if (self.detail_current) |active_detail| self.detail_elapsed[@intFromEnum(active_detail)] += @intCast(now - self.detail_started);
        self.detail_current = null;
    }

    fn transition(raw: *anyopaque, next: ?gravity.dynamics.pipeline.Phase) void {
        const self: *Profile = @ptrCast(@alignCast(raw));
        const now = nowNs();
        self.closeDetail(now);
        if (self.current) |phase| self.elapsed[@intFromEnum(phase)] += @intCast(now - self.started);
        self.current = next;
        self.started = now;
    }

    fn detail(raw: *anyopaque, next: gravity.dynamics.pipeline.ProfileDetail) void {
        const self: *Profile = @ptrCast(@alignCast(raw));
        const now = nowNs();
        self.closeDetail(now);
        self.detail_current = next;
        self.detail_started = now;
    }
};

const SnapshotBuffers = struct {
    output: []u8,
    pipeline: []u8,
    bodies: []u8,
    colliders: []u8,
    contacts: []u8,
    joints: []u8,
    sleep: []u8,
    ccd: []u8,

    fn init(allocator: std.mem.Allocator, scene: corpus.Scene) !SnapshotBuffers {
        const body_bytes = 512 + @as(usize, scene.bodies) * 320;
        const collider_bytes = 32 + @as(usize, scene.colliders) * 192;
        const contact_bytes = 32 + @as(usize, scene.contact_patches) * 256;
        const joint_bytes = 32 + @as(usize, scene.joints) * 384;
        const sleep_bytes = 32 + @as(usize, scene.bodies) * 8;
        const ccd_bytes = 32 + @as(usize, scene.colliders) * 2;
        const total = 4_096 + body_bytes + collider_bytes + contact_bytes + joint_bytes + sleep_bytes + ccd_bytes;
        return .{
            .output = try allocator.alloc(u8, total),
            .pipeline = try allocator.alloc(u8, 256),
            .bodies = try allocator.alloc(u8, body_bytes),
            .colliders = try allocator.alloc(u8, collider_bytes),
            .contacts = try allocator.alloc(u8, contact_bytes),
            .joints = try allocator.alloc(u8, joint_bytes),
            .sleep = try allocator.alloc(u8, sleep_bytes),
            .ccd = try allocator.alloc(u8, ccd_bytes),
        };
    }

    fn encode(self: *SnapshotBuffers, host: *replay_tool.Host) ![]const u8 {
        return snapshot.encodeFullSnapshot(host.header, host.state, host.target, host.contacts, host.target_joints, host.sleep, host.ccd_enabled, self.output, self.pipeline, self.bodies, self.colliders, self.contacts, self.joints, self.sleep, self.ccd);
    }
};

const AssetBundle = struct {
    store: asset_store.Store,

    fn init(allocator: std.mem.Allocator, scene: corpus.Scene, status: *fp.MathStatus) !AssetBundle {
        if (scene.kind != .mesh_heavy) {
            const memory = try allocator.alignedAlloc(u8, .of(asset_store.Asset), 1);
            return .{ .store = try asset_store.Store.init(memory, &.{}) };
        }
        const side: usize = scene.mesh_cells_per_axis;
        const vertex_count = (side + 1) * (side + 1);
        const triangle_count = side * side * 2;
        const vertices = try allocator.alloc(geometry.Vec3, vertex_count);
        const triangles = try allocator.alloc(baked.Triangle, triangle_count);
        const nodes = try allocator.alloc(baked.BvhNode, triangle_count * 2 - 1);
        const primitives = try allocator.alloc(u32, triangle_count);
        for (0..side + 1) |z| for (0..side + 1) |x| {
            vertices[z * (side + 1) + x] = .{ .x = fp.Fp.fromInt(@intCast(x)), .z = fp.Fp.fromInt(@intCast(z)) };
        };
        var at: usize = 0;
        for (0..side) |z| for (0..side) |x| {
            const a: u32 = @intCast(z * (side + 1) + x);
            const b: u32 = a + 1;
            const c: u32 = @intCast((z + 1) * (side + 1) + x);
            const d: u32 = c + 1;
            triangles[at] = .{ .a = a, .b = b, .c = d };
            triangles[at + 1] = .{ .a = a, .b = d, .c = c };
            at += 2;
        };
        const built = try baked.buildTriangleBvh(vertices, triangles, nodes, primitives);
        const encoded_capacity = triangle_count * 160 + vertex_count * 32 + 4_096;
        const encoded_memory = try allocator.alloc(u8, encoded_capacity);
        const scratch = try allocator.alloc(u8, encoded_capacity);
        const encoded = try baked.encodeMesh(.{ .source_id = mesh_source_id, .vertices = vertices, .triangles = triangles, .nodes = built.nodes, .primitives = built.primitives }, encoded_memory, scratch);
        const required = try asset_store.Store.memoryRequired(&.{encoded.bytes});
        const store_memory = try allocator.alignedAlloc(u8, .of(asset_store.Asset), required);
        _ = status;
        return .{ .store = try asset_store.Store.init(store_memory, &.{encoded.bytes}) };
    }
};

fn configuration(scene: corpus.Scene) !gravity.core.config.SimulationConfig {
    var value = gravity.core.config.SimulationConfig.default;
    value.capacities.body = scene.bodies;
    value.capacities.collider = scene.colliders;
    value.capacities.joint = scene.joints;
    value.capacities.command_per_tick = 64;
    value.capacities.broad_pair = scene.broad_pairs;
    value.capacities.contact_patch = scene.contact_patches;
    value.capacities.contact_point = scene.contact_points;
    value.capacities.sensor_overlap = scene.contact_patches;
    value.capacities.event_per_tick = @max(scene.contact_patches * 2, 64);
    value.capacities.rollback_window = 120;
    try value.validate();
    return value;
}

fn identityInertia() geometry.SymmetricMat3 {
    return .{ .xx = .one, .yy = .one, .zz = .one, .xy = .zero, .xz = .zero, .yz = .zero };
}

fn addBody(host: *replay_tool.Host, body_type: shapes.BodyType, position: geometry.Vec3) !gravity.core.ids.BodyId {
    return host.target.create(.{ .body_type = body_type, .transform = .{ .position = position }, .inverse_mass = if (body_type == .dynamic) .one else .zero, .inverse_inertia_local = identityInertia() }, &host.status);
}

fn addSphere(host: *replay_tool.Host, body: gravity.core.ids.BodyId, radius: fp.Fp) !gravity.core.ids.ColliderId {
    return host.target.createCollider(.{ .body = body, .shape = .{ .sphere = .{ .radius = radius } } });
}

fn populateGrouped(host: *replay_tool.Host, scene: corpus.Scene) ![]gravity.core.ids.BodyId {
    const ids = try host.allocator.alloc(gravity.core.ids.BodyId, scene.bodies);
    const group_size: usize = scene.group_size;
    for (ids, 0..) |*id, index| {
        const group = index / group_size;
        const within = index % group_size;
        const base = fp.Fp.fromInt(@intCast(group * 8));
        const offset = fp.Fp.fromRatio(@intCast(within), 4, &host.status);
        id.* = try addBody(host, .dynamic, .{ .x = base.add(offset, &host.status) });
        _ = try addSphere(host, id.*, .one);
    }
    return ids;
}

fn populateJointHeavy(host: *replay_tool.Host, scene: corpus.Scene) ![]gravity.core.ids.BodyId {
    const ids = try host.allocator.alloc(gravity.core.ids.BodyId, scene.bodies);
    for (ids, 0..) |*id, index| {
        id.* = try addBody(host, .dynamic, .{ .x = fp.Fp.fromInt(@intCast(index * 4)) });
        _ = try addSphere(host, id.*, .one);
    }
    return ids;
}

fn populateCcd(host: *replay_tool.Host, scene: corpus.Scene) ![]gravity.core.ids.BodyId {
    const ids = try host.allocator.alloc(gravity.core.ids.BodyId, scene.bodies);
    const pairs: usize = scene.ccd_colliders;
    for (0..pairs) |index| {
        const position = geometry.Vec3{ .x = fp.Fp.fromInt(@intCast(index * 4)) };
        ids[index * 2] = try addBody(host, .static, position);
        _ = try addSphere(host, ids[index * 2], .one);
        ids[index * 2 + 1] = try addBody(host, .dynamic, .{ .x = position.x, .y = .one });
        const collider = try addSphere(host, ids[index * 2 + 1], .one);
        host.ccd_enabled[collider.index()] = true;
        host.target.storage.linear_velocity[ids[index * 2 + 1].index()] = .{ .y = fp.Fp.fromInt(-60) };
    }
    return ids;
}

fn populateMesh(host: *replay_tool.Host, scene: corpus.Scene) ![]gravity.core.ids.BodyId {
    const ids = try host.allocator.alloc(gravity.core.ids.BodyId, scene.bodies);
    ids[0] = try addBody(host, .static, .{});
    _ = try host.target.createCollider(.{ .body = ids[0], .shape = .{ .triangle_mesh = .{ .source_id = mesh_source_id } } });
    const side: usize = scene.mesh_cells_per_axis;
    for (1..scene.bodies) |raw| {
        const index: usize = @intCast(raw - 1);
        const x = index % side;
        const z = index / side;
        ids[raw] = try addBody(host, .dynamic, .{ .x = fp.Fp.fromRatio(@intCast(x * 2 + 1), 2, &host.status), .y = fp.Fp.fromRatio(1, 2, &host.status), .z = fp.Fp.fromRatio(@intCast(z * 2 + 1), 2, &host.status) });
        _ = try addSphere(host, ids[raw], fp.Fp.fromRatio(3, 4, &host.status));
    }
    return ids;
}

fn populateJoints(host: *replay_tool.Host, ids: []const gravity.core.ids.BodyId, count: u32, group_size: u8, mixed: bool) !void {
    const kinds = [_]gravity.dynamics.joints.Kind{ .distance, .ball_socket, .hinge, .slider, .fixed, .cone_twist };
    for (0..count) |raw| {
        const index: usize = @intCast(raw);
        const a_index = if (mixed) index % ids.len else blk: {
            const groups = ids.len / group_size;
            const group = index % groups;
            const ordinal = (index / groups) % group_size;
            break :blk group * group_size + ordinal;
        };
        const b_index = if (mixed) (index * 17 + 1) % ids.len else (a_index / group_size) * group_size + (a_index + 1) % group_size;
        const a = ids[a_index];
        var b = ids[b_index];
        if (a.value == b.value) b = ids[(index + 1) % ids.len];
        const kind = if (mixed) kinds[index % kinds.len] else .distance;
        _ = try host.target_joints.create(host.target, .{ .kind = kind, .body_a = a, .body_b = b, .reference = if (kind == .distance) .one else null, .motor = .{ .enabled = kind == .hinge or kind == .slider, .target_velocity = .one, .max_force = fp.Fp.fromInt(10) }, .spring = .{ .enabled = kind == .distance, .frequency = fp.Fp.fromInt(2), .damping_ratio = fp.Fp.fromRatio(1, 2, &host.status) } }, &host.status);
    }
}

fn populate(host: *replay_tool.Host, scene: corpus.Scene) !void {
    const ids = switch (scene.kind) {
        .small, .medium, .stress => try populateGrouped(host, scene),
        .joint_heavy => try populateJointHeavy(host, scene),
        .ccd => try populateCcd(host, scene),
        .mesh_heavy => try populateMesh(host, scene),
    };
    if (scene.kind == .mesh_heavy or scene.kind == .ccd) return;
    try populateJoints(host, ids, scene.joints, scene.group_size, scene.kind == .joint_heavy);
}

fn nowNs() i96 {
    return std.Io.Clock.Timestamp.now(std.Options.debug_io, .awake).raw.nanoseconds;
}

fn percentile(samples: []u64, numerator: usize) u64 {
    std.sort.heap(u64, samples, {}, std.sort.asc(u64));
    const index = @min(samples.len - 1, (samples.len * numerator + 99) / 100 - 1);
    return samples[index];
}

fn measure(options: Options) !void {
    var gpa = std.heap.DebugAllocator(.{ .enable_memory_limit = true }){};
    const allocator = gpa.allocator();
    var status: fp.MathStatus = .{};
    const assets = try allocator.create(AssetBundle);
    assets.* = try AssetBundle.init(allocator, options.scene, &status);
    const config = try configuration(options.scene);
    const header = snapshot.Header{ .configuration = config, .asset_set = assets.store.asset_set_hash };
    const host = try replay_tool.Host.init(allocator, header, &assets.store);
    try populate(host, options.scene);

    var buffers = try SnapshotBuffers.init(allocator, options.scene);
    const initial = try buffers.encode(host);
    const initial_copy = try allocator.dupe(u8, initial);
    var input_storage: [4]u8 = undefined;
    const input = try replay.encodeCommands(&.{}, &input_storage);
    const tick_samples = try allocator.alloc(u64, options.samples);
    const snapshot_samples = try allocator.alloc(u64, options.samples);
    const rollback_samples = try allocator.alloc(u64, options.samples);
    const expected_hashes = try allocator.alloc([16]u8, 8);

    const first = replay.Entry{ .tick = 1, .input = input, .expected_hash = [_]u8{0} ** 16 };
    _ = replay.FullWorldHost.load(&host.full, initial_copy) catch return error.SnapshotLoadFailed;
    const expected = replay.FullWorldHost.step(&host.full, first) catch |err| {
        std.debug.print("scene {s} first Tick failed: {s}, fault={any}, broad_pairs={d}, contacts={d}\n", .{ options.scene.name, @errorName(err), host.state.fault, host.solver_pipeline.contacts.broadphase.buffers.pair_count, host.contacts.len });
        return err;
    };
    if (host.contacts.len < options.scene.expected_contacts) return error.SceneBecameCheaper;
    for (0..8) |index| {
        if (index != 0) {
            const entry = replay.Entry{ .tick = index + 1, .input = input, .expected_hash = [_]u8{0} ** 16 };
            expected_hashes[index] = try replay.FullWorldHost.step(&host.full, entry);
        } else expected_hashes[index] = expected;
    }

    var executor_storage: ?spindle.executor.WorkStealingExecutor = null;
    var adapter: spindle_adapter.Dispatcher = undefined;
    var custom: contract.Custom = undefined;
    if (options.workers != 0) {
        executor_storage = try spindle.executor.WorkStealingExecutor.init(allocator, .{ .workers = options.workers, .local_capacity = 64, .injection_capacity = 128, .urgent_capacity = 16 });
        const slots = try allocator.alloc(spindle_adapter.Dispatcher.Slot, contract.maximum_batch_jobs);
        adapter = spindle_adapter.Dispatcher.init(&executor_storage.?, slots, contract.maximum_batch_jobs);
        custom = adapter.custom();
        host.workspace.dispatcher = .{ .custom = &custom };
    }
    defer if (executor_storage) |*executor| executor.deinit();

    // Prove that Tick, snapshot, rollback, and Spindle dispatch cannot allocate
    // from the benchmark-owned allocator after setup.
    const setup_bytes = gpa.total_requested_bytes;
    gpa.requested_memory_limit = setup_bytes;

    for (0..options.warmup) |_| {
        _ = try replay.FullWorldHost.load(&host.full, initial_copy);
        const actual = try replay.FullWorldHost.step(&host.full, first);
        if (!std.mem.eql(u8, &actual, &expected)) return error.HashMismatch;
    }
    for (tick_samples) |*sample| {
        _ = try replay.FullWorldHost.load(&host.full, initial_copy);
        const started = nowNs();
        const actual = try replay.FullWorldHost.step(&host.full, first);
        sample.* = @intCast(nowNs() - started);
        if (!std.mem.eql(u8, &actual, &expected)) return error.HashMismatch;
    }
    for (snapshot_samples) |*sample| {
        _ = try replay.FullWorldHost.load(&host.full, initial_copy);
        const started = nowNs();
        _ = try buffers.encode(host);
        sample.* = @intCast(nowNs() - started);
    }
    for (rollback_samples) |*sample| {
        const started = nowNs();
        _ = try replay.FullWorldHost.load(&host.full, initial_copy);
        for (0..8) |index| {
            const entry = replay.Entry{ .tick = index + 1, .input = input, .expected_hash = expected_hashes[index] };
            const actual = try replay.FullWorldHost.step(&host.full, entry);
            if (!std.mem.eql(u8, &actual, &expected_hashes[index])) return error.HashMismatch;
        }
        sample.* = @intCast(nowNs() - started);
    }

    const tick_p50 = percentile(tick_samples, 50);
    const tick_p95 = percentile(tick_samples, 95);
    const tick_p99 = percentile(tick_samples, 99);
    const snapshot_p95 = percentile(snapshot_samples, 95);
    const rollback_p95 = percentile(rollback_samples, 95);
    var profile: Profile = .{};
    const observer = gravity.dynamics.pipeline.PhaseObserver{ .context = &profile, .transition_fn = Profile.transition, .detail_fn = Profile.detail };
    _ = try replay.FullWorldHost.load(&host.full, initial_copy);
    host.workspace.observer = &observer;
    const profile_hash = try replay.FullWorldHost.step(&host.full, first);
    host.workspace.observer = null;
    if (!std.mem.eql(u8, &profile_hash, &expected)) return error.HashMismatch;
    const budget_pass = tick_p95 <= options.scene.budget.native_p95_ns and tick_p99 <= options.scene.budget.native_p99_ns and snapshot_p95 <= options.scene.budget.snapshot_p95_ns and rollback_p95 <= options.scene.budget.rollback_8_p95_ns and setup_bytes <= options.scene.budget.max_workspace_bytes;
    std.debug.print("{{\"schema\":\"gravity.performance.v1\",\"scene\":\"{s}\",\"backend\":\"{s}\",\"workers\":{d},\"samples\":{d},\"bodies\":{d},\"colliders\":{d},\"joints\":{d},\"contacts\":{d},\"tick_p50_ns\":{d},\"tick_p95_ns\":{d},\"tick_p99_ns\":{d},\"snapshot_bytes\":{d},\"snapshot_p95_ns\":{d},\"rollback_8_p95_ns\":{d},\"workspace_bytes\":{d},\"tick_allocations\":0,\"budget_pass\":{s}}}\n", .{ options.scene.name, if (options.workers == 0) "serial" else "spindle", options.workers, options.samples, options.scene.bodies, options.scene.colliders, options.scene.joints, host.contacts.len, tick_p50, tick_p95, tick_p99, initial_copy.len, snapshot_p95, rollback_p95, setup_bytes, if (budget_pass) "true" else "false" });
    inline for (@typeInfo(gravity.dynamics.pipeline.Phase).@"enum".fields) |field| {
        const phase: gravity.dynamics.pipeline.Phase = @enumFromInt(field.value);
        std.debug.print("{{\"schema\":\"gravity.profile.v1\",\"scene\":\"{s}\",\"phase\":\"{s}\",\"elapsed_ns\":{d}}}\n", .{ options.scene.name, field.name, profile.elapsed[@intFromEnum(phase)] });
    }
    inline for (@typeInfo(gravity.dynamics.pipeline.ProfileDetail).@"enum".fields) |field| {
        const detail: gravity.dynamics.pipeline.ProfileDetail = @enumFromInt(field.value);
        std.debug.print("{{\"schema\":\"gravity.profile-detail.v1\",\"scene\":\"{s}\",\"detail\":\"{s}\",\"elapsed_ns\":{d}}}\n", .{ options.scene.name, field.name, profile.detail_elapsed[@intFromEnum(detail)] });
    }
    if (options.enforcement == .fixed_runner and !budget_pass) return error.ProductBudgetExceeded;
    if (options.enforcement == .shared_ci) {
        const noise_multiplier = 2;
        const significant_regression = tick_p95 > options.scene.budget.native_p95_ns * noise_multiplier or tick_p99 > options.scene.budget.native_p99_ns * noise_multiplier or snapshot_p95 > options.scene.budget.snapshot_p95_ns * noise_multiplier or rollback_p95 > options.scene.budget.rollback_8_p95_ns * noise_multiplier or setup_bytes > options.scene.budget.max_workspace_bytes;
        if (significant_regression) return error.SignificantPerformanceRegression;
    }
}

fn parseOptions(init: std.process.Init) !Options {
    var args = try init.minimal.args.iterateAllocator(init.gpa);
    defer args.deinit();
    _ = args.next();
    const scene_name = args.next() orelse "Small";
    const scene = corpus.byName(scene_name) orelse return error.UnknownScene;
    var result = Options{ .scene = scene };
    if (args.next()) |samples| result.samples = try std.fmt.parseInt(usize, samples, 10);
    if (args.next()) |warmup| result.warmup = try std.fmt.parseInt(usize, warmup, 10);
    if (args.next()) |workers| result.workers = try std.fmt.parseInt(usize, workers, 10);
    if (args.next()) |mode| {
        if (std.mem.eql(u8, mode, "gate")) result.enforcement = .fixed_runner else if (std.mem.eql(u8, mode, "ci")) result.enforcement = .shared_ci else return error.InvalidArguments;
    }
    if (args.next() != null or result.samples == 0) return error.InvalidArguments;
    return result;
}

pub fn main(init: std.process.Init) !void {
    try corpus.validate();
    const options = try parseOptions(init);
    if (options.enforcement == .fixed_runner and builtin.os.tag == .windows) {
        const process = std.os.windows.GetCurrentProcess();
        if (!SetPriorityClass(process, windows_high_priority_class).toBool()) return error.PriorityConfigurationFailed;
        if (!SetProcessAffinityMask(process, reference_p_core_affinity).toBool()) return error.AffinityConfigurationFailed;
    }
    try measure(options);
}

test "Task 24 corpus remains valid in the benchmark compilation unit" {
    try corpus.validate();
}
