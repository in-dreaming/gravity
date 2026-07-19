//! Stable `gravity_v1_*` C ABI. All persistent storage is carved from memory
//! supplied by the caller; pointers borrowed by an ABI call never escape it.
const std = @import("std");
const version = @import("../version.zig");
const fp = @import("../math/fp.zig");
const g = @import("../math/geometry.zig");
const ids = @import("../core/ids.zig");
const config = @import("../core/config.zig");
const shapes = @import("../collision/shapes.zig");
const contacts = @import("../collision/contact_cache.zig");
const broadphase = @import("../collision/broadphase.zig");
const gjk = @import("../collision/gjk.zig");
const mesh = @import("../collision/mesh.zig");
const baked = @import("../geometry/baked.zig");
const world_mod = @import("../dynamics/world.zig");
const pipeline = @import("../dynamics/pipeline.zig");
const solver = @import("../dynamics/contact_solver.zig");
const constraints = @import("../dynamics/constraints.zig");
const queries = @import("../query/queries.zig");
const snapshot = @import("../state/snapshot.zig");
const store_mod = @import("../assets/store.zig");
const build_options = @import("build_options");
const jobs = @import("gravity_jobs");
const host_jobs = @import("gravity_host_jobs");

pub const ok: u32 = 0;
pub const invalid_argument: u32 = 1;
pub const bad_struct: u32 = 2;
pub const misaligned: u32 = 3;
pub const insufficient_memory: u32 = 4;
pub const capacity: u32 = 5;
pub const invalid_id: u32 = 6;
pub const invalid_state: u32 = 7;
pub const corrupt_input: u32 = 8;
pub const callback_error: u32 = 9;
pub const reentrant: u32 = 10;
pub const buffer_too_small: u32 = 11;
pub const unsupported: u32 = 12;
pub const internal: u32 = 13;

pub const Vec3 = extern struct { x: i64, y: i64, z: i64 };
pub const Quat = extern struct { x: i64, y: i64, z: i64, w: i64 };
pub const Transform = extern struct { position: Vec3, orientation: Quat };
pub const Hash128 = extern struct { bytes: [16]u8 };
pub const Hash256 = extern struct { bytes: [32]u8 };
pub const BuildInfo = extern struct {
    struct_size: u32,
    reserved: u32,
    abi_version: u32,
    protocol_version: u32,
    snapshot_format_version: u32,
    asset_format_version: u32,
    commit: [*c]const u8,
    commit_length: u32,
    zig_version: [*c]const u8,
    zig_version_length: u32,
};
pub const AssetBlob = extern struct { data: [*c]const u8, length: u64 };
pub const AssetStoreDesc = extern struct { struct_size: u32, reserved: u32, assets: [*c]const AssetBlob, asset_count: u32, reserved1: u32 };
pub const WorldDesc = extern struct {
    struct_size: u32,
    reserved: u32,
    body_capacity: u32,
    collider_capacity: u32,
    command_capacity: u32,
    contact_capacity: u32,
    gravity: Vec3,
    linear_damping: i64,
    angular_damping: i64,
    max_linear_speed: i64,
    max_angular_speed: i64,
    substeps: u32,
    tick_hz: u32,
    assets: ?*const AssetStore,
};
pub const BodyDesc = extern struct {
    struct_size: u32,
    reserved: u32,
    body_type: u32,
    dof_locks: u32,
    transform: Transform,
    inverse_mass: i64,
    inverse_inertia_xx: i64,
    inverse_inertia_yy: i64,
    inverse_inertia_zz: i64,
    inverse_inertia_xy: i64,
    inverse_inertia_xz: i64,
    inverse_inertia_yz: i64,
};
pub const BodyState = extern struct {
    struct_size: u32,
    reserved: u32,
    id: u64,
    body_type: u32,
    dof_locks: u32,
    transform: Transform,
    linear_velocity: Vec3,
    angular_velocity: Vec3,
};
pub const ColliderDesc = extern struct {
    struct_size: u32,
    reserved: u32,
    body: u64,
    shape_kind: u32,
    flags: u32,
    local: Transform,
    dimensions: Vec3,
    asset_source_id: u64,
    friction: i64,
    restitution: i64,
    category: u32,
    mask: u32,
    group: i32,
    revision: u32,
};
pub const Command = extern struct {
    struct_size: u32,
    reserved: u32,
    type: u32,
    phase_priority: u32,
    issuer: u32,
    sequence: u32,
    body: u64,
    first: Vec3,
    second: Vec3,
    transform: Transform,
    dof_locks: u32,
    reserved1: u32,
};
pub const Event = extern struct { struct_size: u32, reserved: u32, type: u32, reserved1: u32, collider_a: u64, collider_b: u64, feature_a: u64, feature_b: u64 };
pub const Filter = extern struct { category: u32, mask: u32, group: i32, reserved: u32 };
pub const RayQuery = extern struct { struct_size: u32, reserved: u32, origin: Vec3, direction: Vec3, max_fraction: i64, filter: Filter, mode: u32, reserved1: u32 };
pub const PointQuery = extern struct { struct_size: u32, reserved: u32, point: Vec3, filter: Filter, mode: u32, reserved1: u32 };
pub const AabbQuery = extern struct { struct_size: u32, reserved: u32, min: Vec3, max: Vec3, filter: Filter, mode: u32, reserved1: u32 };
pub const ShapeQuery = extern struct { struct_size: u32, reserved: u32, shape: ColliderDesc, transform: Transform, filter: Filter, mode: u32, reserved1: u32 };
pub const QueryHit = extern struct { struct_size: u32, reserved: u32, collider: u64, fraction: i64, point: Vec3, normal: Vec3, primitive: u32, reserved1: u32 };

pub const RunJobFn = *const fn (?*anyopaque, u32) callconv(.c) u32;
pub const DispatchBatchFn = *const fn (?*anyopaque, u32, RunJobFn, ?*anyopaque) callconv(.c) u32;
pub const Dispatcher = extern struct { struct_size: u32, reserved: u32, user: ?*anyopaque, dispatch_batch: ?DispatchBatchFn };

pub const AssetStore = struct {
    magic: u64,
    active: u32,
    reserved: u32,
    value: store_mod.Store,
};

const asset_magic: u64 = 0x4752_4156_4153_5431;
const world_magic: u64 = 0x4752_4156_574F_5231;

pub const World = struct {
    magic: u64,
    active: u32,
    in_call: u32,
    callback_violation: u32,
    last_error: u32,
    reserved: u32,
    assets: *const AssetStore,
    simulation: config.SimulationConfig,
    state: pipeline.State,
    value: world_mod.World,
    stage: world_mod.World,
    commands: []world_mod.Command,
    host_dispatch_seen: []u8,
    trace: []pipeline.Phase,
    cache: contacts.Cache,
    broadphase_buffers: broadphase.Buffers,
    broadphase_workspace: pipeline.BroadphaseWorkspace,
    contact_workspace: pipeline.AnalyticContactWorkspace,
    convex_workspace: pipeline.ConvexNarrowWorkspace,
    surface_workspace: pipeline.SurfaceNarrowWorkspace,
    solver_workspace: pipeline.AnalyticSolverWorkspace,
    island_workspace: pipeline.IslandWorkspace,
    solver_pipeline: pipeline.AnalyticSolverPipelineWorkspace,
    event_count: usize,
    stage_contacts: []contacts.Patch,
    contact_scratch: []contacts.Patch,
    query_colliders: []shapes.Collider,
    query_items: []queries.Item,
    query_candidates: []queries.Hit,
    query_output: []queries.Hit,
    query_flags: []u8,
    query_u32_a: []u32,
    query_u32_b: []u32,
    query_mesh_hits: []queries.MeshRayHit,
    query_height_triangles: []mesh.HeightTriangle,
    query_leaves: []shapes.CompoundLeaf,
    query_nodes_a: []baked.BvhNode,
    query_nodes_b: []baked.BvhNode,
    query_node_pairs: []mesh.NodePair,
    query_pair_a: []mesh.PrimitivePair,
    query_pair_b: []mesh.PrimitivePair,
    query_pair_c: []mesh.PrimitivePair,
    pipeline_payload: []u8,
    bodies_payload: []u8,
    colliders_payload: []u8,
    contacts_payload: []u8,
    snapshot_output: []u8,
    dispatcher: Dispatcher,
};

comptime {
    if (@sizeOf(Vec3) != 24 or @sizeOf(Quat) != 32 or @sizeOf(Hash128) != 16) @compileError("C ABI scalar layout drift");
    if (@offsetOf(BodyState, "id") != 8 or @offsetOf(Command, "body") != 24) @compileError("C ABI field layout drift");
}

fn validStruct(value: anytype) bool {
    const T = @TypeOf(value.*);
    return value.struct_size >= @sizeOf(T) and value.reserved == 0;
}

fn mapError(err: anyerror) u32 {
    return switch (err) {
        error.InsufficientMemory, error.OutOfMemory => insufficient_memory,
        error.MisalignedMemory => misaligned,
        error.CapacityExceeded, error.OutOfSpace => capacity,
        error.InvalidBody, error.InvalidCollider => invalid_id,
        error.InvalidShape, error.InvalidBodyShape, error.InvalidMass, error.InvalidCommand, error.InvalidQuery, error.InvalidConfig, error.InvalidCapacity, error.InvalidIteration, error.InvalidTolerance, error.InvalidEnvelope, error.InvalidFeatureFlags => invalid_argument,
        error.UnsupportedShape => unsupported,
        error.Reentrant => reentrant,
        error.Backpressure, error.Cancelled, error.WorkerFault, error.CallbackFailed, error.Shutdown => callback_error,
        error.Faulted => invalid_state,
        error.InvalidMagic, error.InvalidProtocol, error.HeaderMismatch, error.InvalidData, error.InvalidSection, error.UnknownRequiredSection => corrupt_input,
        else => internal,
    };
}

fn bytesFrom(ptr: [*c]const u8, len: u64) ?[]const u8 {
    if (len > std.math.maxInt(usize) or (len != 0 and ptr == null)) return null;
    return ptr[0..@intCast(len)];
}

fn vec(value: Vec3) g.Vec3 {
    return .{ .x = .{ .raw = value.x }, .y = .{ .raw = value.y }, .z = .{ .raw = value.z } };
}
fn abiVec(value: g.Vec3) Vec3 {
    return .{ .x = value.x.raw, .y = value.y.raw, .z = value.z.raw };
}
fn quat(value: Quat) g.Quat {
    return .{ .x = .{ .raw = value.x }, .y = .{ .raw = value.y }, .z = .{ .raw = value.z }, .w = .{ .raw = value.w } };
}
fn abiQuat(value: g.Quat) Quat {
    return .{ .x = value.x.raw, .y = value.y.raw, .z = value.z.raw, .w = value.w.raw };
}
fn transform(value: Transform) g.Transform3 {
    return .{ .position = vec(value.position), .orientation = quat(value.orientation) };
}
fn abiTransform(value: g.Transform3) Transform {
    return .{ .position = abiVec(value.position), .orientation = abiQuat(value.orientation) };
}
fn compose(parent: g.Transform3, local: g.Transform3, status: *fp.MathStatus) g.Transform3 {
    return .{ .position = parent.apply(local.position, status), .orientation = parent.orientation.mul(local.orientation, status).canonicalize(status) };
}
fn locks(value: u32) ?world_mod.DofLock {
    if ((value & ~@as(u32, 0x3f)) != 0) return null;
    return @bitCast(@as(u8, @intCast(value)));
}

fn estimateWorldBytes(desc: *const WorldDesc) ?usize {
    if (desc.body_capacity == 0 or desc.collider_capacity == 0 or desc.command_capacity == 0 or desc.contact_capacity == 0) return null;
    var total: usize = 256 * 1024;
    total = std.math.add(usize, total, std.math.mul(usize, desc.body_capacity, 4096) catch return null) catch return null;
    total = std.math.add(usize, total, std.math.mul(usize, desc.collider_capacity, 4096) catch return null) catch return null;
    total = std.math.add(usize, total, std.math.mul(usize, desc.command_capacity, 512) catch return null) catch return null;
    total = std.math.add(usize, total, desc.command_capacity) catch return null;
    total = std.math.add(usize, total, std.math.mul(usize, desc.contact_capacity, 8192) catch return null) catch return null;
    return total;
}

fn allocSlice(allocator: std.mem.Allocator, comptime T: type, count: usize) ![]T {
    return allocator.alloc(T, count);
}

fn allocStorage(allocator: std.mem.Allocator, count: usize) !world_mod.Storage {
    return .{
        .body_type = try allocSlice(allocator, shapes.BodyType, count),
        .position = try allocSlice(allocator, g.Vec3, count),
        .orientation = try allocSlice(allocator, g.Quat, count),
        .linear_velocity = try allocSlice(allocator, g.Vec3, count),
        .angular_velocity = try allocSlice(allocator, g.Vec3, count),
        .inverse_mass = try allocSlice(allocator, fp.Fp, count),
        .inverse_inertia_local = try allocSlice(allocator, g.SymmetricMat3, count),
        .force = try allocSlice(allocator, g.Vec3, count),
        .torque = try allocSlice(allocator, g.Vec3, count),
        .locks = try allocSlice(allocator, world_mod.DofLock, count),
        .generation = try allocSlice(allocator, u32, count),
        .alive = try allocSlice(allocator, bool, count),
        .retired = try allocSlice(allocator, bool, count),
        .has_target = try allocSlice(allocator, bool, count),
        .target_position = try allocSlice(allocator, g.Vec3, count),
        .target_orientation = try allocSlice(allocator, g.Quat, count),
    };
}

fn allocColliderStorage(allocator: std.mem.Allocator, count: usize) !world_mod.ColliderStorage {
    return .{
        .body = try allocSlice(allocator, ids.BodyId, count),
        .local = try allocSlice(allocator, g.Transform3, count),
        .shape = try allocSlice(allocator, shapes.Shape, count),
        .material = try allocSlice(allocator, shapes.Material, count),
        .category = try allocSlice(allocator, u32, count),
        .mask = try allocSlice(allocator, u32, count),
        .group = try allocSlice(allocator, i32, count),
        .sensor = try allocSlice(allocator, bool, count),
        .enabled = try allocSlice(allocator, bool, count),
        .revision = try allocSlice(allocator, u32, count),
        .generation = try allocSlice(allocator, u32, count),
        .alive = try allocSlice(allocator, bool, count),
        .retired = try allocSlice(allocator, bool, count),
    };
}

fn enter(world: *World) u32 {
    if (world.magic != world_magic or world.active == 0) return invalid_state;
    if (world.in_call != 0) return reentrant;
    world.in_call = 1;
    world.callback_violation = 0;
    return ok;
}
fn leave(world: *World, result: u32) u32 {
    world.last_error = result;
    world.in_call = 0;
    return result;
}
fn checked(world: ?*World) ?*World {
    const value = world orelse return null;
    if (value.magic != world_magic or value.active == 0) return null;
    return value;
}
fn checkedConst(world: ?*const World) ?*const World {
    const value = world orelse return null;
    if (value.magic != world_magic or value.active == 0) return null;
    return value;
}
fn rejectCallbackReentry(world: *const World) bool {
    if (world.in_call == 0) return false;
    @constCast(world).callback_violation = 1;
    return true;
}

pub export fn gravity_v1_abi_version() callconv(.c) u32 {
    return version.abi_version;
}

pub export fn gravity_v1_build_info(out_info: ?*BuildInfo) callconv(.c) u32 {
    const output = out_info orelse return invalid_argument;
    if (!validStruct(output)) return bad_struct;
    output.* = .{
        .struct_size = @sizeOf(BuildInfo),
        .reserved = 0,
        .abi_version = version.abi_version,
        .protocol_version = version.protocol_version,
        .snapshot_format_version = version.snapshot_format_version,
        .asset_format_version = version.asset_format_version,
        .commit = build_options.commit.ptr,
        .commit_length = @intCast(build_options.commit.len),
        .zig_version = build_options.zig_version.ptr,
        .zig_version_length = @intCast(build_options.zig_version.len),
    };
    return ok;
}

pub export fn gravity_v1_result_string(result: u32) callconv(.c) [*:0]const u8 {
    return switch (result) {
        ok => "ok",
        invalid_argument => "invalid argument",
        bad_struct => "bad struct",
        misaligned => "misaligned memory",
        insufficient_memory => "insufficient memory",
        capacity => "capacity exceeded",
        invalid_id => "invalid id",
        invalid_state => "invalid state",
        corrupt_input => "corrupt input",
        callback_error => "callback contract violation",
        reentrant => "reentrant call",
        buffer_too_small => "buffer too small",
        unsupported => "unsupported",
        else => "internal error",
    };
}

pub export fn gravity_v1_asset_store_memory_required(desc_ptr: ?*const AssetStoreDesc, out_size: ?*u64, out_alignment: ?*u32) callconv(.c) u32 {
    const desc = desc_ptr orelse return invalid_argument;
    const size = out_size orelse return invalid_argument;
    const alignment = out_alignment orelse return invalid_argument;
    if (!validStruct(desc) or desc.reserved1 != 0 or (desc.asset_count != 0 and desc.assets == null)) return bad_struct;
    var total: usize = std.math.add(usize, @sizeOf(AssetStore) + 64, std.math.mul(usize, desc.asset_count, @sizeOf([]const u8)) catch return capacity) catch return capacity;
    total = std.math.add(usize, total, std.math.mul(usize, desc.asset_count, @sizeOf(store_mod.Asset)) catch return capacity) catch return capacity;
    var i: usize = 0;
    while (i < desc.asset_count) : (i += 1) {
        const blob = desc.assets[i];
        if (bytesFrom(blob.data, blob.length) == null) return invalid_argument;
        total = std.math.add(usize, total, @intCast(blob.length)) catch return capacity;
    }
    size.* = total;
    alignment.* = @alignOf(AssetStore);
    return ok;
}

pub export fn gravity_v1_asset_store_init(memory: ?*anyopaque, memory_size: u64, desc_ptr: ?*const AssetStoreDesc, out_store: ?**AssetStore) callconv(.c) u32 {
    const raw = memory orelse return invalid_argument;
    const desc = desc_ptr orelse return invalid_argument;
    const output = out_store orelse return invalid_argument;
    var required: u64 = 0;
    var alignment: u32 = 0;
    const check = gravity_v1_asset_store_memory_required(desc, &required, &alignment);
    if (check != ok) return check;
    if (memory_size < required or memory_size > std.math.maxInt(usize)) return insufficient_memory;
    if (@intFromPtr(raw) % alignment != 0) return misaligned;
    const bytes: []u8 = @as([*]u8, @ptrCast(raw))[0..@intCast(memory_size)];
    const result: *AssetStore = @ptrCast(@alignCast(bytes.ptr));
    var fba = std.heap.FixedBufferAllocator.init(bytes[@sizeOf(AssetStore)..]);
    const allocator = fba.allocator();
    const inputs = allocator.alloc([]const u8, desc.asset_count) catch return insufficient_memory;
    for (inputs, 0..) |*slot, i| slot.* = bytesFrom(desc.assets[i].data, desc.assets[i].length) orelse return invalid_argument;
    const needed = store_mod.Store.memoryRequired(inputs) catch |err| return mapError(err);
    const store_memory = allocator.alignedAlloc(u8, .fromByteUnits(@alignOf(store_mod.Asset)), needed) catch return insufficient_memory;
    const value = store_mod.Store.init(store_memory, inputs) catch |err| return mapError(err);
    result.* = .{ .magic = asset_magic, .active = 1, .reserved = 0, .value = value };
    output.* = result;
    return ok;
}

pub export fn gravity_v1_asset_store_deinit(store: ?*AssetStore) callconv(.c) u32 {
    const value = store orelse return invalid_argument;
    if (value.magic != asset_magic or value.active == 0) return invalid_state;
    value.active = 0;
    return ok;
}
pub export fn gravity_v1_asset_store_hash(store: ?*const AssetStore, out_hash: ?*Hash256) callconv(.c) u32 {
    const value = store orelse return invalid_argument;
    const output = out_hash orelse return invalid_argument;
    if (value.magic != asset_magic or value.active == 0) return invalid_state;
    output.bytes = value.value.asset_set_hash;
    return ok;
}

pub export fn gravity_v1_world_memory_required(desc_ptr: ?*const WorldDesc, out_size: ?*u64, out_alignment: ?*u32) callconv(.c) u32 {
    const desc = desc_ptr orelse return invalid_argument;
    const output = out_size orelse return invalid_argument;
    const alignment = out_alignment orelse return invalid_argument;
    if (!validStruct(desc) or desc.assets == null) return bad_struct;
    const total = estimateWorldBytes(desc) orelse return invalid_argument;
    output.* = total;
    alignment.* = @alignOf(World);
    return ok;
}

pub export fn gravity_v1_world_init(memory: ?*anyopaque, memory_size: u64, desc_ptr: ?*const WorldDesc, out_world: ?**World) callconv(.c) u32 {
    const raw = memory orelse return invalid_argument;
    const desc = desc_ptr orelse return invalid_argument;
    const output = out_world orelse return invalid_argument;
    var required: u64 = 0;
    var alignment: u32 = 0;
    const check = gravity_v1_world_memory_required(desc, &required, &alignment);
    if (check != ok) return check;
    if (memory_size < required or memory_size > std.math.maxInt(usize)) return insufficient_memory;
    if (@intFromPtr(raw) % alignment != 0) return misaligned;
    const asset_store = desc.assets orelse return invalid_argument;
    if (asset_store.magic != asset_magic or asset_store.active == 0) return invalid_state;
    if (desc.substeps == 0 or desc.tick_hz == 0) return invalid_argument;
    const bytes: []u8 = @as([*]u8, @ptrCast(raw))[0..@intCast(memory_size)];
    const result: *World = @ptrCast(@alignCast(bytes.ptr));
    var fba = std.heap.FixedBufferAllocator.init(bytes[@sizeOf(World)..]);
    const allocator = fba.allocator();
    const body_storage = allocStorage(allocator, desc.body_capacity) catch return insufficient_memory;
    const collider_storage = allocColliderStorage(allocator, desc.collider_capacity) catch return insufficient_memory;
    const stage_body = allocStorage(allocator, desc.body_capacity) catch return insufficient_memory;
    const stage_collider = allocColliderStorage(allocator, desc.collider_capacity) catch return insufficient_memory;
    var value = world_mod.World.initWithColliders(body_storage, collider_storage) catch |err| return mapError(err);
    const stage = world_mod.World.initWithColliders(stage_body, stage_collider) catch |err| return mapError(err);
    value.settings = .{ .gravity = vec(desc.gravity), .linear_damping = .{ .raw = desc.linear_damping }, .angular_damping = .{ .raw = desc.angular_damping }, .max_linear_speed = .{ .raw = desc.max_linear_speed }, .max_angular_speed = .{ .raw = desc.max_angular_speed } };
    var simulation = config.SimulationConfig.default;
    simulation.capacities.body = desc.body_capacity;
    simulation.capacities.collider = desc.collider_capacity;
    simulation.capacities.command_per_tick = desc.command_capacity;
    simulation.capacities.contact_patch = desc.contact_capacity;
    simulation.capacities.contact_point = @max(desc.contact_capacity, simulation.capacities.contact_point);
    simulation.iterations.substeps = desc.substeps;
    simulation.iterations.tick_hz = desc.tick_hz;
    const command_buffer = allocSlice(allocator, world_mod.Command, desc.command_capacity) catch return insufficient_memory;
    const host_dispatch_seen = allocSlice(allocator, u8, jobs.maximum_batch_jobs) catch return insufficient_memory;
    const trace = allocSlice(allocator, pipeline.Phase, 64) catch return insufficient_memory;
    const cache_patches = allocSlice(allocator, contacts.Patch, desc.contact_capacity) catch return insufficient_memory;
    const collider_views = allocSlice(allocator, shapes.Collider, desc.collider_capacity) catch return insufficient_memory;
    const proxies = allocSlice(allocator, broadphase.Proxy, desc.collider_capacity) catch return insufficient_memory;
    const endpoints = allocSlice(allocator, broadphase.Endpoint, @as(usize, desc.collider_capacity) * 2) catch return insufficient_memory;
    const endpoint_scratch = allocSlice(allocator, broadphase.Endpoint, @as(usize, desc.collider_capacity) * 2) catch return insufficient_memory;
    const active = allocSlice(allocator, u32, desc.collider_capacity) catch return insufficient_memory;
    const pairs = allocSlice(allocator, broadphase.PairKey, desc.contact_capacity) catch return insufficient_memory;
    const pair_work = allocSlice(allocator, broadphase.PairKey, desc.contact_capacity) catch return insufficient_memory;
    const pair_scratch = allocSlice(allocator, broadphase.PairKey, desc.contact_capacity) catch return insufficient_memory;
    const narrow_patches = allocSlice(allocator, contacts.Patch, desc.contact_capacity) catch return insufficient_memory;
    const cache_next = allocSlice(allocator, contacts.Patch, desc.contact_capacity) catch return insufficient_memory;
    const contact_events = allocSlice(allocator, contacts.Event, @as(usize, desc.contact_capacity) * 2) catch return insufficient_memory;
    const epa_vertices = allocSlice(allocator, gjk.SupportVertex, 256) catch return insufficient_memory;
    const epa_faces = allocSlice(allocator, gjk.EpaFace, 256) catch return insufficient_memory;
    const epa_visible = allocSlice(allocator, bool, 256) catch return insufficient_memory;
    const epa_horizon = allocSlice(allocator, gjk.HorizonEdge, 768) catch return insufficient_memory;
    const clip_reference = allocSlice(allocator, gjk.ClipVertex, 512) catch return insufficient_memory;
    const clip_incident = allocSlice(allocator, gjk.ClipVertex, 512) catch return insufficient_memory;
    const clip_scratch_a = allocSlice(allocator, gjk.ClipVertex, 512) catch return insufficient_memory;
    const clip_scratch_b = allocSlice(allocator, gjk.ClipVertex, 512) catch return insufficient_memory;
    const manifold_contacts = allocSlice(allocator, gjk.ContactPoint, 512) catch return insufficient_memory;
    const solver_contacts = allocSlice(allocator, solver.Contact, desc.contact_capacity) catch return insufficient_memory;
    const solver_points = allocSlice(allocator, solver.Point, @as(usize, desc.contact_capacity) * 4) catch return insufficient_memory;
    const restitution_bias = allocSlice(allocator, fp.Fp, @as(usize, desc.contact_capacity) * 4) catch return insufficient_memory;
    const pseudo_linear = allocSlice(allocator, g.Vec3, desc.body_capacity) catch return insufficient_memory;
    const pseudo_angular = allocSlice(allocator, g.Vec3, desc.body_capacity) catch return insufficient_memory;
    const edge_capacity = @as(usize, desc.contact_capacity) + @as(usize, desc.body_capacity) * 6;
    const island_edges = allocSlice(allocator, constraints.Edge, edge_capacity) catch return insufficient_memory;
    const edge_scratch = allocSlice(allocator, constraints.Edge, edge_capacity) catch return insufficient_memory;
    const islands = allocSlice(allocator, constraints.Island, desc.body_capacity) catch return insufficient_memory;
    const members = allocSlice(allocator, ids.BodyId, desc.body_capacity) catch return insufficient_memory;
    const lock_rows = allocSlice(allocator, constraints.ConstraintRow, @as(usize, desc.body_capacity) * 6) catch return insufficient_memory;
    const substep_events = allocSlice(allocator, contacts.Event, @as(usize, desc.contact_capacity) * 2) catch return insufficient_memory;
    const previous = allocSlice(allocator, contacts.Patch, desc.contact_capacity) catch return insufficient_memory;
    const event_next = allocSlice(allocator, contacts.Patch, desc.contact_capacity) catch return insufficient_memory;
    const tick_events = allocSlice(allocator, contacts.Event, @as(usize, desc.contact_capacity) * 2) catch return insufficient_memory;
    const stage_contacts = allocSlice(allocator, contacts.Patch, desc.contact_capacity) catch return insufficient_memory;
    const contact_scratch = allocSlice(allocator, contacts.Patch, desc.contact_capacity) catch return insufficient_memory;
    const query_items = allocSlice(allocator, queries.Item, desc.collider_capacity) catch return insufficient_memory;
    const query_colliders = allocSlice(allocator, shapes.Collider, desc.collider_capacity) catch return insufficient_memory;
    const query_capacity = @max(desc.collider_capacity, desc.contact_capacity);
    const query_candidates = allocSlice(allocator, queries.Hit, query_capacity) catch return insufficient_memory;
    const query_output = allocSlice(allocator, queries.Hit, query_capacity) catch return insufficient_memory;
    const query_flags = allocSlice(allocator, u8, desc.collider_capacity) catch return insufficient_memory;
    const query_u32_a = allocSlice(allocator, u32, query_capacity) catch return insufficient_memory;
    const query_u32_b = allocSlice(allocator, u32, query_capacity) catch return insufficient_memory;
    const query_mesh_hits = allocSlice(allocator, queries.MeshRayHit, query_capacity) catch return insufficient_memory;
    const query_height_triangles = allocSlice(allocator, mesh.HeightTriangle, query_capacity) catch return insufficient_memory;
    const query_leaves = allocSlice(allocator, shapes.CompoundLeaf, @as(usize, desc.collider_capacity) * 8) catch return insufficient_memory;
    const query_nodes_a = allocSlice(allocator, baked.BvhNode, query_capacity) catch return insufficient_memory;
    const query_nodes_b = allocSlice(allocator, baked.BvhNode, query_capacity) catch return insufficient_memory;
    const query_node_pairs = allocSlice(allocator, mesh.NodePair, query_capacity) catch return insufficient_memory;
    const query_pair_a = allocSlice(allocator, mesh.PrimitivePair, query_capacity) catch return insufficient_memory;
    const query_pair_b = allocSlice(allocator, mesh.PrimitivePair, query_capacity) catch return insufficient_memory;
    const query_pair_c = allocSlice(allocator, mesh.PrimitivePair, query_capacity) catch return insufficient_memory;
    const surface_contacts = allocSlice(allocator, gjk.ContactPoint, @as(usize, query_capacity) * 4) catch return insufficient_memory;
    const surface_merged = allocSlice(allocator, gjk.ContactPoint, @as(usize, query_capacity) * 4) catch return insufficient_memory;
    const pipeline_payload = allocSlice(allocator, u8, 256) catch return insufficient_memory;
    const bodies_payload = allocSlice(allocator, u8, @as(usize, desc.body_capacity) * 256 + 256) catch return insufficient_memory;
    const colliders_payload = allocSlice(allocator, u8, @as(usize, desc.collider_capacity) * 256 + 256) catch return insufficient_memory;
    const contacts_payload = allocSlice(allocator, u8, @as(usize, desc.contact_capacity) * 512 + 256) catch return insufficient_memory;
    const snapshot_output = allocSlice(allocator, u8, pipeline_payload.len + bodies_payload.len + colliders_payload.len + contacts_payload.len + 1024) catch return insufficient_memory;
    result.* = .{
        .magic = world_magic,
        .active = 1,
        .in_call = 0,
        .callback_violation = 0,
        .last_error = ok,
        .reserved = 0,
        .assets = asset_store,
        .simulation = simulation,
        .state = .{},
        .value = value,
        .stage = stage,
        .commands = command_buffer,
        .host_dispatch_seen = host_dispatch_seen,
        .trace = trace,
        .cache = .{ .patches = cache_patches },
        .broadphase_buffers = undefined,
        .broadphase_workspace = undefined,
        .contact_workspace = undefined,
        .convex_workspace = undefined,
        .surface_workspace = undefined,
        .solver_workspace = undefined,
        .island_workspace = undefined,
        .solver_pipeline = undefined,
        .event_count = 0,
        .stage_contacts = stage_contacts,
        .contact_scratch = contact_scratch,
        .query_colliders = query_colliders,
        .query_items = query_items,
        .query_candidates = query_candidates,
        .query_output = query_output,
        .query_flags = query_flags,
        .query_u32_a = query_u32_a,
        .query_u32_b = query_u32_b,
        .query_mesh_hits = query_mesh_hits,
        .query_height_triangles = query_height_triangles,
        .query_leaves = query_leaves,
        .query_nodes_a = query_nodes_a,
        .query_nodes_b = query_nodes_b,
        .query_node_pairs = query_node_pairs,
        .query_pair_a = query_pair_a,
        .query_pair_b = query_pair_b,
        .query_pair_c = query_pair_c,
        .pipeline_payload = pipeline_payload,
        .bodies_payload = bodies_payload,
        .colliders_payload = colliders_payload,
        .contacts_payload = contacts_payload,
        .snapshot_output = snapshot_output,
        .dispatcher = .{ .struct_size = @sizeOf(Dispatcher), .reserved = 0, .user = null, .dispatch_batch = null },
    };
    result.broadphase_buffers = .{ .endpoints = endpoints, .endpoint_scratch = endpoint_scratch, .active = active, .pairs = pairs, .pair_work = pair_work, .pair_scratch = pair_scratch };
    result.broadphase_workspace = .{ .assets = &asset_store.value, .collider_views = collider_views, .proxies = proxies, .buffers = &result.broadphase_buffers };
    result.convex_workspace = .{ .manifold = .{ .epa = .{ .vertices = epa_vertices, .faces = epa_faces, .visible = epa_visible, .horizon = epa_horizon }, .reference = clip_reference, .incident = clip_incident, .scratch_a = clip_scratch_a, .scratch_b = clip_scratch_b, .contacts = manifold_contacts } };
    const sphere_mesh = mesh.SphereMeshWorkspace{ .nodes = query_nodes_a, .primitives = query_u32_a, .work = query_node_pairs, .pair_scratch = query_pair_a, .pair_output = query_pair_b, .contacts = surface_contacts };
    const sphere_heightfield = mesh.SphereHeightfieldWorkspace{ .tile_nodes = query_nodes_b, .work = query_node_pairs, .pair_scratch = query_pair_a, .pair_output = query_pair_b, .triangles = query_height_triangles, .contacts = surface_contacts };
    const convex_mesh = mesh.ConvexMeshPatchWorkspace{ .query = .{ .nodes = query_nodes_a, .primitives = query_u32_a, .work = query_node_pairs, .pair_scratch = query_pair_a, .pair_output = query_pair_b, .intersections = query_u32_b }, .triangle = .{ .epa = result.convex_workspace.manifold.epa }, .contacts = surface_contacts };
    const convex_heightfield = mesh.ConvexHeightfieldPatchWorkspace{ .query = .{ .tile_nodes = query_nodes_b, .work = query_node_pairs, .pair_scratch = query_pair_a, .pair_output = query_pair_b, .triangles = query_height_triangles, .intersections = query_u32_b }, .triangle = .{ .epa = result.convex_workspace.manifold.epa }, .contacts = surface_contacts };
    result.surface_workspace = .{
        .sphere_mesh = sphere_mesh,
        .sphere_heightfield = sphere_heightfield,
        .convex_mesh = convex_mesh,
        .convex_heightfield = convex_heightfield,
        .mesh_mesh = .{ .query = .{ .nodes_a = query_nodes_a, .primitives_a = query_u32_a, .nodes_b = query_nodes_b, .primitives_b = query_u32_b, .work = query_node_pairs, .pair_scratch = query_pair_a, .pair_output = query_pair_b, .overlaps = query_pair_c }, .contacts = surface_contacts },
        .sphere_compound = .{ .leaves = query_leaves, .mesh = sphere_mesh, .heightfield = sphere_heightfield, .merged = surface_merged },
        .convex_compound = .{ .leaves = query_leaves, .mesh = convex_mesh, .heightfield = convex_heightfield, .merged = surface_merged },
    };
    result.contact_workspace = .{ .broadphase = &result.broadphase_workspace, .narrow = narrow_patches, .convex = &result.convex_workspace, .surface = &result.surface_workspace, .cache = &result.cache, .cache_next = cache_next, .events = contact_events };
    result.solver_workspace = .{ .contacts = solver_contacts, .points = solver_points, .restitution_bias = restitution_bias, .pseudo = .{ .linear = pseudo_linear, .angular = pseudo_angular }, .manifold = result.convex_workspace.manifold, .surface = result.surface_workspace };
    result.island_workspace = .{ .edges = island_edges, .edge_scratch = edge_scratch, .islands = islands, .members = members, .lock_rows = lock_rows };
    result.solver_pipeline = .{ .contacts = &result.contact_workspace, .solver = &result.solver_workspace, .islands = &result.island_workspace, .substep_events = substep_events, .previous = previous, .event_next = event_next, .tick_events = tick_events };
    output.* = result;
    return ok;
}

pub export fn gravity_v1_world_deinit(world: ?*World) callconv(.c) u32 {
    const value = checked(world) orelse return invalid_state;
    if (value.in_call != 0) return reentrant;
    value.active = 0;
    return ok;
}
pub export fn gravity_v1_world_set_dispatcher(world: ?*World, dispatcher: ?*const Dispatcher) callconv(.c) u32 {
    const value = checked(world) orelse return invalid_state;
    const d = dispatcher orelse {
        value.dispatcher.dispatch_batch = null;
        value.dispatcher.user = null;
        return ok;
    };
    if (!validStruct(d) or d.dispatch_batch == null) return bad_struct;
    value.dispatcher = d.*;
    return ok;
}
pub export fn gravity_v1_world_tick(world: ?*const World, out_tick: ?*u64) callconv(.c) u32 {
    const value = checkedConst(world) orelse return invalid_state;
    if (rejectCallbackReentry(value)) return reentrant;
    const output = out_tick orelse return invalid_argument;
    output.* = value.state.tick;
    return ok;
}
pub export fn gravity_v1_world_last_error(world: ?*const World, out_error: ?*u32) callconv(.c) u32 {
    const value = checkedConst(world) orelse return invalid_state;
    if (rejectCallbackReentry(value)) return reentrant;
    const output = out_error orelse return invalid_argument;
    output.* = value.last_error;
    return ok;
}
pub export fn gravity_v1_world_hash(world: ?*const World, out_hash: ?*Hash128) callconv(.c) u32 {
    const value = checkedConst(world) orelse return invalid_state;
    if (rejectCallbackReentry(value)) return reentrant;
    const output = out_hash orelse return invalid_argument;
    output.bytes = pipeline.canonicalStateHash(&value.value, &value.state, value.simulation, .{ .cache = &value.cache });
    return ok;
}

fn convertCommand(value: Command) ?world_mod.Command {
    if (!validStruct(&value) or value.reserved1 != 0 or value.phase_priority > std.math.maxInt(u8)) return null;
    const key: world_mod.CommandKey = .{ .phase_priority = @intCast(value.phase_priority), .issuer = value.issuer, .sequence = value.sequence, .discriminant = 0 };
    const body = ids.BodyId{ .value = value.body };
    return .{ .key = key, .op = switch (value.type) {
        0 => .{ .force = .{ .body = body, .value = vec(value.first) } },
        1 => .{ .torque = .{ .body = body, .value = vec(value.first) } },
        2 => .{ .impulse_at_point = .{ .body = body, .impulse = vec(value.first), .point = vec(value.second) } },
        3 => .{ .velocity = .{ .body = body, .linear = vec(value.first), .angular = vec(value.second) } },
        4 => .{ .kinematic_target = .{ .body = body, .target = transform(value.transform) } },
        5 => .{ .locks = .{ .body = body, .value = locks(value.dof_locks) orelse return null } },
        else => return null,
    } };
}

pub export fn gravity_v1_world_step(world_ptr: ?*World, input: [*c]const Command, count: u32) callconv(.c) u32 {
    const world = checked(world_ptr) orelse return invalid_state;
    const entered = enter(world);
    if (entered != ok) return entered;
    if (count > world.commands.len or (count != 0 and input == null)) return leave(world, if (count > world.commands.len) capacity else invalid_argument);
    var i: usize = 0;
    while (i < count) : (i += 1) world.commands[i] = convertCommand(input[i]) orelse return leave(world, bad_struct);
    const rollback_bytes = snapshotBytes(world) catch |err| return leave(world, mapError(err));
    var status = fp.MathStatus{};
    var workspace = pipeline.Workspace{ .commands = world.commands, .trace = world.trace };
    var host: host_jobs.Dispatcher = undefined;
    var custom: jobs.Custom = undefined;
    if (world.dispatcher.dispatch_batch) |callback| {
        host = .{ .user = world.dispatcher.user, .dispatch_batch = callback, .seen = world.host_dispatch_seen };
        custom = host.custom();
        workspace.dispatcher = .{ .custom = &custom };
    }
    const stepped = pipeline.stepWithAnalyticSolver(&world.value, &world.state, world.simulation, world.commands[0..count], &workspace, &world.solver_pipeline, &status) catch |err| {
        const failure = mapError(err);
        const restored = snapshot.decodePipelineBodiesContactsSnapshotChecked(rollback_bytes, .{ .configuration = world.simulation, .asset_set = world.assets.value.asset_set_hash }, &world.value, &world.stage, &world.cache, world.stage_contacts, world.contact_scratch) catch return leave(world, internal);
        world.state = restored.state;
        world.event_count = 0;
        return leave(world, failure);
    };
    if (world.callback_violation != 0) {
        const restored = snapshot.decodePipelineBodiesContactsSnapshotChecked(rollback_bytes, .{ .configuration = world.simulation, .asset_set = world.assets.value.asset_set_hash }, &world.value, &world.stage, &world.cache, world.stage_contacts, world.contact_scratch) catch return leave(world, internal);
        world.state = restored.state;
        world.event_count = 0;
        return leave(world, callback_error);
    }
    world.event_count = stepped.events.len;
    return leave(world, ok);
}

pub export fn gravity_v1_world_create_body(world_ptr: ?*World, desc_ptr: ?*const BodyDesc, out_id: ?*u64) callconv(.c) u32 {
    const world = checked(world_ptr) orelse return invalid_state;
    const desc = desc_ptr orelse return invalid_argument;
    const output = out_id orelse return invalid_argument;
    const entered = enter(world);
    if (entered != ok) return entered;
    if (!validStruct(desc)) return leave(world, bad_struct);
    const body_type: shapes.BodyType = switch (desc.body_type) {
        0 => .static,
        1 => .dynamic,
        2 => .kinematic,
        else => return leave(world, invalid_argument),
    };
    const dof = locks(desc.dof_locks) orelse return leave(world, invalid_argument);
    var status = fp.MathStatus{};
    const id = world.value.create(.{ .body_type = body_type, .transform = transform(desc.transform), .inverse_mass = .{ .raw = desc.inverse_mass }, .inverse_inertia_local = .{ .xx = .{ .raw = desc.inverse_inertia_xx }, .yy = .{ .raw = desc.inverse_inertia_yy }, .zz = .{ .raw = desc.inverse_inertia_zz }, .xy = .{ .raw = desc.inverse_inertia_xy }, .xz = .{ .raw = desc.inverse_inertia_xz }, .yz = .{ .raw = desc.inverse_inertia_yz } }, .locks = dof }, &status) catch |err| return leave(world, mapError(err));
    output.* = id.value;
    return leave(world, ok);
}
pub export fn gravity_v1_world_destroy_body(world_ptr: ?*World, id: u64) callconv(.c) u32 {
    const world = checked(world_ptr) orelse return invalid_state;
    const entered = enter(world);
    if (entered != ok) return entered;
    world.value.destroy(.{ .value = id }) catch |err| return leave(world, mapError(err));
    return leave(world, ok);
}

pub export fn gravity_v1_world_body_states(world_ptr: ?*const World, output: [*c]BodyState, output_capacity: u32, out_required: ?*u32) callconv(.c) u32 {
    const world = checkedConst(world_ptr) orelse return invalid_state;
    if (rejectCallbackReentry(world)) return reentrant;
    const required = out_required orelse return invalid_argument;
    var count: u32 = 0;
    for (world.value.storage.alive) |alive| if (alive) {
        count += 1;
    };
    required.* = count;
    if (output_capacity < count or (count != 0 and output == null)) return buffer_too_small;
    var at: usize = 0;
    for (world.value.storage.alive, 0..) |alive, index| {
        if (!alive) continue;
        const id = world.value.bodyIdAt(index).?;
        output[at] = .{ .struct_size = @sizeOf(BodyState), .reserved = 0, .id = id.value, .body_type = @intFromEnum(world.value.storage.body_type[index]), .dof_locks = @as(u8, @bitCast(world.value.storage.locks[index])), .transform = .{ .position = abiVec(world.value.storage.position[index]), .orientation = abiQuat(world.value.storage.orientation[index]) }, .linear_velocity = abiVec(world.value.storage.linear_velocity[index]), .angular_velocity = abiVec(world.value.storage.angular_velocity[index]) };
        at += 1;
    }
    return ok;
}

fn shapeFrom(desc: *const ColliderDesc) ?shapes.Shape {
    if (!validStruct(desc) or desc.flags & ~@as(u32, 3) != 0 or desc.revision == 0) return null;
    return switch (desc.shape_kind) {
        0 => .{ .sphere = .{ .radius = .{ .raw = desc.dimensions.x } } },
        1 => .{ .box = .{ .half_extents = vec(desc.dimensions) } },
        2 => .{ .capsule = .{ .radius = .{ .raw = desc.dimensions.x }, .half_height = .{ .raw = desc.dimensions.y } } },
        3 => .{ .convex_hull = .{ .source_id = desc.asset_source_id, .revision = desc.revision } },
        4 => .{ .compound = .{ .source_id = desc.asset_source_id, .revision = desc.revision } },
        5 => .{ .triangle_mesh = .{ .source_id = desc.asset_source_id, .revision = desc.revision } },
        6 => .{ .height_field = .{ .source_id = desc.asset_source_id, .revision = desc.revision } },
        else => null,
    };
}
pub export fn gravity_v1_world_create_collider(world_ptr: ?*World, desc_ptr: ?*const ColliderDesc, out_id: ?*u64) callconv(.c) u32 {
    const world = checked(world_ptr) orelse return invalid_state;
    const desc = desc_ptr orelse return invalid_argument;
    const output = out_id orelse return invalid_argument;
    const entered = enter(world);
    if (entered != ok) return entered;
    const shape = shapeFrom(desc) orelse return leave(world, bad_struct);
    const id = world.value.createCollider(.{ .body = .{ .value = desc.body }, .local = transform(desc.local), .shape = shape, .material = .{ .friction = .{ .raw = desc.friction }, .restitution = .{ .raw = desc.restitution } }, .category = desc.category, .mask = desc.mask, .group = desc.group, .sensor = desc.flags & 1 != 0, .enabled = desc.flags & 2 == 0, .revision = desc.revision }) catch |err| return leave(world, mapError(err));
    output.* = id.value;
    return leave(world, ok);
}
pub export fn gravity_v1_world_destroy_collider(world_ptr: ?*World, id: u64) callconv(.c) u32 {
    const world = checked(world_ptr) orelse return invalid_state;
    const entered = enter(world);
    if (entered != ok) return entered;
    world.value.destroyCollider(.{ .value = id }) catch |err| return leave(world, mapError(err));
    return leave(world, ok);
}
pub export fn gravity_v1_world_events(world_ptr: ?*const World, output: [*c]Event, output_capacity: u32, out_required: ?*u32) callconv(.c) u32 {
    const world = checkedConst(world_ptr) orelse return invalid_state;
    if (rejectCallbackReentry(world)) return reentrant;
    const required = out_required orelse return invalid_argument;
    required.* = @intCast(world.event_count);
    if (output_capacity < world.event_count or (world.event_count != 0 and output == null)) return buffer_too_small;
    for (world.solver_pipeline.tick_events[0..world.event_count], 0..) |value, i| output[i] = .{ .struct_size = @sizeOf(Event), .reserved = 0, .type = @intFromEnum(value.kind), .reserved1 = 0, .collider_a = value.key.collider_a.value, .collider_b = value.key.collider_b.value, .feature_a = value.key.primitive_a, .feature_b = value.key.primitive_b };
    return ok;
}

fn mode(value: u32) ?queries.Mode {
    return switch (value) {
        0 => .any,
        1 => .closest,
        2 => .all,
        else => null,
    };
}
fn filter(value: Filter) ?queries.Filter {
    if (value.reserved != 0) return null;
    return .{ .category = value.category, .mask = value.mask, .group = value.group };
}
fn collectItems(world: *World) usize {
    const cs = world.value.colliders.?;
    var count: usize = 0;
    var status = fp.MathStatus{};
    for (cs.alive, 0..) |alive, index| {
        if (!alive or !cs.enabled[index]) continue;
        const body_index = world.value.bodyIndex(cs.body[index]) orelse continue;
        world.query_colliders[count] = .{ .body = cs.body[index], .local = cs.local[index], .shape = cs.shape[index], .material = cs.material[index], .category = cs.category[index], .mask = cs.mask[index], .group = cs.group[index], .sensor = cs.sensor[index], .enabled = cs.enabled[index], .revision = cs.revision[index] };
        world.query_items[count] = .{ .id = ids.ColliderId.init(@intCast(index), cs.generation[index]), .collider = &world.query_colliders[count], .transform = compose(.{ .position = world.value.storage.position[body_index], .orientation = world.value.storage.orientation[body_index] }, cs.local[index], &status) };
        count += 1;
    }
    return count;
}
fn writeHits(publication: queries.Publication, output: [*c]QueryHit, output_capacity: u32, out_required: *u32) u32 {
    out_required.* = @intCast(publication.required);
    if (publication.hits.len != publication.required) return capacity;
    if (output_capacity < publication.required or (publication.required != 0 and output == null)) return buffer_too_small;
    for (publication.hits, 0..) |hit, i| output[i] = .{ .struct_size = @sizeOf(QueryHit), .reserved = 0, .collider = hit.collider.value, .fraction = hit.fraction.raw, .point = abiVec(hit.point), .normal = abiVec(hit.normal), .primitive = hit.primitive, .reserved1 = 0 };
    return ok;
}
fn rayWorkspace(world: *World) queries.RayWorkspace {
    return .{ .bvh_nodes = world.query_u32_a, .mesh_hits = world.query_mesh_hits, .height_triangles = world.query_height_triangles, .compound_leaves = world.query_leaves };
}
fn overlapWorkspace(world: *World) queries.SurfaceOverlapWorkspace {
    return .{
        .compound_leaves = world.query_leaves,
        .mesh = .{ .nodes = world.query_nodes_a, .primitives = world.query_u32_a, .work = world.query_node_pairs, .pair_scratch = world.query_pair_a, .pair_output = world.query_pair_b, .intersections = world.query_u32_b },
        .heightfield = .{ .tile_nodes = world.query_nodes_b, .work = world.query_node_pairs, .pair_scratch = world.query_pair_a, .pair_output = world.query_pair_b, .triangles = world.query_height_triangles, .intersections = world.query_u32_b },
    };
}
pub export fn gravity_v1_world_query_ray(world_ptr: ?*World, query_ptr: ?*const RayQuery, output: [*c]QueryHit, output_capacity: u32, out_required: ?*u32) callconv(.c) u32 {
    const world = checked(world_ptr) orelse return invalid_state;
    const query = query_ptr orelse return invalid_argument;
    const required = out_required orelse return invalid_argument;
    const entered = enter(world);
    if (entered != ok) return entered;
    if (!validStruct(query) or query.reserved1 != 0) return leave(world, bad_struct);
    const m = mode(query.mode) orelse return leave(world, invalid_argument);
    const f = filter(query.filter) orelse return leave(world, bad_struct);
    var status = fp.MathStatus{};
    const count = collectItems(world);
    const ray = queries.Ray{ .origin = vec(query.origin), .delta = vec(query.direction).scale(.{ .raw = query.max_fraction }, &status) };
    var analytic_only = true;
    for (world.query_items[0..count]) |item| switch (item.collider.shape) {
        .sphere, .box, .capsule => {},
        else => analytic_only = false,
    };
    const publication = if (!analytic_only) queries.rayShapes(ray, f, world.query_items[0..count], &world.assets.value, rayWorkspace(world), world.query_candidates, world.query_output, m, &status) catch |err| return leave(world, mapError(err)) else block: {
        const plan = jobs.RangePlan.initBounded(count, 32, jobs.maximum_batch_jobs) catch return leave(world, capacity);
        const Kernel = struct {
            ray: queries.Ray,
            filter: queries.Filter,
            items: []const queries.Item,
            slots: []queries.Hit,
            flags: []u8,
            plan: jobs.RangePlan,
            errors: [jobs.maximum_batch_jobs]u32 = [_]u32{ok} ** jobs.maximum_batch_jobs,
            statuses: [jobs.maximum_batch_jobs]fp.MathStatus = [_]fp.MathStatus{.{}} ** jobs.maximum_batch_jobs,
            fn run(raw: *anyopaque, logical_job: u32) !void {
                const self: *@This() = @ptrCast(@alignCast(raw));
                const range = self.plan.range(logical_job) catch {
                    self.errors[logical_job] = internal;
                    return error.CallbackFailed;
                };
                var index: usize = range.begin;
                while (index < range.end) : (index += 1) {
                    self.flags[index] = 0;
                    const item = self.items[index];
                    if (!queries.passesFilter(self.filter, item.collider)) continue;
                    self.slots[index] = queries.rayPrimitive(self.ray, item, &self.statuses[logical_job]) catch |err| {
                        self.errors[logical_job] = mapError(err);
                        return error.CallbackFailed;
                    } orelse continue;
                    self.flags[index] = 1;
                }
            }
        };
        var kernel = Kernel{ .ray = ray, .filter = f, .items = world.query_items[0..count], .slots = world.query_candidates[0..count], .flags = world.query_flags[0..count], .plan = plan };
        if (plan.job_count != 0) {
            var host: host_jobs.Dispatcher = undefined;
            var custom: jobs.Custom = undefined;
            var dispatcher = jobs.Dispatcher{ .serial = {} };
            if (world.dispatcher.dispatch_batch) |callback| {
                host = .{ .user = world.dispatcher.user, .dispatch_batch = callback, .seen = world.host_dispatch_seen };
                custom = host.custom();
                dispatcher = .{ .custom = &custom };
            }
            dispatcher.dispatch(.{ .context = &kernel, .job_count = plan.job_count, .run = Kernel.run }) catch {
                for (kernel.errors[0..plan.job_count]) |failure| if (failure != ok) return leave(world, failure);
                return leave(world, callback_error);
            };
            if (world.callback_violation != 0) return leave(world, callback_error);
            for (kernel.errors[0..plan.job_count]) |failure| if (failure != ok) return leave(world, failure);
            for (kernel.statuses[0..plan.job_count]) |job_status| {
                if (status.fault == .none and job_status.fault != .none) status.fault = job_status.fault;
            }
        }
        var found: usize = 0;
        for (world.query_flags[0..count], 0..) |flag, index| if (flag == 1) {
            world.query_candidates[found] = world.query_candidates[index];
            found += 1;
        };
        break :block queries.publish(m, world.query_candidates[0..found], world.query_output) catch |err| return leave(world, mapError(err));
    };
    return leave(world, writeHits(publication, output, output_capacity, required));
}

fn overlapQuery(world: *World, query_filter: queries.Filter, query_mode: queries.Mode, kind: enum { point, aabb, shape }, point: g.Vec3, bounds: g.Aabb3, shape: shapes.Shape, shape_transform: g.Transform3, output: [*c]QueryHit, output_capacity: u32, required: *u32) u32 {
    const count = collectItems(world);
    var status = fp.MathStatus{};
    const plan = jobs.RangePlan.initBounded(count, 32, jobs.maximum_batch_jobs) catch return capacity;
    const Kernel = struct {
        world: *World,
        items: []const queries.Item,
        flags: []u8,
        filter: queries.Filter,
        kind: @TypeOf(kind),
        point: g.Vec3,
        bounds: g.Aabb3,
        shape: shapes.Shape,
        shape_transform: g.Transform3,
        plan: jobs.RangePlan,
        errors: [jobs.maximum_batch_jobs]u32 = [_]u32{ok} ** jobs.maximum_batch_jobs,
        statuses: [jobs.maximum_batch_jobs]fp.MathStatus = [_]fp.MathStatus{.{}} ** jobs.maximum_batch_jobs,

        fn run(raw: *anyopaque, logical_job: u32) !void {
            const self: *@This() = @ptrCast(@alignCast(raw));
            const range = self.plan.range(logical_job) catch {
                self.errors[logical_job] = internal;
                return error.CallbackFailed;
            };
            var index: usize = range.begin;
            while (index < range.end) : (index += 1) {
                const item = self.items[index];
                self.flags[index] = 0;
                if (!queries.passesFilter(self.filter, item.collider)) continue;
                // Asset-backed targets require shared traversal scratch and are
                // deliberately deferred to the canonical serial merge below.
                switch (item.collider.shape) {
                    .sphere, .box, .capsule => {},
                    else => {
                        self.flags[index] = 2;
                        continue;
                    },
                }
                const hit = switch (self.kind) {
                    .point => queries.pointOverlapsPrimitive(self.point, item.collider.shape, item.transform, &self.statuses[logical_job]),
                    .aabb => queries.aabbOverlapsConvex(self.bounds, item.collider.shape, item.transform, &self.world.assets.value, &self.statuses[logical_job]),
                    .shape => queries.convexOverlaps(self.shape, self.shape_transform, item.collider.shape, item.transform, &self.world.assets.value, &self.statuses[logical_job]),
                } catch |err| {
                    self.errors[logical_job] = mapError(err);
                    return error.CallbackFailed;
                };
                self.flags[index] = @intFromBool(hit);
            }
        }
    };
    var kernel = Kernel{ .world = world, .items = world.query_items[0..count], .flags = world.query_flags[0..count], .filter = query_filter, .kind = kind, .point = point, .bounds = bounds, .shape = shape, .shape_transform = shape_transform, .plan = plan };
    if (plan.job_count != 0) {
        var host: host_jobs.Dispatcher = undefined;
        var custom: jobs.Custom = undefined;
        var dispatcher = jobs.Dispatcher{ .serial = {} };
        if (world.dispatcher.dispatch_batch) |callback| {
            host = .{ .user = world.dispatcher.user, .dispatch_batch = callback, .seen = world.host_dispatch_seen };
            custom = host.custom();
            dispatcher = .{ .custom = &custom };
        }
        dispatcher.dispatch(.{ .context = &kernel, .job_count = plan.job_count, .run = Kernel.run }) catch {
            for (kernel.errors[0..plan.job_count]) |failure| if (failure != ok) return failure;
            return callback_error;
        };
        if (world.callback_violation != 0) return callback_error;
        for (kernel.errors[0..plan.job_count]) |failure| if (failure != ok) return failure;
        for (kernel.statuses[0..plan.job_count]) |job_status| {
            if (status.fault == .none and job_status.fault != .none) status.fault = job_status.fault;
        }
    }
    var found: usize = 0;
    for (world.query_items[0..count], 0..) |item, index| {
        var hit = kernel.flags[index] == 1;
        if (kernel.flags[index] == 2) hit = switch (kind) {
            .point => queries.pointOverlapsShape(point, item.collider.shape, item.transform, &world.assets.value, rayWorkspace(world), &status) catch |err| return mapError(err),
            .aabb => queries.aabbOverlapsShape(bounds, item.collider.shape, item.transform, &world.assets.value, overlapWorkspace(world), &status) catch |err| return mapError(err),
            .shape => queries.convexOverlapsShape(shape, shape_transform, item.collider.shape, item.transform, &world.assets.value, overlapWorkspace(world), &status) catch |err| return mapError(err),
        };
        if (hit) {
            world.query_candidates[found] = .{ .collider = item.id, .fraction = .zero, .point = if (kind == .point) point else item.transform.position, .normal = .{}, .primitive = 0 };
            found += 1;
        }
    }
    const publication = queries.publish(query_mode, world.query_candidates[0..found], world.query_output) catch |err| return mapError(err);
    return writeHits(publication, output, output_capacity, required);
}
pub export fn gravity_v1_world_query_point(world_ptr: ?*World, query_ptr: ?*const PointQuery, output: [*c]QueryHit, output_capacity: u32, out_required: ?*u32) callconv(.c) u32 {
    const world = checked(world_ptr) orelse return invalid_state;
    const query = query_ptr orelse return invalid_argument;
    const required = out_required orelse return invalid_argument;
    const entered = enter(world);
    if (entered != ok) return entered;
    if (!validStruct(query) or query.reserved1 != 0) return leave(world, bad_struct);
    const m = mode(query.mode) orelse return leave(world, invalid_argument);
    const f = filter(query.filter) orelse return leave(world, bad_struct);
    return leave(world, overlapQuery(world, f, m, .point, vec(query.point), undefined, undefined, undefined, output, output_capacity, required));
}
pub export fn gravity_v1_world_query_aabb(world_ptr: ?*World, query_ptr: ?*const AabbQuery, output: [*c]QueryHit, output_capacity: u32, out_required: ?*u32) callconv(.c) u32 {
    const world = checked(world_ptr) orelse return invalid_state;
    const query = query_ptr orelse return invalid_argument;
    const required = out_required orelse return invalid_argument;
    const entered = enter(world);
    if (entered != ok) return entered;
    if (!validStruct(query) or query.reserved1 != 0) return leave(world, bad_struct);
    const m = mode(query.mode) orelse return leave(world, invalid_argument);
    const f = filter(query.filter) orelse return leave(world, bad_struct);
    const bounds = g.Aabb3{ .min = vec(query.min), .max = vec(query.max) };
    if (bounds.min.x.raw > bounds.max.x.raw or bounds.min.y.raw > bounds.max.y.raw or bounds.min.z.raw > bounds.max.z.raw) return leave(world, invalid_argument);
    return leave(world, overlapQuery(world, f, m, .aabb, undefined, bounds, undefined, undefined, output, output_capacity, required));
}
pub export fn gravity_v1_world_query_shape(world_ptr: ?*World, query_ptr: ?*const ShapeQuery, output: [*c]QueryHit, output_capacity: u32, out_required: ?*u32) callconv(.c) u32 {
    const world = checked(world_ptr) orelse return invalid_state;
    const query = query_ptr orelse return invalid_argument;
    const required = out_required orelse return invalid_argument;
    const entered = enter(world);
    if (entered != ok) return entered;
    if (!validStruct(query) or query.reserved1 != 0) return leave(world, bad_struct);
    const m = mode(query.mode) orelse return leave(world, invalid_argument);
    const f = filter(query.filter) orelse return leave(world, bad_struct);
    const shape = shapeFrom(&query.shape) orelse return leave(world, bad_struct);
    return leave(world, overlapQuery(world, f, m, .shape, undefined, undefined, shape, transform(query.transform), output, output_capacity, required));
}

fn snapshotBytes(world: *World) ![]const u8 {
    return snapshot.encodePipelineBodiesContactsSnapshot(.{ .configuration = world.simulation, .asset_set = world.assets.value.asset_set_hash }, world.state, &world.value, &world.cache, world.snapshot_output, world.pipeline_payload, world.bodies_payload, world.colliders_payload, world.contacts_payload);
}
pub export fn gravity_v1_world_snapshot_size(world_ptr: ?*World, out_size: ?*u64) callconv(.c) u32 {
    const world = checked(world_ptr) orelse return invalid_state;
    const output = out_size orelse return invalid_argument;
    const entered = enter(world);
    if (entered != ok) return entered;
    const bytes = snapshotBytes(world) catch |err| return leave(world, mapError(err));
    output.* = bytes.len;
    return leave(world, ok);
}
pub export fn gravity_v1_world_snapshot_save(world_ptr: ?*World, output: [*c]u8, output_capacity: u64, out_required: ?*u64) callconv(.c) u32 {
    const world = checked(world_ptr) orelse return invalid_state;
    const required = out_required orelse return invalid_argument;
    const entered = enter(world);
    if (entered != ok) return entered;
    const bytes = snapshotBytes(world) catch |err| return leave(world, mapError(err));
    required.* = bytes.len;
    if (output_capacity < bytes.len or (bytes.len != 0 and output == null)) return leave(world, buffer_too_small);
    @memcpy(output[0..bytes.len], bytes);
    return leave(world, ok);
}
pub export fn gravity_v1_world_snapshot_load(world_ptr: ?*World, input: [*c]const u8, length: u64) callconv(.c) u32 {
    const world = checked(world_ptr) orelse return invalid_state;
    const entered = enter(world);
    if (entered != ok) return entered;
    const bytes = bytesFrom(input, length) orelse return leave(world, invalid_argument);
    const decoded = snapshot.decodePipelineBodiesContactsSnapshotChecked(bytes, .{ .configuration = world.simulation, .asset_set = world.assets.value.asset_set_hash }, &world.value, &world.stage, &world.cache, world.stage_contacts, world.contact_scratch) catch |err| return leave(world, mapError(err));
    world.state = decoded.state;
    return leave(world, ok);
}

test "ABI layouts remain frozen" {
    try std.testing.expectEqual(@as(usize, 24), @sizeOf(Vec3));
    try std.testing.expectEqual(@as(usize, 32), @sizeOf(Quat));
    try std.testing.expectEqual(@as(usize, 8), @offsetOf(BodyState, "id"));
}
