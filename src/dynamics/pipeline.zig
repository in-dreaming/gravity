//! Fixed-order World-step foundation.  Persistent state and all temporary
//! buffers are caller-owned; no allocator or wall clock participates.
const std = @import("std");
const fp = @import("../math/fp.zig");
const config = @import("../core/config.zig");
const geometry = @import("../math/geometry.zig");
const broadphase = @import("../collision/broadphase.zig");
const analytic = @import("../collision/analytic.zig");
const contact_cache = @import("../collision/contact_cache.zig");
const gjk = @import("../collision/gjk.zig");
const mesh = @import("../collision/mesh.zig");
const runtime_view = @import("../assets/runtime_view.zig");
const contact_solver = @import("contact_solver.zig");
const constraints = @import("constraints.zig");
const joints = @import("joints.zig");
const sleeping = @import("sleeping.zig");
const ccd = @import("ccd.zig");
const queries = @import("../query/queries.zig");
const store = @import("../assets/store.zig");
const shapes = @import("../collision/shapes.zig");
const hash = @import("../state/hash.zig");
const world_mod = @import("world.zig");
const jobs = @import("gravity_jobs");

const max_phase_jobs: u32 = jobs.maximum_batch_jobs;
const preferred_phase_grain: u32 = 256;

pub const Error = error{ Reentrant, Faulted, TraceCapacity, CcdFault, BroadphaseFailure, ContactFailure } || jobs.Error || world_mod.Error || config.ConfigError || sleeping.Error || constraints.Error || ccd.Error;
pub const Phase = enum(u8) { prevalidate, commit, integrate, broadphase, narrowphase, islands, solve, ccd, sleep, events, hash };
pub const FaultCode = enum { math, world, shape, broadphase, contact, ccd };
/// Stable, serializable cause carried by a runtime fault. It intentionally
/// avoids Zig error-set identity so snapshots and cross-target hashes retain
/// the same meaning.
pub const FaultDetail = enum(u8) { none, math, invalid_body, invalid_collider, invalid_shape, invalid_asset, invalid_transform, capacity_exceeded, unsupported_shape, ccd, contact, broadphase, other };
/// First runtime failure for a Tick. `object` is reserved for the canonical
/// body/collider/joint owner once the failing producer has one; zero-allocation
/// phases that have no single owner leave it null.
pub const RuntimeFault = struct { tick: u64, phase: Phase, object: ?u64 = null, code: FaultCode, detail: FaultDetail = .none, math_fault: fp.MathFault = .none };
pub const State = struct { tick: u64 = 0, in_step: bool = false, fault: ?RuntimeFault = null };
pub const Diagnostics = struct {
    /// Number of visits to every fixed pipeline phase during this Tick. This
    /// is derived, caller-owned profiling data and never participates in
    /// simulation state or hashes.
    phase_count: [11]u32 = [_]u32{0} ** 11,
    pub fn reset(self: *Diagnostics) void {
        self.* = .{};
    }
    pub fn visits(self: *const Diagnostics, phase: Phase) u32 {
        return self.phase_count[@intFromEnum(phase)];
    }
};
/// Optional derived phase-boundary observer. Core never reads a host clock;
/// profiling tools may timestamp these deterministic transitions externally.
/// `next == null` closes the final phase of a Tick.
pub const ProfileDetail = enum { solve_contacts, solve_wake, solve_joints, solve_validate, solve_clear, solve_pgs };
pub const PhaseObserver = struct {
    context: *anyopaque,
    transition_fn: *const fn (*anyopaque, ?Phase) void,
    detail_fn: ?*const fn (*anyopaque, ProfileDetail) void = null,
    pub fn transition(self: *const PhaseObserver, next: ?Phase) void {
        self.transition_fn(self.context, next);
    }
    pub fn detail(self: *const PhaseObserver, value: ProfileDetail) void {
        if (self.detail_fn) |call| call(self.context, value);
    }
};
pub const Workspace = struct {
    commands: []world_mod.Command,
    trace: []Phase,
    trace_len: usize = 0,
    diagnostics: ?*Diagnostics = null,
    observer: ?*const PhaseObserver = null,
    /// Synchronous executor seam. The default serial backend is the golden
    /// oracle; native/host adapters borrow each phase context until barrier.
    dispatcher: jobs.Dispatcher = .{ .serial = {} },
};
/// Caller-owned storage for the SAP phase. `collider_views` backs proxy
/// pointers and must remain valid until the caller has consumed `pairs`.
pub const BroadphaseWorkspace = struct {
    assets: *const store.Store,
    collider_views: []shapes.Collider,
    proxies: []broadphase.Proxy,
    buffers: *broadphase.Buffers,
};
/// Caller-owned transient and persistent contact-cache storage. Narrow output
/// is overwritten per substep; only the final substep is merged so public
/// contact events retain Tick, rather than substep, semantics.
pub const AnalyticContactWorkspace = struct {
    broadphase: *BroadphaseWorkspace,
    narrow: []contact_cache.Patch,
    convex: ?*ConvexNarrowWorkspace = null,
    surface: ?*SurfaceNarrowWorkspace = null,
    cache: *contact_cache.Cache,
    cache_next: []contact_cache.Patch,
    events: []contact_cache.Event,
};
/// Scratch required by the runtime convex producer. It deliberately owns only
/// Task 10 query storage: persistent manifold/cache memory remains in
/// `AnalyticContactWorkspace` and is shared with every narrow producer.
pub const ConvexNarrowWorkspace = struct { manifold: gjk.ManifoldWorkspace };
pub const SurfaceNarrowWorkspace = struct {
    sphere_mesh: ?mesh.SphereMeshWorkspace = null,
    sphere_heightfield: ?mesh.SphereHeightfieldWorkspace = null,
    convex_mesh: ?mesh.ConvexMeshPatchWorkspace = null,
    convex_heightfield: ?mesh.ConvexHeightfieldPatchWorkspace = null,
    mesh_mesh: ?mesh.MeshMeshPatchWorkspace = null,
    sphere_compound: ?mesh.SphereCompoundSurfaceWorkspace = null,
    convex_compound: ?mesh.ConvexCompoundSurfaceWorkspace = null,
};
pub const Result = struct { tick: u64, trace: []const Phase, diagnostics: ?Diagnostics = null, state_hash: ?hash.Hash128 = null };
pub const ContactResult = struct { step: Result, events: []const contact_cache.Event };
/// Persistent pieces that live outside Task 13's World layout but affect the
/// next Tick. All are included in the canonical pipeline state hash.
pub const HashInputs = struct {
    cache: *const contact_cache.Cache,
    joint: ?*const joints.Pool = null,
    sleep: ?sleeping.Storage = null,
    ccd_enabled: ?[]const bool = null,
};
/// Per-layer diagnostic hashes make a replay mismatch actionable without
/// weakening the single composite hash used by deterministic simulation.
pub const LayerHashes = struct {
    composite: hash.Hash128,
    world: hash.Hash128,
    bodies: hash.Hash128,
    colliders: hash.Hash128,
    contacts: hash.Hash128,
    joints: ?hash.Hash128,
    sleep: ?hash.Hash128,
    ccd: ?hash.Hash128,
    events: hash.Hash128,
};

/// Hashes the complete future-relevant state owned by the fixed World
/// pipeline. Scratch SAP/island/row/trace/profile buffers are intentionally
/// absent because they are reconstructed uniquely on the next Tick.
pub fn canonicalStateHash(world: *const world_mod.World, state: *const State, simulation: config.SimulationConfig, inputs: HashInputs) hash.Hash128 {
    var sink = hash.Sink.init(.state);
    var visitor = HashVisitor{ .sink = &sink };
    visitor.writeU64(state.tick);
    if (state.fault) |fault| {
        visitor.writeU8(1);
        visitor.writeU64(fault.tick);
        visitor.writeU8(@intFromEnum(fault.phase));
        if (fault.object) |object| {
            visitor.writeU8(1);
            visitor.writeU64(object);
        } else visitor.writeU8(0);
        visitor.writeU8(@intFromEnum(fault.code));
        visitor.writeU8(@intFromEnum(fault.detail));
        visitor.writeU8(@intFromEnum(fault.math_fault));
    } else visitor.writeU8(0);
    simulation.visitCanonical(&visitor);
    world.visitCanonical(&visitor);
    contact_cache.visitCanonical(inputs.cache, &visitor);
    if (inputs.joint) |joint| {
        visitor.writeU8(1);
        joints.visitCanonical(joint, &visitor);
    } else visitor.writeU8(0);
    if (inputs.sleep) |sleep| {
        visitor.writeU8(1);
        sleeping.visitCanonical(sleep, &visitor);
    } else visitor.writeU8(0);
    if (inputs.ccd_enabled) |enabled| {
        visitor.writeU8(1);
        visitor.writeU64(enabled.len);
        for (enabled) |value| visitor.writeU8(@intFromBool(value));
    } else visitor.writeU8(0);
    return sink.final128();
}
pub fn layeredStateHashes(world: *const world_mod.World, state: *const State, simulation: config.SimulationConfig, inputs: HashInputs, events: []const contact_cache.Event) LayerHashes {
    var world_sink = hash.Sink.init(.state);
    var world_visitor = HashVisitor{ .sink = &world_sink };
    world.visitCanonical(&world_visitor);
    var body_sink = hash.Sink.init(.state);
    var body_visitor = HashVisitor{ .sink = &body_sink };
    world.visitBodiesCanonical(&body_visitor);
    var collider_sink = hash.Sink.init(.state);
    var collider_visitor = HashVisitor{ .sink = &collider_sink };
    world.visitCollidersCanonical(&collider_visitor);
    var contact_sink = hash.Sink.init(.state);
    var contact_visitor = HashVisitor{ .sink = &contact_sink };
    contact_cache.visitCanonical(inputs.cache, &contact_visitor);
    var event_sink = hash.Sink.init(.state);
    var event_visitor = HashVisitor{ .sink = &event_sink };
    contact_cache.visitEventsCanonical(events, &event_visitor);
    const joint_hash = if (inputs.joint) |joint| block: {
        var sink = hash.Sink.init(.state);
        var visitor = HashVisitor{ .sink = &sink };
        joints.visitCanonical(joint, &visitor);
        break :block sink.final128();
    } else null;
    const sleep_hash = if (inputs.sleep) |storage| block: {
        var sink = hash.Sink.init(.state);
        var visitor = HashVisitor{ .sink = &sink };
        sleeping.visitCanonical(storage, &visitor);
        break :block sink.final128();
    } else null;
    const ccd_hash = if (inputs.ccd_enabled) |enabled| block: {
        var sink = hash.Sink.init(.state);
        var visitor = HashVisitor{ .sink = &sink };
        visitor.writeU64(enabled.len);
        for (enabled) |value| visitor.writeU8(@intFromBool(value));
        break :block sink.final128();
    } else null;
    return .{ .composite = canonicalStateHash(world, state, simulation, inputs), .world = world_sink.final128(), .bodies = body_sink.final128(), .colliders = collider_sink.final128(), .contacts = contact_sink.final128(), .joints = joint_hash, .sleep = sleep_hash, .ccd = ccd_hash, .events = event_sink.final128() };
}
const HashVisitor = struct {
    sink: *hash.Sink,
    pub fn writeU8(self: *@This(), value: u8) void {
        self.sink.update(&[_]u8{value});
    }
    pub fn writeU32(self: *@This(), value: u32) void {
        var bytes: [4]u8 = undefined;
        std.mem.writeInt(u32, &bytes, value, .little);
        self.sink.update(&bytes);
    }
    pub fn writeU64(self: *@This(), value: u64) void {
        var bytes: [8]u8 = undefined;
        std.mem.writeInt(u64, &bytes, value, .little);
        self.sink.update(&bytes);
    }
    pub fn writeI64(self: *@This(), value: i64) void {
        self.writeU64(@bitCast(value));
    }
    pub fn field(self: *@This(), name: []const u8, value: anytype) void {
        self.writeU8(@intCast(name.len));
        self.sink.update(name);
        switch (@TypeOf(value)) {
            bool => self.writeU8(@intFromBool(value)),
            u8 => self.writeU8(value),
            u32 => self.writeU32(value),
            i64 => self.writeI64(value),
            usize => self.writeU64(value),
            else => @compileError("unsupported canonical config field type"),
        }
    }
};
/// Scratch storage for converting analytic cache patches into Task 15 solver
/// contacts. One analytic patch has one point; complex manifold producers use
/// their own multi-point bridge before the same solver boundary.
pub const AnalyticSolverWorkspace = struct {
    contacts: []contact_solver.Contact,
    points: []contact_solver.Point,
    restitution_bias: []fp.Fp,
    pseudo: contact_solver.PseudoVelocities,
    partition: ?SolverPartitionWorkspace = null,
    /// Required only when cache patches originate from Task 10 convex/compound
    /// narrowing. The old primitive path remains allocation-free and does not
    /// touch this workspace.
    manifold: ?gjk.ManifoldWorkspace = null,
    surface: ?SurfaceNarrowWorkspace = null,
};
pub const SolverPartitionWorkspace = struct {
    body_island: []u32,
    contact_indices: []u32,
    row_indices: []u32,
    contact_offsets: []u32,
    row_offsets: []u32,
    contact_cursor: []u32,
    row_cursor: []u32,
};
/// Additional caller-owned storage for a full analytic contact/solver tick.
/// `previous` snapshots cache state before any substep; it is used only to
/// derive one Tick-level event sequence after substep warm-start updates.
pub const AnalyticSolverPipelineWorkspace = struct {
    contacts: *AnalyticContactWorkspace,
    solver: *AnalyticSolverWorkspace,
    islands: *IslandWorkspace,
    joint: ?*JointWorkspace = null,
    sleep: ?*SleepWorkspace = null,
    ccd: ?*CcdPipelineWorkspace = null,
    substep_events: []contact_cache.Event,
    previous: []contact_cache.Patch,
    event_next: []contact_cache.Patch,
    tick_events: []contact_cache.Event,
};
/// Caller-owned joint pool and Task 16 row buffers. When present, its rows
/// are built each substep and interleaved before contact rows in the combined
/// PGS loop.
pub const JointWorkspace = struct {
    pool: *joints.Pool,
    rows: []constraints.ConstraintRow,
    scratch: joints.PoolRowScratch,
};
/// Caller-owned Task 14 island and lock-row storage. Analytic non-sensor
/// cache patches provide the contact edges; joint edges are appended by the
/// joint-aware pipeline entry point.
pub const IslandWorkspace = struct {
    edges: []constraints.Edge,
    edge_scratch: []constraints.Edge,
    islands: []constraints.Island,
    members: []@import("../core/ids.zig").BodyId,
    lock_rows: []constraints.ConstraintRow,
    union_parents: ?[]u32 = null,
    edge_len: usize = 0,
};
/// Caller-owned persistent Task 18 state and event/wake scratch. Sleep events
/// remain available in `sleep_events` for the outer Tick event merge.
pub const SleepWorkspace = struct {
    storage: sleeping.Storage,
    requests: []sleeping.Request,
    graph_scratch: []@import("../core/ids.zig").BodyId,
    wake_events: []sleeping.Event,
    sleep_events: []sleeping.Event,
};
/// Caller-owned CCD flags and item storage. CCD enablement is mutable World
/// policy, deliberately separate from immutable collider shape data.
pub const CcdItemWorkspace = struct { enabled: []const bool, items: []ccd.Item };
/// Caller-owned CCD scan and TOI-patch storage.  The pipeline performs a
/// non-mutating `prepareCcdToi` before it advances World motion; this leaves
/// enough time to reject missing contact/cache/solver capacity without
/// publishing a partial TOI.
pub const CcdPipelineWorkspace = struct {
    assets: *const store.Store,
    items: *CcdItemWorkspace,
    pairs: []ccd.Pair,
    surface: queries.SurfaceCastWorkspace,
    patches: []contact_cache.Patch,
    /// Full next-cache input used to merge an additional TOI without ending
    /// unrelated persistent contacts.
    merge_input: []contact_cache.Patch,
};

/// Rebuilds Task 08 SAP proxies directly from live World collider storage.
/// The caller owns proxy and SAP buffers, so this path performs no Tick heap
/// allocation and preserves the collider slot order before SAP canonicalizes.
pub fn rebuildBroadphase(world: *const world_mod.World, assets: *const store.Store, simulation: config.SimulationConfig, dt: fp.Fp, collider_views: []shapes.Collider, proxies: []broadphase.Proxy, buffers: *broadphase.Buffers, status: *fp.MathStatus) (world_mod.Error || broadphase.Error || shapes.Error)![]const broadphase.PairKey {
    const colliders = world.colliders orelse return error.InvalidCollider;
    var required: usize = 0;
    for (colliders.alive, 0..) |alive, index| {
        if (alive and colliders.enabled[index]) required += 1;
    }
    if (required > proxies.len or required > collider_views.len) return error.CapacityExceeded;
    var count: usize = 0;
    for (colliders.alive, 0..) |alive, index| {
        if (!alive or !colliders.enabled[index]) continue;
        const body_index = world.bodyIndex(colliders.body[index]) orelse return error.InvalidBody;
        const transform = compose(.{ .position = world.storage.position[body_index], .orientation = world.storage.orientation[body_index] }, colliders.local[index], status);
        const exact = try shapes.worldAabb(colliders.shape[index], assets, transform, status);
        collider_views[count] = colliderAt(&colliders, index);
        proxies[count] = .{ .id = .init(@intCast(index), colliders.generation[index]), .collider = &collider_views[count], .body_type = world.storage.body_type[body_index], .world_bounds = exact, .fat_bounds = broadphase.fatAabb(exact, simulation.tolerances.aabb_margin, status), .swept_bounds = broadphase.sweptAabb(exact, world.storage.linear_velocity[body_index], dt, status) };
        count += 1;
    }
    return broadphase.rebuild(proxies[0..count], buffers);
}

/// Task 23 production path: each logical job owns collider slots directly;
/// the main thread then compacts those slots in canonical collider order and
/// performs the order-sensitive SAP rebuild.
fn rebuildBroadphaseRanges(workspace: *Workspace, world: *const world_mod.World, assets: *const store.Store, simulation: config.SimulationConfig, dt: fp.Fp, collider_views: []shapes.Collider, proxies: []broadphase.Proxy, buffers: *broadphase.Buffers, status: *fp.MathStatus) Error![]const broadphase.PairKey {
    const colliders = world.colliders orelse return error.InvalidCollider;
    if (collider_views.len < colliders.alive.len or proxies.len < colliders.alive.len) return error.CapacityExceeded;
    const Build = struct {
        world: *const world_mod.World,
        colliders: *const world_mod.ColliderStorage,
        assets: *const store.Store,
        simulation: config.SimulationConfig,
        dt: fp.Fp,
        collider_views: []shapes.Collider,
        proxies: []broadphase.Proxy,

        fn run(self: *@This(), range: jobs.Range, job_status: *fp.MathStatus) Error!void {
            var index: usize = range.begin;
            while (index < range.end) : (index += 1) {
                if (!self.colliders.alive[index] or !self.colliders.enabled[index]) continue;
                const body_index = self.world.bodyIndex(self.colliders.body[index]) orelse return error.InvalidBody;
                const transform = compose(.{ .position = self.world.storage.position[body_index], .orientation = self.world.storage.orientation[body_index] }, self.colliders.local[index], job_status);
                const exact = shapes.worldAabb(self.colliders.shape[index], self.assets, transform, job_status) catch return error.InvalidShape;
                self.collider_views[index] = colliderAt(self.colliders, index);
                self.proxies[index] = .{
                    .id = .init(@intCast(index), self.colliders.generation[index]),
                    .collider = &self.collider_views[index],
                    .body_type = self.world.storage.body_type[body_index],
                    .world_bounds = exact,
                    .fat_bounds = broadphase.fatAabb(exact, self.simulation.tolerances.aabb_margin, job_status),
                    .swept_bounds = broadphase.sweptAabb(exact, self.world.storage.linear_velocity[body_index], self.dt, job_status),
                };
            }
        }
    };
    var build = Build{ .world = world, .colliders = &colliders, .assets = assets, .simulation = simulation, .dt = dt, .collider_views = collider_views, .proxies = proxies };
    try dispatchRanges(workspace, colliders.alive.len, &build, status, Build.run);
    var count: usize = 0;
    for (colliders.alive, 0..) |alive, index| {
        if (!alive or !colliders.enabled[index]) continue;
        if (count != index) {
            collider_views[count] = collider_views[index];
            proxies[count] = proxies[index];
        }
        proxies[count].collider = &collider_views[count];
        count += 1;
    }
    return broadphase.rebuild(proxies[0..count], buffers) catch |err| return switch (err) {
        error.PairCapacity, error.InsufficientEndpoints, error.InsufficientActive, error.ScratchLengthMismatch => error.CapacityExceeded,
        else => error.BroadphaseFailure,
    };
}

fn integratePositionRanges(workspace: *Workspace, world: *world_mod.World, awake: ?[]const bool, dt: fp.Fp, status: *fp.MathStatus) Error!void {
    const Integrate = struct {
        world: *world_mod.World,
        awake: ?[]const bool,
        dt: fp.Fp,
        fn run(self: *@This(), range: jobs.Range, job_status: *fp.MathStatus) Error!void {
            try self.world.integratePositionSlots(range.begin, range.end, self.awake, self.dt, job_status);
        }
    };
    var integrate = Integrate{ .world = world, .awake = awake, .dt = dt };
    try dispatchRanges(workspace, world.storage.alive.len, &integrate, status, Integrate.run);
}

/// Builds deterministic Task 19 items from live collider slots. Dynamic and
/// kinematic body velocity supplies the complete substep translation; static
/// colliders remain valid zero-delta targets. Capacity is checked before any
/// caller-visible item count is returned.
pub fn buildCcdItems(world: *const world_mod.World, dt: fp.Fp, workspace: *CcdItemWorkspace, status: *fp.MathStatus) world_mod.Error![]ccd.Item {
    const colliders = world.colliders orelse return error.InvalidCollider;
    if (workspace.enabled.len != colliders.alive.len) return error.CapacityExceeded;
    var required: usize = 0;
    for (colliders.alive, 0..) |alive, index| {
        if (alive and colliders.enabled[index]) required += 1;
    }
    if (required > workspace.items.len) return error.CapacityExceeded;
    var count: usize = 0;
    for (colliders.alive, 0..) |alive, index| {
        if (!alive or !colliders.enabled[index]) continue;
        const body = world.bodyIndex(colliders.body[index]) orelse return error.InvalidBody;
        const transform = compose(.{ .position = world.storage.position[body], .orientation = world.storage.orientation[body] }, colliders.local[index], status);
        const dynamic_motion = world.storage.body_type[body] != .static;
        workspace.items[count] = .{ .id = .init(@intCast(index), colliders.generation[index]), .shape = colliders.shape[index], .transform = transform, .delta = if (dynamic_motion) world.storage.linear_velocity[body].scale(dt, status) else .zero, .enabled = true, .ccd_enabled = workspace.enabled[index] };
        count += 1;
    }
    return workspace.items[0..count];
}

/// Builds a single canonical cache patch for a prepared TOI without mutating
/// the cache.  It is intentionally a separate preflight boundary: callers
/// must reserve cache/event/solver storage before advancing bodies to the
/// impact fraction.
pub fn buildToiPatch(world: *const world_mod.World, toi: ccd.Toi, output: []contact_cache.Patch, status: *fp.MathStatus) (world_mod.Error || error{CapacityExceeded})![]const contact_cache.Patch {
    if (output.len == 0) return error.CapacityExceeded;
    const caster = try colliderFor(world, toi.caster);
    const target = try colliderFor(world, toi.target);
    const ordered = colliderLess(toi.caster, toi.target);
    const a = if (ordered) caster.collider else target.collider;
    const b = if (ordered) target.collider else caster.collider;
    const normal = if (ordered) toi.normal else toi.normal.scale(fp.Fp.fromInt(-1), status);
    output[0] = .{
        .key = .{
            .collider_a = if (ordered) toi.caster else toi.target,
            .collider_b = if (ordered) toi.target else toi.caster,
            .path_a = if (ordered) toi.child_path else .{},
            .path_b = if (ordered) .{} else toi.child_path,
            .primitive_b = toi.primitive,
            .shape_revision_a = a.revision,
            .shape_revision_b = b.revision,
        },
        .normal = normal,
        .points = .{contact_cache.CachedPoint{ .feature_a = if (ordered) 0 else toi.feature, .feature_b = if (ordered) toi.feature else 0 }} ** 4,
        .len = 1,
        .sensor = a.sensor or b.sensor,
    };
    return output[0..1];
}

/// Performs the CCD scan and TOI cache-patch preflight for one remaining
/// substep interval.  Neither World state nor the persistent contact cache is
/// modified.  A returned hit can therefore be followed by a capacity failure
/// without violating the World pipeline's pre-commit rule.
pub fn prepareCcdToi(world: *const world_mod.World, dt: fp.Fp, workspace: *CcdPipelineWorkspace, status: *fp.MathStatus) Error!struct { prepared: ccd.Prepared, patches: []const contact_cache.Patch } {
    const items = try buildCcdItems(world, dt, workspace.items, status);
    const prepared = try ccd.prepare(items, workspace.pairs, workspace.assets, workspace.surface, status);
    const patches = if (prepared.toi) |toi| try buildToiPatch(world, toi, workspace.patches, status) else &.{};
    return .{ .prepared = prepared, .patches = patches };
}

/// Constructs the one-point solver contact for an already-published TOI
/// patch.  `patch` must be the persistent cache row after its merge so the
/// impulse written by the solver is retained for the remainder of the Tick.
pub fn buildToiSolverContact(world: *const world_mod.World, toi: ccd.Toi, patch: *contact_cache.Patch, workspace: *AnalyticSolverWorkspace) (world_mod.Error || error{ CapacityExceeded, InvalidContact })![]const contact_solver.Contact {
    if (workspace.contacts.len == 0 or workspace.points.len == 0 or workspace.restitution_bias.len == 0) return error.CapacityExceeded;
    if (patch.len != 1) return error.InvalidContact;
    const a = try colliderFor(world, patch.key.collider_a);
    const b = try colliderFor(world, patch.key.collider_b);
    workspace.points[0] = .{ .world_point = toi.point };
    workspace.restitution_bias[0] = .zero;
    workspace.contacts[0] = .{
        .body_a = a.collider.body,
        .body_b = b.collider.body,
        .friction_a = a.collider.material.friction,
        .friction_b = b.collider.material.friction,
        .restitution_a = a.collider.material.restitution,
        .restitution_b = b.collider.material.restitution,
        .points = workspace.points[0..1],
        .restitution_bias = workspace.restitution_bias[0..1],
        .patch = patch,
    };
    return workspace.contacts[0..1];
}

/// Adds a prepared TOI patch to the complete next-cache input. Existing
/// patches remain present and are therefore persisted, rather than being
/// spuriously ended by Task 12's full-snapshot merge contract.
pub fn addToiCache(cache: *contact_cache.Cache, patch: contact_cache.Patch, output: []contact_cache.Patch) error{CapacityExceeded}![]const contact_cache.Patch {
    if (cache.len >= output.len) return error.CapacityExceeded;
    var count: usize = 0;
    var replaced = false;
    for (cache.active()) |old| {
        if (keyEqual(old.key, patch.key)) {
            output[count] = patch;
            replaced = true;
        } else output[count] = old;
        count += 1;
    }
    if (!replaced) {
        output[count] = patch;
        count += 1;
    }
    var i: usize = 1;
    while (i < count) : (i += 1) {
        const value = output[i];
        var at = i;
        while (at > 0 and contact_cache.keyLess(value.key, output[at - 1].key)) : (at -= 1) output[at] = output[at - 1];
        output[at] = value;
    }
    return output[0..count];
}

/// Resolves one substep's continuous motion. Every iteration scans the
/// current remaining interval, validates the cache/solver transaction, moves
/// World state to the earliest TOI, solves that contact, then rebuilds the
/// sweep from the new velocities. No smaller-time-step approximation is used.
fn resolveCcdSubstep(world: *world_mod.World, simulation: config.SimulationConfig, dt: fp.Fp, awake: ?[]const bool, ccd_workspace: *CcdPipelineWorkspace, contacts: *AnalyticContactWorkspace, solver: *AnalyticSolverWorkspace, joint_rows: []constraints.ConstraintRow, status: *fp.MathStatus) Error!ccd.Cursor {
    var cursor = ccd.Cursor{};
    const maximum: usize = @intCast(simulation.iterations.ccd_toi_per_substep);
    while (cursor.remaining.raw > 0) {
        const remaining_dt = dt.mul(cursor.remaining, status);
        const prepared = try prepareCcdToi(world, remaining_dt, ccd_workspace, status);
        if (prepared.prepared.fault != .none) return error.CcdFault;
        const toi = prepared.prepared.toi orelse {
            try integrateCcdPositions(world, awake, remaining_dt, status);
            cursor.elapsed = .one;
            cursor.remaining = .zero;
            return cursor;
        };
        // A cast may report the already-solved touching contact at local zero
        // on the next scan. If the full point velocity is no longer closing,
        // it is not a new TOI: consume the remainder with the solved state
        // instead of burning the deterministic TOI budget on zero progress.
        if (toi.fraction.raw == 0 and !toiClosing(world, toi, status)) {
            try integrateCcdPositions(world, awake, remaining_dt, status);
            cursor.elapsed = .one;
            cursor.remaining = .zero;
            return cursor;
        }
        if (cursor.processed >= maximum) return error.CcdFault;
        const patches = try addToiCache(contacts.cache, prepared.patches[0], ccd_workspace.merge_input);
        // All fallible cache/solver capacity checks have now happened while
        // World transforms are still at the start of this TOI interval.
        if (solver.contacts.len == 0 or solver.points.len == 0 or solver.restitution_bias.len == 0) return error.CapacityExceeded;
        const toi_dt = remaining_dt.mul(toi.fraction, status);
        try integrateCcdPositions(world, awake, toi_dt, status);
        _ = contact_cache.merge(contacts.cache, patches, .{ .next = contacts.cache_next, .events = contacts.events }, simulation.tolerances.warmstart_normal_cos_min, status) catch return error.CapacityExceeded;
        const patch = findCachePatch(contacts.cache, prepared.patches[0].key) orelse return error.InvalidContact;
        if (!patch.sensor) {
            const toi_contact = try buildToiSolverContact(world, toi, patch, solver);
            try contact_solver.solveAdditionalContactWithJointRows(world, joint_rows, toi_contact, solver.pseudo, solverSettings(simulation), status);
        }
        try ccd.advance(&cursor, toi.fraction, maximum, status);
        if (cursor.fault != .none) return error.CcdFault;
    }
    return cursor;
}
fn integrateCcdPositions(world: *world_mod.World, awake: ?[]const bool, dt: fp.Fp, status: *fp.MathStatus) world_mod.Error!void {
    if (awake) |mask| try world.integratePositionsAwake(mask, dt, status) else world.integratePositions(dt, status);
}

/// Produces canonical one-point patches for supported analytic primitive
/// pairs. Pair order is already canonical from SAP, and unsupported shapes
/// are intentionally skipped for the later GJK/mesh producer rather than
/// being approximated here. `output` is scratch, never persistent cache data.
pub fn narrowAnalyticPairs(world: *const world_mod.World, pairs: []const broadphase.PairKey, output: []contact_cache.Patch, status: *fp.MathStatus) (world_mod.Error || error{CapacityExceeded})![]const contact_cache.Patch {
    var count: usize = 0;
    for (pairs) |pair| {
        const a = try colliderFor(world, pair.a);
        const b = try colliderFor(world, pair.b);
        const a_body = world.bodyIndex(a.collider.body) orelse return error.InvalidBody;
        const b_body = world.bodyIndex(b.collider.body) orelse return error.InvalidBody;
        const a_transform = compose(.{ .position = world.storage.position[a_body], .orientation = world.storage.orientation[a_body] }, a.collider.local, status);
        const b_transform = compose(.{ .position = world.storage.position[b_body], .orientation = world.storage.orientation[b_body] }, b.collider.local, status);
        const a_shape = analyticShape(a.collider.shape, a_transform, status) orelse continue;
        const b_shape = analyticShape(b.collider.shape, b_transform, status) orelse continue;
        const relative_velocity = world.storage.linear_velocity[b_body].sub(world.storage.linear_velocity[a_body], status);
        const hit = analytic.collide(a_shape, b_shape, relative_velocity, pair.a, pair.b, status) orelse continue;
        if (hit.separation.raw > 0) continue;
        if (count == output.len) return error.CapacityExceeded;
        const point = contact_cache.CachedPoint{ .feature_a = analyticFeature(hit.feature_a), .feature_b = analyticFeature(hit.feature_b) };
        output[count] = .{ .key = .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, .normal = hit.normal, .points = .{point} ** 4, .len = 1, .sensor = a.collider.sensor or b.collider.sensor };
        count += 1;
    }
    return output[0..count];
}

/// Converts a Task 10 result into cache patches without discarding Compound
/// child paths. A GJK compound result can contain contacts for several child
/// pairs; each child-pair therefore receives its own cache key and warm-start
/// lifetime. The caller owns `output`, so capacity failures never publish
/// partial persistent cache state.
pub fn cachePatchesFromConvexResult(result: gjk.ConvexResult, base_key: contact_cache.ManifoldKey, normal: geometry.Vec3, sensor: bool, output: []contact_cache.Patch) error{CapacityExceeded}![]const contact_cache.Patch {
    var required: usize = 0;
    for (result.patch.points[0..result.patch.len], 0..) |point, point_index| {
        // Count a path group only at its first occurrence. The small patch
        // bound keeps this deterministic preflight simpler than scratch maps.
        var first = true;
        for (result.patch.points[0..point_index]) |prior| {
            if (prior.path_a.len == point.path_a.len and prior.path_b.len == point.path_b.len and std.mem.eql(u32, prior.path_a.values[0..prior.path_a.len], point.path_a.values[0..point.path_a.len]) and std.mem.eql(u32, prior.path_b.values[0..prior.path_b.len], point.path_b.values[0..point.path_b.len])) {
                first = false;
                break;
            }
        }
        if (first) required += 1;
    }
    if (required > output.len) return error.CapacityExceeded;
    var count: usize = 0;
    for (result.patch.points[0..result.patch.len]) |point| {
        var key = base_key;
        key.path_a = point.path_a;
        key.path_b = point.path_b;
        var index: ?usize = null;
        for (output[0..count], 0..) |patch, i| if (!contact_cache.keyLess(patch.key, key) and !contact_cache.keyLess(key, patch.key)) {
            index = i;
            break;
        };
        const at = index orelse blk: {
            output[count] = .{ .key = key, .normal = normal, .sensor = sensor };
            const value = count;
            count += 1;
            break :blk value;
        };
        if (output[at].len == 4) return error.CapacityExceeded;
        output[at].points[output[at].len] = .{ .feature_a = point.feature_a, .feature_b = point.feature_b };
        output[at].len += 1;
    }
    // Compound traversal follows asset order, not cache-key order. Canonical
    // merge requires the latter, so sort the small fixed result in place.
    var i: usize = 1;
    while (i < count) : (i += 1) {
        const value = output[i];
        var at = i;
        while (at > 0 and contact_cache.keyLess(value.key, output[at - 1].key)) : (at -= 1) output[at] = output[at - 1];
        output[at] = value;
    }
    return output[0..count];
}

/// Converts one reduced Task 11 surface patch into its persistent form. The
/// public collider order is always SAP order, while surface queries may have
/// been executed with the surface as A; `swapped` restores the A-to-B normal
/// and feature identity before cache merge/warm-starting observes the patch.
pub fn cachePatchFromSurfaceResult(surface: gjk.ContactPatch, base_key: contact_cache.ManifoldKey, sensor: bool, swapped: bool, status: *fp.MathStatus) error{CapacityExceeded}!contact_cache.Patch {
    if (surface.len == 0) return error.CapacityExceeded;
    const source_normal = surface.points[0].normal;
    var result = contact_cache.Patch{ .key = base_key, .normal = if (swapped) source_normal.scale(fp.Fp.one.neg(status), status) else source_normal, .sensor = sensor };
    for (surface.points[0..surface.len]) |point| {
        if (result.len == 4) return error.CapacityExceeded;
        result.points[result.len] = if (swapped) .{ .feature_a = point.feature_b, .feature_b = point.feature_a } else .{ .feature_a = point.feature_a, .feature_b = point.feature_b };
        result.len += 1;
    }
    return result;
}

/// Surface Compound traversal can produce contacts belonging to several child
/// paths. Preserve those paths as independent cache lifetimes, exactly like
/// the convex manifold bridge, rather than collapsing unrelated children into
/// one warm-start patch.
pub fn cachePatchesFromSurfaceResult(surface: gjk.ContactPatch, base_key: contact_cache.ManifoldKey, sensor: bool, swapped: bool, output: []contact_cache.Patch, status: *fp.MathStatus) error{CapacityExceeded}![]const contact_cache.Patch {
    var required: usize = 0;
    for (surface.points[0..surface.len], 0..) |point, point_index| {
        const path_a = if (swapped) point.path_b else point.path_a;
        const path_b = if (swapped) point.path_a else point.path_b;
        var first = true;
        for (surface.points[0..point_index]) |prior| {
            const prior_a = if (swapped) prior.path_b else prior.path_a;
            const prior_b = if (swapped) prior.path_a else prior.path_b;
            if (pathsEqual(path_a, prior_a) and pathsEqual(path_b, prior_b)) {
                first = false;
                break;
            }
        }
        if (first) required += 1;
    }
    if (required > output.len) return error.CapacityExceeded;
    var count: usize = 0;
    for (surface.points[0..surface.len]) |point| {
        var key = base_key;
        key.path_a = if (swapped) point.path_b else point.path_a;
        key.path_b = if (swapped) point.path_a else point.path_b;
        var index: ?usize = null;
        for (output[0..count], 0..) |patch, i| if (!contact_cache.keyLess(patch.key, key) and !contact_cache.keyLess(key, patch.key)) {
            index = i;
            break;
        };
        const at = index orelse blk: {
            output[count] = .{ .key = key, .normal = if (swapped) point.normal.scale(fp.Fp.one.neg(status), status) else point.normal, .sensor = sensor };
            const value = count;
            count += 1;
            break :blk value;
        };
        if (output[at].len == 4) return error.CapacityExceeded;
        output[at].points[output[at].len] = if (swapped) .{ .feature_a = point.feature_b, .feature_b = point.feature_a } else .{ .feature_a = point.feature_a, .feature_b = point.feature_b };
        output[at].len += 1;
    }
    var i: usize = 1;
    while (i < count) : (i += 1) {
        const value = output[i];
        var at = i;
        while (at > 0 and contact_cache.keyLess(value.key, output[at - 1].key)) : (at -= 1) output[at] = output[at - 1];
        output[at] = value;
    }
    return output[0..count];
}

/// Extends the primitive producer with Task 10 convex-hull pairs. Mesh,
/// HeightField and Compound surface dispatch remain intentionally separate:
/// their triangle/child traversal must not be replaced by a whole-shape
/// support query. A convex hull pair without the required caller workspace is
/// a deterministic capacity/configuration fault, never a silent miss.
pub fn narrowRuntimePairs(world: *const world_mod.World, assets: *const store.Store, pairs: []const broadphase.PairKey, convex: ?*ConvexNarrowWorkspace, surface: ?*SurfaceNarrowWorkspace, output: []contact_cache.Patch, status: *fp.MathStatus) anyerror![]const contact_cache.Patch {
    return narrowRuntimePairsTracked(world, assets, pairs, convex, surface, output, status, null);
}

/// The public producer remains side-effect free. The formal World pipeline
/// supplies `fault_object` so a rejected pair can be attributed without
/// changing the zero-allocation patch result or standalone API.
fn narrowRuntimePairsTracked(world: *const world_mod.World, assets: *const store.Store, pairs: []const broadphase.PairKey, convex: ?*ConvexNarrowWorkspace, surface: ?*SurfaceNarrowWorkspace, output: []contact_cache.Patch, status: *fp.MathStatus, fault_object: ?*?u64) anyerror![]const contact_cache.Patch {
    var count: usize = 0;
    for (pairs) |pair| {
        // PairKey is canonical, so this is a stable representative when a
        // pair-level producer has no narrower single-owner attribution.
        if (fault_object) |object| object.* = pair.a.value;
        const a = try colliderFor(world, pair.a);
        const b = try colliderFor(world, pair.b);
        const a_body = world.bodyIndex(a.collider.body) orelse return error.InvalidBody;
        const b_body = world.bodyIndex(b.collider.body) orelse return error.InvalidBody;
        const a_transform = compose(.{ .position = world.storage.position[a_body], .orientation = world.storage.orientation[a_body] }, a.collider.local, status);
        const b_transform = compose(.{ .position = world.storage.position[b_body], .orientation = world.storage.orientation[b_body] }, b.collider.local, status);
        const a_analytic = analyticShape(a.collider.shape, a_transform, status);
        const b_analytic = analyticShape(b.collider.shape, b_transform, status);
        if (a_analytic != null and b_analytic != null) {
            const relative_velocity = world.storage.linear_velocity[b_body].sub(world.storage.linear_velocity[a_body], status);
            const hit = analytic.collide(a_analytic.?, b_analytic.?, relative_velocity, pair.a, pair.b, status) orelse continue;
            if (hit.separation.raw > 0) continue;
            if (count == output.len) return error.CapacityExceeded;
            const point = contact_cache.CachedPoint{ .feature_a = analyticFeature(hit.feature_a), .feature_b = analyticFeature(hit.feature_b) };
            output[count] = .{ .key = .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, .normal = hit.normal, .points = .{point} ** 4, .len = 1, .sensor = a.collider.sensor or b.collider.sensor };
            count += 1;
            continue;
        }
        if (a.collider.shape == .sphere and b.collider.shape == .triangle_mesh) {
            const workspace = surface orelse return error.InvalidContact;
            const sphere_mesh = workspace.sphere_mesh orelse return error.InvalidContact;
            const source = b.collider.shape.triangle_mesh;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const patch = try mesh.sphereMeshPatchTransformed(view, b_transform, .{ .center = a_transform.position, .radius = a.collider.shape.sphere.radius }, sphere_mesh, status);
            if (patch.len == 0) continue;
            if (count == output.len) return error.CapacityExceeded;
            output[count] = try cachePatchFromSurfaceResult(patch, .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, a.collider.sensor or b.collider.sensor, false, status);
            count += 1;
            continue;
        }
        if (a.collider.shape == .triangle_mesh and b.collider.shape == .sphere) {
            const workspace = surface orelse return error.InvalidContact;
            const sphere_mesh = workspace.sphere_mesh orelse return error.InvalidContact;
            const source = a.collider.shape.triangle_mesh;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const patch = try mesh.sphereMeshPatchTransformed(view, a_transform, .{ .center = b_transform.position, .radius = b.collider.shape.sphere.radius }, sphere_mesh, status);
            if (patch.len == 0) continue;
            if (count == output.len) return error.CapacityExceeded;
            output[count] = try cachePatchFromSurfaceResult(patch, .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, a.collider.sensor or b.collider.sensor, true, status);
            count += 1;
            continue;
        }
        if (a.collider.shape == .capsule and b.collider.shape == .triangle_mesh) {
            const workspace = surface orelse return error.InvalidContact;
            const sphere_mesh = workspace.sphere_mesh orelse return error.InvalidContact;
            const source = b.collider.shape.triangle_mesh;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const value = a.collider.shape.capsule;
            const patch = try mesh.capsuleMeshPatchTransformed(view, b_transform, .{ .segment = .{ .a = a_transform.apply(.{ .y = value.half_height.neg(status) }, status), .b = a_transform.apply(.{ .y = value.half_height }, status) }, .radius = value.radius }, sphere_mesh, status);
            if (patch.len == 0) continue;
            if (count == output.len) return error.CapacityExceeded;
            output[count] = try cachePatchFromSurfaceResult(patch, .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, a.collider.sensor or b.collider.sensor, false, status);
            count += 1;
            continue;
        }
        if (a.collider.shape == .triangle_mesh and b.collider.shape == .capsule) {
            const workspace = surface orelse return error.InvalidContact;
            const sphere_mesh = workspace.sphere_mesh orelse return error.InvalidContact;
            const source = a.collider.shape.triangle_mesh;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const value = b.collider.shape.capsule;
            const patch = try mesh.capsuleMeshPatchTransformed(view, a_transform, .{ .segment = .{ .a = b_transform.apply(.{ .y = value.half_height.neg(status) }, status), .b = b_transform.apply(.{ .y = value.half_height }, status) }, .radius = value.radius }, sphere_mesh, status);
            if (patch.len == 0) continue;
            if (count == output.len) return error.CapacityExceeded;
            output[count] = try cachePatchFromSurfaceResult(patch, .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, a.collider.sensor or b.collider.sensor, true, status);
            count += 1;
            continue;
        }
        if (a.collider.shape == .sphere and b.collider.shape == .height_field) {
            const workspace = surface orelse return error.InvalidContact;
            const sphere_heightfield = workspace.sphere_heightfield orelse return error.InvalidContact;
            const source = b.collider.shape.height_field;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const patch = try mesh.sphereHeightfieldPatchTransformed(view, b_transform, .{ .center = a_transform.position, .radius = a.collider.shape.sphere.radius }, sphere_heightfield, status);
            if (patch.len == 0) continue;
            if (count == output.len) return error.CapacityExceeded;
            output[count] = try cachePatchFromSurfaceResult(patch, .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, a.collider.sensor or b.collider.sensor, false, status);
            count += 1;
            continue;
        }
        if (a.collider.shape == .height_field and b.collider.shape == .sphere) {
            const workspace = surface orelse return error.InvalidContact;
            const sphere_heightfield = workspace.sphere_heightfield orelse return error.InvalidContact;
            const source = a.collider.shape.height_field;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const patch = try mesh.sphereHeightfieldPatchTransformed(view, a_transform, .{ .center = b_transform.position, .radius = b.collider.shape.sphere.radius }, sphere_heightfield, status);
            if (patch.len == 0) continue;
            if (count == output.len) return error.CapacityExceeded;
            output[count] = try cachePatchFromSurfaceResult(patch, .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, a.collider.sensor or b.collider.sensor, true, status);
            count += 1;
            continue;
        }
        if (isConvexSurfaceCaster(a.collider.shape) and b.collider.shape == .triangle_mesh) {
            const workspace = surface orelse return error.InvalidContact;
            const convex_mesh = workspace.convex_mesh orelse return error.InvalidContact;
            const source = b.collider.shape.triangle_mesh;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const patch = try mesh.convexMeshPatch(a.collider.shape, assets, a_transform, view, b_transform, convex_mesh, status);
            if (patch.len == 0) continue;
            if (count == output.len) return error.CapacityExceeded;
            output[count] = try cachePatchFromSurfaceResult(patch, .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, a.collider.sensor or b.collider.sensor, false, status);
            count += 1;
            continue;
        }
        if (a.collider.shape == .triangle_mesh and isConvexSurfaceCaster(b.collider.shape)) {
            const workspace = surface orelse return error.InvalidContact;
            const convex_mesh = workspace.convex_mesh orelse return error.InvalidContact;
            const source = a.collider.shape.triangle_mesh;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const patch = try mesh.convexMeshPatch(b.collider.shape, assets, b_transform, view, a_transform, convex_mesh, status);
            if (patch.len == 0) continue;
            if (count == output.len) return error.CapacityExceeded;
            output[count] = try cachePatchFromSurfaceResult(patch, .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, a.collider.sensor or b.collider.sensor, true, status);
            count += 1;
            continue;
        }
        if (isConvexSurfaceCaster(a.collider.shape) and b.collider.shape == .height_field) {
            const workspace = surface orelse return error.InvalidContact;
            const convex_heightfield = workspace.convex_heightfield orelse return error.InvalidContact;
            const source = b.collider.shape.height_field;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const patch = try mesh.convexHeightfieldPatch(a.collider.shape, assets, a_transform, view, b_transform, convex_heightfield, status);
            if (patch.len == 0) continue;
            if (count == output.len) return error.CapacityExceeded;
            output[count] = try cachePatchFromSurfaceResult(patch, .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, a.collider.sensor or b.collider.sensor, false, status);
            count += 1;
            continue;
        }
        if (a.collider.shape == .height_field and isConvexSurfaceCaster(b.collider.shape)) {
            const workspace = surface orelse return error.InvalidContact;
            const convex_heightfield = workspace.convex_heightfield orelse return error.InvalidContact;
            const source = a.collider.shape.height_field;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const patch = try mesh.convexHeightfieldPatch(b.collider.shape, assets, b_transform, view, a_transform, convex_heightfield, status);
            if (patch.len == 0) continue;
            if (count == output.len) return error.CapacityExceeded;
            output[count] = try cachePatchFromSurfaceResult(patch, .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, a.collider.sensor or b.collider.sensor, true, status);
            count += 1;
            continue;
        }
        if (a.collider.shape == .triangle_mesh and b.collider.shape == .triangle_mesh) {
            const workspace = surface orelse return error.InvalidContact;
            const mesh_mesh = workspace.mesh_mesh orelse return error.InvalidContact;
            const source_a = a.collider.shape.triangle_mesh;
            const source_b = b.collider.shape.triangle_mesh;
            const view_a = try runtime_view.find(assets, if (source_a.source_id != 0) source_a.source_id else source_a.asset.index());
            const view_b = try runtime_view.find(assets, if (source_b.source_id != 0) source_b.source_id else source_b.asset.index());
            const patch = try mesh.meshMeshPatchTransformed(view_a, a_transform, view_b, b_transform, mesh_mesh, status);
            if (patch.len == 0) continue;
            if (count == output.len) return error.CapacityExceeded;
            output[count] = try cachePatchFromSurfaceResult(patch, .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, a.collider.sensor or b.collider.sensor, false, status);
            count += 1;
            continue;
        }
        if (a.collider.shape == .sphere and b.collider.shape == .compound) {
            const workspace = surface orelse return error.InvalidContact;
            const sphere_compound = workspace.sphere_compound orelse return error.InvalidContact;
            const patch = try mesh.sphereCompoundSurfacePatch(b.collider.shape, assets, b_transform, .{ .center = a_transform.position, .radius = a.collider.shape.sphere.radius }, sphere_compound, status);
            if (patch.len == 0) continue;
            const converted = try cachePatchesFromSurfaceResult(patch, .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, a.collider.sensor or b.collider.sensor, false, output[count..], status);
            count += converted.len;
            continue;
        }
        if (a.collider.shape == .compound and b.collider.shape == .sphere) {
            const workspace = surface orelse return error.InvalidContact;
            const sphere_compound = workspace.sphere_compound orelse return error.InvalidContact;
            const patch = try mesh.sphereCompoundSurfacePatch(a.collider.shape, assets, a_transform, .{ .center = b_transform.position, .radius = b.collider.shape.sphere.radius }, sphere_compound, status);
            if (patch.len == 0) continue;
            const converted = try cachePatchesFromSurfaceResult(patch, .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, a.collider.sensor or b.collider.sensor, true, output[count..], status);
            count += converted.len;
            continue;
        }
        if (isConvexSurfaceCaster(a.collider.shape) and b.collider.shape == .compound) {
            const workspace = surface orelse return error.InvalidContact;
            const convex_compound = workspace.convex_compound orelse return error.InvalidContact;
            const surface_patch = mesh.convexCompoundSurfacePatch(b.collider.shape, assets, b_transform, a.collider.shape, a_transform, convex_compound, status) catch |err| switch (err) {
                error.UnsupportedShape => null,
                else => return err,
            };
            if (surface_patch) |patch| {
                if (patch.len == 0) continue;
                const converted = try cachePatchesFromSurfaceResult(patch, .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, a.collider.sensor or b.collider.sensor, false, output[count..], status);
                count += converted.len;
                continue;
            }
        }
        if (a.collider.shape == .compound and isConvexSurfaceCaster(b.collider.shape)) {
            const workspace = surface orelse return error.InvalidContact;
            const convex_compound = workspace.convex_compound orelse return error.InvalidContact;
            const surface_patch = mesh.convexCompoundSurfacePatch(a.collider.shape, assets, a_transform, b.collider.shape, b_transform, convex_compound, status) catch |err| switch (err) {
                error.UnsupportedShape => null,
                else => return err,
            };
            if (surface_patch) |patch| {
                if (patch.len == 0) continue;
                const converted = try cachePatchesFromSurfaceResult(patch, .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, a.collider.sensor or b.collider.sensor, true, output[count..], status);
                count += converted.len;
                continue;
            }
        }
        if (a.collider.shape == .triangle_mesh or a.collider.shape == .height_field or b.collider.shape == .triangle_mesh or b.collider.shape == .height_field) return error.UnsupportedShape;
        const convex_workspace = convex orelse return error.InvalidContact;
        const direction = b_transform.position.sub(a_transform.position, status);
        var shape_pair = gjk.ShapePairContext{ .shape_a = a.collider.shape, .shape_b = b.collider.shape, .assets = assets, .transform_a = a_transform, .transform_b = b_transform };
        const result = try gjk.collideShapes(&shape_pair, if (direction.lengthSquared(status).raw == 0) geometry.Vec3.unit_x else direction, convex_workspace.manifold, status);
        if (result.gjk.status != .intersecting or result.patch.len == 0) continue;
        const normal = if (result.epa) |epa| epa.normal else result.gjk.direction;
        const converted = try cachePatchesFromConvexResult(result, .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, normal, a.collider.sensor or b.collider.sensor, output[count..]);
        count += converted.len;
    }
    var i: usize = 1;
    while (i < count) : (i += 1) {
        const value = output[i];
        var at = i;
        while (at > 0 and contact_cache.keyLess(value.key, output[at - 1].key)) : (at -= 1) output[at] = output[at - 1];
        output[at] = value;
    }
    return output[0..count];
}

const parallel_narrow_sentinel: u8 = std.math.maxInt(u8);
const rejected_primitive_pair = mesh.PrimitivePair{ .a = std.math.maxInt(u32), .b = std.math.maxInt(u32) };

/// Mesh topology traversal freezes one canonical primitive-pair list on the
/// caller. Workers then classify fixed candidate slots, the caller compacts in
/// candidate order, and a second fixed-slot pass fills exact contacts.
pub fn meshMeshPatchRanges(workspace: *Workspace, view_a: runtime_view.View, transform_a: geometry.Transform3, view_b: runtime_view.View, transform_b: geometry.Transform3, mesh_workspace: mesh.MeshMeshPatchWorkspace, status: *fp.MathStatus) anyerror!gjk.ContactPatch {
    const candidates = try mesh.meshMeshCandidatesTransformed(view_a, transform_a, view_b, transform_b, mesh_workspace.query, status);
    if (candidates.len > mesh_workspace.query.pair_scratch.len) return error.CapacityExceeded;
    const Classify = struct {
        view_a: runtime_view.View,
        transform_a: geometry.Transform3,
        view_b: runtime_view.View,
        transform_b: geometry.Transform3,
        candidates: []const mesh.PrimitivePair,
        staging: []mesh.PrimitivePair,

        fn run(self: *@This(), range: jobs.Range, job_status: *fp.MathStatus) Error!void {
            var index: usize = range.begin;
            while (index < range.end) : (index += 1) {
                const pair = self.candidates[index];
                self.staging[index] = if (try mesh.meshTrianglePairOverlaps(self.view_a, self.transform_a, self.view_b, self.transform_b, pair, job_status)) pair else rejected_primitive_pair;
            }
        }
    };
    var classify = Classify{ .view_a = view_a, .transform_a = transform_a, .view_b = view_b, .transform_b = transform_b, .candidates = candidates, .staging = mesh_workspace.query.pair_scratch };
    try dispatchRanges(workspace, candidates.len, &classify, status, Classify.run);

    var overlap_count: usize = 0;
    for (mesh_workspace.query.pair_scratch[0..candidates.len]) |pair| if (pair.a != rejected_primitive_pair.a or pair.b != rejected_primitive_pair.b) {
        if (overlap_count == mesh_workspace.query.overlaps.len) return error.CapacityExceeded;
        mesh_workspace.query.overlaps[overlap_count] = pair;
        overlap_count += 1;
    };
    if (overlap_count > mesh_workspace.contacts.len) return error.CapacityExceeded;

    const Fill = struct {
        view_a: runtime_view.View,
        transform_a: geometry.Transform3,
        view_b: runtime_view.View,
        transform_b: geometry.Transform3,
        overlaps: []const mesh.PrimitivePair,
        contacts: []gjk.ContactPoint,

        fn run(self: *@This(), range: jobs.Range, job_status: *fp.MathStatus) Error!void {
            var index: usize = range.begin;
            while (index < range.end) : (index += 1) self.contacts[index] = try mesh.meshTrianglePairContact(self.view_a, self.transform_a, self.view_b, self.transform_b, self.overlaps[index], job_status);
        }
    };
    var fill = Fill{ .view_a = view_a, .transform_a = transform_a, .view_b = view_b, .transform_b = transform_b, .overlaps = mesh_workspace.query.overlaps[0..overlap_count], .contacts = mesh_workspace.contacts };
    try dispatchRanges(workspace, overlap_count, &fill, status, Fill.run);
    return gjk.reducePatch(mesh_workspace.contacts[0..overlap_count], status);
}

/// Parallel analytic-pair front end. One fixed staging slot belongs to each
/// broadphase pair; complex pairs retain the sentinel and are evaluated later
/// in canonical pair order using their shared bounded GJK/mesh workspaces.
fn narrowRuntimePairsRanges(workspace: *Workspace, world: *const world_mod.World, assets: *const store.Store, pairs: []const broadphase.PairKey, convex: ?*ConvexNarrowWorkspace, surface: ?*SurfaceNarrowWorkspace, staging: []contact_cache.Patch, output: []contact_cache.Patch, status: *fp.MathStatus, fault_object: ?*?u64) anyerror![]const contact_cache.Patch {
    if (staging.len < pairs.len) return error.CapacityExceeded;
    const Prepare = struct {
        world: *const world_mod.World,
        pairs: []const broadphase.PairKey,
        staging: []contact_cache.Patch,

        fn run(self: *@This(), range: jobs.Range, _: *fp.MathStatus) Error!void {
            var pair_index: usize = range.begin;
            while (pair_index < range.end) : (pair_index += 1) {
                const pair = self.pairs[pair_index];
                self.staging[pair_index] = .{ .key = .{ .collider_a = pair.a, .collider_b = pair.b }, .normal = .zero, .len = parallel_narrow_sentinel };
                var pair_status = fp.MathStatus{};
                const a = colliderFor(self.world, pair.a) catch return error.InvalidCollider;
                const b = colliderFor(self.world, pair.b) catch return error.InvalidCollider;
                const a_body = self.world.bodyIndex(a.collider.body) orelse return error.InvalidBody;
                const b_body = self.world.bodyIndex(b.collider.body) orelse return error.InvalidBody;
                const a_transform = compose(.{ .position = self.world.storage.position[a_body], .orientation = self.world.storage.orientation[a_body] }, a.collider.local, &pair_status);
                const b_transform = compose(.{ .position = self.world.storage.position[b_body], .orientation = self.world.storage.orientation[b_body] }, b.collider.local, &pair_status);
                const a_analytic = analyticShape(a.collider.shape, a_transform, &pair_status);
                const b_analytic = analyticShape(b.collider.shape, b_transform, &pair_status);
                if (a_analytic == null or b_analytic == null) continue;
                var slot = contact_cache.Patch{
                    .key = .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision },
                    .normal = .zero,
                };
                const relative_velocity = self.world.storage.linear_velocity[b_body].sub(self.world.storage.linear_velocity[a_body], &pair_status);
                if (analytic.collide(a_analytic.?, b_analytic.?, relative_velocity, pair.a, pair.b, &pair_status)) |hit| {
                    if (hit.separation.raw <= 0) {
                        const point = contact_cache.CachedPoint{ .feature_a = analyticFeature(hit.feature_a), .feature_b = analyticFeature(hit.feature_b) };
                        slot.normal = hit.normal;
                        slot.points = .{point} ** 4;
                        slot.len = 1;
                        slot.sensor = a.collider.sensor or b.collider.sensor;
                    }
                }
                // These primitive fields are zero for analytic manifolds. Use
                // them only inside staging to retain handled/fault metadata.
                slot.key.primitive_a = @intFromEnum(pair_status.fault);
                slot.key.primitive_b = std.math.maxInt(u32);
                self.staging[pair_index] = slot;
            }
        }
    };
    var prepare = Prepare{ .world = world, .pairs = pairs, .staging = staging };
    var ignored_status = fp.MathStatus{};
    try dispatchRanges(workspace, pairs.len, &prepare, &ignored_status, Prepare.run);

    var count: usize = 0;
    for (pairs, 0..) |pair, pair_index| {
        const slot = &staging[pair_index];
        if (slot.len == parallel_narrow_sentinel) {
            if (fault_object) |object| object.* = pair.a.value;
            const a = try colliderFor(world, pair.a);
            const b = try colliderFor(world, pair.b);
            if (a.collider.shape == .triangle_mesh and b.collider.shape == .triangle_mesh) {
                const surface_workspace = surface orelse return error.InvalidContact;
                const mesh_workspace = surface_workspace.mesh_mesh orelse return error.InvalidContact;
                const a_body = world.bodyIndex(a.collider.body) orelse return error.InvalidBody;
                const b_body = world.bodyIndex(b.collider.body) orelse return error.InvalidBody;
                const a_transform = compose(.{ .position = world.storage.position[a_body], .orientation = world.storage.orientation[a_body] }, a.collider.local, status);
                const b_transform = compose(.{ .position = world.storage.position[b_body], .orientation = world.storage.orientation[b_body] }, b.collider.local, status);
                const source_a = a.collider.shape.triangle_mesh;
                const source_b = b.collider.shape.triangle_mesh;
                const view_a = try runtime_view.find(assets, if (source_a.source_id != 0) source_a.source_id else source_a.asset.index());
                const view_b = try runtime_view.find(assets, if (source_b.source_id != 0) source_b.source_id else source_b.asset.index());
                const patch = try meshMeshPatchRanges(workspace, view_a, a_transform, view_b, b_transform, mesh_workspace, status);
                if (patch.len == 0) continue;
                if (count == output.len) return error.CapacityExceeded;
                output[count] = try cachePatchFromSurfaceResult(patch, .{ .collider_a = pair.a, .collider_b = pair.b, .shape_revision_a = a.collider.revision, .shape_revision_b = b.collider.revision }, a.collider.sensor or b.collider.sensor, false, status);
                count += 1;
                continue;
            }
            const produced = try narrowRuntimePairsTracked(world, assets, pairs[pair_index .. pair_index + 1], convex, surface, output[count..], status, fault_object);
            count += produced.len;
            continue;
        }
        if (status.fault == .none) status.fault = std.enums.fromInt(fp.MathFault, slot.key.primitive_a) orelse .none;
        if (slot.len == 0) continue;
        if (count == output.len) return error.CapacityExceeded;
        output[count] = slot.*;
        output[count].key.primitive_a = 0;
        output[count].key.primitive_b = 0;
        count += 1;
    }
    return output[0..count];
}

/// Reconstructs exact primitive or GJK witnesses for persistent cache patches
/// and builds canonical Task 15 solver contacts. Sensor patches are
/// deliberately excluded from impulse solving but remain in the contact cache
/// and events.
pub fn buildAnalyticSolverContacts(world: *const world_mod.World, assets: *const store.Store, cache: *contact_cache.Cache, workspace: *AnalyticSolverWorkspace, status: *fp.MathStatus) anyerror![]const contact_solver.Contact {
    if (workspace.contacts.len < cache.len) return error.CapacityExceeded;
    var contact_count: usize = 0;
    var point_count: usize = 0;
    for (cache.patches[0..cache.len]) |*patch| {
        if (patch.sensor) continue;
        const a = try colliderFor(world, patch.key.collider_a);
        const b = try colliderFor(world, patch.key.collider_b);
        const a_body = world.bodyIndex(a.collider.body) orelse return error.InvalidBody;
        const b_body = world.bodyIndex(b.collider.body) orelse return error.InvalidBody;
        const a_transform = compose(.{ .position = world.storage.position[a_body], .orientation = world.storage.orientation[a_body] }, a.collider.local, status);
        const b_transform = compose(.{ .position = world.storage.position[b_body], .orientation = world.storage.orientation[b_body] }, b.collider.local, status);
        const points_start = point_count;
        const point_len: usize = patch.len;
        if (point_len == 0 or point_count + point_len > workspace.points.len or point_count + point_len > workspace.restitution_bias.len or contact_count == workspace.contacts.len) return error.CapacityExceeded;
        const a_shape = analyticShape(a.collider.shape, a_transform, status);
        const b_shape = analyticShape(b.collider.shape, b_transform, status);
        if (a_shape != null and b_shape != null) {
            const velocity = world.storage.linear_velocity[b_body].sub(world.storage.linear_velocity[a_body], status);
            const hit = analytic.collide(a_shape.?, b_shape.?, velocity, patch.key.collider_a, patch.key.collider_b, status) orelse return error.InvalidContact;
            if (hit.separation.raw > 0 or point_len != 1) return error.InvalidContact;
            const point = hit.witness_a.add(hit.witness_b, status).scale(fp.Fp.fromRatio(1, 2, status), status);
            workspace.points[points_start] = .{ .world_point = point, .penetration = hit.separation.neg(status) };
        } else if (a.collider.shape == .sphere and b.collider.shape == .triangle_mesh) {
            const surface_workspace = workspace.surface orelse return error.InvalidContact;
            const sphere_mesh = surface_workspace.sphere_mesh orelse return error.InvalidContact;
            const source = b.collider.shape.triangle_mesh;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const surface_patch = try mesh.sphereMeshPatchTransformed(view, b_transform, .{ .center = a_transform.position, .radius = a.collider.shape.sphere.radius }, sphere_mesh, status);
            for (patch.points[0..patch.len], 0..) |cached, i| {
                const witness = findSurfaceWitness(surface_patch, patch.key, cached) orelse return error.InvalidContact;
                const point = witness.point_a.add(witness.point_b, status).scale(fp.Fp.fromRatio(1, 2, status), status);
                workspace.points[points_start + i] = .{ .world_point = point, .penetration = witness.separation.neg(status) };
            }
        } else if (a.collider.shape == .triangle_mesh and b.collider.shape == .sphere) {
            const surface_workspace = workspace.surface orelse return error.InvalidContact;
            const sphere_mesh = surface_workspace.sphere_mesh orelse return error.InvalidContact;
            const source = a.collider.shape.triangle_mesh;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const surface_patch = try mesh.sphereMeshPatchTransformed(view, a_transform, .{ .center = b_transform.position, .radius = b.collider.shape.sphere.radius }, sphere_mesh, status);
            for (patch.points[0..patch.len], 0..) |cached, i| {
                const witness = findSurfaceWitnessSwapped(surface_patch, patch.key, cached) orelse return error.InvalidContact;
                const point = witness.point_a.add(witness.point_b, status).scale(fp.Fp.fromRatio(1, 2, status), status);
                workspace.points[points_start + i] = .{ .world_point = point, .penetration = witness.separation.neg(status) };
            }
        } else if (a.collider.shape == .capsule and b.collider.shape == .triangle_mesh) {
            const surface_workspace = workspace.surface orelse return error.InvalidContact;
            const sphere_mesh = surface_workspace.sphere_mesh orelse return error.InvalidContact;
            const source = b.collider.shape.triangle_mesh;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const value = a.collider.shape.capsule;
            const surface_patch = try mesh.capsuleMeshPatchTransformed(view, b_transform, .{ .segment = .{ .a = a_transform.apply(.{ .y = value.half_height.neg(status) }, status), .b = a_transform.apply(.{ .y = value.half_height }, status) }, .radius = value.radius }, sphere_mesh, status);
            for (patch.points[0..patch.len], 0..) |cached, i| {
                const witness = findSurfaceWitness(surface_patch, patch.key, cached) orelse return error.InvalidContact;
                const point = witness.point_a.add(witness.point_b, status).scale(fp.Fp.fromRatio(1, 2, status), status);
                workspace.points[points_start + i] = .{ .world_point = point, .penetration = witness.separation.neg(status) };
            }
        } else if (a.collider.shape == .triangle_mesh and b.collider.shape == .capsule) {
            const surface_workspace = workspace.surface orelse return error.InvalidContact;
            const sphere_mesh = surface_workspace.sphere_mesh orelse return error.InvalidContact;
            const source = a.collider.shape.triangle_mesh;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const value = b.collider.shape.capsule;
            const surface_patch = try mesh.capsuleMeshPatchTransformed(view, a_transform, .{ .segment = .{ .a = b_transform.apply(.{ .y = value.half_height.neg(status) }, status), .b = b_transform.apply(.{ .y = value.half_height }, status) }, .radius = value.radius }, sphere_mesh, status);
            for (patch.points[0..patch.len], 0..) |cached, i| {
                const witness = findSurfaceWitnessSwapped(surface_patch, patch.key, cached) orelse return error.InvalidContact;
                const point = witness.point_a.add(witness.point_b, status).scale(fp.Fp.fromRatio(1, 2, status), status);
                workspace.points[points_start + i] = .{ .world_point = point, .penetration = witness.separation.neg(status) };
            }
        } else if (a.collider.shape == .sphere and b.collider.shape == .height_field) {
            const surface_workspace = workspace.surface orelse return error.InvalidContact;
            const sphere_heightfield = surface_workspace.sphere_heightfield orelse return error.InvalidContact;
            const source = b.collider.shape.height_field;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const surface_patch = try mesh.sphereHeightfieldPatchTransformed(view, b_transform, .{ .center = a_transform.position, .radius = a.collider.shape.sphere.radius }, sphere_heightfield, status);
            for (patch.points[0..patch.len], 0..) |cached, i| {
                const witness = findSurfaceWitness(surface_patch, patch.key, cached) orelse return error.InvalidContact;
                const point = witness.point_a.add(witness.point_b, status).scale(fp.Fp.fromRatio(1, 2, status), status);
                workspace.points[points_start + i] = .{ .world_point = point, .penetration = witness.separation.neg(status) };
            }
        } else if (a.collider.shape == .height_field and b.collider.shape == .sphere) {
            const surface_workspace = workspace.surface orelse return error.InvalidContact;
            const sphere_heightfield = surface_workspace.sphere_heightfield orelse return error.InvalidContact;
            const source = a.collider.shape.height_field;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const surface_patch = try mesh.sphereHeightfieldPatchTransformed(view, a_transform, .{ .center = b_transform.position, .radius = b.collider.shape.sphere.radius }, sphere_heightfield, status);
            for (patch.points[0..patch.len], 0..) |cached, i| {
                const witness = findSurfaceWitnessSwapped(surface_patch, patch.key, cached) orelse return error.InvalidContact;
                const point = witness.point_a.add(witness.point_b, status).scale(fp.Fp.fromRatio(1, 2, status), status);
                workspace.points[points_start + i] = .{ .world_point = point, .penetration = witness.separation.neg(status) };
            }
        } else if (isConvexSurfaceCaster(a.collider.shape) and b.collider.shape == .triangle_mesh) {
            const surface_workspace = workspace.surface orelse return error.InvalidContact;
            const convex_mesh = surface_workspace.convex_mesh orelse return error.InvalidContact;
            const source = b.collider.shape.triangle_mesh;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const surface_patch = try mesh.convexMeshPatch(a.collider.shape, assets, a_transform, view, b_transform, convex_mesh, status);
            for (patch.points[0..patch.len], 0..) |cached, i| {
                const witness = findSurfaceWitness(surface_patch, patch.key, cached) orelse return error.InvalidContact;
                const point = witness.point_a.add(witness.point_b, status).scale(fp.Fp.fromRatio(1, 2, status), status);
                workspace.points[points_start + i] = .{ .world_point = point, .penetration = witness.separation.neg(status) };
            }
        } else if (a.collider.shape == .triangle_mesh and isConvexSurfaceCaster(b.collider.shape)) {
            const surface_workspace = workspace.surface orelse return error.InvalidContact;
            const convex_mesh = surface_workspace.convex_mesh orelse return error.InvalidContact;
            const source = a.collider.shape.triangle_mesh;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const surface_patch = try mesh.convexMeshPatch(b.collider.shape, assets, b_transform, view, a_transform, convex_mesh, status);
            for (patch.points[0..patch.len], 0..) |cached, i| {
                const witness = findSurfaceWitnessSwapped(surface_patch, patch.key, cached) orelse return error.InvalidContact;
                const point = witness.point_a.add(witness.point_b, status).scale(fp.Fp.fromRatio(1, 2, status), status);
                workspace.points[points_start + i] = .{ .world_point = point, .penetration = witness.separation.neg(status) };
            }
        } else if (isConvexSurfaceCaster(a.collider.shape) and b.collider.shape == .height_field) {
            const surface_workspace = workspace.surface orelse return error.InvalidContact;
            const convex_heightfield = surface_workspace.convex_heightfield orelse return error.InvalidContact;
            const source = b.collider.shape.height_field;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const surface_patch = try mesh.convexHeightfieldPatch(a.collider.shape, assets, a_transform, view, b_transform, convex_heightfield, status);
            for (patch.points[0..patch.len], 0..) |cached, i| {
                const witness = findSurfaceWitness(surface_patch, patch.key, cached) orelse return error.InvalidContact;
                const point = witness.point_a.add(witness.point_b, status).scale(fp.Fp.fromRatio(1, 2, status), status);
                workspace.points[points_start + i] = .{ .world_point = point, .penetration = witness.separation.neg(status) };
            }
        } else if (a.collider.shape == .height_field and isConvexSurfaceCaster(b.collider.shape)) {
            const surface_workspace = workspace.surface orelse return error.InvalidContact;
            const convex_heightfield = surface_workspace.convex_heightfield orelse return error.InvalidContact;
            const source = a.collider.shape.height_field;
            const view = try runtime_view.find(assets, if (source.source_id != 0) source.source_id else source.asset.index());
            const surface_patch = try mesh.convexHeightfieldPatch(b.collider.shape, assets, b_transform, view, a_transform, convex_heightfield, status);
            for (patch.points[0..patch.len], 0..) |cached, i| {
                const witness = findSurfaceWitnessSwapped(surface_patch, patch.key, cached) orelse return error.InvalidContact;
                const point = witness.point_a.add(witness.point_b, status).scale(fp.Fp.fromRatio(1, 2, status), status);
                workspace.points[points_start + i] = .{ .world_point = point, .penetration = witness.separation.neg(status) };
            }
        } else if (a.collider.shape == .triangle_mesh and b.collider.shape == .triangle_mesh) {
            const surface_workspace = workspace.surface orelse return error.InvalidContact;
            const mesh_mesh = surface_workspace.mesh_mesh orelse return error.InvalidContact;
            const source_a = a.collider.shape.triangle_mesh;
            const source_b = b.collider.shape.triangle_mesh;
            const view_a = try runtime_view.find(assets, if (source_a.source_id != 0) source_a.source_id else source_a.asset.index());
            const view_b = try runtime_view.find(assets, if (source_b.source_id != 0) source_b.source_id else source_b.asset.index());
            const surface_patch = try mesh.meshMeshPatchTransformed(view_a, a_transform, view_b, b_transform, mesh_mesh, status);
            for (patch.points[0..patch.len], 0..) |cached, i| {
                const witness = findSurfaceWitness(surface_patch, patch.key, cached) orelse return error.InvalidContact;
                const point = witness.point_a.add(witness.point_b, status).scale(fp.Fp.fromRatio(1, 2, status), status);
                workspace.points[points_start + i] = .{ .world_point = point, .penetration = witness.separation.neg(status) };
            }
        } else if (a.collider.shape == .sphere and b.collider.shape == .compound) {
            const surface_workspace = workspace.surface orelse return error.InvalidContact;
            const sphere_compound = surface_workspace.sphere_compound orelse return error.InvalidContact;
            const surface_patch = try mesh.sphereCompoundSurfacePatch(b.collider.shape, assets, b_transform, .{ .center = a_transform.position, .radius = a.collider.shape.sphere.radius }, sphere_compound, status);
            for (patch.points[0..patch.len], 0..) |cached, i| {
                const witness = findSurfaceWitness(surface_patch, patch.key, cached) orelse return error.InvalidContact;
                const point = witness.point_a.add(witness.point_b, status).scale(fp.Fp.fromRatio(1, 2, status), status);
                workspace.points[points_start + i] = .{ .world_point = point, .penetration = witness.separation.neg(status) };
            }
        } else if (a.collider.shape == .compound and b.collider.shape == .sphere) {
            const surface_workspace = workspace.surface orelse return error.InvalidContact;
            const sphere_compound = surface_workspace.sphere_compound orelse return error.InvalidContact;
            const surface_patch = try mesh.sphereCompoundSurfacePatch(a.collider.shape, assets, a_transform, .{ .center = b_transform.position, .radius = b.collider.shape.sphere.radius }, sphere_compound, status);
            for (patch.points[0..patch.len], 0..) |cached, i| {
                const witness = findSurfaceWitnessSwapped(surface_patch, patch.key, cached) orelse return error.InvalidContact;
                const point = witness.point_a.add(witness.point_b, status).scale(fp.Fp.fromRatio(1, 2, status), status);
                workspace.points[points_start + i] = .{ .world_point = point, .penetration = witness.separation.neg(status) };
            }
        } else if (isConvexSurfaceCaster(a.collider.shape) and b.collider.shape == .compound) {
            const surface_workspace = workspace.surface orelse return error.InvalidContact;
            const convex_compound = surface_workspace.convex_compound orelse return error.InvalidContact;
            const surface_patch = try mesh.convexCompoundSurfacePatch(b.collider.shape, assets, b_transform, a.collider.shape, a_transform, convex_compound, status);
            for (patch.points[0..patch.len], 0..) |cached, i| {
                const witness = findSurfaceWitness(surface_patch, patch.key, cached) orelse return error.InvalidContact;
                const point = witness.point_a.add(witness.point_b, status).scale(fp.Fp.fromRatio(1, 2, status), status);
                workspace.points[points_start + i] = .{ .world_point = point, .penetration = witness.separation.neg(status) };
            }
        } else if (a.collider.shape == .compound and isConvexSurfaceCaster(b.collider.shape)) {
            const surface_workspace = workspace.surface orelse return error.InvalidContact;
            const convex_compound = surface_workspace.convex_compound orelse return error.InvalidContact;
            const surface_patch = try mesh.convexCompoundSurfacePatch(a.collider.shape, assets, a_transform, b.collider.shape, b_transform, convex_compound, status);
            for (patch.points[0..patch.len], 0..) |cached, i| {
                const witness = findSurfaceWitnessSwapped(surface_patch, patch.key, cached) orelse return error.InvalidContact;
                const point = witness.point_a.add(witness.point_b, status).scale(fp.Fp.fromRatio(1, 2, status), status);
                workspace.points[points_start + i] = .{ .world_point = point, .penetration = witness.separation.neg(status) };
            }
        } else {
            const manifold = workspace.manifold orelse return error.InvalidContact;
            const direction = b_transform.position.sub(a_transform.position, status);
            var pair = gjk.ShapePairContext{ .shape_a = a.collider.shape, .shape_b = b.collider.shape, .assets = assets, .transform_a = a_transform, .transform_b = b_transform };
            const result = try gjk.collideShapes(&pair, if (direction.lengthSquared(status).raw == 0) geometry.Vec3.unit_x else direction, manifold, status);
            if (result.gjk.status != .intersecting) return error.InvalidContact;
            for (patch.points[0..patch.len], 0..) |cached, i| {
                const witness = findConvexWitness(result.patch, patch.key, cached) orelse return error.InvalidContact;
                const point = witness.point_a.add(witness.point_b, status).scale(fp.Fp.fromRatio(1, 2, status), status);
                workspace.points[points_start + i] = .{ .world_point = point, .penetration = witness.separation.neg(status) };
            }
        }
        for (workspace.restitution_bias[points_start .. points_start + point_len]) |*bias| bias.* = .zero;
        workspace.contacts[contact_count] = .{ .body_a = a.collider.body, .body_b = b.collider.body, .friction_a = a.collider.material.friction, .friction_b = b.collider.material.friction, .restitution_a = a.collider.material.restitution, .restitution_b = b.collider.material.restitution, .points = workspace.points[points_start .. points_start + point_len], .restitution_bias = workspace.restitution_bias[points_start .. points_start + point_len], .patch = patch };
        try contact_solver.prepareContact(world, &workspace.contacts[contact_count], workspace.points[points_start .. points_start + point_len], status);
        contact_count += 1;
        point_count += point_len;
    }
    return workspace.contacts[0..contact_count];
}
fn isConvexSurfaceCaster(shape: shapes.Shape) bool {
    return switch (shape) {
        .sphere, .box, .capsule, .convex_hull => true,
        else => false,
    };
}
fn findConvexWitness(patch: gjk.ContactPatch, key: contact_cache.ManifoldKey, cached: contact_cache.CachedPoint) ?gjk.ContactPoint {
    for (patch.points[0..patch.len]) |point| if (point.feature_a == cached.feature_a and point.feature_b == cached.feature_b and pathsEqual(point.path_a, key.path_a) and pathsEqual(point.path_b, key.path_b)) return point;
    return null;
}
fn findSurfaceWitness(patch: gjk.ContactPatch, key: contact_cache.ManifoldKey, cached: contact_cache.CachedPoint) ?gjk.ContactPoint {
    for (patch.points[0..patch.len]) |point| if (point.feature_a == cached.feature_a and point.feature_b == cached.feature_b and pathsEqual(point.path_a, key.path_a) and pathsEqual(point.path_b, key.path_b)) return point;
    return null;
}
fn findSurfaceWitnessSwapped(patch: gjk.ContactPatch, key: contact_cache.ManifoldKey, cached: contact_cache.CachedPoint) ?gjk.ContactPoint {
    for (patch.points[0..patch.len]) |point| if (point.feature_b == cached.feature_a and point.feature_a == cached.feature_b and pathsEqual(point.path_b, key.path_a) and pathsEqual(point.path_a, key.path_b)) return point;
    return null;
}
fn pathsEqual(a: shapes.ChildPath, b: shapes.ChildPath) bool {
    return a.len == b.len and std.mem.eql(u32, a.values[0..a.len], b.values[0..b.len]);
}

/// Builds deterministic dynamic islands from current non-sensor analytic
/// patches and emits the Task 14 DOF lock rows. Cache order is canonical, so
/// the contact-edge order is independent of broadphase scratch layout.
pub fn buildAnalyticIslands(world: *const world_mod.World, cache: *const contact_cache.Cache, joint_pool: ?*const joints.Pool, workspace: *IslandWorkspace, status: *fp.MathStatus) (world_mod.Error || constraints.Error)!constraints.BuildResult {
    var count: usize = 0;
    for (cache.active()) |patch| {
        if (patch.sensor) continue;
        if (count == workspace.edges.len) return error.CapacityExceeded;
        const a = try colliderFor(world, patch.key.collider_a);
        const b = try colliderFor(world, patch.key.collider_b);
        workspace.edges[count] = .{ .kind = .contact, .body_a = a.collider.body, .body_b = b.collider.body, .owner = (@as(u64, patch.key.collider_a.value) << 1) ^ patch.key.collider_b.value };
        count += 1;
    }
    if (joint_pool) |pool| for (pool.storage.values, 0..) |joint, index| {
        if (!pool.storage.alive[index]) continue;
        if (count == workspace.edges.len) return error.CapacityExceeded;
        workspace.edges[count] = .{ .kind = .joint, .body_a = joint.body_a, .body_b = joint.body_b, .owner = @as(u64, @intCast(index)) | (@as(u64, pool.storage.generation[index]) << 32) };
        count += 1;
    };
    workspace.edge_len = count;
    return if (workspace.union_parents) |parents|
        constraints.buildWithParents(world, workspace.edges[0..count], workspace.edge_scratch, workspace.islands, workspace.members, workspace.lock_rows, parents, status)
    else
        constraints.build(world, workspace.edges[0..count], workspace.edge_scratch, workspace.islands, workspace.members, workspace.lock_rows, status);
}

fn buildSolverPartition(world: *const world_mod.World, islands: []const constraints.Island, members: []const @import("../core/ids.zig").BodyId, contacts: []const contact_solver.Contact, rows: []const constraints.ConstraintRow, partition: *SolverPartitionWorkspace) Error!void {
    const sentinel = std.math.maxInt(u32);
    if (partition.body_island.len < world.storage.alive.len or partition.contact_indices.len < contacts.len or partition.row_indices.len < rows.len or partition.contact_offsets.len < islands.len + 1 or partition.row_offsets.len < islands.len + 1 or partition.contact_cursor.len < islands.len or partition.row_cursor.len < islands.len) return error.CapacityExceeded;
    @memset(partition.body_island[0..world.storage.alive.len], sentinel);
    for (islands, 0..) |island, island_index| {
        const first: usize = island.first_member;
        const count: usize = island.member_count;
        if (first + count > members.len) return error.CapacityExceeded;
        for (members[first .. first + count]) |body| partition.body_island[world.bodyIndex(body) orelse return error.InvalidBody] = @intCast(island_index);
    }
    const contact_offsets = partition.contact_offsets[0 .. islands.len + 1];
    const row_offsets = partition.row_offsets[0 .. islands.len + 1];
    @memset(contact_offsets, 0);
    @memset(row_offsets, 0);
    for (contacts) |contact| {
        const a = world.bodyIndex(contact.body_a) orelse return error.InvalidBody;
        const b = world.bodyIndex(contact.body_b) orelse return error.InvalidBody;
        const dynamic = if (world.storage.body_type[a] == .dynamic) a else if (world.storage.body_type[b] == .dynamic) b else return error.InvalidContact;
        const island = partition.body_island[dynamic];
        if (island == sentinel) return error.InvalidContact;
        contact_offsets[@as(usize, island) + 1] += 1;
    }
    for (rows) |row| {
        const a = world.bodyIndex(row.key.min_body) orelse return error.InvalidBody;
        const b = world.bodyIndex(row.key.max_body) orelse return error.InvalidBody;
        const dynamic = if (world.storage.body_type[a] == .dynamic) a else if (world.storage.body_type[b] == .dynamic) b else return error.InvalidContact;
        const island = partition.body_island[dynamic];
        if (island == sentinel) return error.InvalidContact;
        row_offsets[@as(usize, island) + 1] += 1;
    }
    for (0..islands.len) |index| {
        contact_offsets[index + 1] += contact_offsets[index];
        row_offsets[index + 1] += row_offsets[index];
        partition.contact_cursor[index] = contact_offsets[index];
        partition.row_cursor[index] = row_offsets[index];
    }
    for (contacts, 0..) |contact, contact_index| {
        const a = world.bodyIndex(contact.body_a).?;
        const b = world.bodyIndex(contact.body_b).?;
        const dynamic = if (world.storage.body_type[a] == .dynamic) a else b;
        const island = partition.body_island[dynamic];
        const destination = partition.contact_cursor[island];
        partition.contact_indices[destination] = @intCast(contact_index);
        partition.contact_cursor[island] += 1;
    }
    for (rows, 0..) |row, row_index| {
        const a = world.bodyIndex(row.key.min_body).?;
        const b = world.bodyIndex(row.key.max_body).?;
        const dynamic = if (world.storage.body_type[a] == .dynamic) a else b;
        const island = partition.body_island[dynamic];
        const destination = partition.row_cursor[island];
        partition.row_indices[destination] = @intCast(row_index);
        partition.row_cursor[island] += 1;
    }
}

/// Executes the command/integration prefix of the frozen pipeline.  All
/// commands are validated before any mutation; a prior runtime fault makes
/// the world read-only until a future snapshot loader clears it.
pub fn stepBodies(world: *world_mod.World, state: *State, simulation: config.SimulationConfig, commands: []const world_mod.Command, workspace: *Workspace, status: *fp.MathStatus) Error!Result {
    const substeps: usize = @intCast(simulation.iterations.substeps);
    try begin(world, state, simulation, commands, workspace, status, 2 + substeps);
    defer state.in_step = false;
    const dt = substepDt(simulation, status);
    for (0..substeps) |_| {
        push(workspace, .integrate);
        world.stepSubstep(dt, status);
        try checkMath(state, .integrate, status);
    }
    world.finishTick();
    return finish(state, workspace);
}

/// Runs the command/integration/SAP prefix with a SAP rebuild after every
/// substep. Later phases deliberately consume the returned pair slice from
/// `broadphase_workspace.buffers`; this function never allocates or caches it.
pub fn stepWithBroadphase(world: *world_mod.World, state: *State, simulation: config.SimulationConfig, commands: []const world_mod.Command, workspace: *Workspace, broadphase_workspace: *BroadphaseWorkspace, status: *fp.MathStatus) Error!Result {
    const substeps: usize = @intCast(simulation.iterations.substeps);
    try begin(world, state, simulation, commands, workspace, status, 2 + substeps * 2);
    defer state.in_step = false;
    const dt = substepDt(simulation, status);
    for (0..substeps) |_| {
        push(workspace, .integrate);
        world.stepSubstep(dt, status);
        try checkMath(state, .integrate, status);
        push(workspace, .broadphase);
        _ = rebuildBroadphase(world, broadphase_workspace.assets, simulation, dt, broadphase_workspace.collider_views, broadphase_workspace.proxies, broadphase_workspace.buffers, status) catch |err| {
            recordFaultError(state, .broadphase, err, status.fault);
            return error.Faulted;
        };
        try checkMath(state, .broadphase, status);
    }
    world.finishTick();
    return finish(state, workspace);
}

/// Runs the fixed command/integrate/SAP/analytic-narrow prefix and atomically
/// merges the final substep's canonical patches into the Task 12 cache. This
/// is deliberately limited to analytic primitives; the complete producer will
/// append convex and mesh patches before the same merge boundary.
pub fn stepWithAnalyticContacts(world: *world_mod.World, state: *State, simulation: config.SimulationConfig, commands: []const world_mod.Command, workspace: *Workspace, contacts: *AnalyticContactWorkspace, status: *fp.MathStatus) Error!ContactResult {
    const substeps: usize = @intCast(simulation.iterations.substeps);
    try begin(world, state, simulation, commands, workspace, status, 2 + substeps * 3);
    defer state.in_step = false;
    const dt = substepDt(simulation, status);
    var final_patches: []const contact_cache.Patch = &.{};
    for (0..substeps) |_| {
        push(workspace, .integrate);
        world.integrateVelocities(dt, status);
        try checkMath(state, .integrate, status);
        push(workspace, .broadphase);
        const pairs = rebuildBroadphase(world, contacts.broadphase.assets, simulation, dt, contacts.broadphase.collider_views, contacts.broadphase.proxies, contacts.broadphase.buffers, status) catch |err| {
            recordFaultError(state, .broadphase, err, status.fault);
            return error.Faulted;
        };
        try checkMath(state, .broadphase, status);
        push(workspace, .narrowphase);
        var narrow_fault_object: ?u64 = null;
        final_patches = narrowRuntimePairsTracked(world, contacts.broadphase.assets, pairs, contacts.convex, contacts.surface, contacts.narrow, status, &narrow_fault_object) catch |err| {
            recordFaultErrorFor(state, .narrowphase, err, status.fault, narrow_fault_object);
            return error.Faulted;
        };
        try checkMath(state, .narrowphase, status);
        // The solver phase is inserted at this boundary; position integration
        // must always observe its solved velocities.
        world.integratePositions(dt, status);
        try checkMath(state, .integrate, status);
    }
    const merged = contact_cache.merge(contacts.cache, final_patches, .{ .next = contacts.cache_next, .events = contacts.events }, simulation.tolerances.warmstart_normal_cos_min, status) catch {
        recordFault(state, .narrowphase, .contact, status.fault);
        return error.Faulted;
    };
    try checkMath(state, .narrowphase, status);
    world.finishTick();
    return .{ .step = finish(state, workspace), .events = merged.events };
}

/// Executes analytic contact solving during every substep. Persistent cache
/// patches are updated before each solve for deterministic warm-starting, but
/// events are derived only once from the pre-Tick cache snapshot to avoid
/// substep `begin`/`persist` duplication.
pub fn stepWithAnalyticSolver(world: *world_mod.World, state: *State, simulation: config.SimulationConfig, commands: []const world_mod.Command, workspace: *Workspace, pipeline_workspace: *AnalyticSolverPipelineWorkspace, status: *fp.MathStatus) Error!ContactResult {
    const contacts = pipeline_workspace.contacts;
    const substeps: usize = @intCast(simulation.iterations.substeps);
    const has_ccd = pipeline_workspace.ccd != null and simulation.features.ccd;
    if (pipeline_workspace.previous.len < contacts.cache.len) return error.TraceCapacity;
    const phases_per_substep: usize = 5 + @as(usize, @intFromBool(has_ccd));
    if (pipeline_workspace.sleep) |sleep| try beginSleeping(world, state, simulation, commands, workspace, pipeline_workspace, sleep, status, 5 + substeps * phases_per_substep) else try begin(world, state, simulation, commands, workspace, status, 4 + substeps * phases_per_substep);
    defer state.in_step = false;
    @memcpy(pipeline_workspace.previous[0..contacts.cache.len], contacts.cache.active());
    const initial_len = contacts.cache.len;
    const dt = substepDt(simulation, status);
    for (0..substeps) |_| {
        const Integrate = struct {
            world: *world_mod.World,
            awake: ?[]const bool,
            dt: fp.Fp,
            fn linear(self: *@This(), range: jobs.Range, job_status: *fp.MathStatus) Error!void {
                try self.world.integrateLinearVelocitySlots(range.begin, range.end, self.awake, self.dt, job_status);
            }
            fn angular(self: *@This(), range: jobs.Range, job_status: *fp.MathStatus) Error!void {
                try self.world.integrateAngularVelocitySlots(range.begin, range.end, self.awake, self.dt, job_status);
            }
        };
        var integrate = Integrate{ .world = world, .awake = if (pipeline_workspace.sleep) |sleep| sleep.storage.awake else null, .dt = dt };
        push(workspace, .integrate);
        try dispatchRanges(workspace, world.storage.alive.len, &integrate, status, Integrate.linear);
        try dispatchRanges(workspace, world.storage.alive.len, &integrate, status, Integrate.angular);
        try checkMath(state, .integrate, status);
        push(workspace, .broadphase);
        const pairs = rebuildBroadphaseRanges(workspace, world, contacts.broadphase.assets, simulation, dt, contacts.broadphase.collider_views, contacts.broadphase.proxies, contacts.broadphase.buffers, status) catch |err| {
            if (isDispatchError(err)) return err;
            recordFaultError(state, .broadphase, err, status.fault);
            return error.Faulted;
        };
        try checkMath(state, .broadphase, status);
        const Narrow = struct {
            workspace: *Workspace,
            world: *world_mod.World,
            contacts: *AnalyticContactWorkspace,
            pairs: []const broadphase.PairKey,
            simulation: config.SimulationConfig,
            pipeline_workspace: *AnalyticSolverPipelineWorkspace,
            status: *fp.MathStatus,
            patches: []const contact_cache.Patch = &.{},
            fault_object: ?u64 = null,
            fn run(self: *@This()) Error!void {
                self.patches = narrowRuntimePairsRanges(self.workspace, self.world, self.contacts.broadphase.assets, self.pairs, self.contacts.convex, self.contacts.surface, self.contacts.cache_next, self.contacts.narrow, self.status, &self.fault_object) catch |err| return switch (err) {
                    error.CapacityExceeded, error.OutOfScratch, error.OutOfSpace => error.CapacityExceeded,
                    else => error.ContactFailure,
                };
                _ = contact_cache.merge(self.contacts.cache, self.patches, .{ .next = self.contacts.cache_next, .events = self.pipeline_workspace.substep_events }, self.simulation.tolerances.warmstart_normal_cos_min, self.status) catch |err| return switch (err) {
                    error.CapacityExceeded => error.CapacityExceeded,
                    else => error.ContactFailure,
                };
            }
        };
        var narrow = Narrow{ .workspace = workspace, .world = world, .contacts = contacts, .pairs = pairs, .simulation = simulation, .pipeline_workspace = pipeline_workspace, .status = status };
        push(workspace, .narrowphase);
        narrow.run() catch |err| {
            if (isDispatchError(err)) return err;
            recordFaultErrorFor(state, .narrowphase, err, status.fault, narrow.fault_object);
            return error.Faulted;
        };
        try checkMath(state, .narrowphase, status);
        const Islands = struct {
            world: *world_mod.World,
            contacts: *AnalyticContactWorkspace,
            pipeline_workspace: *AnalyticSolverPipelineWorkspace,
            status: *fp.MathStatus,
            island_len: usize = 0,
            member_len: usize = 0,
            fn run(self: *@This()) Error!void {
                const built = try buildAnalyticIslands(self.world, self.contacts.cache, if (self.pipeline_workspace.joint) |joint| joint.pool else null, self.pipeline_workspace.islands, self.status);
                self.island_len = built.islands.len;
                self.member_len = built.members.len;
            }
        };
        var islands = Islands{ .world = world, .contacts = contacts, .pipeline_workspace = pipeline_workspace, .status = status };
        dispatchPhase(workspace, .islands, &islands, Islands.run) catch |err| {
            if (isDispatchError(err)) return err;
            recordFault(state, .islands, .contact, status.fault);
            return error.Faulted;
        };
        try checkMath(state, .islands, status);
        const Solve = struct {
            workspace: *Workspace,
            world: *world_mod.World,
            contacts: *AnalyticContactWorkspace,
            pipeline_workspace: *AnalyticSolverPipelineWorkspace,
            simulation: config.SimulationConfig,
            dt: fp.Fp,
            status: *fp.MathStatus,
            islands: []const constraints.Island,
            members: []const @import("../core/ids.zig").BodyId,
            solver_contacts: []const contact_solver.Contact = &.{},
            joint_rows: []constraints.ConstraintRow = &.{},
            fn run(self: *@This()) Error!void {
                if (self.workspace.observer) |observer| observer.detail(.solve_contacts);
                self.solver_contacts = buildAnalyticSolverContacts(self.world, self.contacts.broadphase.assets, self.contacts.cache, self.pipeline_workspace.solver, self.status) catch |err| return switch (err) {
                    error.CapacityExceeded, error.OutOfScratch, error.OutOfSpace => error.CapacityExceeded,
                    else => error.ContactFailure,
                };
                if (self.workspace.observer) |observer| observer.detail(.solve_wake);
                if (self.pipeline_workspace.sleep) |sleep| {
                    try wakeContacts(self.world, sleep, self.pipeline_workspace.islands.edges[0..self.pipeline_workspace.islands.edge_len], self.solver_contacts);
                    if (self.pipeline_workspace.joint) |joint| _ = try sleeping.wakeActiveJoints(self.world, sleep.storage, self.pipeline_workspace.islands.edges[0..self.pipeline_workspace.islands.edge_len], joint.pool, sleep.requests, sleep.graph_scratch, sleep.wake_events);
                }
                if (self.workspace.observer) |observer| observer.detail(.solve_joints);
                self.joint_rows = if (self.pipeline_workspace.joint) |joint| blk: {
                    const built = try joints.buildPoolRows(self.world, joint.pool, self.dt, joint.rows, joint.scratch, self.status);
                    break :blk joint.rows[0..built.len];
                } else &.{};
                if (self.workspace.observer) |observer| observer.detail(.solve_validate);
                try contact_solver.validateInputs(self.world, self.solver_contacts, self.pipeline_workspace.solver.pseudo);
                try joints.validateRows(self.world, self.joint_rows);
                if (self.pipeline_workspace.solver.partition) |*partition| try buildSolverPartition(self.world, self.islands, self.members, self.solver_contacts, self.joint_rows, partition);
                // Static/kinematic pseudo slots are read while evaluating a
                // dynamic island even though they are never written by it.
                // Clear the complete read set once before workers start.
                if (self.workspace.observer) |observer| observer.detail(.solve_clear);
                @memset(self.pipeline_workspace.solver.pseudo.linear, geometry.Vec3.zero);
                @memset(self.pipeline_workspace.solver.pseudo.angular, geometry.Vec3.zero);
                const IslandSolve = struct {
                    world: *world_mod.World,
                    islands: []const constraints.Island,
                    members: []const @import("../core/ids.zig").BodyId,
                    joint_rows: []constraints.ConstraintRow,
                    contacts: []const contact_solver.Contact,
                    partition: ?*SolverPartitionWorkspace,
                    pseudo: contact_solver.PseudoVelocities,
                    settings: contact_solver.Settings,
                    fn run(value: *@This(), range: jobs.Range, job_status: *fp.MathStatus) Error!void {
                        var index: usize = range.begin;
                        while (index < range.end) : (index += 1) {
                            const island = value.islands[index];
                            const first: usize = island.first_member;
                            const count: usize = island.member_count;
                            if (value.partition) |partition| {
                                const contact_first: usize = partition.contact_offsets[index];
                                const contact_end: usize = partition.contact_offsets[index + 1];
                                const row_first: usize = partition.row_offsets[index];
                                const row_end: usize = partition.row_offsets[index + 1];
                                contact_solver.solveIslandIndexed(value.world, value.members[first .. first + count], value.joint_rows, partition.row_indices[row_first..row_end], value.contacts, partition.contact_indices[contact_first..contact_end], value.pseudo, value.settings, job_status);
                            } else contact_solver.solveIslandWithJointRows(value.world, value.members[first .. first + count], value.joint_rows, value.contacts, value.pseudo, value.settings, job_status);
                        }
                    }
                };
                var island_solve = IslandSolve{ .world = self.world, .islands = self.islands, .members = self.members, .joint_rows = self.joint_rows, .contacts = self.solver_contacts, .partition = if (self.pipeline_workspace.solver.partition) |*partition| partition else null, .pseudo = self.pipeline_workspace.solver.pseudo, .settings = solverSettings(self.simulation) };
                if (self.workspace.observer) |observer| observer.detail(.solve_pgs);
                try dispatchRangesGrain(self.workspace, self.islands.len, 1, &island_solve, self.status, IslandSolve.run);
            }
        };
        var solve = Solve{ .workspace = workspace, .world = world, .contacts = contacts, .pipeline_workspace = pipeline_workspace, .simulation = simulation, .dt = dt, .status = status, .islands = pipeline_workspace.islands.islands[0..islands.island_len], .members = pipeline_workspace.islands.members[0..islands.member_len] };
        push(workspace, .solve);
        solve.run() catch |err| {
            if (isDispatchError(err)) return err;
            recordFault(state, .solve, .contact, status.fault);
            return error.Faulted;
        };
        const joint_rows = solve.joint_rows;
        try checkMath(state, .solve, status);
        if (pipeline_workspace.ccd) |ccd_workspace| {
            if (simulation.features.ccd) {
                const Ccd = struct {
                    world: *world_mod.World,
                    simulation: config.SimulationConfig,
                    dt: fp.Fp,
                    pipeline_workspace: *AnalyticSolverPipelineWorkspace,
                    ccd_workspace: *CcdPipelineWorkspace,
                    contacts: *AnalyticContactWorkspace,
                    joint_rows: []constraints.ConstraintRow,
                    status: *fp.MathStatus,
                    fn run(self: *@This()) Error!void {
                        _ = try resolveCcdSubstep(self.world, self.simulation, self.dt, if (self.pipeline_workspace.sleep) |sleep| sleep.storage.awake else null, self.ccd_workspace, self.contacts, self.pipeline_workspace.solver, self.joint_rows, self.status);
                        if (self.pipeline_workspace.joint) |joint| try joints.writeBackImpulses(joint.pool, self.joint_rows);
                    }
                };
                var ccd_phase = Ccd{ .world = world, .simulation = simulation, .dt = dt, .pipeline_workspace = pipeline_workspace, .ccd_workspace = ccd_workspace, .contacts = contacts, .joint_rows = joint_rows, .status = status };
                dispatchPhase(workspace, .ccd, &ccd_phase, Ccd.run) catch |err| {
                    if (isDispatchError(err)) return err;
                    recordFault(state, .ccd, if (err == error.CcdFault) .ccd else .contact, status.fault);
                    return error.Faulted;
                };
                try checkMath(state, .ccd, status);
            } else try integratePositionRanges(workspace, world, if (pipeline_workspace.sleep) |sleep| sleep.storage.awake else null, dt, status);
        } else try integratePositionRanges(workspace, world, if (pipeline_workspace.sleep) |sleep| sleep.storage.awake else null, dt, status);
        if (pipeline_workspace.joint) |joint| if (!has_ccd) joints.writeBackImpulses(joint.pool, joint_rows) catch {
            recordFault(state, .solve, .contact, status.fault);
            return error.Faulted;
        };
        try checkMath(state, .integrate, status);
    }
    if (pipeline_workspace.sleep) |sleep| {
        const Sleep = struct {
            world: *world_mod.World,
            contacts: *AnalyticContactWorkspace,
            pipeline_workspace: *AnalyticSolverPipelineWorkspace,
            sleep: *SleepWorkspace,
            simulation: config.SimulationConfig,
            status: *fp.MathStatus,
            fn run(self: *@This()) Error!void {
                const final_islands = try buildAnalyticIslands(self.world, self.contacts.cache, if (self.pipeline_workspace.joint) |joint| joint.pool else null, self.pipeline_workspace.islands, self.status);
                _ = try sleeping.stepConfigured(self.world, final_islands.islands, final_islands.members, self.sleep.storage, self.simulation, self.sleep.sleep_events, self.status);
            }
        };
        var sleep_phase = Sleep{ .world = world, .contacts = contacts, .pipeline_workspace = pipeline_workspace, .sleep = sleep, .simulation = simulation, .status = status };
        dispatchPhase(workspace, .sleep, &sleep_phase, Sleep.run) catch |err| {
            if (isDispatchError(err)) return err;
            recordFault(state, .sleep, .contact, status.fault);
            return error.Faulted;
        };
        try checkMath(state, .sleep, status);
    }
    const Events = struct {
        contacts: *AnalyticContactWorkspace,
        pipeline_workspace: *AnalyticSolverPipelineWorkspace,
        initial_len: usize,
        simulation: config.SimulationConfig,
        status: *fp.MathStatus,
        events: contact_cache.MergeResult = .{ .events = &.{} },
        fn run(self: *@This()) Error!void {
            var previous = contact_cache.Cache{ .patches = self.pipeline_workspace.previous, .len = self.initial_len };
            self.events = contact_cache.merge(&previous, self.contacts.cache.active(), .{ .next = self.pipeline_workspace.event_next, .events = self.pipeline_workspace.tick_events }, self.simulation.tolerances.warmstart_normal_cos_min, self.status) catch |err| return switch (err) {
                error.CapacityExceeded => error.CapacityExceeded,
                else => error.ContactFailure,
            };
        }
    };
    var event_phase = Events{ .contacts = contacts, .pipeline_workspace = pipeline_workspace, .initial_len = initial_len, .simulation = simulation, .status = status };
    dispatchPhase(workspace, .events, &event_phase, Events.run) catch |err| {
        if (isDispatchError(err)) return err;
        recordFault(state, .events, .contact, status.fault);
        return error.Faulted;
    };
    const events = event_phase.events;
    try checkMath(state, .events, status);
    world.finishTick();
    push(workspace, .hash);
    state.tick += 1;
    var result = Result{ .tick = state.tick, .trace = workspace.trace[0..workspace.trace_len], .diagnostics = if (workspace.diagnostics) |diagnostics| diagnostics.* else null };
    const Hash = struct {
        world: *world_mod.World,
        state: *State,
        simulation: config.SimulationConfig,
        contacts: *AnalyticContactWorkspace,
        pipeline_workspace: *AnalyticSolverPipelineWorkspace,
        output: *?hash.Hash128,
        fn run(self: *@This()) Error!void {
            self.output.* = canonicalStateHash(self.world, self.state, self.simulation, .{
                .cache = self.contacts.cache,
                .joint = if (self.pipeline_workspace.joint) |joint| joint.pool else null,
                .sleep = if (self.pipeline_workspace.sleep) |sleep| sleep.storage else null,
                .ccd_enabled = if (self.pipeline_workspace.ccd) |ccd_workspace| ccd_workspace.items.enabled else null,
            });
        }
    };
    var hash_phase = Hash{ .world = world, .state = state, .simulation = simulation, .contacts = contacts, .pipeline_workspace = pipeline_workspace, .output = &result.state_hash };
    try dispatchWork(workspace, &hash_phase, Hash.run);
    if (workspace.observer) |observer| observer.transition(null);
    return .{ .step = result, .events = events.events };
}

fn beginSleeping(world: *world_mod.World, state: *State, simulation: config.SimulationConfig, commands: []const world_mod.Command, workspace: *Workspace, pipeline_workspace: *AnalyticSolverPipelineWorkspace, sleep: *SleepWorkspace, status: *fp.MathStatus, required_trace: usize) Error!void {
    if (state.in_step) return error.Reentrant;
    if (state.fault != null) return error.Faulted;
    try simulation.validate();
    if (workspace.trace.len < required_trace) return error.TraceCapacity;
    try sleep.storage.validate(world);
    status.clear();
    workspace.trace_len = 0;
    if (workspace.diagnostics) |diagnostics| diagnostics.reset();
    state.in_step = true;
    errdefer state.in_step = false;
    const Prevalidate = struct {
        world: *world_mod.World,
        commands: []const world_mod.Command,
        workspace: *Workspace,
        pipeline_workspace: *AnalyticSolverPipelineWorkspace,
        status: *fp.MathStatus,
        fn run(self: *@This()) Error!void {
            _ = try self.world.orderedCommands(self.commands, self.workspace.commands);
            // Command wake uses the previous canonical contact/joint graph.
            _ = try buildAnalyticIslands(self.world, self.pipeline_workspace.contacts.cache, if (self.pipeline_workspace.joint) |joint| joint.pool else null, self.pipeline_workspace.islands, self.status);
        }
    };
    var prevalidate = Prevalidate{ .world = world, .commands = commands, .workspace = workspace, .pipeline_workspace = pipeline_workspace, .status = status };
    try dispatchPhase(workspace, .prevalidate, &prevalidate, Prevalidate.run);
    const Commit = struct {
        world: *world_mod.World,
        commands: []const world_mod.Command,
        workspace: *Workspace,
        pipeline_workspace: *AnalyticSolverPipelineWorkspace,
        sleep: *SleepWorkspace,
        dt: fp.Fp,
        status: *fp.MathStatus,
        fn run(self: *@This()) Error!void {
            _ = try sleeping.executeCommands(self.world, self.sleep.storage, self.pipeline_workspace.islands.edges[0..self.pipeline_workspace.islands.edge_len], self.commands, self.workspace.commands, self.sleep.requests, self.sleep.graph_scratch, self.sleep.wake_events, self.dt, self.status);
        }
    };
    var commit = Commit{ .world = world, .commands = commands, .workspace = workspace, .pipeline_workspace = pipeline_workspace, .sleep = sleep, .dt = tickDt(simulation, status), .status = status };
    try dispatchPhase(workspace, .commit, &commit, Commit.run);
    try checkMath(state, .commit, status);
}
fn wakeContacts(world: *world_mod.World, sleep: *SleepWorkspace, edges: []const constraints.Edge, contacts: []const contact_solver.Contact) sleeping.Error!void {
    var count: usize = 0;
    for (contacts) |contact| {
        if (count + 2 > sleep.requests.len) return error.CapacityExceeded;
        sleep.requests[count] = .{ .body = contact.body_a, .reason = .contact };
        sleep.requests[count + 1] = .{ .body = contact.body_b, .reason = .contact };
        count += 2;
    }
    _ = try sleeping.wakeGraph(world, sleep.storage, edges, sleep.requests[0..count], sleep.graph_scratch, sleep.wake_events);
}

fn begin(world: *world_mod.World, state: *State, simulation: config.SimulationConfig, commands: []const world_mod.Command, workspace: *Workspace, status: *fp.MathStatus, required_trace: usize) Error!void {
    if (state.in_step) return error.Reentrant;
    if (state.fault != null) return error.Faulted;
    try simulation.validate();
    if (workspace.trace.len < required_trace) return error.TraceCapacity;
    status.clear();
    workspace.trace_len = 0;
    if (workspace.diagnostics) |diagnostics| diagnostics.reset();
    state.in_step = true;
    errdefer state.in_step = false;
    const Prevalidate = struct {
        world: *world_mod.World,
        commands: []const world_mod.Command,
        scratch: []world_mod.Command,
        fn run(self: *@This()) Error!void {
            _ = try self.world.orderedCommands(self.commands, self.scratch);
        }
    };
    var prevalidate = Prevalidate{ .world = world, .commands = commands, .scratch = workspace.commands };
    try dispatchPhase(workspace, .prevalidate, &prevalidate, Prevalidate.run);
    const ordered = workspace.commands[0..commands.len];
    const Commit = struct {
        world: *world_mod.World,
        ordered: []const world_mod.Command,
        scratch: []world_mod.Command,
        dt: fp.Fp,
        status: *fp.MathStatus,
        fn run(self: *@This()) Error!void {
            try self.world.execute(self.ordered, self.scratch, self.dt, self.status);
        }
    };
    var commit = Commit{ .world = world, .ordered = ordered, .scratch = workspace.commands, .dt = tickDt(simulation, status), .status = status };
    try dispatchPhase(workspace, .commit, &commit, Commit.run);
    try checkMath(state, .commit, status);
}
fn checkMath(state: *State, phase: Phase, status: *const fp.MathStatus) Error!void {
    if (status.fault == .none) return;
    recordFault(state, phase, .math, status.fault);
    return error.Faulted;
}
fn recordFault(state: *State, phase: Phase, code: FaultCode, math_fault: fp.MathFault) void {
    state.fault = .{ .tick = state.tick, .phase = phase, .code = code, .detail = if (code == .math) .math else if (code == .ccd) .ccd else if (code == .contact) .contact else .other, .math_fault = math_fault };
}
fn recordFaultError(state: *State, phase: Phase, err: anyerror, math_fault: fp.MathFault) void {
    recordFaultErrorFor(state, phase, err, math_fault, null);
}
fn recordFaultErrorFor(state: *State, phase: Phase, err: anyerror, math_fault: fp.MathFault, object: ?u64) void {
    state.fault = .{ .tick = state.tick, .phase = phase, .object = object, .code = faultCode(err), .detail = faultDetail(err), .math_fault = math_fault };
}
fn finish(state: *State, workspace: *Workspace) Result {
    if (workspace.observer) |observer| observer.transition(null);
    state.tick += 1;
    return .{ .tick = state.tick, .trace = workspace.trace[0..workspace.trace_len], .diagnostics = if (workspace.diagnostics) |diagnostics| diagnostics.* else null };
}
fn faultCode(err: anyerror) FaultCode {
    return switch (err) {
        error.InvalidCollider, error.InvalidBody, error.CapacityExceeded => .world,
        error.InvalidShape, error.InvalidBodyShape, error.InvalidAsset, error.InvalidTransform, error.UnsupportedShape => .shape,
        error.InvalidContact, error.ContactFailure => .contact,
        error.BroadphaseFailure => .broadphase,
        else => .broadphase,
    };
}
fn faultDetail(err: anyerror) FaultDetail {
    return switch (err) {
        error.InvalidBody => .invalid_body,
        error.InvalidCollider => .invalid_collider,
        error.InvalidShape, error.InvalidBodyShape => .invalid_shape,
        error.InvalidAsset => .invalid_asset,
        error.InvalidTransform => .invalid_transform,
        error.CapacityExceeded, error.PairCapacity, error.InsufficientEndpoints, error.InsufficientActive => .capacity_exceeded,
        error.UnsupportedShape => .unsupported_shape,
        error.InvalidContact, error.ContactFailure => .contact,
        error.BroadphaseFailure => .broadphase,
        error.CcdFault => .ccd,
        else => .broadphase,
    };
}
fn isDispatchError(err: anyerror) bool {
    return switch (err) {
        error.Backpressure, error.CallbackFailed, error.Cancelled, error.WorkerFault, error.Reentrant, error.Shutdown => true,
        else => false,
    };
}

fn tickDt(simulation: config.SimulationConfig, status: *fp.MathStatus) fp.Fp {
    return fp.Fp.one.div(fp.Fp.fromInt(@intCast(simulation.iterations.tick_hz)), status);
}
fn substepDt(simulation: config.SimulationConfig, status: *fp.MathStatus) fp.Fp {
    return tickDt(simulation, status).div(fp.Fp.fromInt(@intCast(simulation.iterations.substeps)), status);
}
fn solverSettings(simulation: config.SimulationConfig) contact_solver.Settings {
    return .{ .velocity_iterations = @intCast(simulation.iterations.velocity), .position_iterations = @intCast(simulation.iterations.position), .restitution_threshold = simulation.tolerances.restitution_threshold, .max_position_correction = simulation.tolerances.max_position_correction };
}
fn push(workspace: *Workspace, phase: Phase) void {
    if (workspace.observer) |observer| observer.transition(phase);
    workspace.trace[workspace.trace_len] = phase;
    workspace.trace_len += 1;
    if (workspace.diagnostics) |diagnostics| diagnostics.phase_count[@intFromEnum(phase)] += 1;
}

/// Executes one ordered production phase through the configured synchronous
/// batch backend. The context is borrowed until the barrier returns. Kernel
/// errors are preserved instead of being collapsed into a dispatcher error.
fn dispatchPhase(workspace: *Workspace, phase: Phase, context: anytype, comptime run_fn: anytype) Error!void {
    push(workspace, phase);
    try dispatchWork(workspace, context, run_fn);
}

fn dispatchWork(workspace: *Workspace, context: anytype, comptime run_fn: anytype) Error!void {
    const Context = @TypeOf(context.*);
    const Envelope = struct {
        value: *Context,
        failure: ?Error = null,

        fn run(raw: *anyopaque, index: u32) !void {
            if (index != 0) return error.WorkerFault;
            const self: *@This() = @ptrCast(@alignCast(raw));
            run_fn(self.value) catch |err| {
                self.failure = err;
                return err;
            };
        }
    };
    var envelope = Envelope{ .value = context };
    workspace.dispatcher.dispatch(.{ .context = &envelope, .job_count = 1, .run = Envelope.run }) catch |err| {
        if (envelope.failure) |failure| return failure;
        return err;
    };
}

/// Dispatches canonical contiguous ranges and folds per-job math/error state
/// in logical-job order after the synchronous barrier. Workers never share a
/// MathStatus or choose output ownership.
fn dispatchRanges(workspace: *Workspace, item_count: usize, context: anytype, status: *fp.MathStatus, comptime run_fn: anytype) Error!void {
    return dispatchRangesGrain(workspace, item_count, preferred_phase_grain, context, status, run_fn);
}

fn dispatchRangesGrain(workspace: *Workspace, item_count: usize, grain: u32, context: anytype, status: *fp.MathStatus, comptime run_fn: anytype) Error!void {
    const plan = try jobs.RangePlan.initBounded(item_count, grain, max_phase_jobs);
    if (plan.job_count == 0) return;
    const Context = @TypeOf(context.*);
    const Envelope = struct {
        value: *Context,
        plan: jobs.RangePlan,
        failures: [max_phase_jobs]?Error = [_]?Error{null} ** max_phase_jobs,
        statuses: [max_phase_jobs]fp.MathStatus = [_]fp.MathStatus{.{}} ** max_phase_jobs,

        fn run(raw: *anyopaque, index: u32) !void {
            const self: *@This() = @ptrCast(@alignCast(raw));
            const range = self.plan.range(index) catch |err| {
                self.failures[index] = err;
                return err;
            };
            run_fn(self.value, range, &self.statuses[index]) catch |err| {
                self.failures[index] = err;
                return err;
            };
        }
    };
    var envelope = Envelope{ .value = context, .plan = plan };
    workspace.dispatcher.dispatch(.{ .context = &envelope, .job_count = plan.job_count, .run = Envelope.run }) catch |dispatch_error| {
        for (envelope.failures[0..plan.job_count]) |failure| if (failure) |err| return err;
        return dispatch_error;
    };
    for (envelope.failures[0..plan.job_count]) |failure| if (failure) |err| return err;
    if (status.fault == .none) {
        for (envelope.statuses[0..plan.job_count]) |job_status| if (job_status.fault != .none) {
            status.fault = job_status.fault;
            break;
        };
    }
}
fn compose(parent: geometry.Transform3, local: geometry.Transform3, status: *fp.MathStatus) geometry.Transform3 {
    return .{ .position = parent.apply(local.position, status), .orientation = parent.orientation.mul(local.orientation, status) };
}
fn colliderAt(storage: *const world_mod.ColliderStorage, index: usize) shapes.Collider {
    return .{ .body = storage.body[index], .local = storage.local[index], .shape = storage.shape[index], .material = storage.material[index], .category = storage.category[index], .mask = storage.mask[index], .group = storage.group[index], .sensor = storage.sensor[index], .enabled = storage.enabled[index], .revision = storage.revision[index] };
}
const ColliderRef = struct { collider: shapes.Collider };
fn colliderFor(world: *const world_mod.World, id: @import("../core/ids.zig").ColliderId) world_mod.Error!ColliderRef {
    const storage = world.colliders orelse return error.InvalidCollider;
    const index: usize = id.index();
    if (index >= storage.alive.len or !storage.alive[index] or storage.generation[index] != id.generation()) return error.InvalidCollider;
    return .{ .collider = colliderAt(&storage, index) };
}
fn colliderLess(a: @import("../core/ids.zig").ColliderId, b: @import("../core/ids.zig").ColliderId) bool {
    if (a.index() != b.index()) return a.index() < b.index();
    return a.generation() < b.generation();
}
fn findCachePatch(cache: *contact_cache.Cache, key: contact_cache.ManifoldKey) ?*contact_cache.Patch {
    for (cache.patches[0..cache.len]) |*patch| if (keyEqual(patch.key, key)) return patch;
    return null;
}
fn keyEqual(a: contact_cache.ManifoldKey, b: contact_cache.ManifoldKey) bool {
    return !contact_cache.keyLess(a, b) and !contact_cache.keyLess(b, a);
}
fn toiClosing(world: *const world_mod.World, toi: ccd.Toi, status: *fp.MathStatus) bool {
    const caster = colliderFor(world, toi.caster) catch return false;
    const target = colliderFor(world, toi.target) catch return false;
    const a = world.bodyIndex(caster.collider.body) orelse return false;
    const b = world.bodyIndex(target.collider.body) orelse return false;
    const va = world.storage.linear_velocity[a].add(world.storage.angular_velocity[a].cross(toi.point.sub(world.storage.position[a], status), status), status);
    const vb = world.storage.linear_velocity[b].add(world.storage.angular_velocity[b].cross(toi.point.sub(world.storage.position[b], status), status), status);
    return vb.sub(va, status).dot(toi.normal, status).raw < 0;
}
fn analyticShape(shape: shapes.Shape, transform: geometry.Transform3, status: *fp.MathStatus) ?analytic.QueryShape {
    return switch (shape) {
        .sphere => |value| .{ .sphere = .{ .center = transform.position, .radius = value.radius } },
        .box => |value| .{ .box = .{ .center = transform.position, .half_extents = value.half_extents, .orientation = transform.orientation } },
        .capsule => |value| .{ .capsule = .{ .segment = .{ .a = transform.apply(.{ .y = value.half_height.neg(status) }, status), .b = transform.apply(.{ .y = value.half_height }, status) }, .radius = value.radius } },
        else => null,
    };
}
fn analyticFeature(feature: analytic.Feature) u32 {
    return switch (feature) {
        .sphere => 0,
        .capsule_side => 1,
        .capsule_end => |value| 2 + @as(u32, value),
        .box_face => |value| 4 + @as(u32, value),
        .box_edge => |value| 16 + @as(u32, value),
        .box_vertex => |value| 32 + @as(u32, value),
        .primitive => |value| 64 + value,
    };
}
