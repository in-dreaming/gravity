//! Replays a GRAVREPL through Task 20's production serial solver profile.
const std = @import("std");
const gravity = @import("gravity");

const fp = gravity.math.fp;
const geometry = gravity.math.geometry;
const shapes = gravity.collision.shapes;
const broadphase = gravity.collision.broadphase;
const contact_cache = gravity.collision.contact_cache;
const mesh = gravity.collision.mesh;
const gjk = gravity.collision.gjk;
const contact_solver = gravity.dynamics.contact_solver;
const constraints = gravity.dynamics.constraints;
const world = gravity.dynamics.world;
const joints = gravity.dynamics.joints;
const sleeping = gravity.dynamics.sleeping;
const ccd = gravity.dynamics.ccd;
const pipeline = gravity.dynamics.pipeline;
const asset_store = gravity.assets.store;
const runtime_view = gravity.assets.runtime_view;
const baked = gravity.geometry.baked;
const snapshot = gravity.state.snapshot;
const replay = gravity.state.replay;
const rollback = gravity.state.rollback;

const AssetBounds = struct {
    mesh_nodes: usize = 0,
    mesh_primitives: usize = 0,
    height_nodes: usize = 0,
    height_triangles: usize = 0,
};

fn assetBounds(assets: *const asset_store.Store) !AssetBounds {
    var result = AssetBounds{};
    for (assets.assets) |*asset| {
        const view = try runtime_view.View.init(asset);
        result.mesh_nodes = @max(result.mesh_nodes, view.nodeCount());
        result.mesh_primitives = @max(result.mesh_primitives, @max(view.primitiveCount(), view.triangleCount()));
        result.height_nodes = @max(result.height_nodes, view.heightTileNodeCount());
        result.height_triangles = @max(result.height_triangles, view.heightCellCount() * 2);
    }
    return result;
}

pub fn main(init: std.process.Init) !void {
    var args = try std.process.Args.Iterator.initAllocator(init.minimal.args, init.gpa);
    defer args.deinit();
    _ = args.next();
    const replay_path = args.next() orelse return error.InvalidArguments;

    const recording_bytes = try readFile(init.gpa, init.io, replay_path);
    // A GRAVREPL entry is at least tick(u64)+length(u32)+hash128.  Size the
    // caller-owned decode table from the file rather than imposing a hidden
    // recording-length limit on valid replay corpora.
    const max_entries = recording_bytes.len / 28 + 1;
    const entries = try init.gpa.alloc(replay.Entry, max_entries);
    defer init.gpa.free(entries);
    const decode_arena = try init.gpa.alloc(u8, recording_bytes.len);
    defer init.gpa.free(decode_arena);
    const recording = try replay.decode(recording_bytes, entries, decode_arena);

    var reader = gravity.state.codec.Reader.init(recording.initial_snapshot);
    const header = try snapshot.readHeader(&reader);
    try header.configuration.validate();

    var asset_inputs = try std.ArrayList([]const u8).initCapacity(init.gpa, 0);
    defer asset_inputs.deinit(init.gpa);
    var asset_owned = try std.ArrayList([]u8).initCapacity(init.gpa, 0);
    defer asset_owned.deinit(init.gpa);
    while (args.next()) |path| {
        const bytes = try readFile(init.gpa, init.io, path);
        errdefer init.gpa.free(bytes);
        try asset_owned.append(init.gpa, bytes);
        try asset_inputs.append(init.gpa, bytes);
    }
    defer for (asset_owned.items) |bytes| init.gpa.free(bytes);

    const required_store_memory = try asset_store.Store.memoryRequired(asset_inputs.items);
    const store_memory = try init.gpa.alignedAlloc(u8, std.mem.Alignment.fromByteUnits(@alignOf(asset_store.Asset)), @max(required_store_memory, 1));
    defer init.gpa.free(store_memory);
    const assets = try asset_store.Store.init(store_memory, asset_inputs.items);
    if (!std.mem.eql(u8, &assets.asset_set_hash, &header.asset_set)) return error.AssetSetMismatch;

    var host = try Host.init(init.gpa, header, &assets);
    defer host.deinit();
    const result = try replay.run(&host.full, recording, replay.FullWorldHost.load, replay.FullWorldHost.step);
    if (result.first_mismatch) |index| {
        const entry = recording.entries[index];
        std.debug.print("GRAVREPL mismatch: entry {d}, tick {d}\n", .{ index, entry.tick });
        return error.HashMismatch;
    }
    std.debug.print("GRAVREPL verified: {d} ticks\n", .{recording.entries.len});
}

fn readFile(allocator: std.mem.Allocator, io: std.Io, path: []const u8) ![]u8 {
    var file = try std.Io.Dir.openFile(.cwd(), io, path, .{});
    defer file.close(io);
    const length: usize = @intCast(try file.length(io));
    const bytes = try allocator.alloc(u8, length);
    errdefer allocator.free(bytes);
    var buffer: [4096]u8 = undefined;
    var file_reader = file.reader(io, &buffer);
    try file_reader.interface.readSliceAll(bytes);
    return bytes;
}

fn epaWorkspace(allocator: std.mem.Allocator, faces: usize) !gjk.EpaWorkspace {
    return .{ .vertices = try allocator.alloc(gjk.SupportVertex, faces), .faces = try allocator.alloc(gjk.EpaFace, faces), .visible = try allocator.alloc(bool, faces), .horizon = try allocator.alloc(gjk.HorizonEdge, faces * 3) };
}

fn triangleWorkspace(allocator: std.mem.Allocator, faces: usize) !mesh.ConvexTriangleWorkspace {
    return .{ .epa = try epaWorkspace(allocator, faces) };
}

fn manifoldWorkspace(allocator: std.mem.Allocator, vertices: usize, faces: usize, contacts: usize) !gjk.ManifoldWorkspace {
    return .{ .epa = try epaWorkspace(allocator, faces), .reference = try allocator.alloc(gjk.ClipVertex, vertices), .incident = try allocator.alloc(gjk.ClipVertex, vertices), .scratch_a = try allocator.alloc(gjk.ClipVertex, vertices), .scratch_b = try allocator.alloc(gjk.ClipVertex, vertices), .contacts = try allocator.alloc(gjk.ContactPoint, contacts) };
}

fn sphereMeshWorkspace(allocator: std.mem.Allocator, b: AssetBounds, pairs: usize, contacts: usize) !mesh.SphereMeshWorkspace {
    return .{ .nodes = try allocator.alloc(baked.BvhNode, b.mesh_nodes), .primitives = try allocator.alloc(u32, b.mesh_primitives), .work = try allocator.alloc(mesh.NodePair, pairs), .pair_scratch = try allocator.alloc(mesh.PrimitivePair, pairs), .pair_output = try allocator.alloc(mesh.PrimitivePair, pairs), .contacts = try allocator.alloc(gjk.ContactPoint, contacts) };
}

fn sphereHeightfieldWorkspace(allocator: std.mem.Allocator, b: AssetBounds, pairs: usize, contacts: usize) !mesh.SphereHeightfieldWorkspace {
    return .{ .tile_nodes = try allocator.alloc(baked.BvhNode, b.height_nodes), .work = try allocator.alloc(mesh.NodePair, pairs), .pair_scratch = try allocator.alloc(mesh.PrimitivePair, pairs), .pair_output = try allocator.alloc(mesh.PrimitivePair, pairs), .triangles = try allocator.alloc(mesh.HeightTriangle, b.height_triangles), .contacts = try allocator.alloc(gjk.ContactPoint, contacts) };
}

fn convexMeshPatchWorkspace(allocator: std.mem.Allocator, b: AssetBounds, pairs: usize, contacts: usize, faces: usize) !mesh.ConvexMeshPatchWorkspace {
    return .{ .query = .{ .nodes = try allocator.alloc(baked.BvhNode, b.mesh_nodes), .primitives = try allocator.alloc(u32, b.mesh_primitives), .work = try allocator.alloc(mesh.NodePair, pairs), .pair_scratch = try allocator.alloc(mesh.PrimitivePair, pairs), .pair_output = try allocator.alloc(mesh.PrimitivePair, pairs), .intersections = try allocator.alloc(u32, pairs) }, .triangle = try triangleWorkspace(allocator, faces), .contacts = try allocator.alloc(gjk.ContactPoint, contacts) };
}

fn convexHeightfieldPatchWorkspace(allocator: std.mem.Allocator, b: AssetBounds, pairs: usize, contacts: usize, faces: usize) !mesh.ConvexHeightfieldPatchWorkspace {
    return .{ .query = .{ .tile_nodes = try allocator.alloc(baked.BvhNode, b.height_nodes), .work = try allocator.alloc(mesh.NodePair, pairs), .pair_scratch = try allocator.alloc(mesh.PrimitivePair, pairs), .pair_output = try allocator.alloc(mesh.PrimitivePair, pairs), .triangles = try allocator.alloc(mesh.HeightTriangle, b.height_triangles), .intersections = try allocator.alloc(u32, pairs) }, .triangle = try triangleWorkspace(allocator, faces), .contacts = try allocator.alloc(gjk.ContactPoint, contacts) };
}

fn surfaceNarrowWorkspace(allocator: std.mem.Allocator, b: AssetBounds, pairs: usize, contacts: usize, leaves: usize, faces: usize) !pipeline.SurfaceNarrowWorkspace {
    return .{ .sphere_mesh = try sphereMeshWorkspace(allocator, b, pairs, contacts), .sphere_heightfield = try sphereHeightfieldWorkspace(allocator, b, pairs, contacts), .convex_mesh = try convexMeshPatchWorkspace(allocator, b, pairs, contacts, faces), .convex_heightfield = try convexHeightfieldPatchWorkspace(allocator, b, pairs, contacts, faces), .mesh_mesh = .{ .query = .{ .nodes_a = try allocator.alloc(baked.BvhNode, b.mesh_nodes), .primitives_a = try allocator.alloc(u32, b.mesh_primitives), .nodes_b = try allocator.alloc(baked.BvhNode, b.mesh_nodes), .primitives_b = try allocator.alloc(u32, b.mesh_primitives), .work = try allocator.alloc(mesh.NodePair, pairs), .pair_scratch = try allocator.alloc(mesh.PrimitivePair, pairs), .pair_output = try allocator.alloc(mesh.PrimitivePair, pairs), .overlaps = try allocator.alloc(mesh.PrimitivePair, pairs) }, .contacts = try allocator.alloc(gjk.ContactPoint, contacts) }, .sphere_compound = .{ .leaves = try allocator.alloc(shapes.CompoundLeaf, leaves), .mesh = try sphereMeshWorkspace(allocator, b, pairs, contacts), .heightfield = try sphereHeightfieldWorkspace(allocator, b, pairs, contacts), .merged = try allocator.alloc(gjk.ContactPoint, contacts) }, .convex_compound = .{ .leaves = try allocator.alloc(shapes.CompoundLeaf, leaves), .mesh = try convexMeshPatchWorkspace(allocator, b, pairs, contacts, faces), .heightfield = try convexHeightfieldPatchWorkspace(allocator, b, pairs, contacts, faces), .merged = try allocator.alloc(gjk.ContactPoint, contacts) } };
}

fn surfaceCastWorkspace(allocator: std.mem.Allocator, b: AssetBounds, leaves: usize) !gravity.query.queries.SurfaceCastWorkspace {
    return .{ .compound_leaves = try allocator.alloc(shapes.CompoundLeaf, leaves), .mesh = .{ .nodes = try allocator.alloc(baked.BvhNode, b.mesh_nodes), .primitives = try allocator.alloc(u32, b.mesh_primitives), .stack = try allocator.alloc(u32, b.mesh_nodes) }, .heightfield = .{ .stack = try allocator.alloc(u32, b.height_nodes), .triangles = try allocator.alloc(mesh.HeightTriangle, b.height_triangles) } };
}

/// Tool-owned storage.  It is created once before `replay.run`; no member is
/// allocated or resized while `FullWorldHost.step` is executing.
const Host = struct {
    allocator: std.mem.Allocator,
    header: snapshot.Header,
    assets: *const asset_store.Store,
    target: *world.World,
    stage: *world.World,
    target_joints: *joints.Pool,
    stage_joints: *joints.Pool,
    state: pipeline.State = .{},
    contacts: *contact_cache.Cache,
    stage_contacts: []contact_cache.Patch,
    contact_scratch: []contact_cache.Patch,
    sleep: sleeping.Storage,
    stage_sleep: sleeping.Storage,
    ccd_enabled: []bool,
    stage_ccd: []bool,
    decoded_commands: []world.Command,
    workspace: pipeline.Workspace,
    solver_pipeline: pipeline.AnalyticSolverPipelineWorkspace,
    status: fp.MathStatus = .{},
    base: replay.IntegrationOnlyHost = undefined,
    full: replay.FullWorldHost = undefined,

    fn init(allocator: std.mem.Allocator, header: snapshot.Header, assets: *const asset_store.Store) !*Host {
        const c = header.configuration.capacities;
        const body_count: usize = c.body;
        const collider_count: usize = c.collider;
        const joint_count: usize = c.joint;
        const patch_count: usize = c.contact_patch;
        const point_count: usize = c.contact_point;
        const pair_count: usize = c.broad_pair;
        const event_count: usize = c.event_per_tick;
        const bounds = try assetBounds(assets);
        const compound_leaves: usize = c.compound_children * c.compound_depth;
        const epa_faces: usize = header.configuration.iterations.epa_max_faces;
        const hull_vertices: usize = c.convex_hull_vertices;

        const target = try allocator.create(world.World);
        target.* = try makeWorld(allocator, body_count, collider_count);
        const stage = try allocator.create(world.World);
        stage.* = try makeWorld(allocator, body_count, collider_count);
        const target_joints = try allocator.create(joints.Pool);
        target_joints.* = try makeJoints(allocator, joint_count);
        const stage_joints = try allocator.create(joints.Pool);
        stage_joints.* = try makeJoints(allocator, joint_count);
        const contacts = try allocator.create(contact_cache.Cache);
        contacts.* = .{ .patches = try allocator.alloc(contact_cache.Patch, patch_count) };
        const stage_contacts = try allocator.alloc(contact_cache.Patch, patch_count);
        const contact_scratch = try allocator.alloc(contact_cache.Patch, patch_count);
        const sleep = try makeSleep(allocator, body_count);
        const stage_sleep = try makeSleep(allocator, body_count);
        const ccd_enabled = try allocator.alloc(bool, collider_count);
        const stage_ccd = try allocator.alloc(bool, collider_count);
        @memset(ccd_enabled, false);

        const commands = try allocator.alloc(world.Command, c.command_per_tick);
        const trace = try allocator.alloc(pipeline.Phase, 6 + @as(usize, header.configuration.iterations.substeps) * 6);
        const workspace = pipeline.Workspace{ .commands = commands, .trace = trace };

        const collider_views = try allocator.alloc(shapes.Collider, collider_count);
        const proxies = try allocator.alloc(broadphase.Proxy, collider_count);
        const sap = try allocator.create(broadphase.Buffers);
        sap.* = .{ .endpoints = try allocator.alloc(broadphase.Endpoint, collider_count * 2), .endpoint_scratch = try allocator.alloc(broadphase.Endpoint, collider_count * 2), .active = try allocator.alloc(u32, collider_count), .pairs = try allocator.alloc(broadphase.PairKey, pair_count), .pair_work = try allocator.alloc(broadphase.PairKey, pair_count), .pair_scratch = try allocator.alloc(broadphase.PairKey, pair_count) };
        const broad = try allocator.create(pipeline.BroadphaseWorkspace);
        broad.* = .{ .assets = assets, .collider_views = collider_views, .proxies = proxies, .buffers = sap };
        const contact_workspace = try allocator.create(pipeline.AnalyticContactWorkspace);
        const convex_workspace = try allocator.create(pipeline.ConvexNarrowWorkspace);
        convex_workspace.* = .{ .manifold = try manifoldWorkspace(allocator, hull_vertices, epa_faces, point_count) };
        const surface_workspace = try allocator.create(pipeline.SurfaceNarrowWorkspace);
        surface_workspace.* = try surfaceNarrowWorkspace(allocator, bounds, pair_count, point_count, compound_leaves, epa_faces);
        contact_workspace.* = .{ .broadphase = broad, .narrow = try allocator.alloc(contact_cache.Patch, patch_count), .convex = convex_workspace, .surface = surface_workspace, .cache = contacts, .cache_next = try allocator.alloc(contact_cache.Patch, patch_count), .events = try allocator.alloc(contact_cache.Event, event_count) };

        const solver = try allocator.create(pipeline.AnalyticSolverWorkspace);
        solver.* = .{ .contacts = try allocator.alloc(contact_solver.Contact, patch_count), .points = try allocator.alloc(contact_solver.Point, point_count), .restitution_bias = try allocator.alloc(fp.Fp, point_count), .pseudo = .{ .linear = try allocator.alloc(geometry.Vec3, body_count), .angular = try allocator.alloc(geometry.Vec3, body_count) }, .manifold = try manifoldWorkspace(allocator, hull_vertices, epa_faces, point_count), .surface = surface_workspace.* };
        const islands = try allocator.create(pipeline.IslandWorkspace);
        islands.* = .{ .edges = try allocator.alloc(constraints.Edge, patch_count + joint_count + body_count * 6), .edge_scratch = try allocator.alloc(constraints.Edge, patch_count + joint_count + body_count * 6), .islands = try allocator.alloc(constraints.Island, body_count), .members = try allocator.alloc(gravity.core.ids.BodyId, body_count), .lock_rows = try allocator.alloc(constraints.ConstraintRow, body_count * 6) };
        const joint_workspace = try allocator.create(pipeline.JointWorkspace);
        // Task 16's widest joint produces twelve temporary rows even though
        // its persisted impulse vector contains eight entries.
        joint_workspace.* = .{ .pool = target_joints, .rows = try allocator.alloc(constraints.ConstraintRow, joint_count * 12), .scratch = .{ .authored = try allocator.alloc(constraints.ConstraintRow, joint_count * 12), .build = try allocator.alloc(constraints.ConstraintRow, joint_count * 12), .states = try allocator.alloc(joints.MutableState, joint_count) } };
        const sleep_workspace = try allocator.create(pipeline.SleepWorkspace);
        sleep_workspace.* = .{ .storage = sleep, .requests = try allocator.alloc(sleeping.Request, patch_count * 2 + joint_count + c.command_per_tick), .graph_scratch = try allocator.alloc(gravity.core.ids.BodyId, body_count), .wake_events = try allocator.alloc(sleeping.Event, event_count), .sleep_events = try allocator.alloc(sleeping.Event, event_count) };

        const ccd_workspace = try allocator.create(pipeline.CcdPipelineWorkspace);
        const ccd_items = try allocator.create(pipeline.CcdItemWorkspace);
        ccd_items.* = .{ .enabled = ccd_enabled, .items = try allocator.alloc(ccd.Item, collider_count) };
        ccd_workspace.* = .{ .assets = assets, .items = ccd_items, .pairs = try allocator.alloc(ccd.Pair, pair_count), .surface = try surfaceCastWorkspace(allocator, bounds, compound_leaves), .patches = try allocator.alloc(contact_cache.Patch, patch_count), .merge_input = try allocator.alloc(contact_cache.Patch, patch_count) };

        const result = try allocator.create(Host);
        result.* = .{ .allocator = allocator, .header = header, .assets = assets, .target = target, .stage = stage, .target_joints = target_joints, .stage_joints = stage_joints, .contacts = contacts, .stage_contacts = stage_contacts, .contact_scratch = contact_scratch, .sleep = sleep, .stage_sleep = stage_sleep, .ccd_enabled = ccd_enabled, .stage_ccd = stage_ccd, .decoded_commands = try allocator.alloc(world.Command, c.command_per_tick), .workspace = workspace, .solver_pipeline = .{ .contacts = contact_workspace, .solver = solver, .islands = islands, .joint = joint_workspace, .sleep = sleep_workspace, .ccd = ccd_workspace, .substep_events = try allocator.alloc(contact_cache.Event, event_count), .previous = try allocator.alloc(contact_cache.Patch, patch_count), .event_next = try allocator.alloc(contact_cache.Patch, patch_count), .tick_events = try allocator.alloc(contact_cache.Event, event_count) } };
        const Rebuild = struct {
            fn run(_: ?*anyopaque) void {}
        };
        result.base = .{ .header = header, .state = &result.state, .value = result.target, .stage_world = result.stage, .contacts = result.contacts, .stage_contacts = result.stage_contacts, .contact_scratch = result.contact_scratch, .joint_pool = result.target_joints, .stage_joint_pool = result.stage_joints, .sleep = result.sleep, .stage_sleep = result.stage_sleep, .ccd_enabled = result.ccd_enabled, .stage_ccd = result.stage_ccd, .decoded_commands = result.decoded_commands, .workspace = &result.workspace, .math_status = &result.status, .rebuild_context = null, .rebuild = Rebuild.run };
        result.full = .{ .base = &result.base, .solver_workspace = &result.solver_pipeline };
        return result;
    }

    fn deinit(self: *Host) void {
        // The CLI allocator owns the process-lifetime workspace.  Releasing
        // each nested slice is unnecessary at process exit and would obscure
        // the single ownership rule used by the simulation itself.
        _ = self;
    }
};

fn makeWorld(allocator: std.mem.Allocator, bodies: usize, colliders: usize) !world.World {
    return world.World.initWithColliders(.{ .body_type = try allocator.alloc(shapes.BodyType, bodies), .position = try allocator.alloc(geometry.Vec3, bodies), .orientation = try allocator.alloc(geometry.Quat, bodies), .linear_velocity = try allocator.alloc(geometry.Vec3, bodies), .angular_velocity = try allocator.alloc(geometry.Vec3, bodies), .inverse_mass = try allocator.alloc(fp.Fp, bodies), .inverse_inertia_local = try allocator.alloc(geometry.SymmetricMat3, bodies), .force = try allocator.alloc(geometry.Vec3, bodies), .torque = try allocator.alloc(geometry.Vec3, bodies), .locks = try allocator.alloc(world.DofLock, bodies), .generation = try allocator.alloc(u32, bodies), .alive = try allocator.alloc(bool, bodies), .retired = try allocator.alloc(bool, bodies), .has_target = try allocator.alloc(bool, bodies), .target_position = try allocator.alloc(geometry.Vec3, bodies), .target_orientation = try allocator.alloc(geometry.Quat, bodies) }, .{ .body = try allocator.alloc(gravity.core.ids.BodyId, colliders), .local = try allocator.alloc(geometry.Transform3, colliders), .shape = try allocator.alloc(shapes.Shape, colliders), .material = try allocator.alloc(shapes.Material, colliders), .category = try allocator.alloc(u32, colliders), .mask = try allocator.alloc(u32, colliders), .group = try allocator.alloc(i32, colliders), .sensor = try allocator.alloc(bool, colliders), .enabled = try allocator.alloc(bool, colliders), .revision = try allocator.alloc(u32, colliders), .generation = try allocator.alloc(u32, colliders), .alive = try allocator.alloc(bool, colliders), .retired = try allocator.alloc(bool, colliders) });
}

fn makeJoints(allocator: std.mem.Allocator, count: usize) !joints.Pool {
    return joints.Pool.init(.{ .values = try allocator.alloc(joints.Joint, count), .generation = try allocator.alloc(u32, count), .alive = try allocator.alloc(bool, count), .retired = try allocator.alloc(bool, count) });
}

fn makeSleep(allocator: std.mem.Allocator, count: usize) !sleeping.Storage {
    const value = sleeping.Storage{ .awake = try allocator.alloc(bool, count), .counter = try allocator.alloc(u32, count), .reason = try allocator.alloc(sleeping.WakeReason, count) };
    try sleeping.init(value);
    return value;
}

test "production replay host loads an asset-free snapshot and runs the serial pipeline" {
    var configuration = gravity.core.config.SimulationConfig.default;
    configuration.capacities.body = 1;
    configuration.capacities.collider = 1;
    configuration.capacities.joint = 1;
    configuration.capacities.command_per_tick = 1;
    configuration.capacities.broad_pair = 1;
    configuration.capacities.contact_patch = 1;
    configuration.capacities.contact_point = 1;
    configuration.capacities.sensor_overlap = 1;
    configuration.capacities.event_per_tick = 1;
    configuration.capacities.rollback_window = 1;
    try configuration.validate();
    var asset_memory: [1]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&asset_memory, &.{});
    const header = snapshot.Header{ .configuration = configuration, .asset_set = assets.asset_set_hash };
    const allocator = std.heap.page_allocator;
    var source = try Host.init(allocator, header, &assets);
    var snapshot_bytes: [4096]u8 = undefined;
    var pipeline_bytes: [128]u8 = undefined;
    var bodies_bytes: [1024]u8 = undefined;
    var colliders_bytes: [1024]u8 = undefined;
    var contacts_bytes: [1024]u8 = undefined;
    var joints_bytes: [1024]u8 = undefined;
    var sleep_bytes: [1024]u8 = undefined;
    var ccd_bytes: [1024]u8 = undefined;
    const initial = try snapshot.encodeFullSnapshot(header, source.state, source.target, source.contacts, source.target_joints, source.sleep, source.ccd_enabled, &snapshot_bytes, &pipeline_bytes, &bodies_bytes, &colliders_bytes, &contacts_bytes, &joints_bytes, &sleep_bytes, &ccd_bytes);
    var input_bytes: [4]u8 = undefined;
    const input = try replay.encodeCommands(&.{}, &input_bytes);
    const provisional = replay.Entry{ .tick = 1, .input = input, .expected_hash = [_]u8{0} ** 16 };
    const expected = try replay.FullWorldHost.step(&source.full, provisional);
    var target = try Host.init(allocator, header, &assets);
    const entries = [_]replay.Entry{.{ .tick = 1, .input = input, .expected_hash = expected }};
    const result = try replay.run(&target.full, .{ .initial_snapshot = initial, .entries = &entries }, replay.FullWorldHost.load, replay.FullWorldHost.step);
    try std.testing.expect(result.first_mismatch == null);
}

test "production replay host replays an asset-backed mesh contact" {
    const vertices = [_]geometry.Vec3{
        .{ .x = fp.Fp.fromInt(-2), .z = fp.Fp.fromInt(-2) },
        .{ .x = fp.Fp.fromInt(2), .z = fp.Fp.fromInt(-2) },
        .{ .z = fp.Fp.fromInt(2) },
    };
    const triangles = [_]baked.Triangle{.{ .a = 0, .b = 1, .c = 2 }};
    const nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = .{ .x = fp.Fp.fromInt(-2), .z = fp.Fp.fromInt(-2) }, .max = .{ .x = fp.Fp.fromInt(2), .z = fp.Fp.fromInt(2) } }, 0, 1)};
    const primitives = [_]u32{0};
    var asset_bytes: [512]u8 = undefined;
    var bake_scratch: [256]u8 = undefined;
    const encoded_asset = try baked.encodeMesh(.{ .source_id = 61, .vertices = &vertices, .triangles = &triangles, .nodes = &nodes, .primitives = &primitives }, &asset_bytes, &bake_scratch);
    var asset_memory: [1024]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&asset_memory, &.{encoded_asset.bytes});

    var configuration = gravity.core.config.SimulationConfig.default;
    configuration.capacities.body = 2;
    configuration.capacities.collider = 2;
    configuration.capacities.joint = 1;
    configuration.capacities.command_per_tick = 1;
    configuration.capacities.broad_pair = 4;
    configuration.capacities.contact_patch = 4;
    configuration.capacities.contact_point = 16;
    configuration.capacities.sensor_overlap = 4;
    configuration.capacities.event_per_tick = 16;
    configuration.capacities.rollback_window = 120;
    try configuration.validate();
    const header = snapshot.Header{ .configuration = configuration, .asset_set = assets.asset_set_hash };
    const allocator = std.heap.page_allocator;
    var source = try Host.init(allocator, header, &assets);
    const inertia = geometry.SymmetricMat3{ .xx = .one, .yy = .one, .zz = .one, .xy = .zero, .xz = .zero, .yz = .zero };
    const mesh_body = try source.target.create(.{ .body_type = .static, .inverse_inertia_local = inertia }, &source.status);
    const box_body = try source.target.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia, .transform = .{ .position = .{ .y = fp.Fp.fromRatio(1, 2, &source.status) } } }, &source.status);
    _ = try source.target.createCollider(.{ .body = mesh_body, .shape = .{ .triangle_mesh = .{ .source_id = 61 } } });
    const box_collider = try source.target.createCollider(.{ .body = box_body, .shape = .{ .box = .{ .half_extents = .{ .x = .one, .y = .one, .z = .one } } } });
    _ = try source.target_joints.create(source.target, .{ .kind = .distance, .body_a = mesh_body, .body_b = box_body }, &source.status);
    source.sleep.counter[box_body.index()] = 7;
    source.ccd_enabled[box_collider.index()] = true;

    var snapshot_bytes: [16384]u8 = undefined;
    var pipeline_bytes: [128]u8 = undefined;
    var bodies_bytes: [2048]u8 = undefined;
    var colliders_bytes: [2048]u8 = undefined;
    var contacts_bytes: [4096]u8 = undefined;
    var joints_bytes: [1024]u8 = undefined;
    var sleep_bytes: [1024]u8 = undefined;
    var ccd_bytes: [1024]u8 = undefined;
    const initial = try snapshot.encodeFullSnapshot(header, source.state, source.target, source.contacts, source.target_joints, source.sleep, source.ccd_enabled, &snapshot_bytes, &pipeline_bytes, &bodies_bytes, &colliders_bytes, &contacts_bytes, &joints_bytes, &sleep_bytes, &ccd_bytes);
    var input_bytes: [4]u8 = undefined;
    const input = try replay.encodeCommands(&.{}, &input_bytes);
    const expected = try replay.FullWorldHost.step(&source.full, .{ .tick = 1, .input = input, .expected_hash = [_]u8{0} ** 16 });
    try std.testing.expect(source.contacts.len > 0);

    var target = try Host.init(allocator, header, &assets);
    const entries = [_]replay.Entry{.{ .tick = 1, .input = input, .expected_hash = expected }};
    const result = try replay.run(&target.full, .{ .initial_snapshot = initial, .entries = &entries }, replay.FullWorldHost.load, replay.FullWorldHost.step);
    try std.testing.expectEqual(@as(?usize, null), result.first_mismatch);
    try std.testing.expect(target.contacts.len > 0);
    try std.testing.expect(target.target_joints.storage.alive[0]);
    try std.testing.expect(target.ccd_enabled[box_collider.index()]);
}

test "production replay host survives 100k random full snapshot rollbacks" {
    var configuration = gravity.core.config.SimulationConfig.default;
    configuration.capacities.body = 1;
    configuration.capacities.collider = 1;
    configuration.capacities.joint = 1;
    configuration.capacities.command_per_tick = 1;
    configuration.capacities.broad_pair = 1;
    configuration.capacities.contact_patch = 1;
    configuration.capacities.contact_point = 1;
    configuration.capacities.sensor_overlap = 1;
    configuration.capacities.event_per_tick = 1;
    configuration.capacities.rollback_window = 120;
    try configuration.validate();
    var asset_memory: [1]u8 align(@alignOf(asset_store.Asset)) = undefined;
    const assets = try asset_store.Store.init(&asset_memory, &.{});
    const header = snapshot.Header{ .configuration = configuration, .asset_set = assets.asset_set_hash };
    const allocator = std.heap.page_allocator;
    var continuous = try Host.init(allocator, header, &assets);
    var restored = try Host.init(allocator, header, &assets);
    const inertia = geometry.SymmetricMat3{ .xx = .one, .yy = .one, .zz = .one, .xy = .zero, .xz = .zero, .yz = .zero };
    const body = try continuous.target.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia }, &continuous.status);
    continuous.target.storage.linear_velocity[body.index()] = .{ .x = .one };

    var ticks: [120]u64 = undefined;
    var snapshot_lengths: [120]usize = undefined;
    var input_lengths: [120]usize = undefined;
    var hashes: [120]gravity.state.hash.Hash128 = undefined;
    var snapshots: [120 * 4096]u8 = undefined;
    var inputs: [120 * 4]u8 = undefined;
    var valid: [120]bool = undefined;
    var ring = try rollback.Ring.init(&ticks, &snapshot_lengths, &input_lengths, &hashes, &snapshots, &inputs, &valid, 4096, 4);
    var input_bytes: [4]u8 = undefined;
    const input = try replay.encodeCommands(&.{}, &input_bytes);
    var snapshot_bytes: [4096]u8 = undefined;
    var pipeline_bytes: [128]u8 = undefined;
    var bodies_bytes: [1024]u8 = undefined;
    var colliders_bytes: [512]u8 = undefined;
    var contacts_bytes: [512]u8 = undefined;
    var joints_bytes: [512]u8 = undefined;
    var sleep_bytes: [512]u8 = undefined;
    var ccd_bytes: [512]u8 = undefined;
    var seed: u32 = 0x21c0_ffee;
    for (1..100_001) |tick_value| {
        const tick: u64 = @intCast(tick_value);
        const actual_hash = try replay.FullWorldHost.step(&continuous.full, .{ .tick = tick, .input = input, .expected_hash = [_]u8{0} ** 16 });
        const encoded = try snapshot.encodeFullSnapshot(header, continuous.state, continuous.target, continuous.contacts, continuous.target_joints, continuous.sleep, continuous.ccd_enabled, &snapshot_bytes, &pipeline_bytes, &bodies_bytes, &colliders_bytes, &contacts_bytes, &joints_bytes, &sleep_bytes, &ccd_bytes);
        try ring.save(tick, encoded, input, actual_hash);
        if (tick < 2) continue;
        seed = seed *% 1_664_525 +% 1_013_904_223;
        const span = @min(tick - 2, @as(u64, 118));
        const rollback_tick = tick - 1 - @as(u64, seed % @as(u32, @intCast(span + 1)));
        const before = try ring.get(rollback_tick);
        const expected = try ring.get(rollback_tick + 1);
        restored.status = .{};
        try replay.FullWorldHost.load(&restored.full, before.snapshot);
        const replayed_hash = try replay.FullWorldHost.step(&restored.full, .{ .tick = rollback_tick + 1, .input = before.input, .expected_hash = expected.state_hash });
        try std.testing.expectEqualSlices(u8, &expected.state_hash, &replayed_hash);
    }
}
