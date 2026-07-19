const std = @import("std");
const gravity = @import("gravity");
const fp = gravity.math.fp;
const g = gravity.math.geometry;
const shapes = gravity.collision.shapes;
const world = gravity.dynamics.world;
const pipeline = gravity.dynamics.pipeline;
const broadphase = gravity.collision.broadphase;
const contact_cache = gravity.collision.contact_cache;
const gjk = gravity.collision.gjk;
const mesh = gravity.collision.mesh;
const baked = gravity.geometry.baked;
const store = gravity.assets.store;
const joints = gravity.dynamics.joints;
const constraints = gravity.dynamics.constraints;
const jobs = gravity.jobs;

const Fixture = struct {
    types: [2]shapes.BodyType = undefined,
    position: [2]g.Vec3 = undefined,
    orientation: [2]g.Quat = undefined,
    linear: [2]g.Vec3 = undefined,
    angular: [2]g.Vec3 = undefined,
    mass: [2]fp.Fp = undefined,
    inertia: [2]g.SymmetricMat3 = undefined,
    force: [2]g.Vec3 = undefined,
    torque: [2]g.Vec3 = undefined,
    locks: [2]world.DofLock = undefined,
    generation: [2]u32 = undefined,
    alive: [2]bool = undefined,
    retired: [2]bool = undefined,
    target: [2]bool = undefined,
    target_position: [2]g.Vec3 = undefined,
    target_orientation: [2]g.Quat = undefined,
    fn init(self: *Fixture) !world.World {
        return world.World.init(.{ .body_type = &self.types, .position = &self.position, .orientation = &self.orientation, .linear_velocity = &self.linear, .angular_velocity = &self.angular, .inverse_mass = &self.mass, .inverse_inertia_local = &self.inertia, .force = &self.force, .torque = &self.torque, .locks = &self.locks, .generation = &self.generation, .alive = &self.alive, .retired = &self.retired, .has_target = &self.target, .target_position = &self.target_position, .target_orientation = &self.target_orientation });
    }
};
fn inertia() g.SymmetricMat3 {
    return .{ .xx = .one, .yy = .one, .zz = .one, .xy = .zero, .xz = .zero, .yz = .zero };
}

test "pipeline command prefix is ordered and invalid input is atomic" {
    var fixture: Fixture = .{};
    var state = try fixture.init();
    var math = fp.MathStatus{};
    const body = try state.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &math);
    var persistent = pipeline.State{};
    var command_scratch: [2]world.Command = undefined;
    var trace: [4]pipeline.Phase = undefined;
    var diagnostics = pipeline.Diagnostics{};
    var workspace = pipeline.Workspace{ .commands = &command_scratch, .trace = &trace, .diagnostics = &diagnostics };
    var hash_patches: [0]contact_cache.Patch = .{};
    var hash_cache = contact_cache.Cache{ .patches = &hash_patches };
    const hash_before = pipeline.canonicalStateHash(&state, &persistent, .default, .{ .cache = &hash_cache });
    const hash_repeat = pipeline.canonicalStateHash(&state, &persistent, .default, .{ .cache = &hash_cache });
    try std.testing.expectEqualSlices(u8, &hash_before, &hash_repeat);
    state.settings.gravity.x = fp.Fp.one;
    const changed_settings_hash = pipeline.canonicalStateHash(&state, &persistent, .default, .{ .cache = &hash_cache });
    try std.testing.expect(!std.mem.eql(u8, &hash_before, &changed_settings_hash));
    state.settings.gravity.x = fp.Fp.zero;
    const command = world.Command{ .key = .{ .phase_priority = 0, .issuer = 1, .sequence = 1, .discriminant = 0 }, .op = .{ .velocity = .{ .body = body, .linear = .{ .x = .one }, .angular = .{} } } };
    const result = try pipeline.stepBodies(&state, &persistent, .default, &.{command}, &workspace, &math);
    try std.testing.expectEqual(@as(u64, 1), result.tick);
    try std.testing.expectEqualSlices(pipeline.Phase, &.{ .prevalidate, .commit, .integrate, .integrate }, result.trace);
    try std.testing.expectEqual(@as(u32, 2), result.diagnostics.?.visits(.integrate));
    try std.testing.expectEqual(@as(u32, 1), result.diagnostics.?.visits(.prevalidate));
    try std.testing.expect(state.storage.position[body.index()].x.raw > 0);
    const hash_after = pipeline.canonicalStateHash(&state, &persistent, .default, .{ .cache = &hash_cache });
    try std.testing.expect(!std.mem.eql(u8, &hash_before, &hash_after));
    const before = state.storage.position[body.index()];
    const invalid = world.Command{ .key = command.key, .op = .{ .force = .{ .body = gravity.core.ids.BodyId.invalid, .value = .{ .x = .one } } } };
    try std.testing.expectError(error.InvalidBody, pipeline.stepBodies(&state, &persistent, .default, &.{invalid}, &workspace, &math));
    try std.testing.expectEqualDeep(before, state.storage.position[body.index()]);
}

test "pipeline step rebuilds SAP pairs for every substep" {
    var fixture: Fixture = .{};
    const state = try fixture.init();
    var math = fp.MathStatus{};
    var collider_body: [2]gravity.core.ids.BodyId = undefined;
    var collider_local: [2]g.Transform3 = undefined;
    var collider_shape: [2]shapes.Shape = undefined;
    var material: [2]shapes.Material = undefined;
    var category: [2]u32 = undefined;
    var mask: [2]u32 = undefined;
    var group: [2]i32 = undefined;
    var sensor: [2]bool = undefined;
    var enabled: [2]bool = undefined;
    var revision: [2]u32 = undefined;
    var collider_generation: [2]u32 = undefined;
    var collider_alive: [2]bool = undefined;
    var collider_retired: [2]bool = undefined;
    var with_colliders = try world.World.initWithColliders(state.storage, .{ .body = &collider_body, .local = &collider_local, .shape = &collider_shape, .material = &material, .category = &category, .mask = &mask, .group = &group, .sensor = &sensor, .enabled = &enabled, .revision = &revision, .generation = &collider_generation, .alive = &collider_alive, .retired = &collider_retired });
    const first = try with_colliders.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &math);
    const second = try with_colliders.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia(), .transform = .{ .position = .{ .x = fp.Fp.fromInt(1) } } }, &math);
    _ = try with_colliders.createCollider(.{ .body = first, .shape = .{ .sphere = .{ .radius = .one } } });
    _ = try with_colliders.createCollider(.{ .body = second, .shape = .{ .sphere = .{ .radius = .one } } });
    with_colliders.storage.linear_velocity[first.index()] = .{ .x = .one };
    var ccd_enabled: [2]bool = .{ true, false };
    var ccd_items: [2]gravity.dynamics.ccd.Item = undefined;
    var ccd_workspace = pipeline.CcdItemWorkspace{ .enabled = &ccd_enabled, .items = &ccd_items };
    const ccd_built = try pipeline.buildCcdItems(&with_colliders, fp.Fp.fromRatio(1, 120, &math), &ccd_workspace, &math);
    try std.testing.expectEqual(@as(usize, 2), ccd_built.len);
    try std.testing.expect(ccd_built[0].ccd_enabled);
    try std.testing.expectEqual(fp.Fp.fromRatio(1, 120, &math).raw, ccd_built[0].delta.x.raw);
    with_colliders.storage.linear_velocity[first.index()] = .{};
    var memory: [0]u8 align(@alignOf(store.Asset)) = .{};
    const assets = try store.Store.init(&memory, &.{});
    var compound_leaves: [0]shapes.CompoundLeaf = .{};
    var query_work: [0]u32 = .{};
    var query_nodes: [0]baked.BvhNode = .{};
    var query_triangles: [0]mesh.HeightTriangle = .{};
    const surface_workspace = gravity.query.queries.SurfaceCastWorkspace{ .compound_leaves = &compound_leaves, .mesh = .{ .nodes = &query_nodes, .primitives = &query_work, .stack = &query_work }, .heightfield = .{ .stack = &query_work, .triangles = &query_triangles } };
    var ccd_pairs: [2]gravity.dynamics.ccd.Pair = undefined;
    var toi_patches: [1]gravity.collision.contact_cache.Patch = undefined;
    var ccd_merge_input: [2]gravity.collision.contact_cache.Patch = undefined;
    var ccd_pipeline = pipeline.CcdPipelineWorkspace{ .assets = &assets, .items = &ccd_workspace, .pairs = &ccd_pairs, .surface = surface_workspace, .patches = &toi_patches, .merge_input = &ccd_merge_input };
    const prepared_toi = try pipeline.prepareCcdToi(&with_colliders, fp.Fp.fromRatio(1, 120, &math), &ccd_pipeline, &math);
    try std.testing.expect(prepared_toi.prepared.toi != null);
    try std.testing.expectEqual(@as(usize, 1), prepared_toi.patches.len);
    try std.testing.expectEqual(with_colliders.colliders.?.body[0].value, prepared_toi.patches[0].key.collider_a.value);
    var views: [2]shapes.Collider = undefined;
    var proxies: [2]broadphase.Proxy = undefined;
    var endpoints: [4]broadphase.Endpoint = undefined;
    var endpoint_scratch: [4]broadphase.Endpoint = undefined;
    var active: [2]u32 = undefined;
    var pairs: [1]broadphase.PairKey = undefined;
    var pair_work: [1]broadphase.PairKey = undefined;
    var pair_scratch: [1]broadphase.PairKey = undefined;
    var buffers = broadphase.Buffers{ .endpoints = &endpoints, .endpoint_scratch = &endpoint_scratch, .active = &active, .pairs = &pairs, .pair_work = &pair_work, .pair_scratch = &pair_scratch };
    var persistent = pipeline.State{};
    var command_scratch: [1]world.Command = undefined;
    var trace: [18]pipeline.Phase = undefined;
    var workspace = pipeline.Workspace{ .commands = &command_scratch, .trace = &trace };
    var sap_workspace = pipeline.BroadphaseWorkspace{ .assets = &assets, .collider_views = &views, .proxies = &proxies, .buffers = &buffers };
    var cache_patches: [2]gravity.collision.contact_cache.Patch = undefined;
    var cache = gravity.collision.contact_cache.Cache{ .patches = &cache_patches };
    var narrow: [1]gravity.collision.contact_cache.Patch = undefined;
    var cache_next: [2]gravity.collision.contact_cache.Patch = undefined;
    var events: [2]gravity.collision.contact_cache.Event = undefined;
    var contacts = pipeline.AnalyticContactWorkspace{ .broadphase = &sap_workspace, .narrow = &narrow, .cache = &cache, .cache_next = &cache_next, .events = &events };
    const result = try pipeline.stepWithAnalyticContacts(&with_colliders, &persistent, .default, &.{}, &workspace, &contacts, &math);
    try std.testing.expectEqual(@as(u64, 1), result.step.tick);
    try std.testing.expectEqualSlices(pipeline.Phase, &.{ .prevalidate, .commit, .integrate, .broadphase, .narrowphase, .integrate, .broadphase, .narrowphase }, result.step.trace);
    try std.testing.expectEqual(@as(usize, 1), buffers.pair_count);
    try std.testing.expectEqual(@as(usize, 1), cache.len);
    try std.testing.expectEqual(@as(usize, 1), result.events.len);
    try std.testing.expectEqual(gravity.collision.contact_cache.EventKind.begin, result.events[0].kind);
    var solver_contacts: [1]gravity.dynamics.contact_solver.Contact = undefined;
    var solver_points: [1]gravity.dynamics.contact_solver.Point = undefined;
    var restitution_bias: [1]fp.Fp = undefined;
    var pseudo_linear: [2]g.Vec3 = undefined;
    var pseudo_angular: [2]g.Vec3 = undefined;
    var solver = pipeline.AnalyticSolverWorkspace{ .contacts = &solver_contacts, .points = &solver_points, .restitution_bias = &restitution_bias, .pseudo = .{ .linear = &pseudo_linear, .angular = &pseudo_angular } };
    const contacts_to_solve = try pipeline.buildAnalyticSolverContacts(&with_colliders, &assets, &cache, &solver, &math);
    try std.testing.expectEqual(@as(usize, 1), contacts_to_solve.len);
    try gravity.dynamics.contact_solver.solve(&with_colliders, contacts_to_solve, solver.pseudo, .{}, &math);
    var substep_events: [2]gravity.collision.contact_cache.Event = undefined;
    var previous: [2]gravity.collision.contact_cache.Patch = undefined;
    var event_next: [2]gravity.collision.contact_cache.Patch = undefined;
    var tick_events: [2]gravity.collision.contact_cache.Event = undefined;
    var joint_values: [1]joints.Joint = undefined;
    var joint_generation: [1]u32 = undefined;
    var joint_alive: [1]bool = undefined;
    var joint_retired: [1]bool = undefined;
    var joint_pool = try joints.Pool.init(.{ .values = &joint_values, .generation = &joint_generation, .alive = &joint_alive, .retired = &joint_retired });
    _ = try joint_pool.create(&with_colliders, .{ .kind = .distance, .body_a = first, .body_b = second }, &math);
    var joint_rows: [2]constraints.ConstraintRow = undefined;
    var joint_authored: [2]constraints.ConstraintRow = undefined;
    var joint_build: [12]constraints.ConstraintRow = undefined;
    var joint_states: [1]joints.MutableState = undefined;
    var joint_workspace = pipeline.JointWorkspace{ .pool = &joint_pool, .rows = &joint_rows, .scratch = .{ .authored = &joint_authored, .build = &joint_build, .states = &joint_states } };
    var island_edges: [2]constraints.Edge = undefined;
    var edge_scratch: [2]constraints.Edge = undefined;
    var island_values: [2]constraints.Island = undefined;
    var members: [2]gravity.core.ids.BodyId = undefined;
    var lock_rows: [0]constraints.ConstraintRow = .{};
    var islands = pipeline.IslandWorkspace{ .edges = &island_edges, .edge_scratch = &edge_scratch, .islands = &island_values, .members = &members, .lock_rows = &lock_rows };
    var solver_pipeline = pipeline.AnalyticSolverPipelineWorkspace{ .contacts = &contacts, .solver = &solver, .islands = &islands, .joint = &joint_workspace, .substep_events = &substep_events, .previous = &previous, .event_next = &event_next, .tick_events = &tick_events };
    const repeated = try pipeline.stepWithAnalyticSolver(&with_colliders, &persistent, .default, &.{}, &workspace, &solver_pipeline, &math);
    try std.testing.expectEqualSlices(pipeline.Phase, &.{ .prevalidate, .commit, .integrate, .broadphase, .narrowphase, .islands, .solve, .integrate, .broadphase, .narrowphase, .islands, .solve, .events, .hash }, repeated.step.trace);
    try std.testing.expectEqual(@as(u32, 2), islands.islands[0].member_count);
    try std.testing.expectEqual(constraints.RowKind.joint, joint_rows[0].key.kind);
    try std.testing.expectEqual(gravity.collision.contact_cache.EventKind.persist, repeated.events[0].kind);
    try std.testing.expect(repeated.step.state_hash != null);
    var awake: [2]bool = undefined;
    var sleep_counter: [2]u32 = undefined;
    var wake_reason: [2]gravity.dynamics.sleeping.WakeReason = undefined;
    const sleep_storage = gravity.dynamics.sleeping.Storage{ .awake = &awake, .counter = &sleep_counter, .reason = &wake_reason };
    try gravity.dynamics.sleeping.init(sleep_storage);
    var requests: [2]gravity.dynamics.sleeping.Request = undefined;
    var graph_scratch: [2]gravity.core.ids.BodyId = undefined;
    var wake_events: [2]gravity.dynamics.sleeping.Event = undefined;
    var sleep_events: [2]gravity.dynamics.sleeping.Event = undefined;
    var sleep_workspace = pipeline.SleepWorkspace{ .storage = sleep_storage, .requests = &requests, .graph_scratch = &graph_scratch, .wake_events = &wake_events, .sleep_events = &sleep_events };
    solver_pipeline.sleep = &sleep_workspace;
    // Keep CCD phase active while both bodies are sleeping. Disabled caster
    // flags make this a no-hit path; the awake mask must still prevent any
    // residual velocity from advancing a sleeping transform.
    ccd_enabled[0] = false;
    solver_pipeline.ccd = &ccd_pipeline;
    var sleep_config = gravity.core.config.SimulationConfig.default;
    sleep_config.iterations.sleep_ticks = 1;
    const slept = try pipeline.stepWithAnalyticSolver(&with_colliders, &persistent, sleep_config, &.{}, &workspace, &solver_pipeline, &math);
    try std.testing.expectEqualSlices(pipeline.Phase, &.{ .sleep, .events, .hash }, slept.step.trace[slept.step.trace.len - 3 ..]);
    try std.testing.expect(!sleep_storage.awake[first.index()]);
    const asleep_position = with_colliders.storage.position[first.index()];
    _ = try pipeline.stepWithAnalyticSolver(&with_colliders, &persistent, sleep_config, &.{}, &workspace, &solver_pipeline, &math);
    try std.testing.expectEqualDeep(asleep_position, with_colliders.storage.position[first.index()]);
    const wake_command = world.Command{ .key = .{ .phase_priority = 0, .issuer = 1, .sequence = 2, .discriminant = 0 }, .op = .{ .force = .{ .body = first, .value = .{ .x = fp.Fp.fromInt(10) } } } };
    _ = try pipeline.stepWithAnalyticSolver(&with_colliders, &persistent, sleep_config, &.{wake_command}, &workspace, &solver_pipeline, &math);
    try std.testing.expect(sleep_storage.awake[first.index()]);

    // Fast CCD hit through the production World pipeline. The cast reaches
    // the target during the first substep, solves the TOI, then consumes the
    // remaining interval without tunnelling through the target.
    solver_pipeline.sleep = null;
    solver_pipeline.joint = null;
    solver_pipeline.ccd = &ccd_pipeline;
    cache.len = 0;
    with_colliders.storage.position[first.index()] = .{ .x = fp.Fp.fromRatio(-41, 20, &math) };
    with_colliders.storage.position[second.index()] = .{};
    with_colliders.storage.linear_velocity[first.index()] = .{ .x = fp.Fp.fromInt(10) };
    with_colliders.storage.linear_velocity[second.index()] = .{};
    const hit_ccd = try pipeline.stepWithAnalyticSolver(&with_colliders, &persistent, .default, &.{}, &workspace, &solver_pipeline, &math);
    try std.testing.expectEqual(pipeline.Phase.ccd, hit_ccd.step.trace[7]);
    try std.testing.expect(with_colliders.storage.position[first.index()].x.raw <= fp.Fp.fromInt(-1).raw);
}

test "pipeline runs every joint kind through the formal solver step" {
    inline for ([_]joints.Kind{ .distance, .ball_socket, .hinge, .slider, .fixed, .cone_twist }) |kind| {
        var math = fp.MathStatus{};
        var fixture: Fixture = .{};
        const body_world = try fixture.init();
        var collider_body: [0]gravity.core.ids.BodyId = .{};
        var collider_local: [0]g.Transform3 = .{};
        var collider_shape: [0]shapes.Shape = .{};
        var material: [0]shapes.Material = .{};
        var category: [0]u32 = .{};
        var mask: [0]u32 = .{};
        var group: [0]i32 = .{};
        var sensor: [0]bool = .{};
        var enabled: [0]bool = .{};
        var revision: [0]u32 = .{};
        var collider_generation: [0]u32 = .{};
        var collider_alive: [0]bool = .{};
        var collider_retired: [0]bool = .{};
        var runtime = try world.World.initWithColliders(body_world.storage, .{ .body = &collider_body, .local = &collider_local, .shape = &collider_shape, .material = &material, .category = &category, .mask = &mask, .group = &group, .sensor = &sensor, .enabled = &enabled, .revision = &revision, .generation = &collider_generation, .alive = &collider_alive, .retired = &collider_retired });
        const planar = kind == .distance;
        const body_a = try runtime.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia(), .locks = if (planar) .{ .linear_z = true, .angular_x = true, .angular_y = true } else .{} }, &math);
        const body_b = try runtime.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia(), .transform = .{ .position = .{ .x = fp.Fp.one } } }, &math);

        var asset_memory: [0]u8 align(@alignOf(store.Asset)) = .{};
        const assets = try store.Store.init(&asset_memory, &.{});
        var views: [0]shapes.Collider = .{};
        var proxies: [0]broadphase.Proxy = .{};
        var endpoints: [0]broadphase.Endpoint = .{};
        var endpoint_scratch: [0]broadphase.Endpoint = .{};
        var active: [0]u32 = .{};
        var pairs: [0]broadphase.PairKey = .{};
        var pair_work: [0]broadphase.PairKey = .{};
        var pair_scratch: [0]broadphase.PairKey = .{};
        var sap_buffers = broadphase.Buffers{ .endpoints = &endpoints, .endpoint_scratch = &endpoint_scratch, .active = &active, .pairs = &pairs, .pair_work = &pair_work, .pair_scratch = &pair_scratch };
        var broadphase_workspace = pipeline.BroadphaseWorkspace{ .assets = &assets, .collider_views = &views, .proxies = &proxies, .buffers = &sap_buffers };
        var narrow: [0]contact_cache.Patch = .{};
        var cache_storage: [0]contact_cache.Patch = .{};
        var cache = contact_cache.Cache{ .patches = &cache_storage };
        var cache_next: [0]contact_cache.Patch = .{};
        var contact_events: [0]contact_cache.Event = .{};
        var contacts = pipeline.AnalyticContactWorkspace{ .broadphase = &broadphase_workspace, .narrow = &narrow, .cache = &cache, .cache_next = &cache_next, .events = &contact_events };
        var solver_contacts: [0]gravity.dynamics.contact_solver.Contact = .{};
        var solver_points: [0]gravity.dynamics.contact_solver.Point = .{};
        var biases: [0]fp.Fp = .{};
        var pseudo_linear: [2]g.Vec3 = undefined;
        var pseudo_angular: [2]g.Vec3 = undefined;
        var solver = pipeline.AnalyticSolverWorkspace{ .contacts = &solver_contacts, .points = &solver_points, .restitution_bias = &biases, .pseudo = .{ .linear = &pseudo_linear, .angular = &pseudo_angular } };
        var joint_values: [1]joints.Joint = undefined;
        var joint_generation: [1]u32 = undefined;
        var joint_alive: [1]bool = undefined;
        var joint_retired: [1]bool = undefined;
        var joint_pool = try joints.Pool.init(.{ .values = &joint_values, .generation = &joint_generation, .alive = &joint_alive, .retired = &joint_retired });
        _ = try joint_pool.create(&runtime, .{ .kind = kind, .body_a = body_a, .body_b = body_b }, &math);
        var joint_rows: [8]constraints.ConstraintRow = undefined;
        var joint_authored: [8]constraints.ConstraintRow = undefined;
        var joint_build: [12]constraints.ConstraintRow = undefined;
        var joint_states: [1]joints.MutableState = undefined;
        var joint_workspace = pipeline.JointWorkspace{ .pool = &joint_pool, .rows = &joint_rows, .scratch = .{ .authored = &joint_authored, .build = &joint_build, .states = &joint_states } };
        var edges: [1]constraints.Edge = undefined;
        var edge_scratch: [1]constraints.Edge = undefined;
        var islands_storage: [2]constraints.Island = undefined;
        var members: [2]gravity.core.ids.BodyId = undefined;
        var lock_rows: [3]constraints.ConstraintRow = undefined;
        var islands = pipeline.IslandWorkspace{ .edges = &edges, .edge_scratch = &edge_scratch, .islands = &islands_storage, .members = &members, .lock_rows = &lock_rows };
        var substep_events: [0]contact_cache.Event = .{};
        var previous: [0]contact_cache.Patch = .{};
        var event_next: [0]contact_cache.Patch = .{};
        var tick_events: [0]contact_cache.Event = .{};
        var solver_pipeline = pipeline.AnalyticSolverPipelineWorkspace{ .contacts = &contacts, .solver = &solver, .islands = &islands, .joint = &joint_workspace, .substep_events = &substep_events, .previous = &previous, .event_next = &event_next, .tick_events = &tick_events };
        var commands: [1]world.Command = undefined;
        if (planar) commands[0] = .{ .key = .{ .phase_priority = 0, .issuer = 1, .sequence = 1, .discriminant = 0 }, .op = .{ .velocity = .{ .body = body_a, .linear = .{ .x = fp.Fp.one, .z = fp.Fp.one }, .angular = .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.one } } } };
        const tick_commands: []const world.Command = if (planar) &commands else &.{};
        var trace: [14]pipeline.Phase = undefined;
        var workspace = pipeline.Workspace{ .commands = &commands, .trace = &trace };
        var state = pipeline.State{};
        const result = try pipeline.stepWithAnalyticSolver(&runtime, &state, .default, tick_commands, &workspace, &solver_pipeline, &math);
        try std.testing.expectEqual(@as(u64, 1), result.step.tick);
        try std.testing.expectEqual(pipeline.Phase.solve, result.step.trace[6]);
        try std.testing.expectEqual(kind, joint_pool.storage.values[0].kind);
        try std.testing.expect(joint_rows[0].key.kind == .joint);
        if (planar) {
            try std.testing.expectEqual(@as(i64, 0), runtime.storage.linear_velocity[body_a.index()].z.raw);
            try std.testing.expectEqual(@as(i64, 0), runtime.storage.angular_velocity[body_a.index()].x.raw);
            try std.testing.expectEqual(@as(i64, 0), runtime.storage.angular_velocity[body_a.index()].y.raw);
            try std.testing.expect(runtime.storage.linear_velocity[body_a.index()].x.raw != 0);
            try std.testing.expect(runtime.storage.angular_velocity[body_a.index()].z.raw != 0);
        }
    }
}

test "pipeline SAP capacity fault does not publish a tick" {
    var fixture: Fixture = .{};
    const state = try fixture.init();
    var math = fp.MathStatus{};
    var collider_body: [2]gravity.core.ids.BodyId = undefined;
    var collider_local: [2]g.Transform3 = undefined;
    var collider_shape: [2]shapes.Shape = undefined;
    var material: [2]shapes.Material = undefined;
    var category: [2]u32 = undefined;
    var mask: [2]u32 = undefined;
    var group: [2]i32 = undefined;
    var sensor: [2]bool = undefined;
    var enabled: [2]bool = undefined;
    var revision: [2]u32 = undefined;
    var collider_generation: [2]u32 = undefined;
    var collider_alive: [2]bool = undefined;
    var collider_retired: [2]bool = undefined;
    var with_colliders = try world.World.initWithColliders(state.storage, .{ .body = &collider_body, .local = &collider_local, .shape = &collider_shape, .material = &material, .category = &category, .mask = &mask, .group = &group, .sensor = &sensor, .enabled = &enabled, .revision = &revision, .generation = &collider_generation, .alive = &collider_alive, .retired = &collider_retired });
    const first = try with_colliders.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &math);
    const second = try with_colliders.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia(), .transform = .{ .position = .{ .x = fp.Fp.fromInt(1) } } }, &math);
    _ = try with_colliders.createCollider(.{ .body = first, .shape = .{ .sphere = .{ .radius = .one } } });
    _ = try with_colliders.createCollider(.{ .body = second, .shape = .{ .sphere = .{ .radius = .one } } });
    var memory: [0]u8 align(@alignOf(store.Asset)) = .{};
    const assets = try store.Store.init(&memory, &.{});
    var views: [2]shapes.Collider = undefined;
    var proxies: [2]broadphase.Proxy = undefined;
    var endpoints: [4]broadphase.Endpoint = undefined;
    var endpoint_scratch: [4]broadphase.Endpoint = undefined;
    var active: [2]u32 = undefined;
    var pairs: [0]broadphase.PairKey = .{};
    var pair_work: [1]broadphase.PairKey = undefined;
    var pair_scratch: [1]broadphase.PairKey = undefined;
    var buffers = broadphase.Buffers{ .endpoints = &endpoints, .endpoint_scratch = &endpoint_scratch, .active = &active, .pairs = &pairs, .pair_work = &pair_work, .pair_scratch = &pair_scratch };
    var persistent = pipeline.State{ .tick = 17 };
    var command_scratch: [0]world.Command = .{};
    var trace: [6]pipeline.Phase = undefined;
    var workspace = pipeline.Workspace{ .commands = &command_scratch, .trace = &trace };
    var sap_workspace = pipeline.BroadphaseWorkspace{ .assets = &assets, .collider_views = &views, .proxies = &proxies, .buffers = &buffers };
    try std.testing.expectError(error.Faulted, pipeline.stepWithBroadphase(&with_colliders, &persistent, .default, &.{}, &workspace, &sap_workspace, &math));
    try std.testing.expectEqual(@as(u64, 17), persistent.tick);
    try std.testing.expectEqual(pipeline.Phase.broadphase, persistent.fault.?.phase);
    try std.testing.expectEqual(pipeline.FaultCode.broadphase, persistent.fault.?.code);
    try std.testing.expectEqual(pipeline.FaultDetail.capacity_exceeded, persistent.fault.?.detail);
    try std.testing.expectEqual(@as(u64, 17), persistent.fault.?.tick);
}

test "pipeline fault detail participates in canonical state hash" {
    var fixture: Fixture = .{};
    const state = try fixture.init();
    var cache_patches: [0]contact_cache.Patch = .{};
    var cache = contact_cache.Cache{ .patches = &cache_patches };
    var pipeline_state = pipeline.State{ .fault = .{ .tick = 4, .phase = .narrowphase, .code = .shape, .detail = .invalid_asset } };
    const first = pipeline.canonicalStateHash(&state, &pipeline_state, .default, .{ .cache = &cache });
    pipeline_state.fault.?.detail = .unsupported_shape;
    const second = pipeline.canonicalStateHash(&state, &pipeline_state, .default, .{ .cache = &cache });
    try std.testing.expect(!std.mem.eql(u8, &first, &second));
}

test "pipeline preserves compound child paths when bridging convex patches" {
    var first_path = shapes.ChildPath{};
    first_path.values[0] = 7;
    first_path.len = 1;
    var second_path = shapes.ChildPath{};
    second_path.values[0] = 2;
    second_path.len = 1;
    var result = gjk.ConvexResult{ .gjk = .{ .status = .intersecting, .iterations = 0, .simplex = undefined, .simplex_len = 0, .direction = g.Vec3.unit_x } };
    try result.patch.append(.{ .point_a = .{}, .point_b = .{}, .separation = fp.Fp.zero, .feature_a = 11, .feature_b = 21, .path_a = first_path });
    try result.patch.append(.{ .point_a = .{}, .point_b = .{}, .separation = fp.Fp.zero, .feature_a = 12, .feature_b = 22, .path_a = second_path });
    try result.patch.append(.{ .point_a = .{}, .point_b = .{}, .separation = fp.Fp.zero, .feature_a = 13, .feature_b = 23, .path_a = first_path });
    var patches: [2]contact_cache.Patch = undefined;
    const converted = try pipeline.cachePatchesFromConvexResult(result, .{ .collider_a = gravity.core.ids.ColliderId.init(3, 0), .collider_b = gravity.core.ids.ColliderId.init(5, 0) }, g.Vec3.unit_x, false, &patches);
    try std.testing.expectEqual(@as(usize, 2), converted.len);
    try std.testing.expectEqual(@as(u32, 2), converted[0].key.path_a.values[0]);
    try std.testing.expectEqual(@as(u8, 1), converted[0].len);
    try std.testing.expectEqual(@as(u32, 7), converted[1].key.path_a.values[0]);
    try std.testing.expectEqual(@as(u8, 2), converted[1].len);
    try std.testing.expectEqual(@as(u32, 11), converted[1].points[0].feature_a);
    try std.testing.expectEqual(@as(u32, 13), converted[1].points[1].feature_a);
}

test "pipeline surface cache bridge restores SAP orientation" {
    var math = fp.MathStatus{};
    var surface = gjk.ContactPatch{};
    try surface.append(.{ .point_a = .{}, .point_b = .{}, .normal = g.Vec3.unit_y, .separation = fp.Fp.zero, .feature_a = 3, .feature_b = 9 });
    const patch = try pipeline.cachePatchFromSurfaceResult(surface, .{ .collider_a = gravity.core.ids.ColliderId.init(1, 0), .collider_b = gravity.core.ids.ColliderId.init(2, 0) }, false, true, &math);
    try std.testing.expectEqual(fp.Fp.one.neg(&math).raw, patch.normal.y.raw);
    try std.testing.expectEqual(@as(u32, 9), patch.points[0].feature_a);
    try std.testing.expectEqual(@as(u32, 3), patch.points[0].feature_b);
}

test "pipeline surface cache bridge preserves compound child paths" {
    var math = fp.MathStatus{};
    var first = shapes.ChildPath{};
    first.values[0] = 9;
    first.len = 1;
    var second = shapes.ChildPath{};
    second.values[0] = 4;
    second.len = 1;
    var surface = gjk.ContactPatch{};
    try surface.append(.{ .point_a = .{}, .point_b = .{}, .normal = g.Vec3.unit_y, .separation = fp.Fp.zero, .feature_a = 3, .feature_b = 9, .path_b = first });
    try surface.append(.{ .point_a = .{}, .point_b = .{}, .normal = g.Vec3.unit_y, .separation = fp.Fp.zero, .feature_a = 4, .feature_b = 10, .path_b = second });
    var patches: [2]contact_cache.Patch = undefined;
    const converted = try pipeline.cachePatchesFromSurfaceResult(surface, .{ .collider_a = gravity.core.ids.ColliderId.init(1, 0), .collider_b = gravity.core.ids.ColliderId.init(2, 0) }, false, false, &patches, &math);
    try std.testing.expectEqual(@as(usize, 2), converted.len);
    try std.testing.expectEqual(@as(u32, 4), converted[0].key.path_b.values[0]);
    try std.testing.expectEqual(@as(u32, 9), converted[1].key.path_b.values[0]);
}

test "pipeline hull narrow rebuilds cached witnesses for the solver" {
    var math = fp.MathStatus{};
    const hull_points = [_]g.Vec3{ .{}, .{ .x = fp.Fp.one }, .{ .y = fp.Fp.one }, .{ .z = fp.Fp.one } };
    var vertices: [4]g.Vec3 = undefined;
    var triangles: [4]baked.Triangle = undefined;
    var faces: [4]baked.HullFace = undefined;
    var edges: [12]baked.HalfEdge = undefined;
    const hull = try baked.buildConvexHull(&hull_points, &vertices, &triangles, &faces, &edges, &math);
    var encoded: [2048]u8 = undefined;
    var bake_scratch: [1024]u8 = undefined;
    const baked_hull = try baked.encodeConvexHull(hull, 91, &encoded, &bake_scratch);
    var asset_memory: [4096]u8 align(@alignOf(store.Asset)) = undefined;
    const assets = try store.Store.init(&asset_memory, &.{baked_hull.bytes});
    var fixture: Fixture = .{};
    const base = try fixture.init();
    var collider_body: [2]gravity.core.ids.BodyId = undefined;
    var collider_local: [2]g.Transform3 = undefined;
    var collider_shape: [2]shapes.Shape = undefined;
    var material: [2]shapes.Material = undefined;
    var category: [2]u32 = undefined;
    var mask: [2]u32 = undefined;
    var group: [2]i32 = undefined;
    var sensor: [2]bool = undefined;
    var enabled: [2]bool = undefined;
    var revision: [2]u32 = undefined;
    var collider_generation: [2]u32 = undefined;
    var collider_alive: [2]bool = undefined;
    var collider_retired: [2]bool = undefined;
    var runtime = try world.World.initWithColliders(base.storage, .{ .body = &collider_body, .local = &collider_local, .shape = &collider_shape, .material = &material, .category = &category, .mask = &mask, .group = &group, .sensor = &sensor, .enabled = &enabled, .revision = &revision, .generation = &collider_generation, .alive = &collider_alive, .retired = &collider_retired });
    const hull_body = try runtime.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &math);
    const sphere_body = try runtime.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia(), .transform = .{ .position = .{ .x = fp.Fp.fromRatio(1, 4, &math), .y = fp.Fp.fromRatio(1, 4, &math), .z = fp.Fp.fromRatio(1, 4, &math) } } }, &math);
    const hull_collider = try runtime.createCollider(.{ .body = hull_body, .shape = .{ .convex_hull = .{ .source_id = 91 } } });
    const sphere_collider = try runtime.createCollider(.{ .body = sphere_body, .shape = .{ .sphere = .{ .radius = fp.Fp.fromRatio(1, 2, &math) } } });
    var epa_vertices: [130]gjk.SupportVertex = undefined;
    var epa_faces: [256]gjk.EpaFace = undefined;
    var visible: [256]bool = undefined;
    var horizon: [512]gjk.HorizonEdge = undefined;
    var reference: [8]gjk.ClipVertex = undefined;
    var incident: [8]gjk.ClipVertex = undefined;
    var clip_a: [8]gjk.ClipVertex = undefined;
    var clip_b: [8]gjk.ClipVertex = undefined;
    var contacts: [8]gjk.ContactPoint = undefined;
    const manifold = gjk.ManifoldWorkspace{ .epa = .{ .vertices = &epa_vertices, .faces = &epa_faces, .visible = &visible, .horizon = &horizon }, .reference = &reference, .incident = &incident, .scratch_a = &clip_a, .scratch_b = &clip_b, .contacts = &contacts };
    var convex = pipeline.ConvexNarrowWorkspace{ .manifold = manifold };
    var narrow: [4]contact_cache.Patch = undefined;
    const patches = try pipeline.narrowRuntimePairs(&runtime, &assets, &.{broadphase.PairKey.init(hull_collider, sphere_collider)}, &convex, null, &narrow, &math);
    try std.testing.expect(patches.len > 0);
    var cache_storage: [4]contact_cache.Patch = undefined;
    @memcpy(cache_storage[0..patches.len], patches);
    var cache = contact_cache.Cache{ .patches = &cache_storage, .len = patches.len };
    var solver_contacts: [4]gravity.dynamics.contact_solver.Contact = undefined;
    var solver_points: [16]gravity.dynamics.contact_solver.Point = undefined;
    var biases: [16]fp.Fp = undefined;
    var pseudo_linear: [2]g.Vec3 = undefined;
    var pseudo_angular: [2]g.Vec3 = undefined;
    var solver = pipeline.AnalyticSolverWorkspace{ .contacts = &solver_contacts, .points = &solver_points, .restitution_bias = &biases, .pseudo = .{ .linear = &pseudo_linear, .angular = &pseudo_angular }, .manifold = manifold };
    const rebuilt = try pipeline.buildAnalyticSolverContacts(&runtime, &assets, &cache, &solver, &math);
    try std.testing.expectEqual(patches.len, rebuilt.len);
    try std.testing.expect(rebuilt[0].points.len > 0);

    // The hull has already crossed narrow/rebuild above; run it through the
    // complete fixed Tick to cover cache, islands, solve and hash as well.
    cache.len = 0;
    var views: [2]shapes.Collider = undefined;
    var proxies: [2]broadphase.Proxy = undefined;
    var endpoints: [4]broadphase.Endpoint = undefined;
    var endpoint_scratch: [4]broadphase.Endpoint = undefined;
    var active: [2]u32 = undefined;
    var sap_pairs: [1]broadphase.PairKey = undefined;
    var pair_work: [1]broadphase.PairKey = undefined;
    var pair_sort: [1]broadphase.PairKey = undefined;
    var sap_buffers = broadphase.Buffers{ .endpoints = &endpoints, .endpoint_scratch = &endpoint_scratch, .active = &active, .pairs = &sap_pairs, .pair_work = &pair_work, .pair_scratch = &pair_sort };
    var broadphase_workspace = pipeline.BroadphaseWorkspace{ .assets = &assets, .collider_views = &views, .proxies = &proxies, .buffers = &sap_buffers };
    var cache_next: [8]contact_cache.Patch = undefined;
    var events: [8]contact_cache.Event = undefined;
    var contact_workspace = pipeline.AnalyticContactWorkspace{ .broadphase = &broadphase_workspace, .narrow = &narrow, .convex = &convex, .cache = &cache, .cache_next = &cache_next, .events = &events };
    var island_edges: [1]constraints.Edge = undefined;
    var edge_scratch: [1]constraints.Edge = undefined;
    var islands_storage: [2]constraints.Island = undefined;
    var members: [2]gravity.core.ids.BodyId = undefined;
    var lock_rows: [0]constraints.ConstraintRow = .{};
    var islands = pipeline.IslandWorkspace{ .edges = &island_edges, .edge_scratch = &edge_scratch, .islands = &islands_storage, .members = &members, .lock_rows = &lock_rows };
    var substep_events: [8]contact_cache.Event = undefined;
    var previous: [8]contact_cache.Patch = undefined;
    var event_next: [8]contact_cache.Patch = undefined;
    var tick_events: [8]contact_cache.Event = undefined;
    var solver_pipeline = pipeline.AnalyticSolverPipelineWorkspace{ .contacts = &contact_workspace, .solver = &solver, .islands = &islands, .substep_events = &substep_events, .previous = &previous, .event_next = &event_next, .tick_events = &tick_events };
    var persistent = pipeline.State{};
    var commands: [1]world.Command = undefined;
    var trace: [16]pipeline.Phase = undefined;
    var step_workspace = pipeline.Workspace{ .commands = &commands, .trace = &trace };
    const stepped = try pipeline.stepWithAnalyticSolver(&runtime, &persistent, .default, &.{}, &step_workspace, &solver_pipeline, &math);
    try std.testing.expectEqual(@as(u64, 1), stepped.step.tick);
    try std.testing.expect(cache.len > 0);
    try std.testing.expectEqual(pipeline.Phase.solve, stepped.step.trace[6]);
    try std.testing.expectEqual(pipeline.Phase.hash, stepped.step.trace[stepped.step.trace.len - 1]);
}

test "mesh primitive pair ranges match serial under reverse and permuted schedules" {
    const triangle_count = 17;
    const candidate_count = triangle_count * triangle_count;
    const vertices = [_]g.Vec3{ .{ .x = fp.Fp.fromInt(-1), .z = fp.Fp.fromInt(-1) }, .{ .x = fp.Fp.fromInt(1), .z = fp.Fp.fromInt(-1) }, .{ .z = fp.Fp.fromInt(1) } };
    var triangles: [triangle_count]baked.Triangle = undefined;
    var primitives: [triangle_count]u32 = undefined;
    for (&triangles, &primitives, 0..) |*triangle, *primitive, index| {
        triangle.* = .{ .a = 0, .b = 1, .c = 2 };
        primitive.* = @intCast(index);
    }
    const bounds = g.Aabb3{ .min = .{ .x = fp.Fp.fromInt(-1), .z = fp.Fp.fromInt(-1) }, .max = .{ .x = fp.Fp.fromInt(1), .z = fp.Fp.fromInt(1) } };
    const nodes = [_]baked.BvhNode{baked.BvhNode.leaf(bounds, 0, triangle_count)};
    var bytes_a: [8192]u8 = undefined;
    var bytes_b: [8192]u8 = undefined;
    var bake_scratch_a: [4096]u8 = undefined;
    var bake_scratch_b: [4096]u8 = undefined;
    const encoded_a = try baked.encodeMesh(.{ .source_id = 91, .vertices = &vertices, .triangles = &triangles, .nodes = &nodes, .primitives = &primitives }, &bytes_a, &bake_scratch_a);
    const encoded_b = try baked.encodeMesh(.{ .source_id = 92, .vertices = &vertices, .triangles = &triangles, .nodes = &nodes, .primitives = &primitives }, &bytes_b, &bake_scratch_b);
    var asset_memory: [20_000]u8 align(@alignOf(store.Asset)) = undefined;
    const assets = try store.Store.init(&asset_memory, &.{ encoded_a.bytes, encoded_b.bytes });
    const view_a = try gravity.assets.runtime_view.find(&assets, 91);
    const view_b = try gravity.assets.runtime_view.find(&assets, 92);

    var nodes_a: [1]baked.BvhNode = undefined;
    var nodes_b: [1]baked.BvhNode = undefined;
    var decoded_a: [triangle_count]u32 = undefined;
    var decoded_b: [triangle_count]u32 = undefined;
    var work: [1]mesh.NodePair = undefined;
    var pair_scratch: [candidate_count]mesh.PrimitivePair = undefined;
    var pair_output: [candidate_count]mesh.PrimitivePair = undefined;
    var overlaps: [candidate_count]mesh.PrimitivePair = undefined;
    var contacts: [candidate_count]gjk.ContactPoint = undefined;
    const mesh_workspace = mesh.MeshMeshPatchWorkspace{ .query = .{ .nodes_a = &nodes_a, .primitives_a = &decoded_a, .nodes_b = &nodes_b, .primitives_b = &decoded_b, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .overlaps = &overlaps }, .contacts = &contacts };
    var status = fp.MathStatus{};
    const golden = try mesh.meshMeshPatchTransformed(view_a, .{}, view_b, .{}, mesh_workspace, &status);
    try std.testing.expectEqual(fp.MathFault.none, status.fault);
    try std.testing.expectEqual(@as(u8, 4), golden.len);

    const Host = struct {
        order: jobs.TestDispatcher.Order,
        max_jobs: u32 = 0,
        fn dispatch(raw: *anyopaque, batch: jobs.Batch) jobs.Error!void {
            const self: *@This() = @ptrCast(@alignCast(raw));
            self.max_jobs = @max(self.max_jobs, batch.job_count);
            var scheduler = jobs.TestDispatcher{ .order = self.order };
            var custom = scheduler.custom();
            try custom.dispatch_fn(custom.context, batch);
        }
    };
    inline for (.{ jobs.TestDispatcher.Order.reverse, jobs.TestDispatcher.Order.permuted }) |order| {
        var host = Host{ .order = order };
        var custom = jobs.Custom{ .context = &host, .dispatch_fn = Host.dispatch };
        var commands: [0]world.Command = .{};
        var trace: [0]pipeline.Phase = .{};
        var step_workspace = pipeline.Workspace{ .commands = &commands, .trace = &trace, .dispatcher = .{ .custom = &custom } };
        status = .{};
        const actual = try pipeline.meshMeshPatchRanges(&step_workspace, view_a, .{}, view_b, .{}, mesh_workspace, &status);
        try std.testing.expectEqual(fp.MathFault.none, status.fault);
        try std.testing.expect(host.max_jobs > 1);
        try std.testing.expectEqualDeep(golden, actual);
    }
}

test "pipeline reversed convex mesh pair rebuilds solver witnesses" {
    var math = fp.MathStatus{};
    const vertices = [_]g.Vec3{ .{ .x = fp.Fp.fromInt(-2), .z = fp.Fp.fromInt(-2) }, .{ .x = fp.Fp.fromInt(2), .z = fp.Fp.fromInt(-2) }, .{ .z = fp.Fp.fromInt(2) } };
    const triangles = [_]baked.Triangle{.{ .a = 0, .b = 1, .c = 2 }};
    const nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = .{ .x = fp.Fp.fromInt(-2), .z = fp.Fp.fromInt(-2) }, .max = .{ .x = fp.Fp.fromInt(2), .z = fp.Fp.fromInt(2) } }, 0, 1)};
    const primitives = [_]u32{0};
    var bytes: [512]u8 = undefined;
    var bake_scratch: [256]u8 = undefined;
    const encoded = try baked.encodeMesh(.{ .source_id = 61, .vertices = &vertices, .triangles = &triangles, .nodes = &nodes, .primitives = &primitives }, &bytes, &bake_scratch);
    var asset_memory: [1024]u8 align(@alignOf(store.Asset)) = undefined;
    const assets = try store.Store.init(&asset_memory, &.{encoded.bytes});

    var fixture: Fixture = .{};
    const base = try fixture.init();
    var collider_body: [2]gravity.core.ids.BodyId = undefined;
    var collider_local: [2]g.Transform3 = undefined;
    var collider_shape: [2]shapes.Shape = undefined;
    var material: [2]shapes.Material = undefined;
    var category: [2]u32 = undefined;
    var mask: [2]u32 = undefined;
    var group: [2]i32 = undefined;
    var sensor: [2]bool = undefined;
    var enabled: [2]bool = undefined;
    var revision: [2]u32 = undefined;
    var collider_generation: [2]u32 = undefined;
    var collider_alive: [2]bool = undefined;
    var collider_retired: [2]bool = undefined;
    var runtime = try world.World.initWithColliders(base.storage, .{ .body = &collider_body, .local = &collider_local, .shape = &collider_shape, .material = &material, .category = &category, .mask = &mask, .group = &group, .sensor = &sensor, .enabled = &enabled, .revision = &revision, .generation = &collider_generation, .alive = &collider_alive, .retired = &collider_retired });
    const mesh_body = try runtime.create(.{ .body_type = .static, .inverse_inertia_local = inertia() }, &math);
    const box_body = try runtime.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia(), .transform = .{ .position = .{ .y = fp.Fp.fromRatio(1, 2, &math) } } }, &math);
    const mesh_collider = try runtime.createCollider(.{ .body = mesh_body, .shape = .{ .triangle_mesh = .{ .source_id = 61 } } });
    const box_collider = try runtime.createCollider(.{ .body = box_body, .shape = .{ .box = .{ .half_extents = .{ .x = fp.Fp.one, .y = fp.Fp.one, .z = fp.Fp.one } } } });

    var query_nodes: [1]baked.BvhNode = undefined;
    var query_primitives: [1]u32 = undefined;
    var work: [1]mesh.NodePair = undefined;
    var pair_scratch: [1]mesh.PrimitivePair = undefined;
    var pair_output: [1]mesh.PrimitivePair = undefined;
    var hits: [1]u32 = undefined;
    var epa_vertices: [32]gjk.SupportVertex = undefined;
    var epa_faces: [64]gjk.EpaFace = undefined;
    var visible: [64]bool = undefined;
    var horizon: [64]gjk.HorizonEdge = undefined;
    var surface_contacts: [1]gjk.ContactPoint = undefined;
    const convex_mesh = mesh.ConvexMeshPatchWorkspace{ .query = .{ .nodes = &query_nodes, .primitives = &query_primitives, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .intersections = &hits }, .triangle = .{ .epa = .{ .vertices = &epa_vertices, .faces = &epa_faces, .visible = &visible, .horizon = &horizon } }, .contacts = &surface_contacts };
    var surface = pipeline.SurfaceNarrowWorkspace{ .convex_mesh = convex_mesh };
    var narrow: [1]contact_cache.Patch = undefined;
    const patches = try pipeline.narrowRuntimePairs(&runtime, &assets, &.{broadphase.PairKey.init(mesh_collider, box_collider)}, null, &surface, &narrow, &math);
    try std.testing.expectEqual(@as(usize, 1), patches.len);
    try std.testing.expectEqual(mesh_collider.value, patches[0].key.collider_a.value);
    try std.testing.expectEqual(box_collider.value, patches[0].key.collider_b.value);

    var cache_storage: [2]contact_cache.Patch = undefined;
    cache_storage[0] = patches[0];
    var cache = contact_cache.Cache{ .patches = &cache_storage, .len = 1 };
    var solver_contacts: [1]gravity.dynamics.contact_solver.Contact = undefined;
    var solver_points: [1]gravity.dynamics.contact_solver.Point = undefined;
    var biases: [1]fp.Fp = undefined;
    var pseudo_linear: [2]g.Vec3 = undefined;
    var pseudo_angular: [2]g.Vec3 = undefined;
    var solver = pipeline.AnalyticSolverWorkspace{ .contacts = &solver_contacts, .points = &solver_points, .restitution_bias = &biases, .pseudo = .{ .linear = &pseudo_linear, .angular = &pseudo_angular }, .surface = surface };
    const rebuilt = try pipeline.buildAnalyticSolverContacts(&runtime, &assets, &cache, &solver, &math);
    try std.testing.expectEqual(@as(usize, 1), rebuilt.len);
    try std.testing.expect(rebuilt[0].points[0].penetration.raw > 0);

    // Exercise the same reversed Mesh/Box order through the formal Tick,
    // rather than treating the narrow/rebuild bridge as a substitute for it.
    cache.len = 0;
    var views: [2]shapes.Collider = undefined;
    var proxies: [2]broadphase.Proxy = undefined;
    var endpoints: [4]broadphase.Endpoint = undefined;
    var endpoint_scratch: [4]broadphase.Endpoint = undefined;
    var active: [2]u32 = undefined;
    var sap_pairs: [1]broadphase.PairKey = undefined;
    var pair_work: [1]broadphase.PairKey = undefined;
    var pair_sort: [1]broadphase.PairKey = undefined;
    var sap_buffers = broadphase.Buffers{ .endpoints = &endpoints, .endpoint_scratch = &endpoint_scratch, .active = &active, .pairs = &sap_pairs, .pair_work = &pair_work, .pair_scratch = &pair_sort };
    var broadphase_workspace = pipeline.BroadphaseWorkspace{ .assets = &assets, .collider_views = &views, .proxies = &proxies, .buffers = &sap_buffers };
    var cache_next: [2]contact_cache.Patch = undefined;
    var events: [2]contact_cache.Event = undefined;
    var contact_workspace = pipeline.AnalyticContactWorkspace{ .broadphase = &broadphase_workspace, .narrow = &narrow, .surface = &surface, .cache = &cache, .cache_next = &cache_next, .events = &events };
    var island_edges: [1]constraints.Edge = undefined;
    var edge_scratch: [1]constraints.Edge = undefined;
    var islands_storage: [1]constraints.Island = undefined;
    var members: [2]gravity.core.ids.BodyId = undefined;
    var lock_rows: [0]constraints.ConstraintRow = .{};
    var islands = pipeline.IslandWorkspace{ .edges = &island_edges, .edge_scratch = &edge_scratch, .islands = &islands_storage, .members = &members, .lock_rows = &lock_rows };
    var substep_events: [2]contact_cache.Event = undefined;
    var previous: [2]contact_cache.Patch = undefined;
    var event_next: [2]contact_cache.Patch = undefined;
    var tick_events: [2]contact_cache.Event = undefined;
    var solver_pipeline = pipeline.AnalyticSolverPipelineWorkspace{ .contacts = &contact_workspace, .solver = &solver, .islands = &islands, .substep_events = &substep_events, .previous = &previous, .event_next = &event_next, .tick_events = &tick_events };
    var persistent = pipeline.State{};
    var commands: [1]world.Command = undefined;
    var trace: [16]pipeline.Phase = undefined;
    var step_workspace = pipeline.Workspace{ .commands = &commands, .trace = &trace };
    const stepped = try pipeline.stepWithAnalyticSolver(&runtime, &persistent, .default, &.{}, &step_workspace, &solver_pipeline, &math);
    try std.testing.expectEqual(@as(u64, 1), stepped.step.tick);
    try std.testing.expectEqual(mesh_collider.value, cache.active()[0].key.collider_a.value);
    try std.testing.expectEqual(box_collider.value, cache.active()[0].key.collider_b.value);
    try std.testing.expectEqual(pipeline.Phase.solve, stepped.step.trace[6]);
}

test "pipeline steps a sphere against a static heightfield through the solver" {
    var math = fp.MathStatus{};
    const samples = [_]fp.Fp{ .zero, .zero, .zero, .zero };
    const cells = [_]baked.HeightCell{.{ .material_id = 4 }};
    var tile_nodes: [1]baked.BvhNode = undefined;
    const tiles = try baked.buildHeightFieldTiles(2, 2, &samples, &cells, &tile_nodes);
    var bytes: [1024]u8 = undefined;
    var bake_scratch: [512]u8 = undefined;
    const encoded = try baked.encodeHeightField(.{ .source_id = 73, .width = 2, .height = 2, .samples = &samples, .cells = &cells, .tile_nodes = tiles }, &bytes, &bake_scratch);
    const inputs = [_][]const u8{encoded.bytes};
    var asset_memory: [2048]u8 align(@alignOf(store.Asset)) = undefined;
    const assets = try store.Store.init(&asset_memory, &inputs);

    var fixture: Fixture = .{};
    const base = try fixture.init();
    var collider_body: [2]gravity.core.ids.BodyId = undefined;
    var collider_local: [2]g.Transform3 = undefined;
    var collider_shape: [2]shapes.Shape = undefined;
    var material: [2]shapes.Material = undefined;
    var category: [2]u32 = undefined;
    var mask: [2]u32 = undefined;
    var group: [2]i32 = undefined;
    var sensor: [2]bool = undefined;
    var enabled: [2]bool = undefined;
    var revision: [2]u32 = undefined;
    var collider_generation: [2]u32 = undefined;
    var collider_alive: [2]bool = undefined;
    var collider_retired: [2]bool = undefined;
    var runtime = try world.World.initWithColliders(base.storage, .{ .body = &collider_body, .local = &collider_local, .shape = &collider_shape, .material = &material, .category = &category, .mask = &mask, .group = &group, .sensor = &sensor, .enabled = &enabled, .revision = &revision, .generation = &collider_generation, .alive = &collider_alive, .retired = &collider_retired });
    const ground = try runtime.create(.{ .body_type = .static, .inverse_inertia_local = inertia() }, &math);
    const sphere = try runtime.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia(), .transform = .{ .position = .{ .x = fp.Fp.fromRatio(1, 2, &math), .y = fp.Fp.fromRatio(1, 2, &math), .z = fp.Fp.fromRatio(1, 2, &math) } } }, &math);
    const ground_collider = try runtime.createCollider(.{ .body = ground, .shape = .{ .height_field = .{ .source_id = 73 } } });
    const sphere_collider = try runtime.createCollider(.{ .body = sphere, .shape = .{ .sphere = .{ .radius = .one } } });

    var query_tiles: [1]baked.BvhNode = undefined;
    var traversal_work: [1]mesh.NodePair = undefined;
    var pair_scratch: [1]mesh.PrimitivePair = undefined;
    var pair_output: [1]mesh.PrimitivePair = undefined;
    var triangles: [2]mesh.HeightTriangle = undefined;
    var surface_points: [2]gjk.ContactPoint = undefined;
    var surface = pipeline.SurfaceNarrowWorkspace{ .sphere_heightfield = .{ .tile_nodes = &query_tiles, .work = &traversal_work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .triangles = &triangles, .contacts = &surface_points } };
    var views: [2]shapes.Collider = undefined;
    var proxies: [2]broadphase.Proxy = undefined;
    var endpoints: [4]broadphase.Endpoint = undefined;
    var endpoint_scratch: [4]broadphase.Endpoint = undefined;
    var active: [2]u32 = undefined;
    var sap_pairs: [1]broadphase.PairKey = undefined;
    var pair_work: [1]broadphase.PairKey = undefined;
    var pair_sort: [1]broadphase.PairKey = undefined;
    var sap_buffers = broadphase.Buffers{ .endpoints = &endpoints, .endpoint_scratch = &endpoint_scratch, .active = &active, .pairs = &sap_pairs, .pair_work = &pair_work, .pair_scratch = &pair_sort };
    var broadphase_workspace = pipeline.BroadphaseWorkspace{ .assets = &assets, .collider_views = &views, .proxies = &proxies, .buffers = &sap_buffers };
    var narrow: [2]contact_cache.Patch = undefined;
    var cache_storage: [2]contact_cache.Patch = undefined;
    var cache = contact_cache.Cache{ .patches = &cache_storage };
    var cache_next: [2]contact_cache.Patch = undefined;
    var events: [2]contact_cache.Event = undefined;
    var contact_workspace = pipeline.AnalyticContactWorkspace{ .broadphase = &broadphase_workspace, .narrow = &narrow, .surface = &surface, .cache = &cache, .cache_next = &cache_next, .events = &events };
    var solver_contacts: [2]gravity.dynamics.contact_solver.Contact = undefined;
    var solver_points: [8]gravity.dynamics.contact_solver.Point = undefined;
    var biases: [8]fp.Fp = undefined;
    var pseudo_linear: [2]g.Vec3 = undefined;
    var pseudo_angular: [2]g.Vec3 = undefined;
    var solver = pipeline.AnalyticSolverWorkspace{ .contacts = &solver_contacts, .points = &solver_points, .restitution_bias = &biases, .pseudo = .{ .linear = &pseudo_linear, .angular = &pseudo_angular }, .surface = surface };
    var island_edges: [1]constraints.Edge = undefined;
    var edge_scratch: [1]constraints.Edge = undefined;
    var islands_storage: [1]constraints.Island = undefined;
    var members: [2]gravity.core.ids.BodyId = undefined;
    var lock_rows: [0]constraints.ConstraintRow = .{};
    var islands = pipeline.IslandWorkspace{ .edges = &island_edges, .edge_scratch = &edge_scratch, .islands = &islands_storage, .members = &members, .lock_rows = &lock_rows };
    var substep_events: [2]contact_cache.Event = undefined;
    var previous: [2]contact_cache.Patch = undefined;
    var event_next: [2]contact_cache.Patch = undefined;
    var tick_events: [2]contact_cache.Event = undefined;
    var solver_pipeline = pipeline.AnalyticSolverPipelineWorkspace{ .contacts = &contact_workspace, .solver = &solver, .islands = &islands, .substep_events = &substep_events, .previous = &previous, .event_next = &event_next, .tick_events = &tick_events };
    var persistent = pipeline.State{};
    var commands: [1]world.Command = undefined;
    var trace: [16]pipeline.Phase = undefined;
    var step_workspace = pipeline.Workspace{ .commands = &commands, .trace = &trace };
    const result = try pipeline.stepWithAnalyticSolver(&runtime, &persistent, .default, &.{}, &step_workspace, &solver_pipeline, &math);
    try std.testing.expectEqual(@as(u64, 1), result.step.tick);
    try std.testing.expectEqual(ground_collider.value, cache.active()[0].key.collider_a.value);
    try std.testing.expectEqual(sphere_collider.value, cache.active()[0].key.collider_b.value);
    try std.testing.expectEqual(pipeline.Phase.solve, result.step.trace[6]);
    try std.testing.expectEqual(pipeline.Phase.hash, result.step.trace[result.step.trace.len - 1]);
    try std.testing.expect(cache.active()[0].points[0].feature_b < 2);

    // A producer-specific fault is not allowed to publish the failed tick;
    // it retains the canonical pair representative for diagnostics.
    contact_workspace.surface = null;
    try std.testing.expectError(error.Faulted, pipeline.stepWithAnalyticSolver(&runtime, &persistent, .default, &.{}, &step_workspace, &solver_pipeline, &math));
    const fault = persistent.fault.?;
    try std.testing.expectEqual(@as(u64, 1), persistent.tick);
    try std.testing.expectEqual(pipeline.Phase.narrowphase, fault.phase);
    try std.testing.expectEqual(ground_collider.value, fault.object.?);
    try std.testing.expectEqual(pipeline.FaultCode.contact, fault.code);
    try std.testing.expectEqual(pipeline.FaultDetail.contact, fault.detail);
}

test "pipeline sphere compound surface rebuilds child-path witnesses for the solver" {
    var math = fp.MathStatus{};
    const vertices = [_]g.Vec3{ .{}, .{ .x = fp.Fp.one }, .{ .y = fp.Fp.one } };
    const triangles = [_]baked.Triangle{.{ .a = 0, .b = 1, .c = 2 }};
    const mesh_nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = g.Vec3.zero, .max = .{ .x = fp.Fp.one, .y = fp.Fp.one } }, 0, 1)};
    const primitives = [_]u32{0};
    var mesh_bytes: [512]u8 = undefined;
    var bake_scratch: [256]u8 = undefined;
    const encoded_mesh = try baked.encodeMesh(.{ .source_id = 51, .vertices = &vertices, .triangles = &triangles, .nodes = &mesh_nodes, .primitives = &primitives }, &mesh_bytes, &bake_scratch);
    const children = [_]baked.CompoundChild{.{ .ordinal = 0, .content_hash = encoded_mesh.content_hash, .translation = .{ .x = fp.Fp.fromInt(3) }, .rotation = g.Quat.identity }};
    const compound_nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = .{ .x = fp.Fp.fromInt(3) }, .max = .{ .x = fp.Fp.fromInt(4), .y = fp.Fp.one } }, 0, 1)};
    var compound_bytes: [512]u8 = undefined;
    var compound_scratch: [256]u8 = undefined;
    const encoded_compound = try baked.encodeCompound(.{ .source_id = 52, .children = &children, .nodes = &compound_nodes }, &compound_bytes, &compound_scratch);
    const inputs = [_][]const u8{ encoded_mesh.bytes, encoded_compound.bytes };
    var asset_memory: [2048]u8 align(@alignOf(store.Asset)) = undefined;
    const assets = try store.Store.init(&asset_memory, &inputs);

    var fixture: Fixture = .{};
    const base = try fixture.init();
    var collider_body: [2]gravity.core.ids.BodyId = undefined;
    var collider_local: [2]g.Transform3 = undefined;
    var collider_shape: [2]shapes.Shape = undefined;
    var material: [2]shapes.Material = undefined;
    var category: [2]u32 = undefined;
    var mask: [2]u32 = undefined;
    var group: [2]i32 = undefined;
    var sensor: [2]bool = undefined;
    var enabled: [2]bool = undefined;
    var revision: [2]u32 = undefined;
    var collider_generation: [2]u32 = undefined;
    var collider_alive: [2]bool = undefined;
    var collider_retired: [2]bool = undefined;
    var runtime = try world.World.initWithColliders(base.storage, .{ .body = &collider_body, .local = &collider_local, .shape = &collider_shape, .material = &material, .category = &category, .mask = &mask, .group = &group, .sensor = &sensor, .enabled = &enabled, .revision = &revision, .generation = &collider_generation, .alive = &collider_alive, .retired = &collider_retired });
    const compound_body = try runtime.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia() }, &math);
    const sphere_body = try runtime.create(.{ .inverse_mass = .one, .inverse_inertia_local = inertia(), .transform = .{ .position = .{ .x = fp.Fp.fromRatio(13, 4, &math), .y = fp.Fp.fromRatio(1, 2, &math), .z = fp.Fp.fromRatio(1, 4, &math) } } }, &math);
    const compound_collider = try runtime.createCollider(.{ .body = compound_body, .shape = .{ .compound = .{ .source_id = 52 } } });
    const sphere_collider = try runtime.createCollider(.{ .body = sphere_body, .shape = .{ .sphere = .{ .radius = fp.Fp.one } } });

    var leaves: [1]shapes.CompoundLeaf = undefined;
    var loaded_nodes: [1]baked.BvhNode = undefined;
    var loaded_primitives: [1]u32 = undefined;
    var work: [1]mesh.NodePair = undefined;
    var pair_scratch: [1]mesh.PrimitivePair = undefined;
    var pair_output: [1]mesh.PrimitivePair = undefined;
    var mesh_contacts: [1]gjk.ContactPoint = undefined;
    var empty_tiles: [0]baked.BvhNode = .{};
    var empty_triangles: [0]mesh.HeightTriangle = .{};
    var height_contacts: [0]gjk.ContactPoint = .{};
    var merged: [4]gjk.ContactPoint = undefined;
    const sphere_compound = mesh.SphereCompoundSurfaceWorkspace{ .leaves = &leaves, .mesh = .{ .nodes = &loaded_nodes, .primitives = &loaded_primitives, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .contacts = &mesh_contacts }, .heightfield = .{ .tile_nodes = &empty_tiles, .work = &work, .pair_scratch = &pair_scratch, .pair_output = &pair_output, .triangles = &empty_triangles, .contacts = &height_contacts }, .merged = &merged };
    var surface = pipeline.SurfaceNarrowWorkspace{ .sphere_compound = sphere_compound };
    var narrow: [4]contact_cache.Patch = undefined;
    const patches = try pipeline.narrowRuntimePairs(&runtime, &assets, &.{broadphase.PairKey.init(compound_collider, sphere_collider)}, null, &surface, &narrow, &math);
    try std.testing.expectEqual(@as(usize, 1), patches.len);
    try std.testing.expectEqual(@as(u8, 1), patches[0].key.path_a.len);
    try std.testing.expectEqual(@as(u32, 0), patches[0].key.path_a.values[0]);
    var cache_storage: [4]contact_cache.Patch = undefined;
    @memcpy(cache_storage[0..patches.len], patches);
    var cache = contact_cache.Cache{ .patches = &cache_storage, .len = patches.len };
    var solver_contacts: [4]gravity.dynamics.contact_solver.Contact = undefined;
    var solver_points: [16]gravity.dynamics.contact_solver.Point = undefined;
    var biases: [16]fp.Fp = undefined;
    var pseudo_linear: [2]g.Vec3 = undefined;
    var pseudo_angular: [2]g.Vec3 = undefined;
    var solver = pipeline.AnalyticSolverWorkspace{ .contacts = &solver_contacts, .points = &solver_points, .restitution_bias = &biases, .pseudo = .{ .linear = &pseudo_linear, .angular = &pseudo_angular }, .surface = surface };
    const rebuilt = try pipeline.buildAnalyticSolverContacts(&runtime, &assets, &cache, &solver, &math);
    try std.testing.expectEqual(@as(usize, 1), rebuilt.len);
    try std.testing.expectEqual(@as(usize, 1), rebuilt[0].points.len);

    var views: [2]shapes.Collider = undefined;
    var proxies: [2]broadphase.Proxy = undefined;
    var endpoints: [4]broadphase.Endpoint = undefined;
    var endpoint_scratch: [4]broadphase.Endpoint = undefined;
    var active: [2]u32 = undefined;
    var sap_pairs: [1]broadphase.PairKey = undefined;
    var pair_work: [1]broadphase.PairKey = undefined;
    var pair_sort: [1]broadphase.PairKey = undefined;
    var sap_buffers = broadphase.Buffers{ .endpoints = &endpoints, .endpoint_scratch = &endpoint_scratch, .active = &active, .pairs = &sap_pairs, .pair_work = &pair_work, .pair_scratch = &pair_sort };
    var broadphase_workspace = pipeline.BroadphaseWorkspace{ .assets = &assets, .collider_views = &views, .proxies = &proxies, .buffers = &sap_buffers };
    var tick_narrow: [4]contact_cache.Patch = undefined;
    var cache_next: [4]contact_cache.Patch = undefined;
    var events: [4]contact_cache.Event = undefined;
    var tick_cache_storage: [4]contact_cache.Patch = undefined;
    var tick_cache = contact_cache.Cache{ .patches = &tick_cache_storage };
    var contacts_workspace = pipeline.AnalyticContactWorkspace{ .broadphase = &broadphase_workspace, .narrow = &tick_narrow, .surface = &surface, .cache = &tick_cache, .cache_next = &cache_next, .events = &events };
    var persistent = pipeline.State{};
    var commands: [1]world.Command = undefined;
    var trace: [8]pipeline.Phase = undefined;
    var step_workspace = pipeline.Workspace{ .commands = &commands, .trace = &trace };
    const stepped = try pipeline.stepWithAnalyticContacts(&runtime, &persistent, .default, &.{}, &step_workspace, &contacts_workspace, &math);
    try std.testing.expectEqual(@as(u64, 1), stepped.step.tick);
    try std.testing.expect(tick_cache.len > 0);
    try std.testing.expectEqual(@as(u8, 1), tick_cache.active()[0].key.path_a.len);

    tick_cache.len = 0;
    var island_edges: [2]constraints.Edge = undefined;
    var edge_scratch: [2]constraints.Edge = undefined;
    var islands_storage: [2]constraints.Island = undefined;
    var members: [2]gravity.core.ids.BodyId = undefined;
    var lock_rows: [0]constraints.ConstraintRow = .{};
    var islands = pipeline.IslandWorkspace{ .edges = &island_edges, .edge_scratch = &edge_scratch, .islands = &islands_storage, .members = &members, .lock_rows = &lock_rows };
    var substep_events: [4]contact_cache.Event = undefined;
    var previous: [4]contact_cache.Patch = undefined;
    var event_next: [4]contact_cache.Patch = undefined;
    var tick_events: [4]contact_cache.Event = undefined;
    var solver_pipeline = pipeline.AnalyticSolverPipelineWorkspace{ .contacts = &contacts_workspace, .solver = &solver, .islands = &islands, .substep_events = &substep_events, .previous = &previous, .event_next = &event_next, .tick_events = &tick_events };
    var solver_trace: [16]pipeline.Phase = undefined;
    var solver_step_workspace = pipeline.Workspace{ .commands = &commands, .trace = &solver_trace };
    const solved = try pipeline.stepWithAnalyticSolver(&runtime, &persistent, .default, &.{}, &solver_step_workspace, &solver_pipeline, &math);
    try std.testing.expectEqual(@as(u64, 2), solved.step.tick);
    try std.testing.expectEqual(pipeline.Phase.solve, solved.step.trace[6]);
    try std.testing.expect(tick_cache.len > 0);

    // The normal unit suite deliberately stays short. `zig build
    // test-pipeline-long-run` opts into the full million-tick fixed-pipeline
    // execution through this environment switch, using the exact same World,
    // asset, collision, island and solver route above. Each tick receives a
    // deterministic pseudo-random, legal velocity command so every platform
    // exercises the command prevalidation and commit phases as well.
    const long_run = std.process.Environ.getAlloc(
        .{ .block = .global },
        std.testing.allocator,
        "GRAVITY_PIPELINE_LONG_RUN",
    ) catch null;
    defer if (long_run) |value| std.testing.allocator.free(value);
    if (long_run != null) {
        var random: u32 = 0x6d2b_79f5;
        var tick: usize = 2;
        while (tick < 1_000_000) : (tick += 1) {
            random = random *% 1_664_525 +% 1_013_904_223;
            const velocity_x = switch (random % 3) {
                0 => fp.Fp.fromRatio(-1, 32, &math),
                1 => fp.Fp.zero,
                else => fp.Fp.fromRatio(1, 32, &math),
            };
            commands[0] = .{
                .key = .{ .phase_priority = 0, .issuer = 1, .sequence = @intCast(tick), .discriminant = 0 },
                .op = .{ .velocity = .{ .body = sphere_body, .linear = .{ .x = velocity_x }, .angular = .{} } },
            };
            _ = try pipeline.stepWithAnalyticSolver(&runtime, &persistent, .default, &commands, &solver_step_workspace, &solver_pipeline, &math);
        }
        try std.testing.expectEqual(@as(u64, 1_000_000), persistent.tick);
        const final_hash = pipeline.canonicalStateHash(&runtime, &persistent, .default, .{ .cache = &tick_cache });
        try std.testing.expectEqualSlices(u8, &[_]u8{
            0x8e, 0xca, 0x34, 0xd1, 0x2b, 0x43, 0x4d, 0x1b,
            0xdf, 0xe6, 0x23, 0x40, 0x3c, 0xf3, 0x70, 0x69,
        }, &final_hash);
    }
}
