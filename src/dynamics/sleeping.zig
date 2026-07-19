//! Deterministic caller-owned island sleep state.
//!
//! The world pipeline owns when this module is called; this module owns no
//! allocator, clock, or implicit global state.  State is indexed by live body
//! slot and therefore belongs in a future World snapshot verbatim.
const fp = @import("../math/fp.zig");
const geometry = @import("../math/geometry.zig");
const config = @import("../core/config.zig");
const ids = @import("../core/ids.zig");
const world_mod = @import("world.zig");
const constraints = @import("constraints.zig");
const contact_solver = @import("contact_solver.zig");
const joints = @import("joints.zig");

pub const Error = error{ CapacityExceeded, InvalidBody, InvalidIsland } || world_mod.Error || contact_solver.Error || joints.Error;
pub const WakeReason = enum(u8) { none, command, joint, contact, kinematic };
pub const EventKind = enum(u8) { sleep, wake };
pub const Event = struct { kind: EventKind, body: ids.BodyId, reason: WakeReason = .none };
pub const Request = struct { body: ids.BodyId, reason: WakeReason };

/// Per-live-body state.  `reason` is the first reason which woke the body in
/// the current tick; the world pipeline clears it after publishing events.
pub const Storage = struct {
    awake: []bool,
    counter: []u32,
    reason: []WakeReason,

    pub fn validate(self: Storage, world: *const world_mod.World) Error!void {
        const count = world.storage.alive.len;
        if (self.awake.len != count or self.counter.len != count or self.reason.len != count) return error.CapacityExceeded;
    }
    pub fn clearReasons(self: Storage) void {
        @memset(self.reason, .none);
    }
};

pub fn init(storage: Storage) Error!void {
    if (storage.awake.len != storage.counter.len or storage.awake.len != storage.reason.len) return error.CapacityExceeded;
    @memset(storage.awake, true);
    @memset(storage.counter, 0);
    storage.clearReasons();
}

/// Wakes an entire dynamic connected component.  `edges` must be the stable
/// contact/joint edge list used to build islands.  Static and kinematic bodies
/// never bridge two dynamic sleep islands, matching `constraints.build`.
///
/// Requests are processed in supplied canonical command/contact order.  A
/// body emits at most one wake event, and its first request retains the
/// earliest wake reason.
pub fn wakeGraph(world: *world_mod.World, storage: Storage, edges: []const constraints.Edge, requests: []const Request, scratch: []ids.BodyId, events: []Event) Error![]const Event {
    try storage.validate(world);
    if (scratch.len < world.storage.alive.len) return error.CapacityExceeded;
    for (edges) |edge| {
        _ = world.bodyIndex(edge.body_a) orelse return error.InvalidBody;
        if (edge.body_b.isValid()) _ = world.bodyIndex(edge.body_b) orelse return error.InvalidBody;
    }
    for (requests) |request| _ = world.bodyIndex(request.body) orelse return error.InvalidBody;

    // The normal active-simulation path has no sleeping dynamic body. Once
    // edge/request validation is complete, traversing the graph once per
    // contact cannot discover a transition or event. This O(body) proof
    // replaces the former O(requests * bodies * edges) no-op path without
    // weakening malformed-input validation or partially-awake semantics.
    var any_sleeping_dynamic = false;
    for (world.storage.alive, 0..) |alive, index| {
        if (alive and world.storage.body_type[index] == .dynamic and !storage.awake[index]) {
            any_sleeping_dynamic = true;
            break;
        }
    }
    if (!any_sleeping_dynamic) return events[0..0];

    // First pass counts the exact, de-duplicated set.  This makes an event
    // capacity failure transactional even when several requests overlap.
    var marked: usize = 0;
    for (requests) |request| {
        const start = world.bodyIndex(request.body).?;
        if (world.storage.body_type[start] != .dynamic or contains(scratch[0..marked], request.body)) continue;
        const first = marked;
        scratch[marked] = request.body;
        marked += 1;
        var cursor = first;
        while (cursor < marked) : (cursor += 1) {
            const current = scratch[cursor];
            for (edges) |edge| {
                const other = neighbor(edge, current) orelse continue;
                const index = world.bodyIndex(other).?;
                if (world.storage.body_type[index] != .dynamic or contains(scratch[0..marked], other)) continue;
                scratch[marked] = other;
                marked += 1;
            }
        }
    }
    var required: usize = 0;
    for (scratch[0..marked]) |body| {
        if (!storage.awake[world.bodyIndex(body).?]) required += 1;
    }
    if (required > events.len) return error.CapacityExceeded;

    var event_count: usize = 0;
    // Repeat by request, so the first canonical request supplies the reason.
    for (requests) |request| {
        const start = world.bodyIndex(request.body).?;
        if (world.storage.body_type[start] != .dynamic) continue;
        var first: usize = 0;
        scratch[0] = request.body;
        var end: usize = 1;
        while (first < end) : (first += 1) {
            const current = scratch[first];
            for (edges) |edge| {
                const other = neighbor(edge, current) orelse continue;
                const index = world.bodyIndex(other).?;
                if (world.storage.body_type[index] != .dynamic or contains(scratch[0..end], other)) continue;
                scratch[end] = other;
                end += 1;
            }
        }
        sortIds(scratch[0..end]);
        for (scratch[0..end]) |body| {
            const index = world.bodyIndex(body).?;
            if (storage.awake[index]) continue;
            storage.awake[index] = true;
            storage.counter[index] = 0;
            storage.reason[index] = request.reason;
            events[event_count] = .{ .kind = .wake, .body = body, .reason = request.reason };
            event_count += 1;
        }
    }
    sortEvents(events[0..event_count]);
    return events[0..event_count];
}

/// Canonically validates and executes body commands with their required wake
/// transition.  Waking is reserved before `World.execute`, so an insufficient
/// event or graph buffer cannot leave a command committed without its wake.
/// Callers use the returned events in the same tick's ordered event stream.
pub fn executeCommands(world: *world_mod.World, storage: Storage, edges: []const constraints.Edge, commands: []const world_mod.Command, command_scratch: []world_mod.Command, requests: []Request, graph_scratch: []ids.BodyId, events: []Event, dt: fp.Fp, status: *fp.MathStatus) Error![]const Event {
    const ordered = try world.orderedCommands(commands, command_scratch);
    if (ordered.len > requests.len) return error.CapacityExceeded;
    var count: usize = 0;
    for (ordered) |command| {
        const request = commandWake(command) orelse continue;
        requests[count] = request;
        count += 1;
    }
    const woke = try wakeGraph(world, storage, edges, requests[0..count], graph_scratch, events);
    // `ordered` was fully validated above; execute cannot fail on the same
    // static command data after the wake transition succeeds.
    try world.execute(ordered, command_scratch, dt, status);
    return woke;
}

/// Wakes response-contact islands before solving them.  Sensor contacts are
/// deliberately excluded: overlap observation alone cannot disturb sleep.
pub fn solveContacts(world: *world_mod.World, storage: Storage, edges: []const constraints.Edge, contacts: []const contact_solver.Contact, pseudo: contact_solver.PseudoVelocities, settings: contact_solver.Settings, requests: []Request, graph_scratch: []ids.BodyId, events: []Event, status: *fp.MathStatus) Error![]const Event {
    try contact_solver.validateInputs(world, contacts, pseudo);
    var count: usize = 0;
    for (contacts) |contact| {
        if (contact.patch.sensor or contact.points.len == 0) continue;
        if (count + 2 > requests.len) return error.CapacityExceeded;
        requests[count] = .{ .body = contact.body_a, .reason = .contact };
        requests[count + 1] = .{ .body = contact.body_b, .reason = .contact };
        count += 2;
    }
    const woke = try wakeGraph(world, storage, edges, requests[0..count], graph_scratch, events);
    try contact_solver.solve(world, contacts, pseudo, settings, status);
    return woke;
}

/// Wakes islands driven by active joint controls.  Passive equality joints do
/// not by themselves disturb a resting island; motors, springs and a limit
/// currently on either bound do.
pub fn wakeActiveJoints(world: *world_mod.World, storage: Storage, edges: []const constraints.Edge, pool: *const joints.Pool, requests: []Request, graph_scratch: []ids.BodyId, events: []Event) Error![]const Event {
    var count: usize = 0;
    for (pool.storage.values, 0..) |joint, index| {
        if (!pool.storage.alive[index] or !jointActive(joint)) continue;
        if (count + 2 > requests.len) return error.CapacityExceeded;
        requests[count] = .{ .body = joint.body_a, .reason = .joint };
        requests[count + 1] = .{ .body = joint.body_b, .reason = .joint };
        count += 2;
    }
    return wakeGraph(world, storage, edges, requests[0..count], graph_scratch, events);
}

/// Applies the all-members-below-threshold rule at tick end.  Counters are
/// synchronized per island, and sleep commits atomically per island.
pub fn step(world: *world_mod.World, islands: []const constraints.Island, members: []const ids.BodyId, storage: Storage, linear_threshold: fp.Fp, angular_threshold: fp.Fp, sleep_ticks: u32, events: []Event, status: *fp.MathStatus) Error![]const Event {
    try storage.validate(world);
    if (linear_threshold.raw < 0 or angular_threshold.raw < 0) return error.InvalidIsland;
    const linear2 = linear_threshold.mul(linear_threshold, status);
    const angular2 = angular_threshold.mul(angular_threshold, status);

    var required: usize = 0;
    for (islands) |island| {
        const range = try islandRange(island, members);
        const eligible = try isEligible(world, range, linear2, angular2, status);
        const next_counter = islandCounter(world, range, storage, eligible);
        if (!eligible or sleep_ticks == 0 or next_counter < sleep_ticks) continue;
        for (range) |body| {
            if (storage.awake[world.bodyIndex(body).?]) required += 1;
        }
    }
    if (required > events.len) return error.CapacityExceeded;

    var count: usize = 0;
    for (islands) |island| {
        const range = try islandRange(island, members);
        const eligible = try isEligible(world, range, linear2, angular2, status);
        const next_counter = islandCounter(world, range, storage, eligible);
        for (range) |body| storage.counter[world.bodyIndex(body).?] = next_counter;
        if (!eligible or sleep_ticks == 0 or next_counter < sleep_ticks) continue;
        for (range) |body| {
            const index = world.bodyIndex(body).?;
            if (!storage.awake[index]) continue;
            storage.awake[index] = false;
            storage.reason[index] = .none;
            world.storage.linear_velocity[index] = geometry.Vec3.zero;
            world.storage.angular_velocity[index] = geometry.Vec3.zero;
            world.storage.force[index] = geometry.Vec3.zero;
            world.storage.torque[index] = geometry.Vec3.zero;
            events[count] = .{ .kind = .sleep, .body = body };
            count += 1;
        }
    }
    return events[0..count];
}

/// Applies the configured sleep policy.  A disabled feature leaves every
/// dynamic slot awake and does not advance counters, giving callers the same
/// active-body oracle as a world built without sleeping.
pub fn stepConfigured(world: *world_mod.World, islands: []const constraints.Island, members: []const ids.BodyId, storage: Storage, simulation: config.SimulationConfig, events: []Event, status: *fp.MathStatus) Error![]const Event {
    try storage.validate(world);
    if (!simulation.features.sleeping) {
        for (world.storage.alive, 0..) |alive, index| {
            if (alive and world.storage.body_type[index] == .dynamic) storage.awake[index] = true;
        }
        @memset(storage.counter, 0);
        storage.clearReasons();
        return events[0..0];
    }
    return step(world, islands, members, storage, simulation.tolerances.sleep_linear_threshold, simulation.tolerances.sleep_angular_threshold, simulation.iterations.sleep_ticks, events, status);
}

/// Copies only awake islands into caller buffers.  The solver consumes this
/// view after sleep commits, so sleeping islands perform no warm-start or PGS
/// work while their broad-phase proxies remain in the world.
pub fn selectActive(world: *const world_mod.World, islands: []const constraints.Island, members: []const ids.BodyId, storage: Storage, output_islands: []constraints.Island, output_members: []ids.BodyId) Error!constraints.BuildResult {
    try storage.validate(world);
    var island_count: usize = 0;
    var member_count: usize = 0;
    for (islands) |island| {
        const range = try islandRange(island, members);
        const awake = storage.awake[world.bodyIndex(range[0]).?];
        for (range) |body| {
            if (storage.awake[world.bodyIndex(body).?] != awake) return error.InvalidIsland;
        }
        if (!awake) continue;
        if (island_count == output_islands.len or range.len > output_members.len - member_count) return error.CapacityExceeded;
        @memcpy(output_members[member_count .. member_count + range.len], range);
        output_islands[island_count] = .{ .id = island.id, .first_member = @intCast(member_count), .member_count = @intCast(range.len) };
        island_count += 1;
        member_count += range.len;
    }
    return .{ .islands = output_islands[0..island_count], .members = output_members[0..member_count], .rows = &.{} };
}

/// Visits all future-affecting sleep state in slot order.  The visitor shape
/// intentionally matches the existing canonical state visitors.
pub fn visitCanonical(storage: Storage, visitor: anytype) void {
    for (storage.awake, storage.counter, storage.reason, 0..) |awake, counter, reason, index| {
        visitor.field("sleep_alive", awake);
        visitor.field("sleep_counter", counter);
        visitor.field("sleep_reason", @intFromEnum(reason));
        visitor.field("sleep_slot", index);
    }
}

fn islandRange(island: constraints.Island, members: []const ids.BodyId) Error![]const ids.BodyId {
    const first: usize = island.first_member;
    const count: usize = island.member_count;
    if (count == 0 or first > members.len or count > members.len - first) return error.InvalidIsland;
    return members[first .. first + count];
}
fn isEligible(world: *const world_mod.World, bodies: []const ids.BodyId, linear2: fp.Fp, angular2: fp.Fp, status: *fp.MathStatus) Error!bool {
    for (bodies) |body| {
        const index = world.bodyIndex(body) orelse return error.InvalidBody;
        if (world.storage.body_type[index] != .dynamic or !world.storage.alive[index]) return error.InvalidIsland;
        if (world.storage.linear_velocity[index].lengthSquared(status).raw >= linear2.raw or world.storage.angular_velocity[index].lengthSquared(status).raw >= angular2.raw) return false;
    }
    return true;
}
fn islandCounter(world: *const world_mod.World, bodies: []const ids.BodyId, storage: Storage, eligible: bool) u32 {
    if (!eligible) return 0;
    var result = storage.counter[world.bodyIndex(bodies[0]).?];
    for (bodies[1..]) |body| result = @min(result, storage.counter[world.bodyIndex(body).?]);
    return result +| 1;
}
fn neighbor(edge: constraints.Edge, body: ids.BodyId) ?ids.BodyId {
    if (edge.body_a.value == body.value) return if (edge.body_b.isValid()) edge.body_b else null;
    if (edge.body_b.value == body.value) return edge.body_a;
    return null;
}
fn contains(items: []const ids.BodyId, id: ids.BodyId) bool {
    for (items) |item| if (item.value == id.value) return true;
    return false;
}
fn lessThan(a: ids.BodyId, b: ids.BodyId) bool {
    return if (a.index() != b.index()) a.index() < b.index() else a.generation() < b.generation();
}
fn sortIds(items: []ids.BodyId) void {
    var i: usize = 1;
    while (i < items.len) : (i += 1) {
        const value = items[i];
        var j = i;
        while (j > 0 and lessThan(value, items[j - 1])) : (j -= 1) items[j] = items[j - 1];
        items[j] = value;
    }
}
fn sortEvents(items: []Event) void {
    var i: usize = 1;
    while (i < items.len) : (i += 1) {
        const value = items[i];
        var j = i;
        while (j > 0 and lessThan(value.body, items[j - 1].body)) : (j -= 1) items[j] = items[j - 1];
        items[j] = value;
    }
}
fn commandWake(command: world_mod.Command) ?Request {
    return switch (command.op) {
        .force => |value| if (nonZero(value.value)) .{ .body = value.body, .reason = .command } else null,
        .torque => |value| if (nonZero(value.value)) .{ .body = value.body, .reason = .command } else null,
        .impulse_at_point => |value| if (nonZero(value.impulse)) .{ .body = value.body, .reason = .command } else null,
        .velocity => |value| if (nonZero(value.linear) or nonZero(value.angular)) .{ .body = value.body, .reason = .command } else null,
        .kinematic_target => |value| .{ .body = value.body, .reason = .kinematic },
        .locks => null,
    };
}
fn nonZero(value: geometry.Vec3) bool {
    return value.x.raw != 0 or value.y.raw != 0 or value.z.raw != 0;
}
fn jointActive(joint: joints.Joint) bool {
    if (joint.motor.enabled or joint.spring.enabled or joint.limit_state != .inactive) return true;
    return joint.cone_states[0] != .inactive or joint.cone_states[1] != .inactive;
}
