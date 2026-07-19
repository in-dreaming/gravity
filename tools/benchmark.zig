//! Cross-target product-path benchmark used by native, ARM64 and WASI gates.
//! It intentionally executes the real broadphase, narrowphase, solver, CCD,
//! sleeping, event and state-hash pipeline rather than a metadata loop.
const std = @import("std");
const builtin = @import("builtin");
const gravity = @import("gravity");
const replay_tool = @import("replay.zig");

const fp = gravity.math.fp;
const geometry = gravity.math.geometry;
const shapes = gravity.collision.shapes;
const snapshot = gravity.state.snapshot;
const replay = gravity.state.replay;
const asset_store = gravity.assets.store;

const body_count = 48;
const sample_ticks = 120;
const realtime_tick_ns = 16_666_667;

fn identityInertia() geometry.SymmetricMat3 {
    return .{ .xx = .one, .yy = .one, .zz = .one, .xy = .zero, .xz = .zero, .yz = .zero };
}

fn configuration() !gravity.core.config.SimulationConfig {
    var value = gravity.core.config.SimulationConfig.default;
    value.capacities.body = body_count;
    value.capacities.collider = body_count;
    value.capacities.joint = 1;
    value.capacities.command_per_tick = 8;
    value.capacities.broad_pair = 192;
    value.capacities.contact_patch = 192;
    value.capacities.contact_point = 768;
    value.capacities.sensor_overlap = 192;
    value.capacities.event_per_tick = 384;
    value.capacities.rollback_window = 8;
    try value.validate();
    return value;
}

fn nowNs() i96 {
    return std.Io.Clock.Timestamp.now(std.Options.debug_io, .awake).raw.nanoseconds;
}

pub fn main(init: std.process.Init) !void {
    _ = init;
    const allocator = std.heap.page_allocator;
    const store_memory = try allocator.alignedAlloc(u8, .of(asset_store.Asset), 1);
    defer allocator.free(store_memory);
    const assets = try asset_store.Store.init(store_memory, &.{});
    const config = try configuration();
    var host = try replay_tool.Host.init(allocator, .{ .configuration = config, .asset_set = assets.asset_set_hash }, &assets);
    defer host.deinit();

    for (0..body_count) |index| {
        const group = index / 4;
        const within = index % 4;
        const base = fp.Fp.fromInt(@intCast(group * 8));
        const offset = fp.Fp.fromRatio(@intCast(within), 4, &host.status);
        const body = try host.target.create(.{ .body_type = .dynamic, .transform = .{ .position = .{ .x = base.add(offset, &host.status) } }, .inverse_mass = .one, .inverse_inertia_local = identityInertia() }, &host.status);
        _ = try host.target.createCollider(.{ .body = body, .shape = .{ .sphere = .{ .radius = .one } } });
    }

    var input_storage: [4]u8 = undefined;
    const input = try replay.encodeCommands(&.{}, &input_storage);
    var checksum: u8 = 0;
    var peak_contacts: usize = 0;
    const started = nowNs();
    for (0..sample_ticks) |index| {
        const hash = try replay.FullWorldHost.step(&host.full, .{ .tick = index + 1, .input = input, .expected_hash = [_]u8{0} ** 16 });
        checksum ^= hash[0];
        peak_contacts = @max(peak_contacts, host.contacts.len);
    }
    const elapsed_ns: u64 = @intCast(nowNs() - started);
    const average_ns = elapsed_ns / sample_ticks;
    if (peak_contacts < 72 or host.state.fault != null) return error.InvalidProductWorkload;
    if (builtin.target.os.tag == .wasi and average_ns > realtime_tick_ns) return error.WasmRealtimeBudgetExceeded;
    std.debug.print("product benchmark: target={s}, bodies={d}, peak_contacts={d}, ticks={d}, average_tick_ns={d}, checksum={d}\n", .{ @tagName(builtin.target.os.tag), body_count, peak_contacts, sample_ticks, average_ns, checksum });
}
