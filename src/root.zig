const build_options = @import("build_options");
const version = @import("version.zig");

/// Frozen Task 22/23 logical batch contract. Native executor adapters remain
/// outside core; physics imports only this ownership/synchronization surface.
pub const jobs = @import("gravity_jobs");

pub const math = struct {
    pub const fp = @import("math/fp.zig");
    pub const wide = @import("math/wide.zig");
    pub const envelope = @import("math/envelope.zig");
    pub const geometry = @import("math/geometry.zig");
};

pub const core = struct {
    pub const memory = @import("core/memory.zig");
    pub const ids = @import("core/ids.zig");
    pub const radix = @import("core/radix.zig");
    pub const config = @import("core/config.zig");
};

pub const state = struct {
    pub const codec = @import("state/codec.zig");
    pub const hash = @import("state/hash.zig");
    pub const snapshot = @import("state/snapshot.zig");
    pub const rollback = @import("state/rollback.zig");
    pub const replay = @import("state/replay.zig");
    pub const diff = @import("state/diff.zig");
};

pub const geometry = struct {
    pub const baked = @import("geometry/baked.zig");
};
pub const collision = struct {
    pub const shapes = @import("collision/shapes.zig");
    pub const broadphase = @import("collision/broadphase.zig");
    pub const analytic = @import("collision/analytic.zig");
    pub const gjk = @import("collision/gjk.zig");
    pub const mesh = @import("collision/mesh.zig");
    pub const contact_cache = @import("collision/contact_cache.zig");
};
pub const query = struct {
    pub const queries = @import("query/queries.zig");
};

pub const assets = struct {
    pub const store = @import("assets/store.zig");
    pub const source = @import("assets/source.zig");
    pub const runtime_view = @import("assets/runtime_view.zig");
};
pub const dynamics = struct {
    pub const world = @import("dynamics/world.zig");
    pub const constraints = @import("dynamics/constraints.zig");
    pub const contact_solver = @import("dynamics/contact_solver.zig");
    pub const joints = @import("dynamics/joints.zig");
    pub const sleeping = @import("dynamics/sleeping.zig");
    pub const ccd = @import("dynamics/ccd.zig");
    pub const pipeline = @import("dynamics/pipeline.zig");
};

/// Stable C ABI implementation. Core modules never import this upper layer.
pub const abi = @import("abi/root.zig");

pub const abi_version = version.abi_version;
pub const protocol_version = version.protocol_version;
pub const snapshot_format_version = version.snapshot_format_version;
pub const asset_format_version = version.asset_format_version;

/// Returns immutable metadata embedded by the build graph. It contains no host
/// state and is safe to use in deterministic diagnostics.
pub fn buildMetadata() version.BuildMetadata {
    return .{
        .commit = build_options.commit,
        .zig_version = build_options.zig_version,
        .abi = abi_version,
        .protocol = protocol_version,
        .snapshot_format = snapshot_format_version,
        .asset_format = asset_format_version,
    };
}

test "embedded metadata agrees with the public version constants" {
    const metadata = buildMetadata();
    try @import("std").testing.expectEqual(abi_version, metadata.abi);
    try @import("std").testing.expectEqual(protocol_version, metadata.protocol);
    try @import("std").testing.expectEqual(snapshot_format_version, metadata.snapshot_format);
    try @import("std").testing.expectEqual(asset_format_version, metadata.asset_format);
    try @import("std").testing.expectEqualStrings("0.16.0", metadata.zig_version);
}
