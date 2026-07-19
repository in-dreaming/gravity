//! Frozen Task 24 performance corpus and product-budget contract.
//!
//! Counts describe authored scene state. `expected_contacts` is checked after
//! the first full pipeline Tick so a scene cannot silently become cheaper.
const std = @import("std");

pub const Kind = enum { small, medium, stress, mesh_heavy, joint_heavy, ccd };

pub const Budget = struct {
    native_p95_ns: u64,
    native_p99_ns: u64,
    snapshot_p95_ns: u64,
    rollback_8_p95_ns: u64,
    max_workspace_bytes: u64,
};

pub const Scene = struct {
    kind: Kind,
    name: []const u8,
    bodies: u32,
    colliders: u32,
    joints: u32,
    broad_pairs: u32,
    contact_patches: u32,
    contact_points: u32,
    expected_contacts: u32,
    group_size: u8 = 0,
    ccd_colliders: u32 = 0,
    mesh_cells_per_axis: u16 = 0,
    budget: Budget,
};

const realtime_tick_ns = 16_666_667;

pub const scenes = [_]Scene{
    .{ .kind = .small, .name = "Small", .bodies = 128, .colliders = 128, .joints = 64, .broad_pairs = 512, .contact_patches = 512, .contact_points = 2_048, .expected_contacts = 180, .group_size = 4, .budget = .{ .native_p95_ns = 2_000_000, .native_p99_ns = 3_000_000, .snapshot_p95_ns = 2_000_000, .rollback_8_p95_ns = realtime_tick_ns, .max_workspace_bytes = 64 * 1024 * 1024 } },
    // 500 groups of five coincident spheres produce exactly 5,000 pairwise
    // contacts; 512 authored joints satisfies the minimum Medium contract.
    .{ .kind = .medium, .name = "Medium", .bodies = 2_500, .colliders = 2_500, .joints = 512, .broad_pairs = 8_192, .contact_patches = 8_192, .contact_points = 32_768, .expected_contacts = 5_000, .group_size = 5, .budget = .{ .native_p95_ns = realtime_tick_ns, .native_p99_ns = 20_000_000, .snapshot_p95_ns = 12_000_000, .rollback_8_p95_ns = realtime_tick_ns, .max_workspace_bytes = 512 * 1024 * 1024 } },
    // Default product capacities for bodies/joints with a representative
    // contact subset that remains below the frozen 32,768 patch ceiling.
    .{ .kind = .stress, .name = "Stress", .bodies = 8_192, .colliders = 8_192, .joints = 8_192, .broad_pairs = 131_072, .contact_patches = 32_768, .contact_points = 131_072, .expected_contacts = 28_672, .group_size = 8, .budget = .{ .native_p95_ns = 100_000_000, .native_p99_ns = 125_000_000, .snapshot_p95_ns = 40_000_000, .rollback_8_p95_ns = 750_000_000, .max_workspace_bytes = 2 * 1024 * 1024 * 1024 } },
    .{ .kind = .mesh_heavy, .name = "MeshHeavy", .bodies = 257, .colliders = 257, .joints = 1, .broad_pairs = 2_048, .contact_patches = 2_048, .contact_points = 8_192, .expected_contacts = 256, .mesh_cells_per_axis = 16, .budget = .{ .native_p95_ns = 50_000_000, .native_p99_ns = 65_000_000, .snapshot_p95_ns = 8_000_000, .rollback_8_p95_ns = 350_000_000, .max_workspace_bytes = 256 * 1024 * 1024 } },
    .{ .kind = .joint_heavy, .name = "JointHeavy", .bodies = 2_048, .colliders = 2_048, .joints = 8_192, .broad_pairs = 4_096, .contact_patches = 4_096, .contact_points = 16_384, .expected_contacts = 0, .budget = .{ .native_p95_ns = 50_000_000, .native_p99_ns = 65_000_000, .snapshot_p95_ns = 20_000_000, .rollback_8_p95_ns = 350_000_000, .max_workspace_bytes = 768 * 1024 * 1024 } },
    .{ .kind = .ccd, .name = "CCD", .bodies = 1_024, .colliders = 1_024, .joints = 1, .broad_pairs = 4_096, .contact_patches = 4_096, .contact_points = 16_384, .expected_contacts = 512, .ccd_colliders = 512, .budget = .{ .native_p95_ns = 65_000_000, .native_p99_ns = 80_000_000, .snapshot_p95_ns = 16_000_000, .rollback_8_p95_ns = 450_000_000, .max_workspace_bytes = 384 * 1024 * 1024 } },
};

pub fn byName(name: []const u8) ?Scene {
    for (scenes) |scene| if (std.ascii.eqlIgnoreCase(name, scene.name)) return scene;
    return null;
}

pub fn validate() !void {
    for (scenes, 0..) |scene, index| {
        if (scene.bodies == 0 or scene.colliders == 0 or scene.bodies > 8_192 or scene.colliders > 16_384 or scene.joints == 0 or scene.joints > 8_192) return error.InvalidCorpus;
        if (scene.contact_points < scene.contact_patches or scene.expected_contacts > scene.contact_patches) return error.InvalidCorpus;
        if (scene.budget.native_p95_ns > scene.budget.native_p99_ns or scene.budget.rollback_8_p95_ns == 0) return error.InvalidCorpus;
        for (scenes[0..index]) |prior| if (std.mem.eql(u8, prior.name, scene.name)) return error.InvalidCorpus;
    }
    const medium = byName("Medium") orelse return error.InvalidCorpus;
    if (medium.bodies < 2_000 or medium.expected_contacts < 5_000 or medium.joints < 512) return error.InvalidCorpus;
    const stress = byName("Stress") orelse return error.InvalidCorpus;
    if (stress.bodies != 8_192 or stress.joints != 8_192) return error.InvalidCorpus;
}

test "Task 24 corpus and product budgets are frozen and self-consistent" {
    try validate();
    try std.testing.expectEqual(@as(usize, 6), scenes.len);
}
