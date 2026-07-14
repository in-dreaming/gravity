//! Complete deterministic World configuration. Fields are visited in wire order.
const std = @import("std");
const fp = @import("../math/fp.zig");
const envelope = @import("../math/envelope.zig");

pub const CapacityConfig = struct {
    body: u32 = 8_192,
    collider: u32 = 16_384,
    joint: u32 = 8_192,
    command_per_tick: u32 = 16_384,
    broad_pair: u32 = 131_072,
    contact_patch: u32 = 32_768,
    contact_point: u32 = 131_072,
    sensor_overlap: u32 = 32_768,
    event_per_tick: u32 = 131_072,
    rollback_window: u32 = 120,
    convex_hull_vertices: u32 = 256,
    convex_hull_faces: u32 = 512,
    compound_children: u32 = 256,
    compound_depth: u32 = 8,
    mesh_vertices: u32 = 16_777_215,
    mesh_triangles: u32 = 16_777_215,
    heightfield_axis_samples: u32 = 65_535,
};

pub const ToleranceConfig = struct {
    linear_slop: fp.Fp = .{ .raw = 21_474_836 },
    angular_slop: fp.Fp = .{ .raw = 37_480_660 },
    convex_skin: fp.Fp = .{ .raw = 42_949_673 },
    aabb_margin: fp.Fp = .{ .raw = 214_748_365 },
    max_position_correction: fp.Fp = .{ .raw = 858_993_459 },
    max_angular_correction: fp.Fp = .{ .raw = 599_690_565 },
    restitution_threshold: fp.Fp = fp.Fp.one,
    warmstart_normal_cos_min: fp.Fp = .{ .raw = 3_719_551_787 },
    sleep_linear_threshold: fp.Fp = .{ .raw = 128_849_019 },
    sleep_angular_threshold: fp.Fp = .{ .raw = 128_849_019 },
};

pub const IterationConfig = struct {
    tick_hz: u32 = 60,
    substeps: u32 = 2,
    velocity: u32 = 10,
    position: u32 = 4,
    sleep_ticks: u32 = 30,
    gjk: u32 = 32,
    epa: u32 = 64,
    epa_max_faces: u32 = 256,
    shape_cast: u32 = 32,
    ccd_toi_per_substep: u32 = 8,
    mesh_bvh_leaf_triangles: u32 = 4,
};

pub const FeatureFlags = packed struct(u32) {
    ccd: bool = true,
    sleeping: bool = true,
    rollback: bool = true,
    sensors: bool = true,
    reserved: u28 = 0,
};

pub const ConfigError = error{ InvalidCapacity, InvalidIteration, InvalidTolerance, InvalidEnvelope, InvalidFeatureFlags };

pub const SimulationConfig = struct {
    capacities: CapacityConfig = .{},
    tolerances: ToleranceConfig = .{},
    iterations: IterationConfig = .{},
    envelope: envelope.ProductEnvelope = envelope.ProductEnvelope.product_default,
    features: FeatureFlags = .{},

    pub const default = SimulationConfig{};

    pub fn validate(self: SimulationConfig) ConfigError!void {
        const c = self.capacities;
        inline for (.{ c.body, c.collider, c.joint, c.command_per_tick, c.broad_pair, c.contact_patch, c.contact_point, c.sensor_overlap, c.event_per_tick, c.rollback_window, c.convex_hull_vertices, c.convex_hull_faces, c.compound_children, c.compound_depth, c.mesh_vertices, c.mesh_triangles, c.heightfield_axis_samples }) |value| if (value == 0) return error.InvalidCapacity;
        if (c.contact_point < c.contact_patch or c.convex_hull_faces < c.convex_hull_vertices or c.mesh_vertices > 16_777_215 or c.mesh_triangles > 16_777_215) return error.InvalidCapacity;
        const i = self.iterations;
        inline for (.{ i.tick_hz, i.substeps, i.velocity, i.position, i.sleep_ticks, i.gjk, i.epa, i.epa_max_faces, i.shape_cast, i.ccd_toi_per_substep, i.mesh_bvh_leaf_triangles }) |value| if (value == 0) return error.InvalidIteration;
        // Task 15 stores fixed solver loop counters as u8. Rejecting larger
        // values here keeps pipeline configuration validation total rather
        // than allowing a later narrowing cast to trap during a Tick.
        if (i.velocity > std.math.maxInt(u8) or i.position > std.math.maxInt(u8)) return error.InvalidIteration;
        const t = self.tolerances;
        inline for (.{ t.linear_slop, t.angular_slop, t.convex_skin, t.aabb_margin, t.max_position_correction, t.max_angular_correction, t.restitution_threshold, t.warmstart_normal_cos_min, t.sleep_linear_threshold, t.sleep_angular_threshold }) |value| if (value.raw < 0) return error.InvalidTolerance;
        if (t.warmstart_normal_cos_min.raw > fp.Fp.one.raw) return error.InvalidTolerance;
        self.envelope.validate() catch return error.InvalidEnvelope;
        if (self.features.reserved != 0) return error.InvalidFeatureFlags;
    }

    /// Calls `visitor.field(name, value)` in frozen canonical serialization order.
    /// Worker count is deliberately absent: it cannot affect simulation hashing.
    pub fn visitCanonical(self: SimulationConfig, visitor: anytype) void {
        const c = self.capacities;
        inline for (.{ .{ "body", c.body }, .{ "collider", c.collider }, .{ "joint", c.joint }, .{ "command_per_tick", c.command_per_tick }, .{ "broad_pair", c.broad_pair }, .{ "contact_patch", c.contact_patch }, .{ "contact_point", c.contact_point }, .{ "sensor_overlap", c.sensor_overlap }, .{ "event_per_tick", c.event_per_tick }, .{ "rollback_window", c.rollback_window }, .{ "convex_hull_vertices", c.convex_hull_vertices }, .{ "convex_hull_faces", c.convex_hull_faces }, .{ "compound_children", c.compound_children }, .{ "compound_depth", c.compound_depth }, .{ "mesh_vertices", c.mesh_vertices }, .{ "mesh_triangles", c.mesh_triangles }, .{ "heightfield_axis_samples", c.heightfield_axis_samples } }) |entry| visitor.field(entry[0], entry[1]);
        const t = self.tolerances;
        inline for (.{ .{ "linear_slop", t.linear_slop.raw }, .{ "angular_slop", t.angular_slop.raw }, .{ "convex_skin", t.convex_skin.raw }, .{ "aabb_margin", t.aabb_margin.raw }, .{ "max_position_correction", t.max_position_correction.raw }, .{ "max_angular_correction", t.max_angular_correction.raw }, .{ "restitution_threshold", t.restitution_threshold.raw }, .{ "warmstart_normal_cos_min", t.warmstart_normal_cos_min.raw }, .{ "sleep_linear_threshold", t.sleep_linear_threshold.raw }, .{ "sleep_angular_threshold", t.sleep_angular_threshold.raw } }) |entry| visitor.field(entry[0], entry[1]);
        const i = self.iterations;
        inline for (.{ .{ "tick_hz", i.tick_hz }, .{ "substeps", i.substeps }, .{ "velocity", i.velocity }, .{ "position", i.position }, .{ "sleep_ticks", i.sleep_ticks }, .{ "gjk", i.gjk }, .{ "epa", i.epa }, .{ "epa_max_faces", i.epa_max_faces }, .{ "shape_cast", i.shape_cast }, .{ "ccd_toi_per_substep", i.ccd_toi_per_substep }, .{ "mesh_bvh_leaf_triangles", i.mesh_bvh_leaf_triangles } }) |entry| visitor.field(entry[0], entry[1]);
        visitor.field("max_position", self.envelope.max_position.raw);
        visitor.field("max_linear_velocity", self.envelope.max_linear_velocity.raw);
        visitor.field("max_angular_velocity", self.envelope.max_angular_velocity.raw);
        visitor.field("max_dynamic_size", self.envelope.max_dynamic_size.raw);
        visitor.field("max_mass", self.envelope.max_mass.raw);
        visitor.field("features", @as(u32, @bitCast(self.features)));
    }
};
