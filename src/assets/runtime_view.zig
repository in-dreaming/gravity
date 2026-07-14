//! Checked, zero-copy view of a validated geometry asset.
const std = @import("std");
const baked = @import("../geometry/baked.zig");
const codec = @import("../state/codec.zig");
const store = @import("store.zig");
const geometry = @import("../math/geometry.zig");
const hash = @import("../state/hash.zig");

pub const Error = baked.Error || codec.Error || error{ MissingAsset, MissingSection };
pub const HeightDimensions = struct { width: u32, height: u32 };
pub const View = struct {
    asset: *const store.Asset,
    header: baked.Header,
    positions: ?[]const u8 = null,
    triangles: ?[]const u8 = null,
    nodes: ?[]const u8 = null,
    primitives: ?[]const u8 = null,
    heights: ?[]const u8 = null,
    cells: ?[]const u8 = null,
    children: ?[]const u8 = null,
    faces: ?[]const u8 = null,
    half_edges: ?[]const u8 = null,
    mass_bytes: ?[]const u8 = null,
    height_tile_tree: ?[]const u8 = null,

    pub fn vertexCount(self: View) usize {
        return count(self.positions);
    }
    pub fn vertex(self: View, index: usize) Error!geometry.Vec3 {
        const p = self.positions orelse return error.MissingSection;
        if (index >= count(p)) return error.EndOfInput;
        var r = codec.Reader.init(p[4 + index * 24 ..][0..24]);
        const v = try r.vec3();
        try r.finish();
        return v;
    }
    pub fn triangleCount(self: View) usize {
        return count(self.triangles);
    }
    pub fn triangle(self: View, index: usize) Error!baked.Triangle {
        const p = self.triangles orelse return error.MissingSection;
        if (index >= count(p)) return error.EndOfInput;
        var r = codec.Reader.init(p[4 + index * 12 ..][0..12]);
        const value = baked.Triangle{ .a = try r.unsigned(u32), .b = try r.unsigned(u32), .c = try r.unsigned(u32) };
        try r.finish();
        return value;
    }
    pub fn nodeCount(self: View) usize {
        return count(self.nodes);
    }
    pub fn node(self: View, index: usize) Error!baked.BvhNode {
        const p = self.nodes orelse return error.MissingSection;
        if (index >= count(p)) return error.EndOfInput;
        var r = codec.Reader.init(p[4 + index * 64 ..][0..64]);
        const value = baked.BvhNode{ .bounds = .{ .min = try r.vec3(), .max = try r.vec3() }, .first = try r.unsigned(u32), .count = try r.unsigned(u16), .axis = try r.byte(), .flags = try r.byte() };
        _ = try r.unsigned(u32);
        _ = try r.unsigned(u32);
        try r.finish();
        return value;
    }
    pub fn primitiveCount(self: View) usize {
        return count(self.primitives);
    }
    pub fn primitive(self: View, index: usize) Error!u32 {
        const p = self.primitives orelse return error.MissingSection;
        if (index >= count(p)) return error.EndOfInput;
        var r = codec.Reader.init(p[4 + index * 4 ..][0..4]);
        const value = try r.unsigned(u32);
        try r.finish();
        return value;
    }
    pub fn mass(self: View) Error!baked.MassProperties {
        const p = self.mass_bytes orelse return error.MissingSection;
        var r = codec.Reader.init(p);
        const value = baked.MassProperties{ .volume = try r.fpValue(), .center = try r.vec3(), .inertia = .{ .xx = try r.fpValue(), .yy = try r.fpValue(), .zz = try r.fpValue(), .xy = try r.fpValue(), .xz = try r.fpValue(), .yz = try r.fpValue() } };
        try r.finish();
        return value;
    }
    pub fn faceCount(self: View) usize {
        return count(self.faces);
    }
    pub fn face(self: View, index: usize) Error!baked.HullFace {
        const p = self.faces orelse return error.MissingSection;
        if (index >= count(p)) return error.EndOfInput;
        var r = codec.Reader.init(p[4 + index * 8 ..][0..8]);
        const value = baked.HullFace{ .first_half_edge = try r.unsigned(u32), .half_edge_count = try r.unsigned(u32) };
        try r.finish();
        return value;
    }
    pub fn halfEdgeCount(self: View) usize {
        return count(self.half_edges);
    }
    pub fn halfEdge(self: View, index: usize) Error!baked.HalfEdge {
        const p = self.half_edges orelse return error.MissingSection;
        if (index >= count(p)) return error.EndOfInput;
        var r = codec.Reader.init(p[4 + index * 16 ..][0..16]);
        const value = baked.HalfEdge{ .origin = try r.unsigned(u32), .twin = try r.unsigned(u32), .next = try r.unsigned(u32), .face = try r.unsigned(u32) };
        try r.finish();
        return value;
    }
    pub fn childCount(self: View) usize {
        return count(self.children);
    }
    pub fn child(self: View, index: usize) Error!baked.CompoundChild {
        const p = self.children orelse return error.MissingSection;
        if (index >= count(p)) return error.EndOfInput;
        var r = codec.Reader.init(p[4 + index * 92 ..][0..92]);
        const ordinal = try r.unsigned(u32);
        var content_hash: hash.Hash256 = undefined;
        for (&content_hash) |*byte| byte.* = try r.byte();
        const value = baked.CompoundChild{ .ordinal = ordinal, .content_hash = content_hash, .translation = try r.vec3(), .rotation = try r.quat() };
        try r.finish();
        return value;
    }
    pub fn heightDimensions(self: View) Error!HeightDimensions {
        const p = self.heights orelse return error.MissingSection;
        var r = codec.Reader.init(p[0..8]);
        const value: HeightDimensions = .{ .width = try r.unsigned(u32), .height = try r.unsigned(u32) };
        try r.finish();
        return value;
    }
    pub fn heightSample(self: View, index: usize) Error!geometry.Fp {
        const p = self.heights orelse return error.MissingSection;
        const dims = try self.heightDimensions();
        const total = @as(usize, dims.width) * dims.height;
        if (index >= total) return error.EndOfInput;
        var r = codec.Reader.init(p[8 + index * 8 ..][0..8]);
        const value = try r.fpValue();
        try r.finish();
        return value;
    }
    pub fn heightCellCount(self: View) usize {
        return count(self.cells);
    }
    pub fn heightCell(self: View, index: usize) Error!baked.HeightCell {
        const p = self.cells orelse return error.MissingSection;
        if (index >= count(p)) return error.EndOfInput;
        var r = codec.Reader.init(p[4 + index * 5 ..][0..5]);
        const value = baked.HeightCell{ .hole = try r.boolean(), .material_id = try r.unsigned(u32) };
        try r.finish();
        return value;
    }
    pub fn heightTileNodeCount(self: View) usize {
        const p = self.height_tile_tree orelse return 0;
        if (p.len < 12) return 0;
        return std.mem.readInt(u32, @ptrCast(p[8..12].ptr), .little);
    }
    pub fn heightTileNode(self: View, index: usize) Error!baked.BvhNode {
        const p = self.height_tile_tree orelse return error.MissingSection;
        if (p.len < 12) return error.EndOfInput;
        const count_value = std.mem.readInt(u32, @ptrCast(p[8..12].ptr), .little);
        if (index >= @as(usize, count_value)) return error.EndOfInput;
        var r = codec.Reader.init(p[12 + index * 64 ..][0..64]);
        const value = baked.BvhNode{ .bounds = .{ .min = try r.vec3(), .max = try r.vec3() }, .first = try r.unsigned(u32), .count = try r.unsigned(u16), .axis = try r.byte(), .flags = try r.byte() };
        _ = try r.unsigned(u32);
        _ = try r.unsigned(u32);
        try r.finish();
        return value;
    }

    pub fn init(asset: *const store.Asset) Error!View {
        const header = try baked.validateEncoded(asset.bytes);
        var view = View{ .asset = asset, .header = header };
        const Scan = struct {
            fn run(out: *View, section: codec.Section) codec.Error!void {
                switch (section.id) {
                    @intFromEnum(baked.Tag.positions) => out.positions = section.payload,
                    @intFromEnum(baked.Tag.triangles) => out.triangles = section.payload,
                    @intFromEnum(baked.Tag.bvh_nodes) => out.nodes = section.payload,
                    @intFromEnum(baked.Tag.bvh_primitives) => out.primitives = section.payload,
                    @intFromEnum(baked.Tag.height_samples) => out.heights = section.payload,
                    @intFromEnum(baked.Tag.height_cells) => out.cells = section.payload,
                    @intFromEnum(baked.Tag.compound_children) => out.children = section.payload,
                    @intFromEnum(baked.Tag.hull_faces) => out.faces = section.payload,
                    @intFromEnum(baked.Tag.half_edges) => out.half_edges = section.payload,
                    @intFromEnum(baked.Tag.mass_properties) => out.mass_bytes = section.payload,
                    @intFromEnum(baked.Tag.height_tile_tree) => out.height_tile_tree = section.payload,
                    else => {},
                }
            }
        };
        var reader = codec.Reader.init(asset.bytes);
        try codec.readSections(&reader, baked.format_version, View, &view, Scan.run);
        return view;
    }
};
fn count(payload: ?[]const u8) usize {
    const p = payload orelse return 0;
    return if (p.len < 4) 0 else @intCast(std.mem.readInt(u32, @ptrCast(p.ptr), .little));
}

pub fn find(input: *const store.Store, source_id: u64) Error!View {
    return View.init(input.find(source_id) orelse return error.MissingAsset);
}
