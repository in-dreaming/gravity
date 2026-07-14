//! Deterministic, allocation-free validation and canonical wire encoding for
//! the Task 06 geometry records consumed by Task 05's asset store.
const std = @import("std");
const codec = @import("../state/codec.zig");
const hash = @import("../state/hash.zig");
const geometry = @import("../math/geometry.zig");
const fp = @import("../math/fp.zig");

// Version 2 makes the four bytes of BvhNode trailing padding explicit zeroes.
// Version 1 declared 64-byte nodes but serialized only 60 bytes per record.
pub const format_version: u16 = 2;
pub const Tag = enum(u16) {
    header = 0x8001,
    positions = 0x8002,
    triangles = 0x8003,
    bvh_nodes = 0x8004,
    bvh_primitives = 0x8005,
    height_samples = 0x8006,
    height_cells = 0x8007,
    compound_children = 0x8008,
    hull_faces = 0x8009,
    half_edges = 0x800a,
    mass_properties = 0x800b,
    height_tile_tree = 0x800c,
};
pub const Kind = enum(u8) { convex_hull = 1, triangle_mesh = 2, height_field = 3, compound = 4 };
pub const Error = codec.Error || error{ InvalidKind, InvalidHeader, InvalidIndex, DegenerateTriangle, InvalidBvh, InvalidTopology, InvalidMass, OutOfScratch };

pub const Triangle = struct { a: u32, b: u32, c: u32 };
pub const BvhNode = struct {
    bounds: geometry.Aabb3,
    first: u32,
    count: u16,
    axis: u8,
    flags: u8,
    pub const leaf_flag: u8 = 1;
    pub fn leaf(bounds: geometry.Aabb3, first: u32, count: u16) BvhNode {
        return .{ .bounds = bounds, .first = first, .count = count, .axis = 0, .flags = leaf_flag };
    }
};
pub const Mesh = struct { source_id: u64, vertices: []const geometry.Vec3, triangles: []const Triangle, nodes: []const BvhNode, primitives: []const u32 };
pub const Header = struct { kind: Kind, source_id: u64 };
pub const HalfEdge = struct { origin: u32, twin: u32, next: u32, face: u32 };
pub const HullFace = struct { first_half_edge: u32, half_edge_count: u32 };
pub const MassProperties = struct {
    volume: fp.Fp,
    center: geometry.Vec3,
    inertia: geometry.SymmetricMat3,
};
pub const ConvexHull = struct {
    vertices: []const geometry.Vec3,
    triangles: []const Triangle,
    faces: []const HullFace,
    half_edges: []const HalfEdge,
    mass: MassProperties,
};
pub const CompoundChild = struct {
    ordinal: u32,
    content_hash: hash.Hash256,
    translation: geometry.Vec3,
    rotation: geometry.Quat,
};
pub const Compound = struct {
    source_id: u64,
    children: []const CompoundChild,
    nodes: []const BvhNode,
};

/// Builds a deterministic convex hull in caller-provided storage.
///
/// Input points are lexicographically canonicalised and duplicates removed.
/// Every supporting plane is emitted once: all its points are projected by
/// dropping the dominant normal axis, a monotonic 2D hull removes interior
/// coplanar points, and its polygon is fan-triangulated from the canonical
/// first ring vertex.  Faces retain the polygon ring in the half-edge form;
/// `triangles` is therefore only the canonical mass/BVH triangulation, not a
/// one-face-per-triangle view.  This makes cubes and other legal coplanar
/// facets stable without assigning accidental diagonals to the topology.
pub fn buildConvexHull(
    points: []const geometry.Vec3,
    vertices_out: []geometry.Vec3,
    triangles_out: []Triangle,
    faces_out: []HullFace,
    edges_out: []HalfEdge,
    status: *fp.MathStatus,
) Error!ConvexHull {
    if (points.len < 4 or points.len > vertices_out.len or points.len > 256) return error.OutOfScratch;
    for (points, 0..) |point, i| vertices_out[i] = point;
    insertionSortVertices(vertices_out[0..points.len]);
    var vertex_count: usize = 0;
    for (vertices_out[0..points.len]) |point| {
        if (vertex_count == 0 or !sameVertex(vertices_out[vertex_count - 1], point)) {
            vertices_out[vertex_count] = point;
            vertex_count += 1;
        }
    }
    if (vertex_count < 4) return error.InvalidTopology;
    var face_count: usize = 0;
    var triangle_count: usize = 0;
    var edge_count: usize = 0;
    for (0..vertex_count) |a| for (a + 1..vertex_count) |b| for (b + 1..vertex_count) |c| {
        const ab = vertices_out[b].sub(vertices_out[a], status);
        const ac = vertices_out[c].sub(vertices_out[a], status);
        const normal = ab.cross(ac, status);
        if (normal.x.raw == 0 and normal.y.raw == 0 and normal.z.raw == 0) continue;
        var positive = false;
        var negative = false;
        var plane_points: [256]u32 = undefined;
        var plane_count: usize = 0;
        for (vertices_out[0..vertex_count], 0..) |point, pi| {
            const side = normal.dot(point.sub(vertices_out[a], status), status).raw;
            if (side > 0) positive = true else if (side < 0) negative = true;
            if (side == 0) {
                plane_points[plane_count] = @intCast(pi);
                plane_count += 1;
            }
        }
        if (positive and negative) continue;
        var ring: [256]u32 = undefined;
        const ring_count = planarHull(vertices_out[0..vertex_count], plane_points[0..plane_count], normal, &ring);
        if (ring_count < 3) continue;
        // Exactly one triple in the lexicographic enumeration identifies the
        // face: its first non-collinear triple among all plane points.
        const representative = firstPlaneTriple(vertices_out[0..vertex_count], plane_points[0..plane_count]);
        if (a != representative[0] or b != representative[1] or c != representative[2]) continue;
        if (positive) reverseU32(ring[0..ring_count]);
        if (face_count >= faces_out.len or edge_count + ring_count > edges_out.len or triangle_count + ring_count - 2 > triangles_out.len) return error.OutOfScratch;
        faces_out[face_count] = .{ .first_half_edge = @intCast(edge_count), .half_edge_count = @intCast(ring_count) };
        for (ring[0..ring_count], 0..) |origin, i| {
            edges_out[edge_count + i] = .{ .origin = origin, .twin = std.math.maxInt(u32), .next = @intCast(edge_count + (i + 1) % ring_count), .face = @intCast(face_count) };
        }
        for (1..ring_count - 1) |i| {
            triangles_out[triangle_count] = .{ .a = ring[0], .b = ring[i], .c = ring[i + 1] };
            triangle_count += 1;
        }
        edge_count += ring_count;
        face_count += 1;
    };
    if (face_count < 4) return error.InvalidTopology;
    for (edges_out[0..edge_count]) |*edge| {
        const destination = edges_out[edge.next].origin;
        var twin: ?usize = null;
        for (edges_out[0..edge_count], 0..) |other, oi| if (other.origin == destination and edges_out[other.next].origin == edge.origin) {
            if (twin != null) return error.InvalidTopology;
            twin = oi;
        };
        edge.twin = @intCast(twin orelse return error.InvalidTopology);
    }
    const verts = vertices_out[0..vertex_count];
    const triangles = triangles_out[0..triangle_count];
    const faces = faces_out[0..face_count];
    const edges = edges_out[0..edge_count];
    try validateHullTopology(verts, faces, edges);
    return .{ .vertices = verts, .triangles = triangles, .faces = faces, .half_edges = edges, .mass = try meshMassProperties(verts, triangles, status) };
}

fn reverseU32(values: []u32) void {
    var left: usize = 0;
    var right = values.len;
    while (left < right) {
        right -= 1;
        if (left >= right) break;
        const value = values[left];
        values[left] = values[right];
        values[right] = value;
        left += 1;
    }
}

fn firstPlaneTriple(vertices: []const geometry.Vec3, points: []const u32) [3]usize {
    const a: usize = points[0];
    var status = fp.MathStatus{};
    var b_at: usize = 1;
    while (b_at < points.len) : (b_at += 1) {
        const b: usize = points[b_at];
        var c_at = b_at + 1;
        while (c_at < points.len) : (c_at += 1) {
            const c: usize = points[c_at];
            const normal = vertices[b].sub(vertices[a], &status).cross(vertices[c].sub(vertices[a], &status), &status);
            if (normal.x.raw != 0 or normal.y.raw != 0 or normal.z.raw != 0) return .{ a, b, c };
        }
    }
    return .{ a, a, a };
}

fn planarHull(vertices: []const geometry.Vec3, candidates: []const u32, normal: geometry.Vec3, out: *[256]u32) usize {
    var sorted: [256]u32 = undefined;
    for (candidates, 0..) |value, i| sorted[i] = value;
    const axis = dominantNormalAxis(normal);
    var i: usize = 1;
    while (i < candidates.len) : (i += 1) {
        const value = sorted[i];
        var j = i;
        while (j > 0 and projectedLess(vertices[value], vertices[sorted[j - 1]], axis, value, sorted[j - 1])) : (j -= 1) sorted[j] = sorted[j - 1];
        sorted[j] = value;
    }
    var count: usize = 0;
    for (sorted[0..candidates.len]) |value| {
        while (count >= 2 and projectedTurn(vertices[out[count - 2]], vertices[out[count - 1]], vertices[value], axis) <= 0) count -= 1;
        out[count] = value;
        count += 1;
    }
    const lower = count;
    var k = candidates.len;
    while (k > 0) {
        k -= 1;
        const value = sorted[k];
        while (count > lower and projectedTurn(vertices[out[count - 2]], vertices[out[count - 1]], vertices[value], axis) <= 0) count -= 1;
        out[count] = value;
        count += 1;
    }
    // The final entry closes the ring and is deliberately not stored.
    count -= 1;
    if (normalComponent(normal, axis) < 0) reverseU32(out[0..count]);
    return count;
}

fn dominantNormalAxis(normal: geometry.Vec3) u8 {
    const x = absRaw(normal.x.raw);
    const y = absRaw(normal.y.raw);
    const z = absRaw(normal.z.raw);
    return if (y > x and y >= z) 1 else if (z > x and z > y) 2 else 0;
}
fn absRaw(value: i64) i128 {
    return if (value < 0) -@as(i128, value) else value;
}
fn normalComponent(normal: geometry.Vec3, axis: u8) i64 {
    return switch (axis) {
        0 => normal.x.raw,
        1 => normal.y.raw,
        else => normal.z.raw,
    };
}
fn projectedLess(a: geometry.Vec3, b: geometry.Vec3, axis: u8, ai: u32, bi: u32) bool {
    const au, const av = projected(a, axis);
    const bu, const bv = projected(b, axis);
    return au < bu or (au == bu and (av < bv or (av == bv and ai < bi)));
}
fn projected(value: geometry.Vec3, axis: u8) struct { i64, i64 } {
    return switch (axis) {
        0 => .{ value.y.raw, value.z.raw },
        1 => .{ value.z.raw, value.x.raw },
        else => .{ value.x.raw, value.y.raw },
    };
}
fn projectedTurn(a: geometry.Vec3, b: geometry.Vec3, c: geometry.Vec3, axis: u8) i128 {
    const au, const av = projected(a, axis);
    const bu, const bv = projected(b, axis);
    const cu, const cv = projected(c, axis);
    return (@as(i128, bu) - au) * (@as(i128, cv) - av) - (@as(i128, bv) - av) * (@as(i128, cu) - au);
}

fn sameVertex(a: geometry.Vec3, b: geometry.Vec3) bool {
    return a.x.raw == b.x.raw and a.y.raw == b.y.raw and a.z.raw == b.z.raw;
}
fn insertionSortVertices(values: []geometry.Vec3) void {
    var i: usize = 1;
    while (i < values.len) : (i += 1) {
        const value = values[i];
        var j = i;
        while (j > 0 and vertexLess(value, values[j - 1])) : (j -= 1) values[j] = values[j - 1];
        values[j] = value;
    }
}
fn vertexLess(a: geometry.Vec3, b: geometry.Vec3) bool {
    return a.x.raw < b.x.raw or (a.x.raw == b.x.raw and (a.y.raw < b.y.raw or (a.y.raw == b.y.raw and a.z.raw < b.z.raw)));
}

/// Topological facts calculated from the frozen canonical triangle order.
/// `closed_manifold` is deliberately false for an empty mesh and for every
/// boundary, duplicate-directed, or non-manifold edge.
/// Validates a complete frozen geometry TLV without allocating or repairing it.
/// The returned header borrows nothing and is suitable for AssetStore indexing.
pub fn validateEncoded(bytes: []const u8) Error!Header {
    const known = [_]u16{ @intFromEnum(Tag.header), @intFromEnum(Tag.positions), @intFromEnum(Tag.triangles), @intFromEnum(Tag.bvh_nodes), @intFromEnum(Tag.bvh_primitives), @intFromEnum(Tag.height_samples), @intFromEnum(Tag.height_cells), @intFromEnum(Tag.compound_children), @intFromEnum(Tag.hull_faces), @intFromEnum(Tag.half_edges), @intFromEnum(Tag.mass_properties), @intFromEnum(Tag.height_tile_tree) };
    const Context = struct {
        header: ?Header = null,
        seen: u16 = 0,
        vertices: usize = 0,
        triangles: usize = 0,
        nodes: usize = 0,
        primitives: usize = 0,
        faces: usize = 0,
        edges: usize = 0,
        height_width: u32 = 0,
        height_height: u32 = 0,
        height_cells: usize = 0,
        height_tiles_x: u32 = 0,
        height_tiles_z: u32 = 0,
        height_tile_nodes: usize = 0,
    };
    const Visit = struct {
        fn run(ctx: *Context, section: codec.Section) codec.Error!void {
            const bit: u16 = switch (section.id) {
                @intFromEnum(Tag.header) => 1,
                @intFromEnum(Tag.positions) => 2,
                @intFromEnum(Tag.triangles) => 4,
                @intFromEnum(Tag.bvh_nodes) => 8,
                @intFromEnum(Tag.bvh_primitives) => 16,
                @intFromEnum(Tag.height_samples) => 32,
                @intFromEnum(Tag.height_cells) => 64,
                @intFromEnum(Tag.compound_children) => 128,
                @intFromEnum(Tag.hull_faces) => 256,
                @intFromEnum(Tag.half_edges) => 512,
                @intFromEnum(Tag.mass_properties) => 1024,
                @intFromEnum(Tag.height_tile_tree) => 2048,
                else => return,
            };
            ctx.seen |= bit;
            var r = codec.Reader.init(section.payload);
            switch (section.id) {
                @intFromEnum(Tag.header) => {
                    if (section.payload.len != 16) return error.EndOfInput;
                    const kind_raw = try r.byte();
                    const kind = std.enums.fromInt(Kind, kind_raw) orelse return error.InvalidEnum;
                    if (try r.byte() != 1 or try r.unsigned(u16) != 0) return error.InvalidEnum;
                    const source_id = try r.unsigned(u64);
                    if (try r.unsigned(u32) != 0) return error.InvalidEnum;
                    ctx.header = .{ .kind = kind, .source_id = source_id };
                },
                @intFromEnum(Tag.positions) => {
                    const count = try r.unsigned(u32);
                    if (section.payload.len != 4 + @as(usize, count) * 24) return error.EndOfInput;
                    ctx.vertices = count;
                    r.at = r.bytes.len;
                },
                @intFromEnum(Tag.triangles) => {
                    const count = try r.unsigned(u32);
                    if (section.payload.len != 4 + @as(usize, count) * 12) return error.EndOfInput;
                    var i: usize = 0;
                    while (i < count) : (i += 1) {
                        const a = try r.unsigned(u32);
                        const b = try r.unsigned(u32);
                        const c = try r.unsigned(u32);
                        if (a >= ctx.vertices or b >= ctx.vertices or c >= ctx.vertices or a == b or a == c or b == c) return error.InvalidEnum;
                    }
                    ctx.triangles = count;
                },
                @intFromEnum(Tag.bvh_nodes) => {
                    const count = try r.unsigned(u32);
                    if (section.payload.len != 4 + @as(usize, count) * 64) return error.EndOfInput;
                    ctx.nodes = count;
                    r.at = r.bytes.len;
                },
                @intFromEnum(Tag.bvh_primitives) => {
                    const count = try r.unsigned(u32);
                    if (section.payload.len != 4 + @as(usize, count) * 4) return error.EndOfInput;
                    var i: usize = 0;
                    while (i < count) : (i += 1) {
                        const primitive = try r.unsigned(u32);
                        if (primitive >= ctx.triangles) return error.InvalidEnum;
                        var prior_at: usize = 4;
                        while (prior_at < 4 + i * 4) : (prior_at += 4) {
                            if (std.mem.readInt(u32, @ptrCast(section.payload[prior_at..].ptr), .little) == primitive) return error.InvalidEnum;
                        }
                    }
                    ctx.primitives = count;
                },
                @intFromEnum(Tag.height_samples) => {
                    const width = try r.unsigned(u32);
                    const height = try r.unsigned(u32);
                    const samples = std.math.mul(usize, width, height) catch return error.LengthOverflow;
                    if (width == 0 or height == 0 or section.payload.len != 8 + samples * 8) return error.EndOfInput;
                    ctx.height_width = width;
                    ctx.height_height = height;
                    try r.skip(samples * 8);
                },
                @intFromEnum(Tag.height_cells) => {
                    const count = try r.unsigned(u32);
                    if (section.payload.len != 4 + @as(usize, count) * 5) return error.EndOfInput;
                    var i: usize = 0;
                    while (i < count) : (i += 1) {
                        if ((try r.byte() & ~@as(u8, 1)) != 0) return error.InvalidEnum;
                        _ = try r.unsigned(u32);
                    }
                    ctx.height_cells = count;
                },
                @intFromEnum(Tag.compound_children) => {
                    const count = try r.unsigned(u32);
                    if (count > 256 or section.payload.len != 4 + @as(usize, count) * 92) return error.EndOfInput;
                    var i: u32 = 0;
                    while (i < count) : (i += 1) {
                        if (try r.unsigned(u32) != i) return error.InvalidEnum;
                        try r.skip(32 + 24 + 32);
                    }
                },
                @intFromEnum(Tag.hull_faces) => {
                    const count = try r.unsigned(u32);
                    if (section.payload.len != 4 + @as(usize, count) * 8) return error.EndOfInput;
                    ctx.faces = count;
                    try r.skip(@as(usize, count) * 8);
                },
                @intFromEnum(Tag.half_edges) => {
                    const count = try r.unsigned(u32);
                    if (section.payload.len != 4 + @as(usize, count) * 16) return error.EndOfInput;
                    ctx.edges = count;
                    try r.skip(@as(usize, count) * 16);
                },
                @intFromEnum(Tag.mass_properties) => {
                    if (section.payload.len != 80) return error.EndOfInput;
                    if ((try r.signed(i64)) <= 0) return error.InvalidEnum;
                    try r.skip(72);
                },
                @intFromEnum(Tag.height_tile_tree) => {
                    ctx.height_tiles_x = try r.unsigned(u32);
                    ctx.height_tiles_z = try r.unsigned(u32);
                    const count = try r.unsigned(u32);
                    if (section.payload.len != 12 + @as(usize, count) * 64) return error.EndOfInput;
                    ctx.height_tile_nodes = count;
                    try r.skip(@as(usize, count) * 64);
                },
                else => {},
            }
            try r.finish();
        }
    };
    var context = Context{};
    var reader = codec.Reader.init(bytes);
    try codec.readKnownSections(&reader, format_version, &known, Context, &context, Visit.run);
    const header = context.header orelse return error.InvalidHeader;
    switch (header.kind) {
        .triangle_mesh => if ((context.seen & @as(u16, 31)) != 31 or (context.seen & ~@as(u16, 31 | 1024)) != 0 or context.vertices == 0 or context.triangles == 0) return error.InvalidHeader,
        .convex_hull => if ((context.seen & (1 | 2 | 4 | 256 | 512 | 1024)) != (1 | 2 | 4 | 256 | 512 | 1024) or context.faces == 0 or context.edges == 0) return error.InvalidHeader,
        .height_field => {
            if ((context.seen & (1 | 32 | 64 | 2048)) != (1 | 32 | 64 | 2048) or context.height_width < 2 or context.height_height < 2) return error.InvalidHeader;
            const cells = std.math.mul(usize, context.height_width - 1, context.height_height - 1) catch return error.InvalidHeader;
            const tiles_x = (context.height_width - 1 + heightfield_tile_axis - 1) / heightfield_tile_axis;
            const tiles_z = (context.height_height - 1 + heightfield_tile_axis - 1) / heightfield_tile_axis;
            const tiles = std.math.mul(usize, tiles_x, tiles_z) catch return error.InvalidHeader;
            const tree_nodes = std.math.sub(usize, std.math.mul(usize, tiles, 2) catch return error.InvalidHeader, 1) catch return error.InvalidHeader;
            if (context.height_cells != cells or context.height_tiles_x != tiles_x or context.height_tiles_z != tiles_z or context.height_tile_nodes != tree_nodes) return error.InvalidHeader;
        },
        .compound => if ((context.seen & (1 | 8 | 128)) != (1 | 8 | 128)) return error.InvalidHeader,
    }
    return header;
}

/// Returns a Compound child content hash by canonical ordinal. The source is
/// already fully validated before this function observes the payload.
pub fn compoundChildHash(bytes: []const u8, ordinal: u32) Error!?hash.Hash256 {
    if ((try validateEncoded(bytes)).kind != .compound) return null;
    var r = codec.Reader.init(bytes);
    _ = try r.unsigned(u16);
    const sections = try r.unsigned(u16);
    var n: u16 = 0;
    while (n < sections) : (n += 1) {
        const tag = try r.unsigned(u16);
        const length = try r.unsigned(u32);
        if (tag != @intFromEnum(Tag.compound_children)) {
            try r.skip(length);
            continue;
        }
        const count = try r.unsigned(u32);
        if (ordinal >= count) return null;
        try r.skip(@as(usize, ordinal) * 92);
        if (try r.unsigned(u32) != ordinal) return error.InvalidIndex;
        var result: hash.Hash256 = undefined;
        for (&result) |*byte| byte.* = try r.byte();
        return result;
    }
    return null;
}

pub fn validateMesh(mesh: Mesh) Error!void {
    if (mesh.vertices.len == 0 or mesh.triangles.len == 0) return error.DegenerateTriangle;
    for (mesh.triangles) |triangle| {
        if (triangle.a >= mesh.vertices.len or triangle.b >= mesh.vertices.len or triangle.c >= mesh.vertices.len) return error.InvalidIndex;
        if (triangle.a == triangle.b or triangle.a == triangle.c or triangle.b == triangle.c) return error.DegenerateTriangle;
    }
    try validateBvh(mesh.nodes, mesh.primitives, mesh.triangles.len);
}

/// Validates the immutable half-edge representation of a closed convex hull.
/// It deliberately performs no topology repair: every index and reciprocal
/// relation is part of the hashed asset contract.
pub fn validateHullTopology(vertices: []const geometry.Vec3, faces: []const HullFace, edges: []const HalfEdge) Error!void {
    if (vertices.len < 4 or faces.len < 4 or edges.len < 12) return error.InvalidTopology;
    for (faces, 0..) |face, face_index| {
        if (face.half_edge_count < 3) return error.InvalidTopology;
        const end = std.math.add(usize, face.first_half_edge, face.half_edge_count) catch return error.InvalidTopology;
        if (end > edges.len) return error.InvalidTopology;
        var current = face.first_half_edge;
        var n: u32 = 0;
        while (n < face.half_edge_count) : (n += 1) {
            const edge = edges[current];
            if (edge.face != face_index or edge.origin >= vertices.len or edge.twin >= edges.len or edge.next >= edges.len) return error.InvalidTopology;
            if (edges[edge.twin].twin != current) return error.InvalidTopology;
            current = edge.next;
        }
        if (current != face.first_half_edge) return error.InvalidTopology;
    }
    for (edges) |edge| if (edge.face >= faces.len) return error.InvalidTopology;
}

/// Validates raw HeightField storage before deterministic tile construction.
/// A HeightField needs at least one cell in both dimensions; hole flags have
/// no hidden semantics and material IDs are opaque stable integers.
pub fn validateHeightField(width: u32, height: u32, samples: []const fp.Fp, cells: []const HeightCell) Error!void {
    if (width < 2 or height < 2) return error.InvalidTopology;
    const sample_count = std.math.mul(usize, width, height) catch return error.InvalidTopology;
    const cell_count = std.math.mul(usize, width - 1, height - 1) catch return error.InvalidTopology;
    if (samples.len != sample_count or cells.len != cell_count) return error.InvalidTopology;
}

pub const HeightCell = struct { hole: bool = false, material_id: u32 = 0 };
pub const HeightField = struct {
    source_id: u64,
    width: u32,
    height: u32,
    samples: []const fp.Fp,
    cells: []const HeightCell,
    tile_nodes: []const BvhNode,
};
pub const heightfield_tile_axis: u32 = 16;

/// Produces one min/max leaf per 16x16-cell tile in row-major tile order.
/// Holes do not remove samples from a tile range, which keeps boundary
/// traversal conservative and independent of query direction.
pub fn buildHeightFieldTiles(width: u32, height: u32, samples: []const fp.Fp, cells: []const HeightCell, out: []BvhNode) Error![]const BvhNode {
    try validateHeightField(width, height, samples, cells);
    const cells_x = width - 1;
    const cells_z = height - 1;
    const tiles_x = (cells_x + heightfield_tile_axis - 1) / heightfield_tile_axis;
    const tiles_z = (cells_z + heightfield_tile_axis - 1) / heightfield_tile_axis;
    const tile_count = std.math.mul(usize, tiles_x, tiles_z) catch return error.OutOfScratch;
    const needed = std.math.sub(usize, std.math.mul(usize, tile_count, 2) catch return error.OutOfScratch, 1) catch return error.OutOfScratch;
    if (out.len < needed) return error.OutOfScratch;
    var cursor: usize = 1;
    try buildHeightTileNode(width, height, samples, tiles_x, 0, tile_count, out, 0, &cursor);
    return out[0..cursor];
}
fn buildHeightTileNode(width: u32, height: u32, samples: []const fp.Fp, tiles_x: u32, first_tile: usize, count: usize, nodes: []BvhNode, at: usize, cursor: *usize) Error!void {
    if (count == 0 or at >= nodes.len) return error.OutOfScratch;
    if (count == 1) {
        nodes[at] = BvhNode.leaf(heightTileBounds(width, height, samples, tiles_x, @intCast(first_tile)), @intCast(first_tile), 1);
        return;
    }
    const mid = count / 2;
    const left = cursor.*;
    const right = left + 1;
    if (right >= nodes.len) return error.OutOfScratch;
    cursor.* = right + 1;
    try buildHeightTileNode(width, height, samples, tiles_x, first_tile, mid, nodes, left, cursor);
    try buildHeightTileNode(width, height, samples, tiles_x, first_tile + mid, count - mid, nodes, right, cursor);
    const a = nodes[left].bounds;
    const b = nodes[right].bounds;
    nodes[at] = .{ .bounds = unionBounds(a, b), .first = @intCast(left), .count = 0, .axis = 0, .flags = 0 };
}
fn heightTileBounds(width: u32, height: u32, samples: []const fp.Fp, tiles_x: u32, tile: u32) geometry.Aabb3 {
    const x0 = (tile % tiles_x) * heightfield_tile_axis;
    const z0 = (tile / tiles_x) * heightfield_tile_axis;
    const x1 = @min(x0 + heightfield_tile_axis, width - 1);
    const z1 = @min(z0 + heightfield_tile_axis, height - 1);
    var min_y = samples[@as(usize, z0) * width + x0];
    var max_y = min_y;
    var z = z0;
    while (z <= z1) : (z += 1) {
        var x = x0;
        while (x <= x1) : (x += 1) {
            const value = samples[@as(usize, z) * width + x];
            min_y.raw = @min(min_y.raw, value.raw);
            max_y.raw = @max(max_y.raw, value.raw);
        }
    }
    return .{ .min = .{ .x = fp.Fp.fromInt(@intCast(x0)), .y = min_y, .z = fp.Fp.fromInt(@intCast(z0)) }, .max = .{ .x = fp.Fp.fromInt(@intCast(x1)), .y = max_y, .z = fp.Fp.fromInt(@intCast(z1)) } };
}
fn unionBounds(a: geometry.Aabb3, b: geometry.Aabb3) geometry.Aabb3 {
    return .{ .min = .{ .x = .{ .raw = @min(a.min.x.raw, b.min.x.raw) }, .y = .{ .raw = @min(a.min.y.raw, b.min.y.raw) }, .z = .{ .raw = @min(a.min.z.raw, b.min.z.raw) } }, .max = .{ .x = .{ .raw = @max(a.max.x.raw, b.max.x.raw) }, .y = .{ .raw = @max(a.max.y.raw, b.max.y.raw) }, .z = .{ .raw = @max(a.max.z.raw, b.max.z.raw) } } };
}
pub const CompoundChildBounds = struct { ordinal: u32, bounds: geometry.Aabb3 };

/// Builds a deterministic Compound BVH. Leaves contain at most four direct
/// children; inner nodes use the same adjacent-child convention as mesh BVHs.
pub fn buildCompoundBvh(children: []const CompoundChildBounds, nodes: []BvhNode, primitives: []u32) Error!BvhBuildResult {
    if (children.len == 0 or children.len > 256 or nodes.len < children.len * 2 - 1 or primitives.len < children.len) return error.OutOfScratch;
    for (children, 0..) |child, i| {
        if (child.ordinal != i) return error.InvalidIndex;
        primitives[i] = child.ordinal;
    }
    var cursor: usize = 1;
    try buildCompoundBvhNode(children, nodes, primitives[0..children.len], primitives.ptr, 0, &cursor);
    return .{ .nodes = nodes[0..cursor], .primitives = primitives[0..children.len] };
}

fn buildCompoundBvhNode(children: []const CompoundChildBounds, nodes: []BvhNode, ids: []u32, root: [*]u32, at: usize, cursor: *usize) Error!void {
    const bounds = compoundBounds(children, ids);
    if (ids.len <= 4) {
        const first: usize = (@intFromPtr(ids.ptr) - @intFromPtr(root)) / @sizeOf(u32);
        nodes[at] = BvhNode.leaf(bounds, @intCast(first), @intCast(ids.len));
        return;
    }
    const choice = chooseCompoundSahSplit(children, ids);
    insertionSortCompoundIds(children, ids, choice.axis);
    const mid = choice.split;
    const left = cursor.*;
    const right = left + 1;
    if (right >= nodes.len) return error.OutOfScratch;
    cursor.* = right + 1;
    nodes[at] = .{ .bounds = bounds, .first = @intCast(left), .count = 0, .axis = choice.axis, .flags = 0 };
    try buildCompoundBvhNode(children, nodes, ids[0..mid], root, left, cursor);
    try buildCompoundBvhNode(children, nodes, ids[mid..], root, right, cursor);
}

fn chooseCompoundSahSplit(children: []const CompoundChildBounds, ids: []u32) SahSplit {
    var best_axis: u8 = 0;
    var best_split: usize = 1;
    var best_cost: ?i256 = null;
    var best_primitive: u32 = std.math.maxInt(u32);
    var axis: u8 = 0;
    while (axis < 3) : (axis += 1) {
        insertionSortCompoundIds(children, ids, axis);
        var split: usize = 1;
        while (split < ids.len) : (split += 1) {
            const left = compoundBounds(children, ids[0..split]);
            const right = compoundBounds(children, ids[split..]);
            const cost = sahArea(left) * @as(i256, @intCast(split)) + sahArea(right) * @as(i256, @intCast(ids.len - split));
            const primitive = ids[split];
            if (best_cost == null or cost < best_cost.? or (cost == best_cost.? and (axis < best_axis or (axis == best_axis and (split < best_split or (split == best_split and primitive < best_primitive)))))) {
                best_cost = cost;
                best_axis = axis;
                best_split = split;
                best_primitive = primitive;
            }
        }
    }
    return .{ .axis = best_axis, .split = best_split };
}

fn compoundBounds(children: []const CompoundChildBounds, ids: []const u32) geometry.Aabb3 {
    var result = children[ids[0]].bounds;
    for (ids[1..]) |id| {
        const bounds = children[id].bounds;
        result.min.x.raw = @min(result.min.x.raw, bounds.min.x.raw);
        result.min.y.raw = @min(result.min.y.raw, bounds.min.y.raw);
        result.min.z.raw = @min(result.min.z.raw, bounds.min.z.raw);
        result.max.x.raw = @max(result.max.x.raw, bounds.max.x.raw);
        result.max.y.raw = @max(result.max.y.raw, bounds.max.y.raw);
        result.max.z.raw = @max(result.max.z.raw, bounds.max.z.raw);
    }
    return result;
}

fn compoundCentroidKey(child: CompoundChildBounds, axis: u8) i128 {
    return switch (axis) {
        0 => @as(i128, child.bounds.min.x.raw) + child.bounds.max.x.raw,
        1 => @as(i128, child.bounds.min.y.raw) + child.bounds.max.y.raw,
        else => @as(i128, child.bounds.min.z.raw) + child.bounds.max.z.raw,
    };
}

fn insertionSortCompoundIds(children: []const CompoundChildBounds, ids: []u32, axis: u8) void {
    var i: usize = 1;
    while (i < ids.len) : (i += 1) {
        const value = ids[i];
        const key = compoundCentroidKey(children[value], axis);
        var j = i;
        while (j > 0) {
            const prior = ids[j - 1];
            const prior_key = compoundCentroidKey(children[prior], axis);
            if (prior_key < key or (prior_key == key and prior < value)) break;
            ids[j] = prior;
            j -= 1;
        }
        ids[j] = value;
    }
}

/// Validates the immutable HeightField payload and its non-empty min/max tree.
pub fn validateHeightFieldAsset(value: HeightField) Error!void {
    try validateHeightField(value.width, value.height, value.samples, value.cells);
    if (value.tile_nodes.len == 0) return error.InvalidBvh;
}
pub const MeshAdjacency = struct { triangle: u32 = std.math.maxInt(u32), edge: u8 = 0 };
pub const weld_epsilon_raw: i64 = 1;
pub const VertexWeldRef = struct { point: geometry.Vec3, source_index: u32 };
pub const WeldResult = struct { vertices: []const geometry.Vec3, triangles: []const Triangle };

/// Canonicalises a mesh's vertex namespace before topology is examined.  The
/// fixed one-raw-unit tolerance is part of the asset format: input is sorted
/// by `(x,y,z,source-index)`, every point joins the first representative whose
/// coordinate-wise distance is within that tolerance, and triangle winding is
/// preserved while its cyclic start is made minimal.  No allocator or hash-map
/// iteration is involved.
pub fn weldMesh(vertices: []const geometry.Vec3, triangles: []const Triangle, work: []VertexWeldRef, remap: []u32, vertices_out: []geometry.Vec3, triangles_out: []Triangle) Error!WeldResult {
    if (work.len < vertices.len or remap.len < vertices.len or vertices_out.len < vertices.len or triangles_out.len < triangles.len) return error.OutOfScratch;
    for (vertices, 0..) |vertex, i| work[i] = .{ .point = vertex, .source_index = @intCast(i) };
    insertionSortWeldRefs(work[0..vertices.len]);
    var unique: usize = 0;
    for (work[0..vertices.len]) |entry| {
        var representative: ?usize = null;
        var i: usize = 0;
        while (i < unique) : (i += 1) if (withinWeldEpsilon(vertices_out[i], entry.point)) {
            representative = i;
            break;
        };
        const target = representative orelse blk: {
            vertices_out[unique] = entry.point;
            defer unique += 1;
            break :blk unique;
        };
        remap[entry.source_index] = @intCast(target);
    }
    for (triangles, 0..) |triangle, i| {
        if (triangle.a >= vertices.len or triangle.b >= vertices.len or triangle.c >= vertices.len) return error.InvalidIndex;
        var canonical = Triangle{ .a = remap[triangle.a], .b = remap[triangle.b], .c = remap[triangle.c] };
        if (canonical.a == canonical.b or canonical.a == canonical.c or canonical.b == canonical.c) return error.DegenerateTriangle;
        canonical = rotateTriangleMin(canonical);
        triangles_out[i] = canonical;
    }
    insertionSortTriangles(triangles_out[0..triangles.len]);
    return .{ .vertices = vertices_out[0..unique], .triangles = triangles_out[0..triangles.len] };
}

fn insertionSortWeldRefs(values: []VertexWeldRef) void {
    var i: usize = 1;
    while (i < values.len) : (i += 1) {
        const value = values[i];
        var j = i;
        while (j > 0 and (vertexLess(value.point, values[j - 1].point) or (!vertexLess(values[j - 1].point, value.point) and value.source_index < values[j - 1].source_index))) : (j -= 1) values[j] = values[j - 1];
        values[j] = value;
    }
}
fn withinWeldEpsilon(a: geometry.Vec3, b: geometry.Vec3) bool {
    return @abs(@as(i128, a.x.raw) - b.x.raw) <= weld_epsilon_raw and @abs(@as(i128, a.y.raw) - b.y.raw) <= weld_epsilon_raw and @abs(@as(i128, a.z.raw) - b.z.raw) <= weld_epsilon_raw;
}
fn rotateTriangleMin(value: Triangle) Triangle {
    if (value.b < value.a and value.b <= value.c) return .{ .a = value.b, .b = value.c, .c = value.a };
    if (value.c < value.a and value.c < value.b) return .{ .a = value.c, .b = value.a, .c = value.b };
    return value;
}
fn triangleLess(a: Triangle, b: Triangle) bool {
    return a.a < b.a or (a.a == b.a and (a.b < b.b or (a.b == b.b and a.c < b.c)));
}
fn insertionSortTriangles(values: []Triangle) void {
    var i: usize = 1;
    while (i < values.len) : (i += 1) {
        const value = values[i];
        var j = i;
        while (j > 0 and triangleLess(value, values[j - 1])) : (j -= 1) values[j] = values[j - 1];
        values[j] = value;
    }
}

/// Accumulates area-weighted face normals in canonical triangle order and
/// normalizes once per welded vertex.  A zero sum is a topological defect.
pub fn buildWeldedNormals(vertices: []const geometry.Vec3, triangles: []const Triangle, normals_out: []geometry.Vec3, status: *fp.MathStatus) Error![]const geometry.Vec3 {
    if (normals_out.len < vertices.len) return error.OutOfScratch;
    @memset(normals_out[0..vertices.len], geometry.Vec3.zero);
    for (triangles) |triangle| {
        if (triangle.a >= vertices.len or triangle.b >= vertices.len or triangle.c >= vertices.len) return error.InvalidIndex;
        const normal = vertices[triangle.b].sub(vertices[triangle.a], status).cross(vertices[triangle.c].sub(vertices[triangle.a], status), status);
        for ([_]u32{ triangle.a, triangle.b, triangle.c }) |index| normals_out[index] = normals_out[index].add(normal, status);
    }
    for (normals_out[0..vertices.len]) |*normal| {
        const result = normal.normalize(status);
        if (!result.valid) return error.InvalidTopology;
        normal.* = result.value;
    }
    return normals_out[0..vertices.len];
}

/// Dynamic mesh mass is automatic only for a closed manifold.  All other
/// meshes must carry an already-validated explicit override; no best-effort
/// volume calculation may leak into runtime state.
pub fn dynamicMeshMass(vertices: []const geometry.Vec3, triangles: []const Triangle, adjacency: []MeshAdjacency, override: ?MassProperties, status: *fp.MathStatus) Error!MassProperties {
    if (adjacency.len != triangles.len * 3) return error.InvalidTopology;
    const closed = buildMeshAdjacency(triangles, adjacency) catch return error.InvalidTopology;
    try validateMeshSelfIntersection(vertices, triangles);
    if (!closed) {
        const value = override orelse return error.InvalidMass;
        try validateMassProperties(value);
        return value;
    }
    const value = try meshMassProperties(vertices, triangles, status);
    try validateMassProperties(value);
    return value;
}

/// Separating-axis validation for all non-adjacent triangle pairs.  It uses
/// only exact integer projections (face normals and all edge cross axes), so
/// the baker rejects crossings and coplanar overlaps without platform floats.
pub fn validateMeshSelfIntersection(vertices: []const geometry.Vec3, triangles: []const Triangle) Error!void {
    for (triangles, 0..) |a, ai| {
        if (a.a >= vertices.len or a.b >= vertices.len or a.c >= vertices.len) return error.InvalidIndex;
        for (triangles[ai + 1 ..]) |b| {
            if (b.a >= vertices.len or b.b >= vertices.len or b.c >= vertices.len) return error.InvalidIndex;
            if (trianglesShareVertex(a, b)) continue;
            if (trianglesOverlapSat(vertices, a, b)) return error.InvalidTopology;
        }
    }
}
const WideVec = struct { x: i128, y: i128, z: i128 };
fn wideEdge(a: geometry.Vec3, b: geometry.Vec3) WideVec {
    return .{ .x = @as(i128, b.x.raw) - a.x.raw, .y = @as(i128, b.y.raw) - a.y.raw, .z = @as(i128, b.z.raw) - a.z.raw };
}
fn wideCross(a: WideVec, b: WideVec) WideVec {
    return .{ .x = a.y * b.z - a.z * b.y, .y = a.z * b.x - a.x * b.z, .z = a.x * b.y - a.y * b.x };
}
fn wideZero(a: WideVec) bool {
    return a.x == 0 and a.y == 0 and a.z == 0;
}
fn trianglesShareVertex(a: Triangle, b: Triangle) bool {
    return a.a == b.a or a.a == b.b or a.a == b.c or a.b == b.a or a.b == b.b or a.b == b.c or a.c == b.a or a.c == b.b or a.c == b.c;
}
/// Exact wide-integer triangle overlap SAT used by both baking validation and
/// Task 11 runtime mesh primitive pairs. Touching is overlap by convention.
pub fn trianglesOverlapSat(vertices: []const geometry.Vec3, a: Triangle, b: Triangle) bool {
    const av = [_]geometry.Vec3{ vertices[a.a], vertices[a.b], vertices[a.c] };
    const bv = [_]geometry.Vec3{ vertices[b.a], vertices[b.b], vertices[b.c] };
    const ae = [_]WideVec{ wideEdge(av[0], av[1]), wideEdge(av[1], av[2]), wideEdge(av[2], av[0]) };
    const be = [_]WideVec{ wideEdge(bv[0], bv[1]), wideEdge(bv[1], bv[2]), wideEdge(bv[2], bv[0]) };
    const an = wideCross(ae[0], ae[1]);
    const bn = wideCross(be[0], be[1]);
    if (!wideZero(an) and wideParallel(an, bn) and wideDot(an, wideEdge(av[0], bv[0])) == 0) return coplanarOverlap(av, bv, an);
    if (separated(an, av, bv) or separated(bn, av, bv)) return false;
    for (ae) |left| for (be) |right| if (separated(wideCross(left, right), av, bv)) return false;
    return true;
}
/// Coplanar triangles need in-plane edge normals. Pick the projection plane by
/// dropping the dominant face-normal component, with fixed X/Y/Z tie order.
fn coplanarOverlap(a: [3]geometry.Vec3, b: [3]geometry.Vec3, normal: WideVec) bool {
    const ax = @abs(normal.x);
    const ay = @abs(normal.y);
    const az = @abs(normal.z);
    const drop: u2 = if (ax >= ay and ax >= az) 0 else if (ay >= az) 1 else 2;
    const vertices = [_][3]geometry.Vec3{ a, b };
    for (vertices) |triangle| for (0..3) |i| {
        const p = project2(triangle[i], drop);
        const q = project2(triangle[(i + 1) % 3], drop);
        const axis = Wide2{ .x = p.y - q.y, .y = q.x - p.x };
        if (separated2(axis, a, b, drop)) return false;
    };
    return true;
}
const Wide2 = struct { x: i128, y: i128 };
fn project2(value: geometry.Vec3, drop: u2) Wide2 {
    return switch (drop) {
        0 => .{ .x = value.y.raw, .y = value.z.raw },
        1 => .{ .x = value.x.raw, .y = value.z.raw },
        else => .{ .x = value.x.raw, .y = value.y.raw },
    };
}
fn separated2(axis: Wide2, a: [3]geometry.Vec3, b: [3]geometry.Vec3, drop: u2) bool {
    var amin = projectWide2(axis, project2(a[0], drop));
    var amax = amin;
    var bmin = projectWide2(axis, project2(b[0], drop));
    var bmax = bmin;
    for (a[1..]) |value| {
        const p = projectWide2(axis, project2(value, drop));
        amin = @min(amin, p);
        amax = @max(amax, p);
    }
    for (b[1..]) |value| {
        const p = projectWide2(axis, project2(value, drop));
        bmin = @min(bmin, p);
        bmax = @max(bmax, p);
    }
    return amax < bmin or bmax < amin;
}
fn projectWide2(axis: Wide2, value: Wide2) i256 {
    return @as(i256, axis.x) * value.x + @as(i256, axis.y) * value.y;
}
fn wideDot(a: WideVec, b: WideVec) i256 {
    return @as(i256, a.x) * b.x + @as(i256, a.y) * b.y + @as(i256, a.z) * b.z;
}
fn wideParallel(a: WideVec, b: WideVec) bool {
    return @as(i256, a.y) * b.z == @as(i256, a.z) * b.y and @as(i256, a.z) * b.x == @as(i256, a.x) * b.z and @as(i256, a.x) * b.y == @as(i256, a.y) * b.x;
}
fn separated(axis: WideVec, a: [3]geometry.Vec3, b: [3]geometry.Vec3) bool {
    if (wideZero(axis)) return false;
    var amin = wideProject(axis, a[0]);
    var amax = amin;
    var bmin = wideProject(axis, b[0]);
    var bmax = bmin;
    for (a[1..]) |v| {
        const p = wideProject(axis, v);
        amin = @min(amin, p);
        amax = @max(amax, p);
    }
    for (b[1..]) |v| {
        const p = wideProject(axis, v);
        bmin = @min(bmin, p);
        bmax = @max(bmax, p);
    }
    return amax < bmin or bmax < amin;
}
fn wideProject(axis: WideVec, value: geometry.Vec3) i256 {
    return @as(i256, axis.x) * value.x.raw + @as(i256, axis.y) * value.y.raw + @as(i256, axis.z) * value.z.raw;
}

/// Builds canonical triangle-edge adjacency.  Boundary edges retain the
/// sentinel triangle; duplicate directed edges and non-manifold edges are
/// rejected rather than guessed.  `out` is three entries per triangle.
pub fn buildMeshAdjacency(triangles: []const Triangle, out: []MeshAdjacency) Error!bool {
    if (out.len != triangles.len * 3) return error.OutOfScratch;
    @memset(out, MeshAdjacency{});
    var closed = triangles.len != 0;
    for (triangles, 0..) |triangle, ti| {
        const vs = [_]u32{ triangle.a, triangle.b, triangle.c };
        for (0..3) |ei| {
            const a = vs[ei];
            const b = vs[(ei + 1) % 3];
            var matches: usize = 0;
            var mate_t: u32 = 0;
            var mate_e: u8 = 0;
            for (triangles, 0..) |other, oi| {
                const ov = [_]u32{ other.a, other.b, other.c };
                for (0..3) |oe| {
                    if (ov[oe] == b and ov[(oe + 1) % 3] == a) {
                        matches += 1;
                        mate_t = @intCast(oi);
                        mate_e = @intCast(oe);
                    }
                }
                if (oi != ti) for (0..3) |oe| if (ov[oe] == a and ov[(oe + 1) % 3] == b) return error.InvalidTopology;
            }
            if (matches > 1) return error.InvalidTopology;
            if (matches == 0) closed = false else out[ti * 3 + ei] = .{ .triangle = mate_t, .edge = mate_e };
        }
    }
    return closed;
}

/// Exact content of the mass record.  Positivity is validated here rather
/// than inferred by callers, so a corrupt dynamic-mesh mass cannot publish.
pub fn validateMassProperties(value: MassProperties) Error!void {
    if (value.volume.raw <= 0) return error.InvalidMass;
}

/// Wide tetrahedral volume/centroid integration against the origin.  Callers
/// must first establish closed, consistently oriented adjacency.
pub fn meshVolumeCentroid(vertices: []const geometry.Vec3, triangles: []const Triangle, status: *fp.MathStatus) Error!struct { volume: fp.Fp, center: geometry.Vec3 } {
    var volume = fp.Fp.zero;
    var moment = geometry.Vec3.zero;
    const sixth = fp.Fp.fromRatio(1, 6, status);
    const quarter = fp.Fp.fromRatio(1, 4, status);
    for (triangles) |t| {
        if (t.a >= vertices.len or t.b >= vertices.len or t.c >= vertices.len) return error.InvalidIndex;
        const a = vertices[t.a];
        const b = vertices[t.b];
        const c = vertices[t.c];
        const v = a.dot(b.cross(c, status), status).mul(sixth, status);
        volume = volume.add(v, status);
        moment = moment.add(a.add(b, status).add(c, status).scale(v.mul(quarter, status), status), status);
    }
    if (volume.raw <= 0) return error.InvalidMass;
    return .{ .volume = volume, .center = moment.scale(fp.Fp.one.div(volume, status), status) };
}

/// Integrates the complete symmetric inertia tensor of a closed, consistently
/// wound triangle mesh.  The mesh is decomposed into signed tetrahedra whose
/// fourth vertex is the origin.  The formulas are exact polynomial integrals;
/// all scalar operations retain Task 01's wide intermediates until their
/// individual Q32.32 result is rounded.  The returned tensor is about the
/// computed centre of mass, not about the origin.
pub fn meshMassProperties(vertices: []const geometry.Vec3, triangles: []const Triangle, status: *fp.MathStatus) Error!MassProperties {
    const basic = try meshVolumeCentroid(vertices, triangles, status);
    const ten = fp.Fp.fromInt(10);
    const twenty = fp.Fp.fromInt(20);
    var xx = fp.Fp.zero;
    var yy = fp.Fp.zero;
    var zz = fp.Fp.zero;
    var xy = fp.Fp.zero;
    var xz = fp.Fp.zero;
    var yz = fp.Fp.zero;
    for (triangles) |t| {
        if (t.a >= vertices.len or t.b >= vertices.len or t.c >= vertices.len) return error.InvalidIndex;
        const a = vertices[t.a];
        const b = vertices[t.b];
        const c = vertices[t.c];
        const volume = a.dot(b.cross(c, status), status).div(fp.Fp.fromInt(6), status);
        const p = [_]geometry.Vec3{ a, b, c };
        var diagonal = [_]fp.Fp{ fp.Fp.zero, fp.Fp.zero, fp.Fp.zero };
        var products = [_]fp.Fp{ fp.Fp.zero, fp.Fp.zero, fp.Fp.zero };
        for (0..3) |i| {
            const v = p[i];
            diagonal[0] = diagonal[0].add(v.x.mul(v.x, status), status);
            diagonal[1] = diagonal[1].add(v.y.mul(v.y, status), status);
            diagonal[2] = diagonal[2].add(v.z.mul(v.z, status), status);
            products[0] = products[0].add(v.x.mul(v.y, status).mul(fp.Fp.fromInt(2), status), status);
            products[1] = products[1].add(v.x.mul(v.z, status).mul(fp.Fp.fromInt(2), status), status);
            products[2] = products[2].add(v.y.mul(v.z, status).mul(fp.Fp.fromInt(2), status), status);
            for (0..3) |j| if (i > j) {
                const q = p[j];
                diagonal[0] = diagonal[0].add(v.x.mul(q.x, status), status);
                diagonal[1] = diagonal[1].add(v.y.mul(q.y, status), status);
                diagonal[2] = diagonal[2].add(v.z.mul(q.z, status), status);
            };
            for (0..3) |j| if (i != j) {
                const q = p[j];
                products[0] = products[0].add(v.x.mul(q.y, status), status);
                products[1] = products[1].add(v.x.mul(q.z, status), status);
                products[2] = products[2].add(v.y.mul(q.z, status), status);
            };
        }
        xx = xx.add(volume.mul(diagonal[0].div(ten, status), status), status);
        yy = yy.add(volume.mul(diagonal[1].div(ten, status), status), status);
        zz = zz.add(volume.mul(diagonal[2].div(ten, status), status), status);
        xy = xy.add(volume.mul(products[0].div(twenty, status), status), status);
        xz = xz.add(volume.mul(products[1].div(twenty, status), status), status);
        yz = yz.add(volume.mul(products[2].div(twenty, status), status), status);
    }
    const centre = basic.center;
    const volume = basic.volume;
    const inertia = geometry.SymmetricMat3{
        .xx = yy.add(zz, status).sub(volume.mul(centre.y.mul(centre.y, status).add(centre.z.mul(centre.z, status), status), status), status),
        .yy = xx.add(zz, status).sub(volume.mul(centre.x.mul(centre.x, status).add(centre.z.mul(centre.z, status), status), status), status),
        .zz = xx.add(yy, status).sub(volume.mul(centre.x.mul(centre.x, status).add(centre.y.mul(centre.y, status), status), status), status),
        .xy = xy.sub(volume.mul(centre.x.mul(centre.y, status), status), status).neg(status),
        .xz = xz.sub(volume.mul(centre.x.mul(centre.z, status), status), status).neg(status),
        .yz = yz.sub(volume.mul(centre.y.mul(centre.z, status), status), status).neg(status),
    };
    const result = MassProperties{ .volume = volume, .center = centre, .inertia = inertia };
    try validateMassProperties(result);
    return result;
}

pub fn validateBvh(nodes: []const BvhNode, primitives: []const u32, triangle_count: usize) Error!void {
    if (nodes.len == 0 or primitives.len != triangle_count) return error.InvalidBvh;
    var seen_count: usize = 0;
    for (nodes) |node| {
        if ((node.flags & ~BvhNode.leaf_flag) != 0) return error.InvalidBvh;
        if ((node.flags & BvhNode.leaf_flag) != 0) {
            if (node.axis != 0 or node.count == 0 or node.count > 4) return error.InvalidBvh;
            const end = std.math.add(usize, node.first, node.count) catch return error.InvalidBvh;
            if (end > primitives.len) return error.InvalidBvh;
            seen_count += node.count;
        } else {
            if (node.count != 0 or node.axis > 2 or node.first + 1 >= nodes.len) return error.InvalidBvh;
        }
    }
    if (seen_count != primitives.len) return error.InvalidBvh;
    for (primitives, 0..) |primitive, i| {
        if (primitive >= triangle_count) return error.InvalidBvh;
        for (primitives[0..i]) |prior| if (prior == primitive) return error.InvalidBvh;
    }
}

pub const BvhBuildResult = struct { nodes: []const BvhNode, primitives: []const u32 };

/// Builds a bounded, deterministic triangle BVH in caller storage.  Every
/// candidate split is scored by exact integer SAH; ties are broken by
/// `(axis, split position, primitive id)`.  Nodes are emitted with adjacent
/// child roots, independent of traversal or allocator order.
pub fn buildTriangleBvh(vertices: []const geometry.Vec3, triangles: []const Triangle, nodes: []BvhNode, primitives: []u32) Error!BvhBuildResult {
    if (triangles.len == 0 or primitives.len < triangles.len or nodes.len < triangles.len * 2 - 1) return error.OutOfScratch;
    for (triangles, 0..) |triangle, i| {
        if (triangle.a >= vertices.len or triangle.b >= vertices.len or triangle.c >= vertices.len) return error.InvalidIndex;
        primitives[i] = @intCast(i);
    }
    var cursor: usize = 1;
    try buildBvhNode(vertices, triangles, nodes, primitives[0..triangles.len], primitives.ptr, 0, &cursor);
    return .{ .nodes = nodes[0..cursor], .primitives = primitives[0..triangles.len] };
}

fn buildBvhNode(vertices: []const geometry.Vec3, triangles: []const Triangle, nodes: []BvhNode, ids: []u32, root: [*]u32, at: usize, cursor: *usize) Error!void {
    const bounds = trianglesBounds(vertices, triangles, ids) orelse return error.DegenerateTriangle;
    if (ids.len <= 4) {
        const first: usize = (@intFromPtr(ids.ptr) - @intFromPtr(root)) / @sizeOf(u32);
        nodes[at] = BvhNode.leaf(bounds, @intCast(first), @intCast(ids.len));
        return;
    }
    const choice = chooseSahSplit(vertices, triangles, ids);
    insertionSortIds(vertices, triangles, ids, choice.axis);
    const mid = choice.split;
    const left = cursor.*;
    const right = left + 1;
    if (right >= nodes.len) return error.OutOfScratch;
    cursor.* = right + 1;
    nodes[at] = .{ .bounds = bounds, .first = @intCast(left), .count = 0, .axis = choice.axis, .flags = 0 };
    try buildBvhNode(vertices, triangles, nodes, ids[0..mid], root, left, cursor);
    try buildBvhNode(vertices, triangles, nodes, ids[mid..], root, right, cursor);
}
const SahSplit = struct { axis: u8, split: usize };
fn chooseSahSplit(vertices: []const geometry.Vec3, triangles: []const Triangle, ids: []u32) SahSplit {
    var best_axis: u8 = 0;
    var best_split: usize = 1;
    var best_cost: ?i256 = null;
    var best_primitive: u32 = std.math.maxInt(u32);
    var axis: u8 = 0;
    while (axis < 3) : (axis += 1) {
        insertionSortIds(vertices, triangles, ids, axis);
        var split: usize = 1;
        while (split < ids.len) : (split += 1) {
            const left = trianglesBounds(vertices, triangles, ids[0..split]).?;
            const right = trianglesBounds(vertices, triangles, ids[split..]).?;
            const cost = sahArea(left) * @as(i256, @intCast(split)) + sahArea(right) * @as(i256, @intCast(ids.len - split));
            const primitive = ids[split];
            if (best_cost == null or cost < best_cost.? or (cost == best_cost.? and (axis < best_axis or (axis == best_axis and (split < best_split or (split == best_split and primitive < best_primitive)))))) {
                best_cost = cost;
                best_axis = axis;
                best_split = split;
                best_primitive = primitive;
            }
        }
    }
    return .{ .axis = best_axis, .split = best_split };
}
fn sahArea(value: geometry.Aabb3) i256 {
    const x: i256 = @as(i256, value.max.x.raw) - value.min.x.raw;
    const y: i256 = @as(i256, value.max.y.raw) - value.min.y.raw;
    const z: i256 = @as(i256, value.max.z.raw) - value.min.z.raw;
    return x * y + x * z + y * z;
}

fn trianglesBounds(vertices: []const geometry.Vec3, triangles: []const Triangle, ids: []const u32) ?geometry.Aabb3 {
    if (ids.len == 0) return null;
    var first = true;
    var result: geometry.Aabb3 = undefined;
    for (ids) |id| {
        const t = triangles[id];
        for ([_]geometry.Vec3{ vertices[t.a], vertices[t.b], vertices[t.c] }) |p| {
            if (first) {
                result = .{ .min = p, .max = p };
                first = false;
            } else {
                result.min.x.raw = @min(result.min.x.raw, p.x.raw);
                result.min.y.raw = @min(result.min.y.raw, p.y.raw);
                result.min.z.raw = @min(result.min.z.raw, p.z.raw);
                result.max.x.raw = @max(result.max.x.raw, p.x.raw);
                result.max.y.raw = @max(result.max.y.raw, p.y.raw);
                result.max.z.raw = @max(result.max.z.raw, p.z.raw);
            }
        }
    }
    return result;
}
fn largestAxis(b: geometry.Aabb3) u8 {
    const x = @as(i128, b.max.x.raw) - b.min.x.raw;
    const y = @as(i128, b.max.y.raw) - b.min.y.raw;
    const z = @as(i128, b.max.z.raw) - b.min.z.raw;
    return if (y > x and y >= z) 1 else if (z > x and z > y) 2 else 0;
}
fn centroidKey(vertices: []const geometry.Vec3, t: Triangle, axis: u8) i128 {
    const a = vertices[t.a];
    const b = vertices[t.b];
    const c = vertices[t.c];
    return switch (axis) {
        0 => @as(i128, a.x.raw) + b.x.raw + c.x.raw,
        1 => @as(i128, a.y.raw) + b.y.raw + c.y.raw,
        else => @as(i128, a.z.raw) + b.z.raw + c.z.raw,
    };
}
fn insertionSortIds(vertices: []const geometry.Vec3, triangles: []const Triangle, ids: []u32, axis: u8) void {
    var i: usize = 1;
    while (i < ids.len) : (i += 1) {
        const value = ids[i];
        const key = centroidKey(vertices, triangles[value], axis);
        var j = i;
        while (j > 0) {
            const prior = ids[j - 1];
            const pk = centroidKey(vertices, triangles[prior], axis);
            if (pk < key or (pk == key and prior < value)) break;
            ids[j] = prior;
            j -= 1;
        }
        ids[j] = value;
    }
}

/// Emits a canonical TriangleMesh asset. `scratch` is only used for section
/// payloads and can be reused by the caller after this call returns.
pub fn encodeMesh(mesh: Mesh, output: []u8, scratch: []u8) Error!struct { bytes: []const u8, content_hash: hash.Hash256 } {
    try validateMesh(mesh);
    var writer = codec.Writer.init(output);
    try codec.writeHeader(&writer, format_version, 5);
    var header: [16]u8 = [_]u8{0} ** 16;
    header[0] = @intFromEnum(Kind.triangle_mesh);
    header[1] = 1;
    std.mem.writeInt(u64, header[4..12], mesh.source_id, .little);
    try codec.writeSection(&writer, @intFromEnum(Tag.header), &header);
    try writePositions(&writer, mesh.vertices, scratch);
    try writeTriangles(&writer, mesh.triangles, scratch);
    try writeNodes(&writer, mesh.nodes, scratch);
    try writePrimitives(&writer, mesh.primitives, scratch);
    const bytes = output[0..writer.written()];
    return .{ .bytes = bytes, .content_hash = hash.oneShot256(.asset, bytes) };
}

/// Emits a canonical HeightField asset with row-major samples, row-major cell
/// flags/materials, and fixed-size tile min/max leaves.
pub fn encodeHeightField(value: HeightField, output: []u8, scratch: []u8) Error!struct { bytes: []const u8, content_hash: hash.Hash256 } {
    try validateHeightFieldAsset(value);
    var writer = codec.Writer.init(output);
    try codec.writeHeader(&writer, format_version, 4);
    try writeGeometryHeader(&writer, .height_field, value.source_id);
    const sample_size = std.math.add(usize, 8, std.math.mul(usize, value.samples.len, 8) catch return error.OutOfScratch) catch return error.OutOfScratch;
    if (sample_size > scratch.len) return error.OutOfScratch;
    const sample_bytes = scratch[0..sample_size];
    var sw = codec.Writer.init(sample_bytes);
    try sw.unsigned(u32, value.width);
    try sw.unsigned(u32, value.height);
    for (value.samples) |sample| try sw.fpValue(sample);
    try codec.writeSection(&writer, @intFromEnum(Tag.height_samples), sample_bytes);
    const cell_bytes = try payload(scratch, value.cells.len, 5);
    var cw = codec.Writer.init(cell_bytes);
    try cw.unsigned(u32, @intCast(value.cells.len));
    for (value.cells) |cell| {
        try cw.boolean(cell.hole);
        try cw.unsigned(u32, cell.material_id);
    }
    try codec.writeSection(&writer, @intFromEnum(Tag.height_cells), cell_bytes);
    const tree_size = std.math.add(usize, 12, std.math.mul(usize, value.tile_nodes.len, 64) catch return error.OutOfScratch) catch return error.OutOfScratch;
    if (tree_size > scratch.len) return error.OutOfScratch;
    const tree_bytes = scratch[0..tree_size];
    var tw = codec.Writer.init(tree_bytes);
    try tw.unsigned(u32, (value.width - 1 + heightfield_tile_axis - 1) / heightfield_tile_axis);
    try tw.unsigned(u32, (value.height - 1 + heightfield_tile_axis - 1) / heightfield_tile_axis);
    try tw.unsigned(u32, @intCast(value.tile_nodes.len));
    for (value.tile_nodes) |node| try writeNode(&tw, node);
    try codec.writeSection(&writer, @intFromEnum(Tag.height_tile_tree), tree_bytes);
    const bytes = output[0..writer.written()];
    return .{ .bytes = bytes, .content_hash = hash.oneShot256(.asset, bytes) };
}

pub fn encodeConvexHull(value: ConvexHull, source_id: u64, output: []u8, scratch: []u8) Error!struct { bytes: []const u8, content_hash: hash.Hash256 } {
    try validateHullTopology(value.vertices, value.faces, value.half_edges);
    try validateMassProperties(value.mass);
    var writer = codec.Writer.init(output);
    try codec.writeHeader(&writer, format_version, 6);
    try writeGeometryHeader(&writer, .convex_hull, source_id);
    try writePositions(&writer, value.vertices, scratch);
    try writeTriangles(&writer, value.triangles, scratch);
    const faces = try payload(scratch, value.faces.len, 8);
    var fw = codec.Writer.init(faces);
    try fw.unsigned(u32, @intCast(value.faces.len));
    for (value.faces) |face| {
        try fw.unsigned(u32, face.first_half_edge);
        try fw.unsigned(u32, face.half_edge_count);
    }
    try codec.writeSection(&writer, @intFromEnum(Tag.hull_faces), faces);
    const edges = try payload(scratch, value.half_edges.len, 16);
    var ew = codec.Writer.init(edges);
    try ew.unsigned(u32, @intCast(value.half_edges.len));
    for (value.half_edges) |edge| {
        try ew.unsigned(u32, edge.origin);
        try ew.unsigned(u32, edge.twin);
        try ew.unsigned(u32, edge.next);
        try ew.unsigned(u32, edge.face);
    }
    try codec.writeSection(&writer, @intFromEnum(Tag.half_edges), edges);
    try writeMass(&writer, value.mass, scratch);
    const bytes = output[0..writer.written()];
    return .{ .bytes = bytes, .content_hash = hash.oneShot256(.asset, bytes) };
}

pub fn encodeCompound(value: Compound, output: []u8, scratch: []u8) Error!struct { bytes: []const u8, content_hash: hash.Hash256 } {
    if (value.children.len == 0 or value.children.len > 256 or value.nodes.len == 0) return error.InvalidTopology;
    for (value.children, 0..) |child, i| if (child.ordinal != i) return error.InvalidIndex;
    var writer = codec.Writer.init(output);
    try codec.writeHeader(&writer, format_version, 3);
    try writeGeometryHeader(&writer, .compound, value.source_id);
    try writeNodes(&writer, value.nodes, scratch);
    const child_size = std.math.add(usize, 4, std.math.mul(usize, value.children.len, 92) catch return error.OutOfScratch) catch return error.OutOfScratch;
    if (child_size > scratch.len) return error.OutOfScratch;
    const children = scratch[0..child_size];
    var cw = codec.Writer.init(children);
    try cw.unsigned(u32, @intCast(value.children.len));
    for (value.children) |child| {
        try cw.unsigned(u32, child.ordinal);
        for (child.content_hash) |byte| try cw.byte(byte);
        try cw.vec3(child.translation);
        try cw.quat(child.rotation);
    }
    try codec.writeSection(&writer, @intFromEnum(Tag.compound_children), children);
    const bytes = output[0..writer.written()];
    return .{ .bytes = bytes, .content_hash = hash.oneShot256(.asset, bytes) };
}

fn writeGeometryHeader(writer: *codec.Writer, kind: Kind, source_id: u64) Error!void {
    var header: [16]u8 = [_]u8{0} ** 16;
    header[0] = @intFromEnum(kind);
    header[1] = 1;
    std.mem.writeInt(u64, header[4..12], source_id, .little);
    try codec.writeSection(writer, @intFromEnum(Tag.header), &header);
}
fn writeMass(writer: *codec.Writer, mass: MassProperties, scratch: []u8) Error!void {
    if (scratch.len < 80) return error.OutOfScratch;
    var mw = codec.Writer.init(scratch[0..80]);
    try mw.fpValue(mass.volume);
    try mw.vec3(mass.center);
    try mw.fpValue(mass.inertia.xx);
    try mw.fpValue(mass.inertia.yy);
    try mw.fpValue(mass.inertia.zz);
    try mw.fpValue(mass.inertia.xy);
    try mw.fpValue(mass.inertia.xz);
    try mw.fpValue(mass.inertia.yz);
    try codec.writeSection(writer, @intFromEnum(Tag.mass_properties), scratch[0..80]);
}
fn payload(scratch: []u8, count: usize, element_size: usize) Error![]u8 {
    const size = std.math.add(usize, 4, std.math.mul(usize, count, element_size) catch return error.OutOfScratch) catch return error.OutOfScratch;
    if (size > scratch.len) return error.OutOfScratch;
    var writer = codec.Writer.init(scratch[0..size]);
    try writer.unsigned(u32, @intCast(count));
    return scratch[0..size];
}
fn writePositions(writer: *codec.Writer, values: []const geometry.Vec3, scratch: []u8) Error!void {
    const bytes = try payload(scratch, values.len, 24);
    var w = codec.Writer.init(bytes);
    try w.unsigned(u32, @intCast(values.len));
    for (values) |v| try w.vec3(v);
    try codec.writeSection(writer, @intFromEnum(Tag.positions), bytes);
}
fn writeTriangles(writer: *codec.Writer, values: []const Triangle, scratch: []u8) Error!void {
    const bytes = try payload(scratch, values.len, 12);
    var w = codec.Writer.init(bytes);
    try w.unsigned(u32, @intCast(values.len));
    for (values) |v| {
        try w.unsigned(u32, v.a);
        try w.unsigned(u32, v.b);
        try w.unsigned(u32, v.c);
    }
    try codec.writeSection(writer, @intFromEnum(Tag.triangles), bytes);
}
fn writeNodes(writer: *codec.Writer, values: []const BvhNode, scratch: []u8) Error!void {
    const bytes = try payload(scratch, values.len, 64);
    var w = codec.Writer.init(bytes);
    try w.unsigned(u32, @intCast(values.len));
    for (values) |v| try writeNode(&w, v);
    try codec.writeSection(writer, @intFromEnum(Tag.bvh_nodes), bytes);
}
fn writeNode(writer: *codec.Writer, v: BvhNode) Error!void {
    try writer.vec3(v.bounds.min);
    try writer.vec3(v.bounds.max);
    try writer.unsigned(u32, v.first);
    try writer.unsigned(u16, v.count);
    try writer.byte(v.axis);
    try writer.byte(v.flags);
    try writer.unsigned(u32, 0);
    try writer.unsigned(u32, 0);
}
fn writePrimitives(writer: *codec.Writer, values: []const u32, scratch: []u8) Error!void {
    const bytes = try payload(scratch, values.len, 4);
    var w = codec.Writer.init(bytes);
    try w.unsigned(u32, @intCast(values.len));
    for (values) |v| try w.unsigned(u32, v);
    try codec.writeSection(writer, @intFromEnum(Tag.bvh_primitives), bytes);
}
