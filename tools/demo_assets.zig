const std = @import("std");
const gravity = @import("gravity");
const baked = gravity.geometry.baked;
const fp = gravity.math.fp;
const g = gravity.math.geometry;

pub fn main(init: std.process.Init) !void {
    var args = try init.minimal.args.iterateAllocator(init.gpa);
    defer args.deinit();
    _ = args.next();
    const hull_path = args.next() orelse return error.InvalidArguments;
    const mesh_path = args.next() orelse return error.InvalidArguments;
    const height_path = args.next() orelse return error.InvalidArguments;
    const compound_path = args.next() orelse return error.InvalidArguments;
    if (args.next() != null) return error.InvalidArguments;

    const points = [_]g.Vec3{
        .{ .x = fp.Fp.fromInt(-1), .y = fp.Fp.zero, .z = fp.Fp.fromInt(-1) },
        .{ .x = fp.Fp.one, .y = fp.Fp.zero, .z = fp.Fp.fromInt(-1) },
        .{ .x = fp.Fp.zero, .y = fp.Fp.fromInt(2), .z = fp.Fp.zero },
        .{ .x = fp.Fp.zero, .y = fp.Fp.zero, .z = fp.Fp.one },
    };
    var hull_vertices: [4]g.Vec3 = undefined;
    var hull_triangles: [4]baked.Triangle = undefined;
    var hull_faces: [4]baked.HullFace = undefined;
    var hull_edges: [12]baked.HalfEdge = undefined;
    var status = fp.MathStatus{};
    const hull = try baked.buildConvexHull(&points, &hull_vertices, &hull_triangles, &hull_faces, &hull_edges, &status);
    var hull_output: [4096]u8 = undefined;
    var hull_scratch: [2048]u8 = undefined;
    const encoded_hull = try baked.encodeConvexHull(hull, 1001, &hull_output, &hull_scratch);
    try std.Io.Dir.cwd().writeFile(init.io, .{ .sub_path = hull_path, .data = encoded_hull.bytes });

    const mesh_triangles = [_]baked.Triangle{
        .{ .a = 0, .b = 2, .c = 1 }, .{ .a = 0, .b = 1, .c = 3 },
        .{ .a = 0, .b = 3, .c = 2 }, .{ .a = 1, .b = 2, .c = 3 },
    };
    var mesh_nodes: [7]baked.BvhNode = undefined;
    var mesh_primitives: [4]u32 = undefined;
    const mesh_bvh = try baked.buildTriangleBvh(&points, &mesh_triangles, &mesh_nodes, &mesh_primitives);
    var mesh_output: [4096]u8 = undefined;
    var mesh_scratch: [2048]u8 = undefined;
    const encoded_mesh = try baked.encodeMesh(.{ .source_id = 1002, .vertices = &points, .triangles = &mesh_triangles, .nodes = mesh_bvh.nodes, .primitives = mesh_bvh.primitives }, &mesh_output, &mesh_scratch);
    try std.Io.Dir.cwd().writeFile(init.io, .{ .sub_path = mesh_path, .data = encoded_mesh.bytes });

    var samples: [25]fp.Fp = undefined;
    for (&samples, 0..) |*sample, index| {
        const x: i64 = @intCast(index % 5);
        const z: i64 = @intCast(index / 5);
        sample.* = fp.Fp.fromRatio((x - 2) * (x - 2) + (z - 2) * (z - 2), 8, &status);
    }
    var cells: [16]baked.HeightCell = [_]baked.HeightCell{.{}} ** 16;
    cells[5].material_id = 1;
    cells[10].hole = true;
    var height_nodes: [1]baked.BvhNode = undefined;
    const tiles = try baked.buildHeightFieldTiles(5, 5, &samples, &cells, &height_nodes);
    var height_output: [4096]u8 = undefined;
    var height_scratch: [2048]u8 = undefined;
    const encoded_height = try baked.encodeHeightField(.{ .source_id = 1003, .width = 5, .height = 5, .samples = &samples, .cells = &cells, .tile_nodes = tiles }, &height_output, &height_scratch);
    try std.Io.Dir.cwd().writeFile(init.io, .{ .sub_path = height_path, .data = encoded_height.bytes });

    const children = [_]baked.CompoundChild{
        .{ .ordinal = 0, .content_hash = encoded_hull.content_hash, .translation = .{ .x = fp.Fp.fromInt(-1) }, .rotation = g.Quat.identity },
        .{ .ordinal = 1, .content_hash = encoded_hull.content_hash, .translation = .{ .x = fp.Fp.one }, .rotation = g.Quat.identity },
    };
    const compound_nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = .{ .x = fp.Fp.fromInt(-2), .y = fp.Fp.zero, .z = fp.Fp.fromInt(-1) }, .max = .{ .x = fp.Fp.fromInt(2), .y = fp.Fp.fromInt(2), .z = fp.Fp.one } }, 0, 2)};
    var compound_output: [4096]u8 = undefined;
    var compound_scratch: [2048]u8 = undefined;
    const encoded_compound = try baked.encodeCompound(.{ .source_id = 1004, .children = &children, .nodes = &compound_nodes }, &compound_output, &compound_scratch);
    try std.Io.Dir.cwd().writeFile(init.io, .{ .sub_path = compound_path, .data = encoded_compound.bytes });
}
