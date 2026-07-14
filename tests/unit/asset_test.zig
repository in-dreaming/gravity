const std = @import("std");
const gravity = @import("gravity");
const baked = gravity.geometry.baked;
const store = gravity.assets.store;
const g = gravity.math.geometry;
const fp = gravity.math.fp;

const vertices = [_]g.Vec3{ .{ .x = fp.Fp.zero, .y = fp.Fp.zero, .z = fp.Fp.zero }, .{ .x = fp.Fp.one, .y = fp.Fp.zero, .z = fp.Fp.zero }, .{ .x = fp.Fp.zero, .y = fp.Fp.one, .z = fp.Fp.zero } };
const triangles = [_]baked.Triangle{.{ .a = 0, .b = 1, .c = 2 }};
const nodes = [_]baked.BvhNode{baked.BvhNode.leaf(.{ .min = vertices[0], .max = vertices[2] }, 0, 1)};
const primitives = [_]u32{0};

fn encode(source_id: u64, output: []u8) ![]const u8 {
    var scratch: [256]u8 = undefined;
    return (try baked.encodeMesh(.{ .source_id = source_id, .vertices = &vertices, .triangles = &triangles, .nodes = &nodes, .primitives = &primitives }, output, &scratch)).bytes;
}

test "asset store validates before copying and has stable manifest" {
    var first_buf: [512]u8 = undefined;
    var second_buf: [512]u8 = undefined;
    const first = try encode(3, &first_buf);
    const second = try encode(7, &second_buf);
    const inputs = [_][]const u8{ first, second };
    var memory: [2048]u8 align(@alignOf(store.Asset)) = [_]u8{0xA5} ** 2048;
    const result = try store.Store.init(&memory, &inputs);
    try std.testing.expectEqual(@as(usize, 2), result.assets.len);
    try std.testing.expect(result.find(7) != null);
    var manifest: [128]u8 = undefined;
    const one = try store.writeManifest(result.assets, &manifest);
    const two = try store.writeManifest(result.assets, &manifest);
    try std.testing.expectEqualSlices(u8, one.bytes, two.bytes);

    const bad_inputs = [_][]const u8{ second, first };
    var untouched: [2048]u8 align(@alignOf(store.Asset)) = [_]u8{0xA5} ** 2048;
    try std.testing.expectError(error.DuplicateSourceId, store.Store.init(&untouched, &bad_inputs));
    for (untouched) |byte| try std.testing.expectEqual(@as(u8, 0xA5), byte);
}

test "asset store rejects single-byte corruption" {
    var bytes: [512]u8 = undefined;
    const encoded = try encode(1, &bytes);
    bytes[0] ^= 1;
    const inputs = [_][]const u8{bytes[0..encoded.len]};
    var memory: [1024]u8 align(@alignOf(store.Asset)) = undefined;
    try std.testing.expectError(error.InvalidVersion, store.Store.init(&memory, &inputs));
}
