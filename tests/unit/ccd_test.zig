const std = @import("std");
const gravity = @import("gravity");
const fp = gravity.math.fp;
const g = gravity.math.geometry;
const ccd = gravity.dynamics.ccd;
const shapes = gravity.collision.shapes;
const mesh = gravity.collision.mesh;
const store = gravity.assets.store;
const baked = gravity.geometry.baked;

test "CCD rejects surface casters and preserves relative moving-target sweep" {
    var status = fp.MathStatus{};
    try std.testing.expectError(error.InvalidCaster, ccd.validateCaster(.{ .triangle_mesh = .{} }));
    try std.testing.expectError(error.InvalidCaster, ccd.validateCaster(.{ .height_field = .{} }));
    try ccd.validateCaster(.{ .sphere = .{ .radius = .one } });
    const caster = ccd.Item{ .id = .init(2, 0), .shape = .{ .sphere = .{ .radius = .one } }, .transform = .{}, .delta = .{ .x = fp.Fp.fromInt(8) } };
    const target = ccd.Item{ .id = .init(3, 0), .shape = .{ .box = .{ .half_extents = .{ .x = .one, .y = .one, .z = .one } } }, .transform = .{}, .delta = .{ .x = fp.Fp.fromInt(3) } };
    try std.testing.expectEqual(fp.Fp.fromInt(5).raw, ccd.relativeDelta(caster, target, &status).x.raw);
}

test "CCD earliest TOI uses the complete stable key and cap faults" {
    var status = fp.MathStatus{};
    const high = ccd.Toi{ .fraction = fp.Fp.fromInt(1), .caster = .init(1, 0), .target = .init(2, 0) };
    const first = ccd.Toi{ .fraction = fp.Fp.fromRatio(1, 2, &status), .caster = .init(3, 0), .target = .init(4, 0), .primitive = 9 };
    const tied = ccd.Toi{ .fraction = fp.Fp.fromRatio(1, 2, &status), .caster = .init(2, 0), .target = .init(9, 0) };
    const result = ccd.earliest(&.{ high, first, tied }).?;
    try std.testing.expectEqual(tied.caster.value, result.caster.value);
    try std.testing.expectEqual(ccd.Fault.none, ccd.requireToiCapacity(7, 8));
    try std.testing.expectEqual(ccd.Fault.toi_limit, ccd.requireToiCapacity(8, 8));
    var cursor = ccd.Cursor{};
    try ccd.advance(&cursor, fp.Fp.fromRatio(1, 2, &status), 2, &status);
    try ccd.advance(&cursor, fp.Fp.fromRatio(1, 2, &status), 2, &status);
    try std.testing.expectEqual(fp.Fp.fromRatio(3, 4, &status).raw, cursor.elapsed.raw);
    try std.testing.expectEqual(fp.Fp.fromRatio(1, 4, &status).raw, cursor.remaining.raw);
    try ccd.advance(&cursor, .zero, 2, &status);
    try std.testing.expectEqual(ccd.Fault.toi_limit, cursor.fault);
    _ = g;
}

test "CCD scan returns a global earliest convex TOI against a moving target" {
    var status = fp.MathStatus{};
    var memory: [0]u8 align(@alignOf(store.Asset)) = .{};
    const assets = try store.Store.init(&memory, &.{});
    var leaves: [0]shapes.CompoundLeaf = .{};
    var work: [0]u32 = .{};
    var nodes: [0]baked.BvhNode = .{};
    var triangles: [0]mesh.HeightTriangle = .{};
    const workspace = gravity.query.queries.SurfaceCastWorkspace{ .compound_leaves = &leaves, .mesh = .{ .nodes = &nodes, .primitives = &work, .stack = &work }, .heightfield = .{ .stack = &work, .triangles = &triangles } };
    var items = [_]ccd.Item{
        .{ .id = .init(2, 0), .shape = .{ .sphere = .{ .radius = .one } }, .transform = .{ .position = .{ .x = fp.Fp.fromInt(-3) } }, .delta = .{ .x = fp.Fp.fromInt(6) }, .ccd_enabled = true },
        .{ .id = .init(4, 0), .shape = .{ .box = .{ .half_extents = .{ .x = .one, .y = .one, .z = .one } } }, .transform = .{}, .delta = .{ .x = fp.Fp.fromInt(1) } },
    };
    var candidates: [2]ccd.Pair = undefined;
    const pairs = try ccd.sweptCandidates(&items, &assets, &candidates, &status);
    try std.testing.expectEqual(@as(usize, 1), pairs.len);
    const result = try ccd.findEarliestPairs(&items, pairs, &assets, workspace, &status);
    try std.testing.expectEqual(ccd.Fault.none, result.fault);
    try std.testing.expect(result.toi != null);
    try std.testing.expectEqual(items[0].id.value, result.toi.?.caster.value);
    try std.testing.expect(result.toi.?.fraction.raw > 0 and result.toi.?.fraction.raw < fp.Fp.one.raw);
    const Hook = struct {
        fn call(context: ?*anyopaque, _: ccd.Toi, _: fp.Fp) void {
            const count: *usize = @ptrCast(@alignCast(context.?));
            count.* += 1;
        }
    };
    var hook_count: usize = 0;
    const resolved = try ccd.resolve(&items, &candidates, &assets, workspace, 1, Hook.call, &hook_count, &status);
    try std.testing.expectEqual(@as(usize, 1), hook_count);
    try std.testing.expectEqual(ccd.Fault.toi_limit, resolved.fault);
}

test "CCD preparation leaves continuous motion unchanged until World commits" {
    var status = fp.MathStatus{};
    var memory: [0]u8 align(@alignOf(store.Asset)) = .{};
    const assets = try store.Store.init(&memory, &.{});
    var leaves: [0]shapes.CompoundLeaf = .{};
    var work: [0]u32 = .{};
    var nodes: [0]baked.BvhNode = .{};
    var triangles: [0]mesh.HeightTriangle = .{};
    const workspace = gravity.query.queries.SurfaceCastWorkspace{ .compound_leaves = &leaves, .mesh = .{ .nodes = &nodes, .primitives = &work, .stack = &work }, .heightfield = .{ .stack = &work, .triangles = &triangles } };
    var items = [_]ccd.Item{
        .{ .id = .init(2, 0), .shape = .{ .sphere = .{ .radius = .one } }, .transform = .{ .position = .{ .x = fp.Fp.fromInt(-3) } }, .delta = .{ .x = fp.Fp.fromInt(6) }, .ccd_enabled = true },
        .{ .id = .init(4, 0), .shape = .{ .box = .{ .half_extents = .{ .x = .one, .y = .one, .z = .one } } }, .transform = .{}, .delta = .zero },
    };
    const before = items;
    var pairs: [2]ccd.Pair = undefined;
    const prepared = try ccd.prepare(&items, &pairs, &assets, workspace, &status);
    const toi = prepared.toi orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(ccd.Fault.none, prepared.fault);
    try std.testing.expectEqual(before[0].transform.position.x.raw, items[0].transform.position.x.raw);
    try std.testing.expectEqual(before[0].delta.x.raw, items[0].delta.x.raw);
    var cursor = ccd.Cursor{};
    try ccd.commit(&items, &cursor, toi.fraction, 8, &status);
    try std.testing.expect(items[0].transform.position.x.raw != before[0].transform.position.x.raw);
    try std.testing.expect(cursor.elapsed.raw > 0);
}

test "disabled CCD follows the full discrete remaining-time path" {
    var status = fp.MathStatus{};
    var memory: [0]u8 align(@alignOf(store.Asset)) = .{};
    const assets = try store.Store.init(&memory, &.{});
    var leaves: [0]shapes.CompoundLeaf = .{};
    var work: [0]u32 = .{};
    var nodes: [0]baked.BvhNode = .{};
    var triangles: [0]mesh.HeightTriangle = .{};
    const workspace = gravity.query.queries.SurfaceCastWorkspace{ .compound_leaves = &leaves, .mesh = .{ .nodes = &nodes, .primitives = &work, .stack = &work }, .heightfield = .{ .stack = &work, .triangles = &triangles } };
    var items = [_]ccd.Item{.{ .id = .init(7, 0), .shape = .{ .sphere = .{ .radius = .one } }, .transform = .{}, .delta = .{ .x = fp.Fp.fromInt(3) }, .ccd_enabled = false }};
    var pairs: [0]ccd.Pair = .{};
    const Hook = struct {
        fn call(context: ?*anyopaque, _: ccd.Toi, _: fp.Fp) void {
            const count: *usize = @ptrCast(@alignCast(context.?));
            count.* += 1;
        }
    };
    var count: usize = 0;
    const result = try ccd.resolve(&items, &pairs, &assets, workspace, 8, Hook.call, &count, &status);
    try std.testing.expectEqual(ccd.Fault.none, result.fault);
    try std.testing.expectEqual(@as(usize, 0), count);
    try std.testing.expectEqual(fp.Fp.one.raw, result.elapsed.raw);
    try std.testing.expectEqual(@as(i64, 0), result.remaining.raw);
    try std.testing.expectEqual(fp.Fp.fromInt(3).raw, items[0].transform.position.x.raw);
}
