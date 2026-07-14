const std = @import("std");
const gravity = @import("gravity");
const broad = gravity.collision.broadphase;
const shapes = gravity.collision.shapes;
const ids = gravity.core.ids;
const fp = gravity.math.fp;
const geometry = gravity.math.geometry;

fn bounds(x0: i32, x1: i32) geometry.Aabb3 {
    return .{ .min = .{ .x = fp.Fp.fromInt(x0), .y = fp.Fp.zero, .z = fp.Fp.zero }, .max = .{ .x = fp.Fp.fromInt(x1), .y = fp.Fp.one, .z = fp.Fp.one } };
}
fn collider(body: u32) shapes.Collider {
    return .{ .body = ids.BodyId.init(body, 0), .shape = .{ .sphere = .{ .radius = fp.Fp.one } } };
}
fn proxy(id: u32, value: *const shapes.Collider, value_bounds: geometry.Aabb3) broad.Proxy {
    return .{ .id = ids.ColliderId.init(id, 0), .collider = value, .body_type = .dynamic, .world_bounds = value_bounds, .fat_bounds = value_bounds, .swept_bounds = value_bounds };
}

test "SAP matches brute force with endpoint ties and stable pair order" {
    var colliders = [_]shapes.Collider{ collider(1), collider(2), collider(3), collider(4) };
    const proxies = [_]broad.Proxy{ proxy(8, &colliders[0], bounds(2, 4)), proxy(2, &colliders[1], bounds(0, 2)), proxy(6, &colliders[2], bounds(1, 3)), proxy(4, &colliders[3], bounds(9, 10)) };
    var endpoints: [8]broad.Endpoint = undefined;
    var endpoint_scratch: [8]broad.Endpoint = undefined;
    var active: [4]u32 = undefined;
    var pairs: [8]broad.PairKey = undefined;
    var work: [8]broad.PairKey = undefined;
    var scratch: [8]broad.PairKey = undefined;
    var buffers = broad.Buffers{ .endpoints = &endpoints, .endpoint_scratch = &endpoint_scratch, .active = &active, .pairs = &pairs, .pair_work = &work, .pair_scratch = &scratch };
    const actual = try broad.rebuild(&proxies, &buffers);
    var oracle: [8]broad.PairKey = undefined;
    var oracle_scratch: [8]broad.PairKey = undefined;
    const expected = try broad.bruteForce(&proxies, &oracle, &oracle_scratch);
    try std.testing.expectEqualSlices(broad.PairKey, expected, actual);
    try std.testing.expectEqual(@as(usize, 3), actual.len);
    try std.testing.expectEqual(@as(u64, ids.ColliderId.init(2, 0).value), actual[0].a.value);
}

test "SAP capacity failure is transactional and filters do not ghost" {
    var a = collider(1);
    var b = collider(2);
    const proxies = [_]broad.Proxy{ proxy(1, &a, bounds(0, 2)), proxy(2, &b, bounds(1, 3)) };
    var endpoints: [4]broad.Endpoint = undefined;
    var endpoint_scratch: [4]broad.Endpoint = undefined;
    var active: [2]u32 = undefined;
    var pairs = [_]broad.PairKey{.{ .a = ids.ColliderId.init(99, 0), .b = ids.ColliderId.init(100, 0) }};
    var work: [0]broad.PairKey = .{};
    var scratch: [0]broad.PairKey = .{};
    var buffers = broad.Buffers{ .endpoints = &endpoints, .endpoint_scratch = &endpoint_scratch, .active = &active, .pairs = &pairs, .pair_work = &work, .pair_scratch = &scratch, .pair_count = 1 };
    try std.testing.expectError(error.PairCapacity, broad.rebuild(&proxies, &buffers));
    try std.testing.expectEqual(@as(usize, 1), buffers.pair_count);
    try std.testing.expectEqual(@as(u64, 99), buffers.pairs[0].a.index());
    b.enabled = false;
    var enough_work: [1]broad.PairKey = undefined;
    var enough_scratch: [1]broad.PairKey = undefined;
    buffers.pair_work = &enough_work;
    buffers.pair_scratch = &enough_scratch;
    try std.testing.expectEqual(@as(usize, 0), (try broad.rebuild(&proxies, &buffers)).len);
}

test "SAP is input-order independent and retains collider generations" {
    var colliders = [_]shapes.Collider{ collider(1), collider(2), collider(3) };
    const ordered = [_]broad.Proxy{ proxy(5, &colliders[0], bounds(0, 3)), proxy(1, &colliders[1], bounds(1, 4)), .{ .id = ids.ColliderId.init(1, 7), .collider = &colliders[2], .body_type = .dynamic, .world_bounds = bounds(2, 5), .fat_bounds = bounds(2, 5), .swept_bounds = bounds(2, 5) } };
    const shuffled = [_]broad.Proxy{ ordered[2], ordered[0], ordered[1] };
    var endpoints_a: [6]broad.Endpoint = undefined;
    var endpoint_scratch_a: [6]broad.Endpoint = undefined;
    var active_a: [3]u32 = undefined;
    var pairs_a: [4]broad.PairKey = undefined;
    var work_a: [4]broad.PairKey = undefined;
    var scratch_a: [4]broad.PairKey = undefined;
    var endpoints_b: [6]broad.Endpoint = undefined;
    var endpoint_scratch_b: [6]broad.Endpoint = undefined;
    var active_b: [3]u32 = undefined;
    var pairs_b: [4]broad.PairKey = undefined;
    var work_b: [4]broad.PairKey = undefined;
    var scratch_b: [4]broad.PairKey = undefined;
    var a = broad.Buffers{ .endpoints = &endpoints_a, .endpoint_scratch = &endpoint_scratch_a, .active = &active_a, .pairs = &pairs_a, .pair_work = &work_a, .pair_scratch = &scratch_a };
    var b = broad.Buffers{ .endpoints = &endpoints_b, .endpoint_scratch = &endpoint_scratch_b, .active = &active_b, .pairs = &pairs_b, .pair_work = &work_b, .pair_scratch = &scratch_b };
    const first = try broad.rebuild(&ordered, &a);
    const second = try broad.rebuild(&shuffled, &b);
    try std.testing.expectEqualSlices(broad.PairKey, first, second);
    try std.testing.expect(first[0].a.generation() == 0);
    try std.testing.expect(first[first.len - 1].b.generation() == 7);
}

test "SAP rebuild handles the default 8192-body pressure shape without allocation" {
    const count = 8_192;
    var source = collider(99);
    var proxies: [count]broad.Proxy = undefined;
    for (&proxies, 0..) |*item, i| {
        const x: i32 = @intCast(i * 3);
        item.* = proxy(@intCast(i), &source, bounds(x, x + 1));
    }
    var endpoints: [count * 2]broad.Endpoint = undefined;
    var endpoint_scratch: [count * 2]broad.Endpoint = undefined;
    var active: [count]u32 = undefined;
    var no_pairs: [0]broad.PairKey = .{};
    var no_work: [0]broad.PairKey = .{};
    var no_scratch: [0]broad.PairKey = .{};
    var buffers = broad.Buffers{ .endpoints = &endpoints, .endpoint_scratch = &endpoint_scratch, .active = &active, .pairs = &no_pairs, .pair_work = &no_work, .pair_scratch = &no_scratch };
    try std.testing.expectEqual(@as(usize, 0), (try broad.rebuild(&proxies, &buffers)).len);
}
