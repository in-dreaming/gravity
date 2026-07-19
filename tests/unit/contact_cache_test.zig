const std = @import("std");
const gravity = @import("gravity");
const cache = gravity.collision.contact_cache;
const fp = gravity.math.fp;
const g = gravity.math.geometry;

fn key(a: u32, b: u32) cache.ManifoldKey {
    return .{ .collider_a = .init(a, 0), .collider_b = .init(b, 0) };
}
fn patch(value: cache.ManifoldKey, sensor: bool) cache.Patch {
    var result = cache.Patch{ .key = value, .normal = g.Vec3.unit_y, .len = 1, .sensor = sensor };
    result.points[0] = .{ .feature_a = 3, .feature_b = 7 };
    return result;
}

test "contact cache merges sorted patches and preserves matching impulses" {
    var status = fp.MathStatus{};
    var storage: [4]cache.Patch = undefined;
    var state = cache.Cache{ .patches = &storage };
    var next: [8]cache.Patch = undefined;
    var events: [8]cache.Event = undefined;
    const first = [_]cache.Patch{patch(key(1, 2), false)};
    const begin = try cache.merge(&state, &first, .{ .next = &next, .events = &events }, fp.Fp.zero, &status);
    try std.testing.expectEqual(@as(usize, 1), state.len);
    try std.testing.expectEqual(cache.EventKind.begin, begin.events[0].kind);
    state.patches[0].points[0].normal_impulse = fp.Fp.fromInt(4);
    state.patches[0].points[0].tangent_first = fp.Fp.fromInt(2);
    const persist = try cache.merge(&state, &first, .{ .next = &next, .events = &events }, fp.Fp.zero, &status);
    try std.testing.expectEqual(cache.EventKind.persist, persist.events[0].kind);
    try std.testing.expectEqual(fp.Fp.fromInt(4).raw, state.patches[0].points[0].normal_impulse.raw);
    const none = [_]cache.Patch{};
    const end = try cache.merge(&state, &none, .{ .next = &next, .events = &events }, fp.Fp.zero, &status);
    try std.testing.expectEqual(@as(usize, 0), state.len);
    try std.testing.expectEqual(cache.EventKind.end, end.events[0].kind);
}

test "persistent contacts require only live patch capacity" {
    var status = fp.MathStatus{};
    var storage: [5]cache.Patch = undefined;
    var state = cache.Cache{ .patches = &storage };
    var next: [5]cache.Patch = undefined;
    var events: [5]cache.Event = undefined;
    const contacts = [_]cache.Patch{
        patch(key(1, 2), false),
        patch(key(3, 4), false),
        patch(key(5, 6), false),
        patch(key(7, 8), false),
        patch(key(9, 10), false),
    };
    _ = try cache.merge(&state, &contacts, .{ .next = &next, .events = &events }, fp.Fp.zero, &status);
    const persisted = try cache.merge(&state, &contacts, .{ .next = &next, .events = &events }, fp.Fp.zero, &status);
    try std.testing.expectEqual(@as(usize, contacts.len), state.len);
    try std.testing.expectEqual(@as(usize, contacts.len), persisted.events.len);
    for (persisted.events) |event| try std.testing.expectEqual(cache.EventKind.persist, event.kind);
}

test "contact cache clears revision changes and emits sensor transitions transactionally" {
    var status = fp.MathStatus{};
    var storage: [2]cache.Patch = undefined;
    var state = cache.Cache{ .patches = &storage };
    var next: [4]cache.Patch = undefined;
    var events: [4]cache.Event = undefined;
    var contact = patch(key(1, 2), false);
    _ = try cache.merge(&state, &[_]cache.Patch{contact}, .{ .next = &next, .events = &events }, fp.Fp.zero, &status);
    state.patches[0].points[0].normal_impulse = fp.Fp.one;
    contact.key.shape_revision_a = 1;
    _ = try cache.merge(&state, &[_]cache.Patch{contact}, .{ .next = &next, .events = &events }, fp.Fp.zero, &status);
    try std.testing.expectEqual(fp.Fp.zero.raw, state.patches[0].points[0].normal_impulse.raw);
    const sensor = [_]cache.Patch{patch(key(1, 2), true)};
    const transition = try cache.merge(&state, &sensor, .{ .next = &next, .events = &events }, fp.Fp.zero, &status);
    try std.testing.expectEqual(@as(usize, 2), transition.events.len);
    try std.testing.expectEqual(cache.EventKind.end, transition.events[0].kind);
    try std.testing.expectEqual(cache.EventKind.sensor_enter, transition.events[1].kind);
    var tiny_events: [0]cache.Event = .{};
    const before = state.len;
    try std.testing.expectError(error.CapacityExceeded, cache.merge(&state, &[_]cache.Patch{}, .{ .next = &next, .events = &tiny_events }, fp.Fp.zero, &status));
    try std.testing.expectEqual(before, state.len);
}

test "contact cache canonical hash excludes unused storage and follows sorted state" {
    var status = fp.MathStatus{};
    var storage_a: [4]cache.Patch = undefined;
    var storage_b: [4]cache.Patch = undefined;
    var first = cache.Cache{ .patches = &storage_a };
    var second = cache.Cache{ .patches = &storage_b };
    var next_a: [4]cache.Patch = undefined;
    var next_b: [4]cache.Patch = undefined;
    var events: [4]cache.Event = undefined;
    const sorted = [_]cache.Patch{ patch(key(1, 2), false), patch(key(3, 4), true) };
    _ = try cache.merge(&first, &sorted, .{ .next = &next_a, .events = &events }, fp.Fp.zero, &status);
    _ = try cache.merge(&second, &sorted, .{ .next = &next_b, .events = &events }, fp.Fp.zero, &status);
    storage_a[3] = patch(key(99, 100), false);
    try std.testing.expectEqualSlices(u8, &cache.canonicalHash(&first), &cache.canonicalHash(&second));
    second.patches[0].points[0].normal_impulse = fp.Fp.one;
    try std.testing.expect(!std.mem.eql(u8, &cache.canonicalHash(&first), &cache.canonicalHash(&second)));
}

test "contact cache codec round trips transactionally" {
    var status = fp.MathStatus{};
    var source_storage: [2]cache.Patch = undefined;
    var source = cache.Cache{ .patches = &source_storage };
    var next: [2]cache.Patch = undefined;
    var events: [2]cache.Event = undefined;
    _ = try cache.merge(&source, &[_]cache.Patch{patch(key(5, 6), false)}, .{ .next = &next, .events = &events }, fp.Fp.zero, &status);
    source.patches[0].points[0].normal_impulse = fp.Fp.fromInt(3);
    var bytes: [256]u8 = undefined;
    const encoded = try cache.encode(&source, &bytes);
    var target_storage: [2]cache.Patch = undefined;
    var target = cache.Cache{ .patches = &target_storage };
    var scratch: [2]cache.Patch = undefined;
    try cache.decode(encoded, &target, &scratch);
    try std.testing.expectEqualSlices(u8, &cache.canonicalHash(&source), &cache.canonicalHash(&target));
    const before = target.len;
    try std.testing.expectError(error.EndOfInput, cache.decode(encoded[0 .. encoded.len - 1], &target, &scratch));
    try std.testing.expectEqual(before, target.len);
}

test "contact cache clears topology and normal changes and sorts event kinds" {
    var status = fp.MathStatus{};
    var storage: [4]cache.Patch = undefined;
    var state = cache.Cache{ .patches = &storage };
    var next: [8]cache.Patch = undefined;
    var events: [8]cache.Event = undefined;
    var first = patch(key(2, 3), false);
    first.points[0].normal_impulse = fp.Fp.fromInt(5);
    _ = try cache.merge(&state, &[_]cache.Patch{first}, .{ .next = &next, .events = &events }, fp.Fp.zero, &status);
    state.patches[0].points[0].normal_impulse = fp.Fp.fromInt(5);
    var switched = patch(key(2, 3), false);
    switched.points[0].feature_b = 8;
    _ = try cache.merge(&state, &[_]cache.Patch{switched}, .{ .next = &next, .events = &events }, fp.Fp.zero, &status);
    try std.testing.expectEqual(fp.Fp.zero.raw, state.patches[0].points[0].normal_impulse.raw);
    state.patches[0].points[0].normal_impulse = fp.Fp.one;
    switched.normal = g.Vec3.unit_x;
    _ = try cache.merge(&state, &[_]cache.Patch{switched}, .{ .next = &next, .events = &events }, fp.Fp.fromInt(1), &status);
    try std.testing.expectEqual(fp.Fp.zero.raw, state.patches[0].points[0].normal_impulse.raw);
    const mixed = [_]cache.Patch{ patch(key(1, 1), false), patch(key(4, 4), false) };
    const result = try cache.merge(&state, &mixed, .{ .next = &next, .events = &events }, fp.Fp.zero, &status);
    var i: usize = 1;
    while (i < result.events.len) : (i += 1) try std.testing.expect(@intFromEnum(result.events[i - 1].kind) <= @intFromEnum(result.events[i].kind));
}
