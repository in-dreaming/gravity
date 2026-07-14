//! Deterministic 3D sweep-and-prune rebuilt from caller-owned buffers each
//! substep. Only root collider proxies enter this layer; Compound children are
//! deliberately left to narrow phase traversal.
const std = @import("std");
const geometry = @import("../math/geometry.zig");
const fp = @import("../math/fp.zig");
const ids = @import("../core/ids.zig");
const radix = @import("../core/radix.zig");
const shapes = @import("shapes.zig");

pub const Error = radix.Error || error{ InvalidProxy, InsufficientEndpoints, InsufficientActive, PairCapacity };
pub const Proxy = struct {
    id: ids.ColliderId,
    collider: *const shapes.Collider,
    body_type: shapes.BodyType,
    world_bounds: geometry.Aabb3,
    fat_bounds: geometry.Aabb3,
    swept_bounds: geometry.Aabb3,
};
pub const Endpoint = struct { x_raw: i64, is_end: bool, proxy_index: u32, collider: ids.ColliderId };
pub const PairKey = struct {
    a: ids.ColliderId,
    b: ids.ColliderId,
    pub fn init(first: ids.ColliderId, second: ids.ColliderId) PairKey {
        return if (first.value < second.value) .{ .a = first, .b = second } else .{ .a = second, .b = first };
    }
};
pub const Buffers = struct {
    endpoints: []Endpoint,
    endpoint_scratch: []Endpoint,
    active: []u32,
    pairs: []PairKey,
    pair_work: []PairKey,
    pair_scratch: []PairKey,
    pair_count: usize = 0,
};

/// Expands an exact AABB by the protocol AABB margin.
pub fn fatAabb(exact: geometry.Aabb3, margin: fp.Fp, status: *fp.MathStatus) geometry.Aabb3 {
    return .{ .min = .{ .x = exact.min.x.sub(margin, status), .y = exact.min.y.sub(margin, status), .z = exact.min.z.sub(margin, status) }, .max = .{ .x = exact.max.x.add(margin, status), .y = exact.max.y.add(margin, status), .z = exact.max.z.add(margin, status) } };
}
/// Builds a velocity-swept AABB; it includes both exact endpoints.
pub fn sweptAabb(exact: geometry.Aabb3, velocity: geometry.Vec3, dt: fp.Fp, status: *fp.MathStatus) geometry.Aabb3 {
    return exact.swept(velocity.scale(dt, status), status);
}

/// Rebuilds and atomically publishes ordered unique pairs. On capacity failure
/// `buffers.pairs[0..pair_count]` is left untouched.
pub fn rebuild(proxies: []const Proxy, buffers: *Buffers) Error![]const PairKey {
    if (buffers.endpoints.len < proxies.len * 2 or buffers.endpoint_scratch.len < proxies.len * 2) return error.InsufficientEndpoints;
    if (buffers.active.len < proxies.len) return error.InsufficientActive;
    if (buffers.pair_work.len != buffers.pair_scratch.len or buffers.pair_work.len < buffers.pairs.len) return error.PairCapacity;
    for (proxies, 0..) |proxy, index| {
        if (!proxy.id.isValid()) return error.InvalidProxy;
        for (proxies[0..index]) |prior| if (prior.id.value == proxy.id.value) return error.InvalidProxy;
        const at = index * 2;
        buffers.endpoints[at] = .{ .x_raw = proxy.swept_bounds.min.x.raw, .is_end = false, .proxy_index = @intCast(index), .collider = proxy.id };
        buffers.endpoints[at + 1] = .{ .x_raw = proxy.swept_bounds.max.x.raw, .is_end = true, .proxy_index = @intCast(index), .collider = proxy.id };
    }
    const endpoints = buffers.endpoints[0 .. proxies.len * 2];
    // Stable least-significant passes implement `(x_raw,start-before-end,
    // ColliderId)` without truncating the generation half of ColliderId.
    try radix.sortU64(Endpoint, endpoints, buffers.endpoint_scratch[0..endpoints.len], endpointColliderKey);
    try radix.sortU32(Endpoint, endpoints, buffers.endpoint_scratch[0..endpoints.len], endpointKindKey);
    try radix.sortU64(Endpoint, endpoints, buffers.endpoint_scratch[0..endpoints.len], endpointXKey);
    var active_len: usize = 0;
    var work_len: usize = 0;
    for (endpoints) |endpoint| {
        const current = proxies[endpoint.proxy_index];
        if (!endpoint.is_end) {
            for (buffers.active[0..active_len]) |active_index| {
                const other = proxies[active_index];
                if (!current.swept_bounds.overlaps(other.swept_bounds)) continue;
                if (shapes.filter(current.collider, current.body_type, other.collider, other.body_type) == .ignore) continue;
                if (work_len == buffers.pair_work.len) return error.PairCapacity;
                buffers.pair_work[work_len] = PairKey.init(current.id, other.id);
                work_len += 1;
            }
            if (active_len == buffers.active.len) return error.InsufficientActive;
            insertActive(proxies, buffers.active[0..], &active_len, endpoint.proxy_index);
        } else {
            removeActive(proxies, buffers.active[0..active_len], &active_len, endpoint.proxy_index) orelse return error.InvalidProxy;
        }
    }
    const work = buffers.pair_work[0..work_len];
    try radix.sortU128(PairKey, work, buffers.pair_scratch[0..work_len], pairKey);
    var unique: usize = 0;
    for (work) |pair| {
        if (unique == 0 or !samePair(work[unique - 1], pair)) {
            work[unique] = pair;
            unique += 1;
        }
    }
    if (unique > buffers.pairs.len) return error.PairCapacity;
    @memcpy(buffers.pairs[0..unique], work[0..unique]);
    buffers.pair_count = unique;
    return buffers.pairs[0..unique];
}

pub fn bruteForce(proxies: []const Proxy, output: []PairKey, scratch: []PairKey) Error![]const PairKey {
    if (scratch.len < output.len) return error.PairCapacity;
    var count: usize = 0;
    for (proxies, 0..) |a, i| for (proxies[i + 1 ..]) |b| {
        if (!a.swept_bounds.overlaps(b.swept_bounds) or shapes.filter(a.collider, a.body_type, b.collider, b.body_type) == .ignore) continue;
        if (count == output.len) return error.PairCapacity;
        output[count] = PairKey.init(a.id, b.id);
        count += 1;
    };
    try radix.sortU128(PairKey, output[0..count], scratch[0..count], pairKey);
    return output[0..count];
}

fn endpointColliderKey(value: Endpoint) u64 {
    return value.collider.value;
}
fn endpointKindKey(value: Endpoint) u32 {
    return @intFromBool(value.is_end);
}
fn endpointXKey(value: Endpoint) u64 {
    return @bitCast(value.x_raw ^ std.math.minInt(i64));
}
fn pairKey(value: PairKey) u128 {
    return (@as(u128, value.a.value) << 64) | value.b.value;
}
fn samePair(a: PairKey, b: PairKey) bool {
    return a.a.value == b.a.value and a.b.value == b.b.value;
}
fn insertActive(proxies: []const Proxy, active: []u32, len: *usize, value: u32) void {
    var at = len.*;
    while (at > 0 and proxies[active[at - 1]].id.value > proxies[value].id.value) : (at -= 1) active[at] = active[at - 1];
    active[at] = value;
    len.* += 1;
}
fn removeActive(proxies: []const Proxy, active: []u32, len: *usize, value: u32) ?void {
    _ = proxies;
    for (active[0..len.*], 0..) |candidate, i| if (candidate == value) {
        std.mem.copyForwards(u32, active[i .. len.* - 1], active[i + 1 .. len.*]);
        len.* -= 1;
        return {};
    };
    return null;
}
