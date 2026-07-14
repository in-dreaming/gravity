//! Deterministic continuous-collision candidate contracts.
//!
//! This module owns the stable ordering and eligibility boundary used by the
//! later World TOI loop.  It deliberately does not treat a smaller time step
//! as continuous collision detection.
const fp = @import("../math/fp.zig");
const geometry = @import("../math/geometry.zig");
const ids = @import("../core/ids.zig");
const shapes = @import("../collision/shapes.zig");
const store = @import("../assets/store.zig");
const mesh = @import("../collision/mesh.zig");
const queries = @import("../query/queries.zig");

pub const Error = error{ InvalidCaster, CapacityExceeded, InvalidFraction } || shapes.Error || mesh.Error || queries.Error;
pub const Fault = enum(u8) { none, non_converged, toi_limit };

/// One swept collider endpoint over the current substep. `delta` is the
/// complete world-space translation for fraction [0, 1].
pub const Item = struct {
    id: ids.ColliderId,
    shape: shapes.Shape,
    transform: geometry.Transform3,
    delta: geometry.Vec3 = .zero,
    enabled: bool = true,
    ccd_enabled: bool = false,
};

/// A completed cast candidate.  The full key has no allocation-dependent
/// component: fraction, ordered collider pair, child path, primitive, feature.
pub const Toi = struct {
    fraction: fp.Fp,
    caster: ids.ColliderId,
    target: ids.ColliderId,
    child_path: shapes.ChildPath = .{},
    primitive: u32 = 0,
    feature: u32 = 0,
    point: geometry.Vec3 = .zero,
    normal: geometry.Vec3 = .unit_x,
    pub fn lessThan(a: Toi, b: Toi) bool {
        if (a.fraction.raw != b.fraction.raw) return a.fraction.raw < b.fraction.raw;
        if (a.caster.index() != b.caster.index()) return a.caster.index() < b.caster.index();
        if (a.caster.generation() != b.caster.generation()) return a.caster.generation() < b.caster.generation();
        if (a.target.index() != b.target.index()) return a.target.index() < b.target.index();
        if (a.target.generation() != b.target.generation()) return a.target.generation() < b.target.generation();
        if (a.child_path.len != b.child_path.len) return a.child_path.len < b.child_path.len;
        for (a.child_path.values[0..a.child_path.len], b.child_path.values[0..b.child_path.len]) |left, right| if (left != right) return left < right;
        if (a.primitive != b.primitive) return a.primitive < b.primitive;
        return a.feature < b.feature;
    }
};

/// Mesh and height-field casters are outside the frozen CCD range.  Compound
/// is permitted here and is resolved by the cast implementation, which must
/// reject it if any terminal child is a surface shape.
pub fn validateCaster(shape: shapes.Shape) Error!void {
    switch (shape) {
        .triangle_mesh, .height_field => return error.InvalidCaster,
        else => {},
    }
}

/// Returns the relative sweep used for a moving target.  Both components are
/// evaluated in fixed point; no wall-clock delta or float conversion enters.
pub fn relativeDelta(caster: Item, target: Item, status: *fp.MathStatus) geometry.Vec3 {
    return caster.delta.sub(target.delta, status);
}

/// Chooses the globally earliest candidate without depending on producer or
/// worker completion order.  Empty input returns null.
pub fn earliest(candidates: []const Toi) ?Toi {
    var best: ?Toi = null;
    for (candidates) |candidate| {
        if (best == null or candidate.lessThan(best.?)) best = candidate;
    }
    return best;
}

/// Enforces the frozen per-substep TOI cap.  The caller must surface
/// `toi_limit` rather than silently returning a partial continuous result.
pub fn requireToiCapacity(processed: usize, maximum: usize) Fault {
    return if (processed >= maximum) .toi_limit else .none;
}

/// Fractional substep cursor.  After each solved TOI, the next cast is over
/// `remaining`, so repeated impacts cannot accidentally advance a full tick.
pub const Cursor = struct {
    elapsed: fp.Fp = .zero,
    remaining: fp.Fp = .one,
    processed: usize = 0,
    fault: Fault = .none,
};
pub const ToiHook = *const fn (context: ?*anyopaque, toi: Toi, local_fraction: fp.Fp) void;

/// A non-mutating CCD scan.  `toi` is expressed in the current cursor's
/// remaining-time interval.  The World owns all cache, event, and solver
/// preparation between this result and `commit`; therefore a capacity or
/// runtime fault can still leave body motion untouched.
pub const Prepared = struct {
    toi: ?Toi = null,
    fault: Fault = .none,
};

/// Builds swept candidates and scans their globally earliest TOI without
/// advancing `items` or `cursor`.  The pair buffer is caller-owned scratch;
/// changing it is not simulation state and is safe to repeat after a failed
/// pre-commit preparation.
pub fn prepare(items: []const Item, pair_scratch: []Pair, assets: *const store.Store, workspace: queries.SurfaceCastWorkspace, status: *fp.MathStatus) Error!Prepared {
    const pairs = try sweptCandidates(items, assets, pair_scratch, status);
    const scan = try findEarliestPairs(items, pairs, assets, workspace, status);
    return .{ .toi = scan.toi, .fault = scan.fault };
}

/// Commits one local TOI fraction after its contact has been solved.  The
/// result is transactional for invalid fractions and cap exhaustion.
pub fn advance(cursor: *Cursor, local_fraction: fp.Fp, maximum: usize, status: *fp.MathStatus) Error!void {
    if (local_fraction.raw < 0 or local_fraction.raw > fp.Fp.one.raw) return error.InvalidFraction;
    if (cursor.processed >= maximum) {
        cursor.fault = .toi_limit;
        return;
    }
    const consumed = cursor.remaining.mul(local_fraction, status);
    cursor.elapsed = cursor.elapsed.add(consumed, status);
    cursor.remaining = cursor.remaining.mul(fp.Fp.one.sub(local_fraction, status), status);
    cursor.processed += 1;
}

/// Commits a World-approved TOI.  Validation happens before either cursor or
/// item motion changes, so an invalid fraction or exhausted TOI budget is
/// atomic with respect to the continuous-motion state.
pub fn commit(items: []Item, cursor: *Cursor, local_fraction: fp.Fp, maximum: usize, status: *fp.MathStatus) Error!void {
    if (local_fraction.raw < 0 or local_fraction.raw > fp.Fp.one.raw) return error.InvalidFraction;
    if (cursor.processed >= maximum) {
        cursor.fault = .toi_limit;
        return;
    }
    try advance(cursor, local_fraction, maximum, status);
    if (cursor.fault != .none) return;
    advanceItems(items, local_fraction, status);
}

/// Commits the discrete remainder after `prepare` reported no TOI.  This is
/// intentionally separate from `commit`, because it does not consume a TOI
/// budget entry.
pub fn commitRemaining(items: []Item, cursor: *Cursor, status: *fp.MathStatus) void {
    advanceItems(items, fp.Fp.one, status);
    cursor.elapsed = fp.Fp.one;
    cursor.remaining = .zero;
}

/// Resolves a substep with deterministic global TOI ordering.  Before each
/// remaining-time advance, `hook` receives the TOI so the owning World
/// pipeline can publish/cache/solve the contact.  This layer intentionally
/// has no ownership of World, contact cache, or event buffers.
pub fn resolve(items: []Item, pair_scratch: []Pair, assets: *const store.Store, workspace: queries.SurfaceCastWorkspace, maximum: usize, hook: ToiHook, context: ?*anyopaque, status: *fp.MathStatus) Error!Cursor {
    var cursor = Cursor{};
    while (cursor.remaining.raw > 0) {
        const prepared = try prepare(items, pair_scratch, assets, workspace, status);
        if (prepared.fault != .none) {
            cursor.fault = prepared.fault;
            return cursor;
        }
        const hit = prepared.toi orelse {
            commitRemaining(items, &cursor, status);
            return cursor;
        };
        if (cursor.processed >= maximum) {
            cursor.fault = .toi_limit;
            return cursor;
        }
        hook(context, hit, hit.fraction);
        try commit(items, &cursor, hit.fraction, maximum, status);
        if (cursor.fault != .none) return cursor;
    }
    return cursor;
}

pub const ScanResult = struct { toi: ?Toi = null, fault: Fault = .none };
pub const Pair = struct { caster: u32, target: u32 };

/// Builds directed swept broadphase candidates.  Only CCD-enabled casters are
/// emitted; targets retain the full frozen shape range.  The output is
/// transactional on capacity failure.
pub fn sweptCandidates(items: []const Item, assets: *const store.Store, output: []Pair, status: *fp.MathStatus) Error![]const Pair {
    var required: usize = 0;
    for (items, 0..) |caster, caster_index| {
        if (!caster.enabled or !caster.ccd_enabled) continue;
        try validateCaster(caster.shape);
        const caster_bounds = try sweptBounds(caster, assets, status);
        for (items, 0..) |target, target_index| {
            if (!target.enabled or caster_index == target_index) continue;
            if (caster_bounds.overlaps(try sweptBounds(target, assets, status))) required += 1;
        }
    }
    if (required > output.len) return error.CapacityExceeded;
    var count: usize = 0;
    for (items, 0..) |caster, caster_index| {
        if (!caster.enabled or !caster.ccd_enabled) continue;
        const caster_bounds = try sweptBounds(caster, assets, status);
        for (items, 0..) |target, target_index| {
            if (!target.enabled or caster_index == target_index) continue;
            if (!caster_bounds.overlaps(try sweptBounds(target, assets, status))) continue;
            output[count] = .{ .caster = @intCast(caster_index), .target = @intCast(target_index) };
            count += 1;
        }
    }
    return output[0..count];
}

/// Runs exact Task 17 convex-versus-surface casts over the supplied swept
/// candidates and returns the globally earliest TOI.  The target's motion is
/// included through `relativeDelta`; no target is silently treated as static.
/// `workspace` is reused sequentially and remains caller-owned.
pub fn findEarliest(items: []const Item, assets: *const store.Store, workspace: queries.SurfaceCastWorkspace, status: *fp.MathStatus) Error!ScanResult {
    var pairs: [0]Pair = .{};
    // The no-scratch convenience form retains the complete candidate set.
    // World pipeline callers use `findEarliestPairs` after swept broadphase.
    _ = &pairs;
    var result = ScanResult{};
    for (items) |caster| {
        if (!caster.enabled or !caster.ccd_enabled) continue;
        try validateCaster(caster.shape);
        for (items) |target| {
            if (!target.enabled or caster.id.value == target.id.value) continue;
            const cast = queries.convexShapeCastSurface(caster.shape, caster.transform, relativeDelta(caster, target, status), target.shape, target.transform, assets, workspace, status) catch |err| switch (err) {
                error.UnsupportedShape => return error.InvalidCaster,
                else => return err,
            };
            switch (cast.status) {
                .miss => {},
                .non_convergent => result.fault = .non_converged,
                .hit => {
                    const candidate = Toi{ .fraction = cast.fraction, .caster = caster.id, .target = target.id, .primitive = 0, .feature = cast.feature, .point = cast.point, .normal = cast.normal };
                    if (result.toi == null or candidate.lessThan(result.toi.?)) result.toi = candidate;
                },
            }
        }
    }
    return result;
}

/// Finds the earliest TOI from a prebuilt swept candidate list.  Invalid pair
/// indices are rejected rather than becoming address/order dependent reads.
pub fn findEarliestPairs(items: []const Item, pairs: []const Pair, assets: *const store.Store, workspace: queries.SurfaceCastWorkspace, status: *fp.MathStatus) Error!ScanResult {
    var result = ScanResult{};
    for (pairs) |pair| {
        if (pair.caster >= items.len or pair.target >= items.len) return error.CapacityExceeded;
        const caster = items[pair.caster];
        const target = items[pair.target];
        if (!caster.enabled or !caster.ccd_enabled or !target.enabled) continue;
        const cast = queries.convexShapeCastSurface(caster.shape, caster.transform, relativeDelta(caster, target, status), target.shape, target.transform, assets, workspace, status) catch |err| switch (err) {
            error.UnsupportedShape => return error.InvalidCaster,
            else => return err,
        };
        switch (cast.status) {
            .miss => {},
            .non_convergent => result.fault = .non_converged,
            .hit => {
                const candidate = Toi{ .fraction = cast.fraction, .caster = caster.id, .target = target.id, .feature = cast.feature, .point = cast.point, .normal = cast.normal };
                if (result.toi == null or candidate.lessThan(result.toi.?)) result.toi = candidate;
            },
        }
    }
    return result;
}

fn sweptBounds(item: Item, assets: *const store.Store, status: *fp.MathStatus) shapes.Error!geometry.Aabb3 {
    var result = try shapes.worldAabb(item.shape, assets, item.transform, status);
    var end = item.transform;
    end.position = end.position.add(item.delta, status);
    const final = try shapes.worldAabb(item.shape, assets, end, status);
    result.min.x.raw = @min(result.min.x.raw, final.min.x.raw);
    result.min.y.raw = @min(result.min.y.raw, final.min.y.raw);
    result.min.z.raw = @min(result.min.z.raw, final.min.z.raw);
    result.max.x.raw = @max(result.max.x.raw, final.max.x.raw);
    result.max.y.raw = @max(result.max.y.raw, final.max.y.raw);
    result.max.z.raw = @max(result.max.z.raw, final.max.z.raw);
    return result;
}
fn advanceItems(items: []Item, fraction: fp.Fp, status: *fp.MathStatus) void {
    for (items) |*item| {
        const consumed = item.delta.scale(fraction, status);
        item.transform.position = item.transform.position.add(consumed, status);
        item.delta = item.delta.scale(fp.Fp.one.sub(fraction, status), status);
    }
}
