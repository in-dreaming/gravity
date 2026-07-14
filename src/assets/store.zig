//! Immutable caller-memory asset set.  Input is validated in full before any
//! destination byte is written, so a failed load never publishes a half store.
const std = @import("std");
const baked = @import("../geometry/baked.zig");
const hash = @import("../state/hash.zig");

pub const Error = baked.Error || error{ InsufficientMemory, MisalignedMemory, DuplicateSourceId, DuplicateHash, InvalidManifest, UnresolvedReference, CompoundCycle, CompoundDepth, CompoundChildren };

pub const Asset = struct {
    source_id: u64,
    kind: baked.Kind,
    bytes: []const u8,
    content_hash: hash.Hash256,
};

pub const ManifestEntry = struct { source_id: u64, content_hash: hash.Hash256 };

pub const Store = struct {
    assets: []const Asset,
    bytes: []const u8,
    asset_set_hash: hash.Hash256,

    /// Bytes needed for `init`, including aligned immutable index entries.
    pub fn memoryRequired(inputs: []const []const u8) Error!usize {
        var total: usize = std.mem.alignForward(usize, inputs.len * @sizeOf(Asset), @alignOf(Asset));
        for (inputs) |input| total = std.math.add(usize, total, input.len) catch return error.InsufficientMemory;
        return total;
    }

    /// Builds an immutable asset set in caller-owned `memory`. `inputs` must be
    /// in source-id order; this is deliberately checked instead of silently sorted.
    pub fn init(memory: []u8, inputs: []const []const u8) Error!Store {
        const required = try memoryRequired(inputs);
        if (memory.len < required) return error.InsufficientMemory;
        if (@intFromPtr(memory.ptr) % @alignOf(Asset) != 0) return error.MisalignedMemory;
        const index_len = inputs.len * @sizeOf(Asset);
        const index_end = std.mem.alignForward(usize, index_len, @alignOf(Asset));
        const ptr: [*]Asset = @ptrCast(@alignCast(memory.ptr));
        const assets = ptr[0..inputs.len];

        var previous_id: ?u64 = null;
        var cursor: usize = index_end;
        for (inputs, 0..) |input, i| {
            const header = try baked.validateEncoded(input);
            const digest = hash.oneShot256(.asset, input);
            if (previous_id) |id| if (header.source_id <= id) return error.DuplicateSourceId;
            for (inputs[0..i]) |prior_input| {
                const prior_digest = hash.oneShot256(.asset, prior_input);
                if (std.mem.eql(u8, &prior_digest, &digest)) return error.DuplicateHash;
            }
            previous_id = header.source_id;
            cursor = std.math.add(usize, cursor, input.len) catch return error.InsufficientMemory;
        }
        try validateCompoundGraph(inputs);

        // All validation is complete: copying and publication begin here.
        cursor = index_end;
        for (inputs, 0..) |input, i| {
            const header = baked.validateEncoded(input) catch unreachable;
            const digest = hash.oneShot256(.asset, input);
            @memcpy(memory[cursor..][0..input.len], input);
            assets[i] = .{ .source_id = header.source_id, .kind = header.kind, .bytes = memory[cursor..][0..input.len], .content_hash = digest };
            cursor += input.len;
        }
        var sink = hash.Sink.init(.asset_set);
        for (assets) |asset| {
            var id: [8]u8 = undefined;
            std.mem.writeInt(u64, &id, asset.source_id, .little);
            sink.update(&id);
            sink.update(&asset.content_hash);
        }
        return .{ .assets = assets, .bytes = memory[index_end..cursor], .asset_set_hash = sink.final256() };
    }

    pub fn find(self: *const Store, source_id: u64) ?*const Asset {
        var lower: usize = 0;
        var upper = self.assets.len;
        while (lower < upper) {
            const mid = lower + (upper - lower) / 2;
            if (self.assets[mid].source_id < source_id) lower = mid + 1 else upper = mid;
        }
        return if (lower < self.assets.len and self.assets[lower].source_id == source_id) &self.assets[lower] else null;
    }

    /// Resolves a compound child reference. Asset-set order is irrelevant here:
    /// the content hash is the canonical identity embedded in the compound.
    pub fn findByHash(self: *const Store, wanted: *const hash.Hash256) ?*const Asset {
        for (self.assets) |*asset| if (std.mem.eql(u8, &asset.content_hash, wanted)) return asset;
        return null;
    }
};

fn validateCompoundGraph(inputs: []const []const u8) Error!void {
    const Walk = struct {
        fn indexFor(inputs_: []const []const u8, wanted: *const hash.Hash256) ?usize {
            for (inputs_, 0..) |input, index| {
                const digest = hash.oneShot256(.asset, input);
                if (std.mem.eql(u8, &digest, wanted)) return index;
            }
            return null;
        }
        fn run(inputs_: []const []const u8, index: usize, path: *[8]hash.Hash256, depth: u8) Error!void {
            const input = inputs_[index];
            if ((try baked.validateEncoded(input)).kind != .compound) return;
            var ordinal: u32 = 0;
            while (try baked.compoundChildHash(input, ordinal)) |child_hash| : (ordinal += 1) {
                if (depth >= 8) return error.CompoundDepth;
                for (path[0..depth]) |ancestor| if (std.mem.eql(u8, &ancestor, &child_hash)) return error.CompoundCycle;
                const child_index = indexFor(inputs_, &child_hash) orelse return error.UnresolvedReference;
                path[depth] = child_hash;
                try run(inputs_, child_index, path, depth + 1);
            }
        }
    };
    for (inputs, 0..) |input, index| {
        if ((try baked.validateEncoded(input)).kind != .compound) continue;
        var path: [8]hash.Hash256 = undefined;
        path[0] = hash.oneShot256(.asset, input);
        try Walk.run(inputs, index, &path, 1);
    }
}

/// Encodes the manifest's canonical payload (count, then source-id/hash pairs)
/// and returns its asset-set digest. The asset input must already be source-id ordered.
pub fn writeManifest(assets: []const Asset, output: []u8) Error!struct { bytes: []const u8, content_hash: hash.Hash256 } {
    const size = std.math.add(usize, 4, std.math.mul(usize, assets.len, 40) catch return error.InsufficientMemory) catch return error.InsufficientMemory;
    if (output.len < size) return error.InsufficientMemory;
    std.mem.writeInt(u32, output[0..4], @intCast(assets.len), .little);
    var at: usize = 4;
    var prior: ?u64 = null;
    for (assets) |asset| {
        if (prior) |id| if (asset.source_id <= id) return error.InvalidManifest;
        prior = asset.source_id;
        std.mem.writeInt(u64, output[at..][0..8], asset.source_id, .little);
        @memcpy(output[at + 8 ..][0..32], &asset.content_hash);
        at += 40;
    }
    return .{ .bytes = output[0..size], .content_hash = hash.oneShot256(.asset_set, output[0..size]) };
}
