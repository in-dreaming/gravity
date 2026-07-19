//! Task 25 bounded fuzz corpus for untrusted parsers and C ABI envelopes.
const std = @import("std");
const gravity = @import("gravity");

const fp = gravity.math.fp;
const source = gravity.assets.source;
const baked = gravity.geometry.baked;
const snapshot = gravity.state.snapshot;
const replay = gravity.state.replay;
const abi = gravity.abi;

fn next(seed: *u64) u64 {
    seed.* = seed.* *% 6_364_136_223_846_793_005 +% 1_442_695_040_888_963_407;
    return seed.*;
}

test "decimal parser fuzz is bounded and every accepted value round trips" {
    const alphabet = "-+.0123456789eE_x";
    var seed: u64 = 0x25_dec1_a1;
    var input: [64]u8 = undefined;
    var output: [96]u8 = undefined;
    for (0..20_000) |_| {
        const length: usize = @intCast(next(&seed) % (input.len + 1));
        for (input[0..length]) |*byte| byte.* = alphabet[@intCast(next(&seed) % alphabet.len)];
        var status = fp.MathStatus{};
        const value = fp.Fp.parseCanonicalDecimal(input[0..length], &status);
        if (status.fault != .none) continue;
        const text = value.formatCanonical(&output) orelse return error.FormatCapacity;
        var replay_status = fp.MathStatus{};
        const reparsed = fp.Fp.parseCanonicalDecimal(text, &replay_status);
        try std.testing.expectEqual(fp.MathFault.none, replay_status.fault);
        try std.testing.expectEqual(value.raw, reparsed.raw);
    }
}

test "asset JSON and binary TLV fuzz stay within caller bounded storage" {
    var seed: u64 = 0x25_a55e_7;
    var bytes: [512]u8 = undefined;
    for (0..8_000) |_| {
        const length: usize = @intCast(next(&seed) % (bytes.len + 1));
        for (bytes[0..length]) |*byte| byte.* = @truncate(next(&seed));
        _ = source.validate(bytes[0..length], std.testing.allocator) catch {};
        _ = baked.validateEncoded(bytes[0..length]) catch {};
    }
    const regression = @embedFile("corpus/asset-leading-decimal.json");
    try std.testing.expectError(error.InvalidDecimal, source.validate(regression, std.testing.allocator));
}

test "snapshot replay and command decoders reject arbitrary bounded bytes" {
    var seed: u64 = 0x25_57a7_e;
    var bytes: [1_024]u8 = undefined;
    var entries: [16]replay.Entry = undefined;
    var arena: [1_024]u8 = undefined;
    var commands: [32]gravity.dynamics.world.Command = undefined;
    for (0..12_000) |_| {
        const length: usize = @intCast(next(&seed) % (bytes.len + 1));
        for (bytes[0..length]) |*byte| byte.* = @truncate(next(&seed) >> 17);
        _ = snapshot.decodePipelineSnapshot(bytes[0..length]) catch {};
        _ = replay.decode(bytes[0..length], &entries, &arena) catch {};
        _ = replay.decodeCommands(bytes[0..length], &commands) catch {};
    }
}

test "C ABI rejects null misalignment and length overflow before dereference" {
    var size: u64 = 0;
    var alignment: u32 = 0;
    try std.testing.expectEqual(abi.invalid_argument, abi.gravity_v1_asset_store_memory_required(null, &size, &alignment));
    var desc = abi.AssetStoreDesc{ .struct_size = @sizeOf(abi.AssetStoreDesc), .reserved = 0, .assets = null, .asset_count = 0, .reserved1 = 0 };
    try std.testing.expectEqual(abi.ok, abi.gravity_v1_asset_store_memory_required(&desc, &size, &alignment));
    const memory = try std.testing.allocator.alloc(u8, @intCast(size + alignment));
    defer std.testing.allocator.free(memory);
    var store: *abi.AssetStore = undefined;
    try std.testing.expectEqual(abi.misaligned, abi.gravity_v1_asset_store_init(@ptrCast(memory.ptr + 1), size, &desc, &store));
    try std.testing.expectEqual(abi.invalid_argument, abi.gravity_v1_asset_store_init(null, size, &desc, &store));

    var one: [1]u8 = .{0};
    var blob = abi.AssetBlob{ .data = &one, .length = std.math.maxInt(u64) };
    desc.assets = @ptrCast(&blob);
    desc.asset_count = 1;
    try std.testing.expectEqual(abi.capacity, abi.gravity_v1_asset_store_memory_required(&desc, &size, &alignment));
    try std.testing.expectEqual(abi.invalid_state, abi.gravity_v1_world_snapshot_load(null, &one, std.math.maxInt(u64)));
    try std.testing.expectEqual(abi.invalid_state, abi.gravity_v1_world_query_point(null, null, null, std.math.maxInt(u32), null));
}
