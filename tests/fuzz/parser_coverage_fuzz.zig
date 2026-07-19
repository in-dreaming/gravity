//! Single-target Zig coverage-guided harness for all untrusted wire parsers.
const std = @import("std");
const gravity = @import("gravity");

const source = gravity.assets.source;
const baked = gravity.geometry.baked;
const snapshot = gravity.state.snapshot;
const replay = gravity.state.replay;

fn fuzzUntrustedParsers(_: void, smith: *std.testing.Smith) !void {
    var bytes: [1_024]u8 = undefined;
    const length = smith.slice(&bytes);
    const input = bytes[0..length];
    var entries: [16]replay.Entry = undefined;
    var arena: [1_024]u8 = undefined;
    var commands: [32]gravity.dynamics.world.Command = undefined;
    _ = source.validate(input, std.testing.allocator) catch {};
    _ = baked.validateEncoded(input) catch {};
    _ = snapshot.decodePipelineSnapshot(input) catch {};
    _ = replay.decode(input, &entries, &arena) catch {};
    _ = replay.decodeCommands(input, &commands) catch {};
}

test "coverage-guided untrusted parser target" {
    try std.testing.fuzz({}, fuzzUntrustedParsers, .{});
}
