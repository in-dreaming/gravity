//! Deterministic chunk-removal minimizer for Task 25 parser regressions.
const std = @import("std");
const gravity = @import("gravity");

const Kind = enum { asset, asset_tlv, snapshot, replay, commands };
const Probe = struct {
    allocator: std.mem.Allocator,
    entries: []gravity.state.replay.Entry,
    arena: []u8,
    commands: []gravity.dynamics.world.Command,

    fn failure(self: *Probe, kind: Kind, bytes: []const u8) ?[]const u8 {
        switch (kind) {
            .asset => gravity.assets.source.validate(bytes, self.allocator) catch |err| return @errorName(err),
            .asset_tlv => _ = gravity.geometry.baked.validateEncoded(bytes) catch |err| return @errorName(err),
            .snapshot => _ = gravity.state.snapshot.decodePipelineSnapshot(bytes) catch |err| return @errorName(err),
            .replay => _ = gravity.state.replay.decode(bytes, self.entries, self.arena) catch |err| return @errorName(err),
            .commands => _ = gravity.state.replay.decodeCommands(bytes, self.commands) catch |err| return @errorName(err),
        }
        return null;
    }
};

fn parseKind(text: []const u8) ?Kind {
    inline for (@typeInfo(Kind).@"enum".fields) |field| if (std.mem.eql(u8, text, field.name)) return @enumFromInt(field.value);
    return null;
}

pub fn main(init: std.process.Init) !void {
    var args = try init.minimal.args.iterateAllocator(init.gpa);
    defer args.deinit();
    _ = args.next();
    const kind = parseKind(args.next() orelse return error.InvalidArguments) orelse return error.InvalidArguments;
    const input_path = args.next() orelse return error.InvalidArguments;
    const output_path = args.next() orelse return error.InvalidArguments;
    if (args.next() != null) return error.InvalidArguments;
    const input = try std.Io.Dir.cwd().readFileAlloc(init.io, input_path, init.gpa, .limited(16 * 1024 * 1024));
    defer init.gpa.free(input);
    const current = try init.gpa.dupe(u8, input);
    defer init.gpa.free(current);
    const scratch = try init.gpa.alloc(u8, input.len);
    defer init.gpa.free(scratch);
    const arena = try init.gpa.alloc(u8, input.len);
    defer init.gpa.free(arena);
    const entries = try init.gpa.alloc(gravity.state.replay.Entry, @min(input.len / 28 + 1, 65_536));
    defer init.gpa.free(entries);
    const commands = try init.gpa.alloc(gravity.dynamics.world.Command, @min(input.len / 8 + 1, 65_536));
    defer init.gpa.free(commands);
    var probe = Probe{ .allocator = init.gpa, .entries = entries, .arena = arena, .commands = commands };
    const wanted = probe.failure(kind, current) orelse return error.InputDoesNotFail;
    var length = current.len;
    var chunk = std.math.ceilPowerOfTwo(usize, @max(length, 1)) catch return error.InputTooLarge;
    while (chunk != 0) : (chunk /= 2) {
        var at: usize = 0;
        while (at < length) {
            const removed = @min(chunk, length - at);
            @memcpy(scratch[0..at], current[0..at]);
            @memcpy(scratch[at .. length - removed], current[at + removed .. length]);
            const candidate = scratch[0 .. length - removed];
            if (probe.failure(kind, candidate)) |found| {
                if (std.mem.eql(u8, found, wanted)) {
                    @memcpy(current[0..candidate.len], candidate);
                    length = candidate.len;
                    at = 0;
                    continue;
                }
            }
            at += removed;
        }
    }
    try std.Io.Dir.cwd().writeFile(init.io, .{ .sub_path = output_path, .data = current[0..length] });
    std.debug.print("minimized {s}: {d} -> {d} bytes, failure={s}\n", .{ @tagName(kind), input.len, length, wanted });
}
