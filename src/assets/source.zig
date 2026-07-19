//! Strict canonical-JSON gate for offline assets. This parser deliberately
//! accepts no JSON floating-point tokens: physical quantities are strings.
const std = @import("std");

pub const Error = error{ InvalidRoot, InvalidAsset, InvalidKind, DuplicateSourceId, NonCanonicalNumber, InvalidDecimal };

pub fn validate(bytes: []const u8, allocator: std.mem.Allocator) (std.json.ParseError(std.json.Scanner) || std.mem.Allocator.Error || Error)!void {
    var parsed = try std.json.parseFromSlice(std.json.Value, allocator, bytes, .{ .parse_numbers = true });
    defer parsed.deinit();
    try rejectFloatingNumbers(parsed.value);
    const root = switch (parsed.value) {
        .object => |object| object,
        else => return error.InvalidRoot,
    };
    const assets_value = root.get("assets") orelse return error.InvalidRoot;
    const assets = switch (assets_value) {
        .array => |array| array.items,
        else => return error.InvalidRoot,
    };
    var previous: ?u64 = null;
    for (assets) |asset| {
        const object = switch (asset) {
            .object => |value| value,
            else => return error.InvalidAsset,
        };
        const id = integer(object.get("source_id") orelse return error.InvalidAsset) orelse return error.InvalidAsset;
        const kind = string(object.get("kind") orelse return error.InvalidAsset) orelse return error.InvalidAsset;
        if (!validKind(kind)) return error.InvalidKind;
        if (previous) |prior| if (id <= prior) return error.DuplicateSourceId;
        previous = id;
        try validateAssetFields(object);
    }
}

fn rejectFloatingNumbers(value: std.json.Value) Error!void {
    switch (value) {
        .float, .number_string => return error.NonCanonicalNumber,
        .array => |array| for (array.items) |item| try rejectFloatingNumbers(item),
        .object => |object| {
            var it = object.iterator();
            while (it.next()) |entry| try rejectFloatingNumbers(entry.value_ptr.*);
        },
        else => {},
    }
}
fn validateAssetFields(object: std.json.ObjectMap) Error!void {
    var it = object.iterator();
    while (it.next()) |entry| {
        const key = entry.key_ptr.*;
        if (std.mem.eql(u8, key, "source_id") or std.mem.eql(u8, key, "kind")) continue;
        if (isRealKey(key)) {
            const value = string(entry.value_ptr.*) orelse return error.InvalidDecimal;
            if (!decimal(value)) return error.InvalidDecimal;
        }
    }
}
fn integer(value: std.json.Value) ?u64 {
    return switch (value) {
        .integer => |v| if (v >= 0) @intCast(v) else null,
        else => null,
    };
}
fn string(value: std.json.Value) ?[]const u8 {
    return switch (value) {
        .string => |v| v,
        else => null,
    };
}
fn validKind(value: []const u8) bool {
    inline for ([_][]const u8{ "Sphere", "Box", "Capsule", "ConvexHull", "TriangleMesh", "HeightField", "Compound", "Material" }) |kind| if (std.mem.eql(u8, value, kind)) return true;
    return false;
}
fn isRealKey(key: []const u8) bool {
    inline for ([_][]const u8{ "radius", "height", "mass", "friction", "restitution", "x", "y", "z", "w" }) |name| if (std.mem.eql(u8, key, name)) return true;
    return false;
}
fn decimal(value: []const u8) bool {
    if (value.len == 0) return false;
    var at: usize = 0;
    if (value[0] == '-') {
        if (value.len == 1) return false;
        at = 1;
    }
    // Match Fp.parseCanonicalDecimal exactly: the integer component is
    // mandatory, so `.5` and `-.5` are not canonical source values.
    if (value[at] < '0' or value[at] > '9') return false;
    var digits: usize = 0;
    var dot = false;
    while (at < value.len) : (at += 1) switch (value[at]) {
        '0'...'9' => digits += 1,
        '.' => {
            if (dot) return false;
            dot = true;
        },
        else => return false,
    };
    return digits != 0 and value[value.len - 1] != '.';
}
