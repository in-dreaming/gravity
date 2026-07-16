//! Deterministic structural diff for canonical GRAVSNAP data.
const std = @import("std");
const codec = @import("codec.zig");
const snapshot = @import("snapshot.zig");

pub const Error = codec.Error || snapshot.Error || error{DifferentEnvelope};
pub const Difference = struct {
    section: ?u16,
    /// Stable owner identity when the changed field belongs to a body,
    /// collider, contact manifold, or joint. Header and section framing have
    /// no owner, while sleep/CCD are reported by their body-slot field.
    id: ?u64,
    /// Stable field path. `payload_byte` identifies the exact canonical byte
    /// when a section-specific semantic decoder is unavailable.
    field: Field,
    offset: usize,
    left: ?u8,
    right: ?u8,
};
pub const Field = enum {
    header,
    section_id,
    section_length,
    pipeline_tick,
    pipeline_fault,
    body_count,
    body_settings,
    body_slot,
    body_alive,
    body_retired,
    body_generation,
    body_type,
    body_position_x,
    body_position_y,
    body_position_z,
    body_orientation_x,
    body_orientation_y,
    body_orientation_z,
    body_orientation_w,
    body_linear_velocity_x,
    body_linear_velocity_y,
    body_linear_velocity_z,
    body_angular_velocity_x,
    body_angular_velocity_y,
    body_angular_velocity_z,
    body_inverse_mass,
    body_inverse_inertia,
    body_force,
    body_torque,
    body_dof_locks,
    body_has_target,
    body_target_position,
    body_target_orientation,
    collider_presence,
    collider_slot,
    collider_alive,
    collider_retired,
    collider_generation,
    collider_body,
    collider_local_transform,
    collider_shape_kind,
    collider_shape_payload,
    collider_material,
    collider_filter,
    collider_sensor,
    collider_enabled,
    collider_revision,
    contact_patch,
    contact_key,
    contact_normal,
    contact_sensor,
    contact_point_count,
    contact_point,
    joint_slot,
    joint_alive,
    joint_retired,
    joint_generation,
    joint_kind,
    joint_body_a,
    joint_body_b,
    joint_frame_a,
    joint_frame_b,
    joint_reference,
    joint_limit,
    joint_motor,
    joint_spring,
    joint_cone_twist,
    joint_limit_state,
    joint_impulse,
    sleep_slot,
    sleep_awake,
    sleep_counter,
    sleep_reason,
    ccd_enabled,
    payload_byte,
};

/// Finds the earliest difference in canonical order. This is intentionally
/// allocation-free and also works for future optional sections.
pub fn first(left: []const u8, right: []const u8) Error!?Difference {
    var l = codec.Reader.init(left);
    var r = codec.Reader.init(right);
    const lh = try snapshot.readHeader(&l);
    const rh = try snapshot.readHeader(&r);
    if (!std.meta.eql(lh, rh)) return .{ .section = null, .id = null, .field = .header, .offset = 0, .left = null, .right = null };
    var lc = Sections{ .reader = &l };
    var rc = Sections{ .reader = &r };
    try lc.begin();
    try rc.begin();
    while (lc.remaining != 0 and rc.remaining != 0) {
        const ls = try lc.next();
        const rs = try rc.next();
        if (ls.id != rs.id) return .{ .section = null, .id = null, .field = .section_id, .offset = 0, .left = @intCast(ls.id & 0xff), .right = @intCast(rs.id & 0xff) };
        if (ls.payload.len != rs.payload.len) return .{ .section = ls.id, .id = null, .field = .section_length, .offset = 0, .left = null, .right = null };
        for (ls.payload, rs.payload, 0..) |a, b, i| if (a != b) return .{ .section = ls.id, .id = ownerId(ls.id, ls.payload, i), .field = semanticField(ls.id, ls.payload, i), .offset = i, .left = a, .right = b };
    }
    if (lc.remaining != rc.remaining) return .{ .section = null, .id = null, .field = .section_length, .offset = 0, .left = null, .right = null };
    try l.finish();
    try r.finish();
    return null;
}
/// Extracts the stable owner of a changed canonical field without decoding it
/// into World state. A malformed payload is already rejected by snapshot load;
/// diagnostics deliberately return no ID rather than inventing one.
fn ownerId(section: u16, payload: []const u8, offset: usize) ?u64 {
    return switch (section) {
        snapshot.bodies_section => slotOwner(payload, offset, 60, 273),
        snapshot.colliders_section => colliderOwner(payload, offset),
        snapshot.contacts_section => contactOwner(payload, offset),
        snapshot.joints_section => slotOwner(payload, offset, 4, 358),
        else => null,
    };
}
fn slotOwner(payload: []const u8, offset: usize, start: usize, live_size: usize) ?u64 {
    if (payload.len < start or offset < start) return null;
    const count = std.mem.readInt(u32, @ptrCast(payload[0..4].ptr), .little);
    var at = start;
    for (0..count) |index| {
        if (at + 6 > payload.len) return null;
        const alive = payload[at] == 1;
        const generation = std.mem.readInt(u32, @ptrCast(payload[at + 2 .. at + 6].ptr), .little);
        const end = at + if (alive) live_size else 6;
        if (end > payload.len) return null;
        if (offset < end) return (@as(u64, generation) << 32) | @as(u64, @intCast(index));
        at = end;
    }
    return null;
}
fn colliderOwner(payload: []const u8, offset: usize) ?u64 {
    if (payload.len < 5 or payload[0] != 1 or offset < 5) return null;
    const count = std.mem.readInt(u32, @ptrCast(payload[1..5].ptr), .little);
    var at: usize = 5;
    for (0..count) |index| {
        if (at + 6 > payload.len) return null;
        const alive = payload[at] == 1;
        const generation = std.mem.readInt(u32, @ptrCast(payload[at + 2 .. at + 6].ptr), .little);
        var end = at + 6;
        if (alive) {
            // owner body + transform + kind, then kind-specific shape data.
            if (end + 65 > payload.len) return null;
            const shape_bytes: usize = switch (payload[end + 64]) {
                0 => 8,
                1 => 24,
                2 => 16,
                3, 4, 5, 6 => 20,
                else => return null,
            };
            end += 65 + shape_bytes + 30;
        }
        if (end > payload.len) return null;
        if (offset < end) return (@as(u64, generation) << 32) | @as(u64, @intCast(index));
        at = end;
    }
    return null;
}
fn contactOwner(payload: []const u8, offset: usize) ?u64 {
    if (payload.len < 4 or offset < 4) return null;
    const count = std.mem.readInt(u32, @ptrCast(payload[0..4].ptr), .little);
    var at: usize = 4;
    for (0..count) |_| {
        if (at + 18 > payload.len) return null;
        const collider_a = std.mem.readInt(u64, @ptrCast(payload[at .. at + 8].ptr), .little);
        var end = at + 16;
        const path_a = payload[end];
        end += 1 + @as(usize, path_a) * 4;
        if (end >= payload.len) return null;
        const path_b = payload[end];
        end += 1 + @as(usize, path_b) * 4;
        // primitive/revision fields, normal, sensor, point count.
        end += 24 + 24 + 1;
        if (end >= payload.len) return null;
        const points = payload[end];
        end += 1 + @as(usize, points) * 32;
        if (end > payload.len) return null;
        if (offset < end) return collider_a;
        at = end;
    }
    return null;
}
fn semanticField(section: u16, payload: []const u8, offset: usize) Field {
    return switch (section) {
        snapshot.pipeline_section => if (offset < 8) .pipeline_tick else .pipeline_fault,
        snapshot.bodies_section => bodyField(payload, offset),
        snapshot.colliders_section => colliderField(payload, offset),
        snapshot.contacts_section => contactField(payload, offset),
        snapshot.joints_section => jointField(payload, offset),
        snapshot.sleep_section => sleepField(offset),
        snapshot.ccd_section => .ccd_enabled,
        else => .payload_byte,
    };
}

const SlotLocation = struct { relative: usize, alive: bool };
fn fixedSlot(payload: []const u8, offset: usize, start: usize, live_size: usize) ?SlotLocation {
    if (payload.len < 4 or offset < start) return null;
    const count = std.mem.readInt(u32, @ptrCast(payload[0..4].ptr), .little);
    var at = start;
    for (0..count) |_| {
        if (at + 6 > payload.len) return null;
        const alive = payload[at] == 1;
        const end = at + if (alive) live_size else 6;
        if (end > payload.len) return null;
        if (offset < end) return .{ .relative = offset - at, .alive = alive };
        at = end;
    }
    return null;
}
fn component(relative: usize, start: usize, fields: [3]Field, fallback: Field) Field {
    if (relative < start or relative >= start + 24) return fallback;
    return fields[(relative - start) / 8];
}
fn quatComponent(relative: usize, start: usize, fields: [4]Field, fallback: Field) Field {
    if (relative < start or relative >= start + 32) return fallback;
    return fields[(relative - start) / 8];
}
fn bodyField(payload: []const u8, offset: usize) Field {
    if (offset < 4) return .body_count;
    if (offset < 60) return .body_settings;
    const slot = fixedSlot(payload, offset, 60, 273) orelse return .body_slot;
    const r = slot.relative;
    if (r == 0) return .body_alive;
    if (r == 1) return .body_retired;
    if (r < 6) return .body_generation;
    if (!slot.alive) return .body_slot;
    if (r == 6) return .body_type;
    if (r < 31) return component(r, 7, .{ .body_position_x, .body_position_y, .body_position_z }, .body_slot);
    if (r < 63) return quatComponent(r, 31, .{ .body_orientation_x, .body_orientation_y, .body_orientation_z, .body_orientation_w }, .body_slot);
    if (r < 87) return component(r, 63, .{ .body_linear_velocity_x, .body_linear_velocity_y, .body_linear_velocity_z }, .body_slot);
    if (r < 111) return component(r, 87, .{ .body_angular_velocity_x, .body_angular_velocity_y, .body_angular_velocity_z }, .body_slot);
    if (r < 119) return .body_inverse_mass;
    if (r < 167) return .body_inverse_inertia;
    if (r < 191) return .body_force;
    if (r < 215) return .body_torque;
    if (r == 215) return .body_dof_locks;
    if (r == 216) return .body_has_target;
    if (r < 241) return .body_target_position;
    if (r < 273) return .body_target_orientation;
    return .body_slot;
}
fn colliderField(payload: []const u8, offset: usize) Field {
    if (offset == 0) return .collider_presence;
    if (payload.len < 5 or payload[0] != 1 or offset < 5) return .collider_slot;
    const count = std.mem.readInt(u32, @ptrCast(payload[1..5].ptr), .little);
    var at: usize = 5;
    for (0..count) |_| {
        if (at + 6 > payload.len) return .collider_slot;
        const alive = payload[at] == 1;
        var end = at + 6;
        if (!alive) {
            if (offset < end) return switch (offset - at) {
                0 => .collider_alive,
                1 => .collider_retired,
                else => .collider_generation,
            };
            at = end;
            continue;
        }
        if (end + 65 > payload.len) return .collider_slot;
        const shape_bytes: usize = switch (payload[end + 64]) {
            0 => 8,
            1 => 24,
            2 => 16,
            3, 4, 5, 6 => 20,
            else => return .collider_slot,
        };
        end += 65 + shape_bytes + 30;
        if (end > payload.len) return .collider_slot;
        if (offset < end) {
            const r = offset - at;
            if (r == 0) return .collider_alive;
            if (r == 1) return .collider_retired;
            if (r < 6) return .collider_generation;
            if (r < 14) return .collider_body;
            if (r < 70) return .collider_local_transform;
            if (r == 70) return .collider_shape_kind;
            const after_shape = 71 + shape_bytes;
            if (r < after_shape) return .collider_shape_payload;
            if (r < after_shape + 16) return .collider_material;
            if (r < after_shape + 28) return .collider_filter;
            if (r == after_shape + 28) return .collider_sensor;
            if (r == after_shape + 29) return .collider_enabled;
            return .collider_revision;
        }
        at = end;
    }
    return .collider_slot;
}
fn contactField(payload: []const u8, offset: usize) Field {
    if (payload.len < 4 or offset < 4) return .contact_patch;
    const count = std.mem.readInt(u32, @ptrCast(payload[0..4].ptr), .little);
    var at: usize = 4;
    for (0..count) |_| {
        if (at + 18 > payload.len) return .contact_patch;
        var cursor = at + 16;
        const path_a = payload[cursor];
        cursor += 1 + @as(usize, path_a) * 4;
        if (cursor >= payload.len) return .contact_patch;
        const path_b = payload[cursor];
        cursor += 1 + @as(usize, path_b) * 4;
        const key_end = cursor + 24;
        const normal_end = key_end + 24;
        const sensor_end = normal_end + 1;
        if (sensor_end >= payload.len) return .contact_patch;
        const point_count_at = sensor_end;
        const points = payload[point_count_at];
        const end = point_count_at + 1 + @as(usize, points) * 32;
        if (end > payload.len) return .contact_patch;
        if (offset < end) {
            if (offset < key_end) return .contact_key;
            if (offset < normal_end) return .contact_normal;
            if (offset < sensor_end) return .contact_sensor;
            if (offset == point_count_at) return .contact_point_count;
            return .contact_point;
        }
        at = end;
    }
    return .contact_patch;
}
fn jointField(payload: []const u8, offset: usize) Field {
    if (offset < 4) return .joint_slot;
    const slot = fixedSlot(payload, offset, 4, 358) orelse return .joint_slot;
    const r = slot.relative;
    if (r == 0) return .joint_alive;
    if (r == 1) return .joint_retired;
    if (r < 6) return .joint_generation;
    if (!slot.alive) return .joint_slot;
    if (r == 6) return .joint_kind;
    if (r < 15) return .joint_body_a;
    if (r < 23) return .joint_body_b;
    if (r < 95) return .joint_frame_a;
    if (r < 167) return .joint_frame_b;
    if (r < 215) return .joint_reference;
    if (r < 231) return .joint_limit;
    if (r < 247) return .joint_motor;
    if (r < 263) return .joint_spring;
    if (r < 287) return .joint_cone_twist;
    if (r < 294) return .joint_limit_state;
    if (r < 358) return .joint_impulse;
    return .joint_slot;
}
fn sleepField(offset: usize) Field {
    if (offset < 4) return .sleep_slot;
    return switch ((offset - 4) % 6) {
        0 => .sleep_awake,
        1...4 => .sleep_counter,
        5 => .sleep_reason,
        else => .sleep_slot,
    };
}
const Sections = struct {
    reader: *codec.Reader,
    remaining: u16 = 0,
    prior: ?u16 = null,
    fn begin(self: *Sections) codec.Error!void {
        if (try self.reader.unsigned(u16) != 1) return error.InvalidVersion;
        self.remaining = try self.reader.unsigned(u16);
    }
    fn next(self: *Sections) codec.Error!codec.Section {
        if (self.remaining == 0) return error.EndOfInput;
        const id = try self.reader.unsigned(u16);
        const length = try self.reader.unsigned(u32);
        if (self.prior) |prior| if (id <= prior) return error.InvalidSectionOrder;
        if (length > codec.max_section_payload) return error.SectionTooLarge;
        self.prior = id;
        self.remaining -= 1;
        const start = self.reader.at;
        try self.reader.skip(length);
        return .{ .id = id, .payload = self.reader.bytes[start .. start + length] };
    }
};
