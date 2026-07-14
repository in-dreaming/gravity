//! Stable generation identifiers and fixed-capacity slot pools.
const std = @import("std");

pub const Id = struct {
    value: u64 = invalid_value,

    pub const invalid_value = std.math.maxInt(u64);
    pub const invalid = Id{};

    pub fn init(slot_index: u32, slot_generation: u32) Id {
        return .{ .value = (@as(u64, slot_generation) << 32) | slot_index };
    }

    pub fn index(self: Id) u32 {
        return @truncate(self.value);
    }

    pub fn generation(self: Id) u32 {
        return @truncate(self.value >> 32);
    }

    pub fn isValid(self: Id) bool {
        return self.value != invalid_value;
    }
};

pub const BodyId = Id;
pub const ColliderId = Id;
pub const JointId = Id;
pub const AssetId = Id;

/// Fixed slot pool. Allocation always chooses the lowest reusable index.
/// A slot whose generation reaches u32 max is permanently retired on delete.
pub fn SlotPool(comptime T: type) type {
    return struct {
        const Self = @This();

        values: []T,
        generations: []u32,
        occupied: []bool,
        retired: []bool,
        live_count: usize = 0,

        pub const Error = error{ OutOfCapacity, InvalidId };

        pub fn init(values: []T, generations: []u32, occupied: []bool, retired: []bool) !Self {
            if (values.len != generations.len or values.len != occupied.len or values.len != retired.len) return error.InvalidId;
            @memset(generations, 0);
            @memset(occupied, false);
            @memset(retired, false);
            return .{ .values = values, .generations = generations, .occupied = occupied, .retired = retired };
        }

        pub fn allocate(self: *Self, value: T) Error!Id {
            for (self.occupied, 0..) |used, index| {
                if (used or self.retired[index]) continue;
                self.values[index] = value;
                self.occupied[index] = true;
                self.live_count += 1;
                return Id.init(@intCast(index), self.generations[index]);
            }
            return error.OutOfCapacity;
        }

        pub fn get(self: *Self, id: Id) ?*T {
            const index: usize = id.index();
            if (index >= self.values.len or !self.occupied[index] or self.generations[index] != id.generation()) return null;
            return &self.values[index];
        }

        pub fn getConst(self: *const Self, id: Id) ?*const T {
            const index: usize = id.index();
            if (index >= self.values.len or !self.occupied[index] or self.generations[index] != id.generation()) return null;
            return &self.values[index];
        }

        pub fn release(self: *Self, id: Id) Error!void {
            const index: usize = id.index();
            if (index >= self.values.len or !self.occupied[index] or self.generations[index] != id.generation()) return error.InvalidId;
            self.occupied[index] = false;
            self.live_count -= 1;
            if (self.generations[index] == std.math.maxInt(u32)) {
                self.retired[index] = true;
            } else {
                self.generations[index] += 1;
            }
        }
    };
}
