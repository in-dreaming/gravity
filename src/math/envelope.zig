//! Expression-level bounds used by World creation before any state is mutated.
const fp = @import("fp.zig");

pub const ProductEnvelope = struct {
    max_position: fp.Fp,
    max_linear_velocity: fp.Fp,
    max_angular_velocity: fp.Fp,
    max_dynamic_size: fp.Fp,
    max_mass: fp.Fp,

    pub const product_default = ProductEnvelope{
        .max_position = .{ .raw = 1_000_000 << 32 },
        .max_linear_velocity = .{ .raw = 100_000 << 32 },
        .max_angular_velocity = .{ .raw = 1_000 << 32 },
        .max_dynamic_size = .{ .raw = 100_000 << 32 },
        .max_mass = .{ .raw = 1_000_000 << 32 },
    };

    /// Rejects a configuration if any documented position/distance²/cross/
    /// inertia/angular-impulse/GJK determinant expression exceeds i128.
    pub fn validate(self: ProductEnvelope) error{InvalidEnvelope}!void {
        const values = [_]fp.Fp{ self.max_position, self.max_linear_velocity, self.max_angular_velocity, self.max_dynamic_size, self.max_mass };
        for (values) |value| if (value.raw < 0) return error.InvalidEnvelope;
        try squareSum3(self.max_position.raw);
        try squareSum3(self.max_dynamic_size.raw);
        try product3Q32(self.max_mass.raw, self.max_dynamic_size.raw, self.max_dynamic_size.raw); // inertia
        try product3Q32(self.max_mass.raw, self.max_linear_velocity.raw, self.max_dynamic_size.raw); // angular impulse
        try product3Q32(self.max_position.raw, self.max_position.raw, self.max_position.raw); // GJK/EPA determinant term
    }
};

fn squareSum3(raw: i64) error{InvalidEnvelope}!void {
    const square = @mulWithOverflow(@as(i128, raw), @as(i128, raw));
    if (square[1] != 0) return error.InvalidEnvelope;
    const total = @mulWithOverflow(square[0], @as(i128, 3));
    if (total[1] != 0) return error.InvalidEnvelope;
}

fn product3Q32(a: i64, b: i64, c: i64) error{InvalidEnvelope}!void {
    const first = @mulWithOverflow(@as(i128, a), @as(i128, b));
    if (first[1] != 0) return error.InvalidEnvelope;
    // The value is a Q64.64 product.  Scaling it back before the next factor
    // preserves the expression's physical Q32.32 unit and avoids claiming
    // that a representable default envelope is impossible.
    const normalized = @divTrunc(first[0], fp.scale);
    const second = @mulWithOverflow(normalized, @as(i128, c));
    if (second[1] != 0) return error.InvalidEnvelope;
}
