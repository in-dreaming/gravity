//! Wide Q64.64 accumulators.  Construction is controlled so callers cannot
//! silently narrow products before a full expression has been accumulated.
const fp = @import("fp.zig");

pub const WideScalar = struct {
    raw: i128,

    pub fn zero() WideScalar {
        return .{ .raw = 0 };
    }

    pub fn product(a: fp.Fp, b: fp.Fp) WideScalar {
        return .{ .raw = @as(i128, a.raw) * @as(i128, b.raw) };
    }

    pub fn add(self: WideScalar, other: WideScalar, status: *fp.MathStatus) WideScalar {
        const result = @addWithOverflow(self.raw, other.raw);
        if (result[1] == 0) return .{ .raw = result[0] };
        status.record(.overflow);
        return .{ .raw = if (self.raw >= 0) @import("std").math.maxInt(i128) else @import("std").math.minInt(i128) };
    }

    pub fn sub(self: WideScalar, other: WideScalar, status: *fp.MathStatus) WideScalar {
        const result = @subWithOverflow(self.raw, other.raw);
        if (result[1] == 0) return .{ .raw = result[0] };
        status.record(.overflow);
        return .{ .raw = if (self.raw >= 0) @import("std").math.maxInt(i128) else @import("std").math.minInt(i128) };
    }

    pub fn addProduct(self: WideScalar, a: fp.Fp, b: fp.Fp, status: *fp.MathStatus) WideScalar {
        return self.add(product(a, b), status);
    }

    /// The sole conversion from a Q64.64 expression to Q32.32.
    pub fn narrow(self: WideScalar, status: *fp.MathStatus) fp.Fp {
        return fp.narrowRounded(self.raw, status);
    }
};

pub fn dot3(a: [3]fp.Fp, b: [3]fp.Fp, status: *fp.MathStatus) fp.Fp {
    var sum = WideScalar.zero();
    inline for (0..3) |index| sum = sum.addProduct(a[index], b[index], status);
    return sum.narrow(status);
}

/// Computes a*b-c*d without narrowing either product first.
pub fn differenceOfProducts(a: fp.Fp, b: fp.Fp, c: fp.Fp, d: fp.Fp, status: *fp.MathStatus) fp.Fp {
    return WideScalar.product(a, b).sub(WideScalar.product(c, d), status).narrow(status);
}
