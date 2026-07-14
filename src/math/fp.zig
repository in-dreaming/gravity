//! Deterministic Q32.32 scalar arithmetic.  No operation in this module uses
//! floating point or platform math routines.
const std = @import("std");

pub const fractional_bits: u6 = 32;
pub const scale: i128 = @as(i128, 1) << fractional_bits;

/// Fault values are deliberately ordered by their stable wire value.  A status
/// records the first operation fault, never a global or thread-local value.
pub const MathFault = enum(u8) {
    none = 0,
    overflow = 1,
    divide_by_zero = 2,
    negative_sqrt = 3,
    invalid_decimal = 4,
};

pub const MathStatus = struct {
    fault: MathFault = .none,

    pub fn record(self: *MathStatus, fault: MathFault) void {
        if (self.fault == .none) self.fault = fault;
    }

    pub fn clear(self: *MathStatus) void {
        self.fault = .none;
    }
};

pub const Fp = struct {
    raw: i64,

    pub const zero = Fp{ .raw = 0 };
    pub const one = Fp{ .raw = @as(i64, 1) << fractional_bits };
    pub const min = Fp{ .raw = std.math.minInt(i64) };
    pub const max = Fp{ .raw = std.math.maxInt(i64) };

    pub fn fromInt(value: i32) Fp {
        return .{ .raw = @as(i64, value) << fractional_bits };
    }

    pub fn add(a: Fp, b: Fp, status: *MathStatus) Fp {
        const result = @addWithOverflow(a.raw, b.raw);
        if (result[1] == 0) return .{ .raw = result[0] };
        status.record(.overflow);
        return if (a.raw >= 0) max else min;
    }

    pub fn sub(a: Fp, b: Fp, status: *MathStatus) Fp {
        const result = @subWithOverflow(a.raw, b.raw);
        if (result[1] == 0) return .{ .raw = result[0] };
        status.record(.overflow);
        return if (a.raw >= 0) max else min;
    }

    pub fn neg(a: Fp, status: *MathStatus) Fp {
        if (a.raw != std.math.minInt(i64)) return .{ .raw = -a.raw };
        status.record(.overflow);
        return max;
    }

    pub fn abs(a: Fp, status: *MathStatus) Fp {
        return if (a.raw < 0) a.neg(status) else a;
    }

    pub fn mul(a: Fp, b: Fp, status: *MathStatus) Fp {
        return narrowRounded(@as(i128, a.raw) * @as(i128, b.raw), status);
    }

    pub fn div(a: Fp, b: Fp, status: *MathStatus) Fp {
        if (b.raw == 0) {
            status.record(.divide_by_zero);
            if (a.raw > 0) return max;
            if (a.raw < 0) return min;
            return zero;
        }
        return narrowI128(roundDivTiesEven(@as(i128, a.raw) << fractional_bits, @as(i128, b.raw)), status);
    }

    pub fn reciprocal(a: Fp, status: *MathStatus) Fp {
        return one.div(a, status);
    }

    /// Converts the exact integer ratio numerator / denominator to Q32.32.
    /// A zero denominator follows the same signed saturation rule as div.
    pub fn fromRatio(numerator: i64, denominator: i64, status: *MathStatus) Fp {
        if (denominator == 0) {
            status.record(.divide_by_zero);
            if (numerator > 0) return max;
            if (numerator < 0) return min;
            return zero;
        }
        return narrowI128(roundDivTiesEven(@as(i128, numerator) << fractional_bits, @as(i128, denominator)), status);
    }

    /// Integer restoring square root of raw * 2^32, rounded ties-to-even.
    pub fn sqrt(a: Fp, status: *MathStatus) Fp {
        if (a.raw < 0) {
            status.record(.negative_sqrt);
            return zero;
        }
        const radicand: u128 = @as(u128, @intCast(a.raw)) << fractional_bits;
        var root = isqrt(radicand);
        const lower = root * root;
        const upper = (root + 1) * (root + 1);
        const below = radicand - lower;
        const above = upper - radicand;
        if (below > above or (below == above and (root & 1) != 0)) root += 1;
        return .{ .raw = @intCast(root) };
    }

    /// Parses only -?[0-9]+(\.[0-9]+)?.  The accepted practical input limit is
    /// 36 significant decimal digits, sufficient to decide every Q32.32 value.
    pub fn parseCanonicalDecimal(text: []const u8, status: *MathStatus) Fp {
        if (text.len == 0) return invalidDecimal(status);
        var at: usize = 0;
        var negative = false;
        if (text[at] == '-') {
            negative = true;
            at += 1;
            if (at == text.len) return invalidDecimal(status);
        }

        var integer: i128 = 0;
        var saw_integer = false;
        var point_at: ?usize = null;
        while (at < text.len) : (at += 1) {
            const c = text[at];
            if (c == '.') {
                if (point_at != null) return invalidDecimal(status);
                point_at = at;
                continue;
            }
            if (c < '0' or c > '9') return invalidDecimal(status);
            if (point_at == null) {
                saw_integer = true;
                const next = @mulWithOverflow(integer, @as(i128, 10));
                if (next[1] != 0) return decimalOverflow(negative, status);
                const with_digit = @addWithOverflow(next[0], @as(i128, c - '0'));
                if (with_digit[1] != 0) return decimalOverflow(negative, status);
                integer = with_digit[0];
            }
        }
        if (!saw_integer or text[text.len - 1] == '.') return invalidDecimal(status);
        const shifted = @mulWithOverflow(integer, scale);
        if (shifted[1] != 0) return decimalOverflow(negative, status);
        var rounded = shifted[0];
        if (point_at) |point| {
            const fraction = text[point + 1 ..];
            const fraction_raw = decimalFractionRaw(fraction, status) orelse return zero;
            const total = @addWithOverflow(rounded, fraction_raw);
            if (total[1] != 0) return decimalOverflow(negative, status);
            rounded = total[0];
        }
        if (negative) rounded = -rounded;
        return narrowI128(rounded, status);
    }

    /// Writes an exact, locale-independent terminating decimal and removes
    /// unnecessary trailing fractional zeroes.
    pub fn formatCanonical(self: Fp, buffer: []u8) ?[]const u8 {
        var value: i128 = self.raw;
        const negative = value < 0;
        if (negative) value = -value;
        const integer = @divTrunc(value, scale);
        var fraction: i128 = @rem(value, scale);
        var out: usize = 0;
        if (negative) {
            if (out == buffer.len) return null;
            buffer[out] = '-';
            out += 1;
        }
        var integer_storage: [32]u8 = undefined;
        const integer_text = std.fmt.bufPrint(&integer_storage, "{d}", .{integer}) catch return null;
        if (out + integer_text.len > buffer.len) return null;
        @memcpy(buffer[out .. out + integer_text.len], integer_text);
        out += integer_text.len;
        if (fraction == 0) return buffer[0..out];
        if (out == buffer.len) return null;
        buffer[out] = '.';
        out += 1;
        var fraction_storage: [32]u8 = undefined;
        var count: usize = 0;
        while (fraction != 0 and count < fraction_storage.len) : (count += 1) {
            fraction *= 10;
            const digit = @divTrunc(fraction, scale);
            fraction = @rem(fraction, scale);
            fraction_storage[count] = @intCast('0' + digit);
        }
        while (count > 0 and fraction_storage[count - 1] == '0') count -= 1;
        if (out + count > buffer.len) return null;
        @memcpy(buffer[out .. out + count], fraction_storage[0..count]);
        return buffer[0 .. out + count];
    }
};

pub fn roundDivTiesEven(numerator: i128, denominator: i128) i128 {
    std.debug.assert(denominator != 0);
    var quotient = @divTrunc(numerator, denominator);
    const remainder = @rem(numerator, denominator);
    const abs_remainder = if (remainder < 0) -remainder else remainder;
    const abs_denominator = if (denominator < 0) -denominator else denominator;
    const comparison = abs_remainder * 2;
    if (comparison > abs_denominator or (comparison == abs_denominator and (quotient & 1) != 0)) {
        quotient += if ((numerator < 0) != (denominator < 0)) -1 else 1;
    }
    return quotient;
}

pub fn narrowI128(value: i128, status: *MathStatus) Fp {
    if (value > std.math.maxInt(i64)) {
        status.record(.overflow);
        return Fp.max;
    }
    if (value < std.math.minInt(i64)) {
        status.record(.overflow);
        return Fp.min;
    }
    return .{ .raw = @intCast(value) };
}

pub fn narrowRounded(unscaled: i128, status: *MathStatus) Fp {
    return narrowI128(roundDivTiesEven(unscaled, scale), status);
}

fn isqrt(value: u128) u128 {
    var remainder: u128 = 0;
    var root: u128 = 0;
    var bit: i16 = 126;
    while (bit >= 0) : (bit -= 2) {
        root <<= 1;
        remainder = (remainder << 2) | ((value >> @intCast(bit)) & 3);
        const candidate = (root << 1) | 1;
        if (remainder >= candidate) {
            remainder -= candidate;
            root += 1;
        }
    }
    return root;
}

fn invalidDecimal(status: *MathStatus) Fp {
    status.record(.invalid_decimal);
    return Fp.zero;
}

fn decimalOverflow(negative: bool, status: *MathStatus) Fp {
    status.record(.overflow);
    return if (negative) Fp.min else Fp.max;
}

/// Converts a finite decimal fraction to its rounded Q0.32 raw value by
/// decimal long multiplication.  This avoids constructing a 10^32 * 2^32
/// intermediate (which does not fit i128) and is still wholly integer based.
fn decimalFractionRaw(text: []const u8, status: *MathStatus) ?i128 {
    if (text.len > 36) {
        status.record(.invalid_decimal);
        return null;
    }
    var digits: [36]u8 = undefined;
    for (text, 0..) |c, index| digits[index] = c - '0';
    var result: i128 = 0;
    var bit: u6 = 0;
    while (bit < fractional_bits) : (bit += 1) {
        var carry: u8 = 0;
        var index = text.len;
        while (index > 0) {
            index -= 1;
            const doubled = digits[index] * 2 + carry;
            digits[index] = doubled % 10;
            carry = doubled / 10;
        }
        result = (result << 1) | carry;
    }
    var carry: u8 = 0;
    var index = text.len;
    while (index > 0) {
        index -= 1;
        const doubled = digits[index] * 2 + carry;
        digits[index] = doubled % 10;
        carry = doubled / 10;
    }
    var more = false;
    for (digits[0..text.len]) |digit| more = more or digit != 0;
    if (carry != 0 and (more or (result & 1) != 0)) result += 1;
    return result;
}
