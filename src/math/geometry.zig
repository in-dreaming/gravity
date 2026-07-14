//! Deterministic three-dimensional Q32.32 geometry.  This module deliberately
//! contains no floating point operations or platform math calls.
const std = @import("std");
const fp = @import("fp.zig");
const wide = @import("wide.zig");

pub const Fp = fp.Fp;
pub const MathStatus = fp.MathStatus;
pub const pi = Fp{ .raw = 13_493_037_705 }; // nearest-even Q32.32 pi
pub const tau = Fp{ .raw = 26_986_075_410 };

pub const Vec3 = struct {
    x: Fp = Fp.zero,
    y: Fp = Fp.zero,
    z: Fp = Fp.zero,

    pub const zero = Vec3{};
    pub const unit_x = Vec3{ .x = Fp.one };
    pub const unit_y = Vec3{ .y = Fp.one };
    pub const unit_z = Vec3{ .z = Fp.one };

    pub fn add(a: Vec3, b: Vec3, s: *MathStatus) Vec3 {
        return .{ .x = a.x.add(b.x, s), .y = a.y.add(b.y, s), .z = a.z.add(b.z, s) };
    }
    pub fn sub(a: Vec3, b: Vec3, s: *MathStatus) Vec3 {
        return .{ .x = a.x.sub(b.x, s), .y = a.y.sub(b.y, s), .z = a.z.sub(b.z, s) };
    }
    pub fn scale(a: Vec3, b: Fp, s: *MathStatus) Vec3 {
        return .{ .x = a.x.mul(b, s), .y = a.y.mul(b, s), .z = a.z.mul(b, s) };
    }
    pub fn dot(a: Vec3, b: Vec3, s: *MathStatus) Fp {
        return wide.dot3(.{ a.x, a.y, a.z }, .{ b.x, b.y, b.z }, s);
    }
    pub fn cross(a: Vec3, b: Vec3, s: *MathStatus) Vec3 {
        return .{ .x = wide.differenceOfProducts(a.y, b.z, a.z, b.y, s), .y = wide.differenceOfProducts(a.z, b.x, a.x, b.z, s), .z = wide.differenceOfProducts(a.x, b.y, a.y, b.x, s) };
    }
    pub fn lengthSquared(a: Vec3, s: *MathStatus) Fp {
        return a.dot(a, s);
    }
    pub fn normalize(a: Vec3, s: *MathStatus) struct { value: Vec3, valid: bool } {
        const length = a.lengthSquared(s).sqrt(s);
        if (length.raw == 0) {
            s.record(.negative_sqrt);
            return .{ .value = zero, .valid = false };
        }
        return .{ .value = a.scale(Fp.one.div(length, s), s), .valid = true };
    }
};

pub const Mat3 = struct {
    // Row-major matrix; multiplication is matrix * column-vector.
    m: [9]Fp,
    pub const identity = Mat3{ .m = .{ Fp.one, Fp.zero, Fp.zero, Fp.zero, Fp.one, Fp.zero, Fp.zero, Fp.zero, Fp.one } };
    pub fn at(self: Mat3, row: usize, col: usize) Fp {
        return self.m[row * 3 + col];
    }
    pub fn mulVec(a: Mat3, b: Vec3, s: *MathStatus) Vec3 {
        return .{ .x = wide.dot3(.{ a.m[0], a.m[1], a.m[2] }, .{ b.x, b.y, b.z }, s), .y = wide.dot3(.{ a.m[3], a.m[4], a.m[5] }, .{ b.x, b.y, b.z }, s), .z = wide.dot3(.{ a.m[6], a.m[7], a.m[8] }, .{ b.x, b.y, b.z }, s) };
    }
    pub fn mul(a: Mat3, b: Mat3, s: *MathStatus) Mat3 {
        var r: Mat3 = undefined;
        inline for (0..3) |i| {
            inline for (0..3) |j| r.m[i * 3 + j] = wide.dot3(.{ a.m[i * 3], a.m[i * 3 + 1], a.m[i * 3 + 2] }, .{ b.m[j], b.m[3 + j], b.m[6 + j] }, s);
        }
        return r;
    }
    pub fn transpose(a: Mat3) Mat3 {
        return .{ .m = .{ a.m[0], a.m[3], a.m[6], a.m[1], a.m[4], a.m[7], a.m[2], a.m[5], a.m[8] } };
    }
    pub fn inverse(a: Mat3, s: *MathStatus) struct { value: Mat3, valid: bool } {
        const c00 = wide.differenceOfProducts(a.m[4], a.m[8], a.m[5], a.m[7], s);
        // These are cofactors of the first *row* (not the first row of the
        // adjugate), so their order is suitable for the determinant expansion.
        const c01 = wide.differenceOfProducts(a.m[5], a.m[6], a.m[3], a.m[8], s);
        const c02 = wide.differenceOfProducts(a.m[3], a.m[7], a.m[4], a.m[6], s);
        const det = wide.dot3(.{ a.m[0], a.m[1], a.m[2] }, .{ c00, c01, c02 }, s);
        // One raw unit is the fixed, protocol-level singularity boundary.
        if (det.raw >= -1 and det.raw <= 1) {
            s.record(.divide_by_zero);
            return .{ .value = identity, .valid = false };
        }
        const inv = Fp.one.div(det, s);
        return .{ .valid = true, .value = .{ .m = .{ c00.mul(inv, s), wide.differenceOfProducts(a.m[2], a.m[7], a.m[1], a.m[8], s).mul(inv, s), wide.differenceOfProducts(a.m[1], a.m[5], a.m[2], a.m[4], s).mul(inv, s), wide.differenceOfProducts(a.m[5], a.m[6], a.m[3], a.m[8], s).mul(inv, s), wide.differenceOfProducts(a.m[0], a.m[8], a.m[2], a.m[6], s).mul(inv, s), wide.differenceOfProducts(a.m[2], a.m[3], a.m[0], a.m[5], s).mul(inv, s), wide.differenceOfProducts(a.m[3], a.m[7], a.m[4], a.m[6], s).mul(inv, s), wide.differenceOfProducts(a.m[1], a.m[6], a.m[0], a.m[7], s).mul(inv, s), wide.differenceOfProducts(a.m[0], a.m[4], a.m[1], a.m[3], s).mul(inv, s) } } };
    }
};

pub const SymmetricMat3 = struct {
    xx: Fp,
    yy: Fp,
    zz: Fp,
    xy: Fp,
    xz: Fp,
    yz: Fp,
    pub fn toMat3(self: SymmetricMat3) Mat3 {
        return .{ .m = .{ self.xx, self.xy, self.xz, self.xy, self.yy, self.yz, self.xz, self.yz, self.zz } };
    }
    pub fn rotate(self: SymmetricMat3, q: Quat, s: *MathStatus) SymmetricMat3 {
        const rotation = q.toMat3(s);
        const m = Mat3.mul(Mat3.mul(rotation, self.toMat3(), s), rotation.transpose(), s);
        return .{ .xx = m.m[0], .yy = m.m[4], .zz = m.m[8], .xy = m.m[1], .xz = m.m[2], .yz = m.m[5] };
    }
};

pub const Quat = struct {
    x: Fp = Fp.zero,
    y: Fp = Fp.zero,
    z: Fp = Fp.zero,
    w: Fp = Fp.one,
    pub const identity = Quat{};
    pub fn canonicalize(q: Quat, s: *MathStatus) Quat {
        var r = q;
        const n = r.normalize(s);
        r = n.value;
        if (!n.valid) return identity;
        if (r.w.raw < 0 or (r.w.raw == 0 and (r.x.raw < 0 or (r.x.raw == 0 and (r.y.raw < 0 or (r.y.raw == 0 and r.z.raw < 0)))))) {
            r.x = r.x.neg(s);
            r.y = r.y.neg(s);
            r.z = r.z.neg(s);
            r.w = r.w.neg(s);
        }
        return r;
    }
    pub fn normalize(q: Quat, s: *MathStatus) struct { value: Quat, valid: bool } {
        const l2 = wide.dot3(.{ q.x, q.y, q.z }, .{ q.x, q.y, q.z }, s).add(q.w.mul(q.w, s), s);
        const l = l2.sqrt(s);
        if (l.raw == 0) {
            s.record(.negative_sqrt);
            return .{ .value = identity, .valid = false };
        }
        const k = Fp.one.div(l, s);
        return .{ .value = .{ .x = q.x.mul(k, s), .y = q.y.mul(k, s), .z = q.z.mul(k, s), .w = q.w.mul(k, s) }, .valid = true };
    }
    /// Hamilton product.  It composes local-to-world rotations: parent * local.
    pub fn mul(a: Quat, b: Quat, s: *MathStatus) Quat {
        return canonicalize(.{ .x = a.w.mul(b.x, s).add(a.x.mul(b.w, s), s).add(a.y.mul(b.z, s), s).sub(a.z.mul(b.y, s), s), .y = a.w.mul(b.y, s).sub(a.x.mul(b.z, s), s).add(a.y.mul(b.w, s), s).add(a.z.mul(b.x, s), s), .z = a.w.mul(b.z, s).add(a.x.mul(b.y, s), s).sub(a.y.mul(b.x, s), s).add(a.z.mul(b.w, s), s), .w = a.w.mul(b.w, s).sub(a.x.mul(b.x, s), s).sub(a.y.mul(b.y, s), s).sub(a.z.mul(b.z, s), s) }, s);
    }
    pub fn conjugate(q: Quat, s: *MathStatus) Quat {
        return .{ .x = q.x.neg(s), .y = q.y.neg(s), .z = q.z.neg(s), .w = q.w };
    }
    pub fn rotate(q: Quat, v: Vec3, s: *MathStatus) Vec3 {
        const u = Vec3{ .x = q.x, .y = q.y, .z = q.z };
        const t = u.cross(v, s).scale(Fp.fromInt(2), s);
        return v.add(t.scale(q.w, s), s).add(u.cross(t, s), s);
    }
    pub fn inverseRotate(q: Quat, v: Vec3, s: *MathStatus) Vec3 {
        return q.conjugate(s).rotate(v, s);
    }
    pub fn integrate(q: Quat, omega_world: Vec3, dt: Fp, s: *MathStatus) Quat {
        const h = dt.mul(Fp.fromRatio(1, 2, s), s);
        const dq = Quat{ .x = omega_world.x.mul(q.w, s).add(omega_world.y.mul(q.z, s), s).sub(omega_world.z.mul(q.y, s), s).mul(h, s), .y = omega_world.y.mul(q.w, s).add(omega_world.z.mul(q.x, s), s).sub(omega_world.x.mul(q.z, s), s).mul(h, s), .z = omega_world.z.mul(q.w, s).add(omega_world.x.mul(q.y, s), s).sub(omega_world.y.mul(q.x, s), s).mul(h, s), .w = omega_world.x.mul(q.x, s).add(omega_world.y.mul(q.y, s), s).add(omega_world.z.mul(q.z, s), s).neg(s).mul(h, s) };
        return canonicalize(.{ .x = q.x.add(dq.x, s), .y = q.y.add(dq.y, s), .z = q.z.add(dq.z, s), .w = q.w.add(dq.w, s) }, s);
    }
    pub fn toMat3(q: Quat, s: *MathStatus) Mat3 {
        const two = Fp.fromInt(2);
        return .{ .m = .{ Fp.one.sub(two.mul(q.y.mul(q.y, s).add(q.z.mul(q.z, s), s), s), s), two.mul(q.x.mul(q.y, s).sub(q.z.mul(q.w, s), s), s), two.mul(q.x.mul(q.z, s).add(q.y.mul(q.w, s), s), s), two.mul(q.x.mul(q.y, s).add(q.z.mul(q.w, s), s), s), Fp.one.sub(two.mul(q.x.mul(q.x, s).add(q.z.mul(q.z, s), s), s), s), two.mul(q.y.mul(q.z, s).sub(q.x.mul(q.w, s), s), s), two.mul(q.x.mul(q.z, s).sub(q.y.mul(q.w, s), s), s), two.mul(q.y.mul(q.z, s).add(q.x.mul(q.w, s), s), s), Fp.one.sub(two.mul(q.x.mul(q.x, s).add(q.y.mul(q.y, s), s), s), s) } };
    }
};

pub const Transform3 = struct {
    position: Vec3 = Vec3.zero,
    orientation: Quat = Quat.identity,
    pub fn apply(self: Transform3, p: Vec3, s: *MathStatus) Vec3 {
        return self.orientation.rotate(p, s).add(self.position, s);
    }
    pub fn inverseApply(self: Transform3, p: Vec3, s: *MathStatus) Vec3 {
        return self.orientation.inverseRotate(p.sub(self.position, s), s);
    }
};
pub const Plane = struct {
    normal: Vec3,
    offset: Fp,
    pub fn signedDistance(self: Plane, p: Vec3, s: *MathStatus) Fp {
        return self.normal.dot(p, s).add(self.offset, s);
    }
};
pub const Ray = struct { origin: Vec3, direction: Vec3 };
pub const Aabb3 = struct {
    min: Vec3,
    max: Vec3,
    pub fn overlaps(a: Aabb3, b: Aabb3) bool {
        return a.min.x.raw <= b.max.x.raw and a.max.x.raw >= b.min.x.raw and a.min.y.raw <= b.max.y.raw and a.max.y.raw >= b.min.y.raw and a.min.z.raw <= b.max.z.raw and a.max.z.raw >= b.min.z.raw;
    }
    pub fn swept(a: Aabb3, delta: Vec3, s: *MathStatus) Aabb3 {
        return .{ .min = .{ .x = if (delta.x.raw < 0) a.min.x.add(delta.x, s) else a.min.x, .y = if (delta.y.raw < 0) a.min.y.add(delta.y, s) else a.min.y, .z = if (delta.z.raw < 0) a.min.z.add(delta.z, s) else a.min.z }, .max = .{ .x = if (delta.x.raw > 0) a.max.x.add(delta.x, s) else a.max.x, .y = if (delta.y.raw > 0) a.max.y.add(delta.y, s) else a.max.y, .z = if (delta.z.raw > 0) a.max.z.add(delta.z, s) else a.max.z } };
    }
};

/// 1024 samples at i*tau/1024.  CORDIC uses exactly 96 iterations and its
/// Q32.224 working representation; output is rounded-to-even Q32.32.
pub const trig_table_len = 1024;
pub const TrigSample = struct { sin: Fp, cos: Fp };
pub const TrigTable = [trig_table_len]TrigSample;
pub const trig_hash_domain = "gravity/trig-table/v1\x00";

/// BLAKE3 of the frozen little-endian serialized table, including its domain.
pub fn trigTableHash(table: *const TrigTable) [std.crypto.hash.Blake3.digest_length]u8 {
    var hasher = std.crypto.hash.Blake3.init(.{});
    hasher.update(trig_hash_domain);
    var bytes: [16]u8 = undefined;
    for (table) |entry| {
        std.mem.writeInt(i64, bytes[0..8], entry.sin.raw, .little);
        std.mem.writeInt(i64, bytes[8..16], entry.cos.raw, .little);
        hasher.update(&bytes);
    }
    var digest: [std.crypto.hash.Blake3.digest_length]u8 = undefined;
    hasher.final(&digest);
    return digest;
}
pub fn generateTrigTable() TrigTable {
    var r: TrigTable = undefined;
    for (&r, 0..) |*entry, i| {
        const angle = Fp{ .raw = @as(i64, @intCast(@divTrunc(@as(i128, tau.raw) * @as(i128, @intCast(i)), trig_table_len))) };
        entry.* = cordic(angle);
    }
    return r;
}
pub fn cordic(angle: Fp) TrigSample {
    var z: i256 = @as(i256, angle.raw) << 192;
    const half_pi: i256 = @as(i256, pi.raw) << 191;
    const pi_q224: i256 = @as(i256, pi.raw) << 192;
    const full: i256 = @as(i256, tau.raw) << 192;
    while (z > pi_q224) z -= full;
    while (z < -pi_q224) z += full;
    var sign: i256 = 1;
    if (z > half_pi) {
        z -= pi_q224;
        sign = -1;
    } else if (z < -half_pi) {
        z += pi_q224;
        sign = -1;
    }
    var x: i256 = cordic_k;
    var y: i256 = 0;
    inline for (0..96) |i| {
        const shift: @TypeOf(i) = i;
        const a = atan_table[i];
        if (z >= 0) {
            const nx = x - (y >> @intCast(shift));
            y = y + (x >> @intCast(shift));
            x = nx;
            z -= a;
        } else {
            const nx = x + (y >> @intCast(shift));
            y = y - (x >> @intCast(shift));
            x = nx;
            z += a;
        }
    }
    return .{ .sin = fromQ224(y * sign), .cos = fromQ224(x * sign) };
}
/// Deterministic principal angle in [-pi, pi]. Zero/zero has the frozen
/// result zero and records the same invalid-normalization fault boundary used
/// by other direction-derived operations.
pub fn atan2(y_input: Fp, x_input: Fp, status: *MathStatus) Fp {
    if (x_input.raw == 0 and y_input.raw == 0) {
        status.record(.negative_sqrt);
        return .zero;
    }
    var x: i256 = @as(i256, x_input.raw) << 192;
    var y: i256 = @as(i256, y_input.raw) << 192;
    var z: i256 = 0;
    const pi_q224: i256 = @as(i256, pi.raw) << 192;
    if (x < 0) {
        x = -x;
        y = -y;
        z = if (y_input.raw >= 0) pi_q224 else -pi_q224;
    }
    inline for (0..96) |i| {
        const shift: @TypeOf(i) = i;
        const previous_x = x;
        if (y >= 0) {
            x += y >> @intCast(shift);
            y -= previous_x >> @intCast(shift);
            z += atan_table[i];
        } else {
            x -= y >> @intCast(shift);
            y += previous_x >> @intCast(shift);
            z -= atan_table[i];
        }
    }
    return fromQ224(z);
}
fn fromQ224(value: i256) Fp {
    const q = value >> 192;
    const rem = @abs(value) & ((@as(i256, 1) << 192) - 1);
    var v = q;
    const half: @TypeOf(rem) = @as(i256, 1) << 191;
    if (rem > half or (rem == half and (q & 1) != 0)) v += if (value < 0) -1 else 1;
    return .{ .raw = @intCast(v) };
}
// Frozen, independently generated, nearest-even Q32.224 constants.  Keeping
// these literals avoids target-dependent compile-time series evaluation.
const cordic_k: i256 = 16371506741310132295104929292316916377101202411377066175221274463730;
const atan_table = [_]i256{
    21174292597673270169193562049053717791882423761323585056162680913574, 12499914811013645833761485874711319765883910652669829946987218701171, 6604611692490120586944639910135010248809357292697829139972665286047, 3352604020774496936862054129658993847206988430336546630370172496624,
    1682807788518019357186361167149206718549869328879509292343835994616,  842224243170702353395458569806629780254048336929574404050008935406,   421214890350090397824033603942867944560753926433350652932779245057,  210620298325917051633246592160516207267430599618713456697334538927,
    105311756027446365983361945758704861881810998754133081724431780013,   52656078878679998360880131025414534677065639196793670735355148346,    26328064547675105204208083021860480415124701497684159018232722833,   13164035412386175430535135611071022770644300704741291942537955378,
    6582018098511876024869067828752609252424257936368002414165927253,     3291009098295793127890619386806108073820283700582553157238232558,     1645504555277878658900976642585684447633553022177747308763906670,    822752278405187097742561217394601879299139740525761516194306223,
    411376139298374520108496426795836431114417589881644015208773378,      205688069661159881465172275241845639160100441725585802143885054,      102844034832076518409147647994717424877617109602009861546696234,     51422017416225331414150137876687280749536165661517268832535658,
    25711008708136049733272299582091101830214962000342391766848176,       12855504354070947869910809603039964608759145757565401623617198,       6427752177035839310364737464941970923180445731758113644616194,       3213876088517965327108535321240068004790767899372816636011094,
    1606938044258988372545038484398711006728307384913458436478166,        803469022129494899896365595177394556606796078168601935596125,         401734511064747539151163591711130423324385431464134252356910,        200867255532373780725954395120874925519992923986144768127639,
    100433627766186891756773772468601351077539512531397008210252,         50216813883093446052611458097821167018628825848820085185702,          25108406941546723048083800531850645114301990005607465709139,         12554203470773361526764159201292830263087104431181761592233,
    6277101735386680763722361967567353594951586590801520352734,           3138550867693340381903716279648794105406985739072820932335,           1569275433846670190957175051807536716195054054582003398519,          784637716923335095479252139901660816033977241971281607612,
    392318858461667547739709146700566965259045056150350192094,            196159429230833773869864957944000552284779587418566094575,            98079714615416886934933777046214909849296926281825744759,            49039857307708443467467050782384284138011854717312547772,
    24519928653854221733733545673601745720676351305857228254,             12259964326927110866866775372102073316796978646333452015,             6129982163463555433433388002963686715455839697342478199,             3064991081731777716716694041095924614860088645443212732,
    1532495540865888858358347025499722464571565422318103214,              766247770432944429179173513368831251928472848608613717,               383123885216472214589586756761786878419572691485502123,              191561942608236107294793378390564845766703379140400469,
    95780971304118053647396689196491348702966318744906411,                47890485652059026823698344598396790078934988019291477,                23945242826029513411849172299217284505398972590500523,               11972621413014756705924586149611003435940921117857109,
    5986310706507378352962293074805796865875639911754411,                 2993155353253689176481146537402935326425967374980437,                 1496577676626844588240573268701472274899002114878123,                748288838313422294120286634350736713910253360862549,
    374144419156711147060143317175368429012720718359211,                  187072209578355573530071658587684223513559613920597,                  93536104789177786765035829293842112882679713802923,                  46768052394588893382517914646921056582077345256789,
    23384026197294446691258957323460528308630858672811,                   11692013098647223345629478661730264156514452591957,                   5846006549323611672814739330865132078532104202923,                   2923003274661805836407369665432566039300411839829,
    1461501637330902918203684832716283019654500887211,                    730750818665451459101842416358141509827787314517,                     365375409332725729550921208179070754913960766123,                    182687704666362864775460604089535377456988771669,
    91343852333181432387730302044767688728495434411,                      45671926166590716193865151022383844364247848277,                      22835963083295358096932575511191922182123940523,                     11417981541647679048466287755595961091061972309,
    5708990770823839524233143877797980545530986411,                       2854495385411919762116571938898990272765493237,                       1427247692705959881058285969449495136382746623,                      713623846352979940529142984724747568191373312,
    356811923176489970264571492362373784095686656,                        178405961588244985132285746181186892047843328,                        89202980794122492566142873090593446023921664,                        44601490397061246283071436545296723011960832,
    22300745198530623141535718272648361505980416,                         11150372599265311570767859136324180752990208,                         5575186299632655785383929568162090376495104,                         2787593149816327892691964784081045188247552,
    1393796574908163946345982392040522594123776,                          696898287454081973172991196020261297061888,                           348449143727040986586495598010130648530944,                          174224571863520493293247799005065324265472,
    87112285931760246646623899502532662132736,                            43556142965880123323311949751266331066368,                            21778071482940061661655974875633165533184,                           10889035741470030830827987437816582766592,
    5444517870735015415413993718908291383296,                             2722258935367507707706996859454145691648,                             1361129467683753853853498429727072845824,                            680564733841876926926749214863536422912,
};
