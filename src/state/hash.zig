//! Domain-separated BLAKE3 helpers for every persistent Gravity format.
const std = @import("std");

pub const Hash128 = [16]u8;
pub const Hash256 = [std.crypto.hash.Blake3.digest_length]u8;

pub const Domain = enum {
    config,
    asset,
    asset_set,
    state,
    snapshot,
    replay,

    pub fn bytes(self: Domain) []const u8 {
        return switch (self) {
            .config => "gravity/config/v1\x00",
            .asset => "gravity/asset/v1\x00",
            .asset_set => "gravity/asset-set/v1\x00",
            .state => "gravity/state/v1\x00",
            .snapshot => "gravity/snapshot/v1\x00",
            .replay => "gravity/replay/v1\x00",
        };
    }
};

pub const Sink = struct {
    hasher: std.crypto.hash.Blake3,

    pub fn init(domain: Domain) Sink {
        var result = Sink{ .hasher = std.crypto.hash.Blake3.init(.{}) };
        result.hasher.update(domain.bytes());
        return result;
    }

    pub fn update(self: *Sink, bytes: []const u8) void {
        self.hasher.update(bytes);
    }

    pub fn final256(self: *Sink) Hash256 {
        var result: Hash256 = undefined;
        self.hasher.final(&result);
        return result;
    }

    pub fn final128(self: *Sink) Hash128 {
        const full = self.final256();
        return full[0..16].*;
    }
};

pub fn oneShot256(domain: Domain, payload: []const u8) Hash256 {
    var sink = Sink.init(domain);
    sink.update(payload);
    return sink.final256();
}

pub fn oneShot128(domain: Domain, payload: []const u8) Hash128 {
    var sink = Sink.init(domain);
    sink.update(payload);
    return sink.final128();
}
