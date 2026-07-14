/// Build and wire-format versions. Changing a simulation-affecting value requires
/// a protocol-version review before it is released.
pub const abi_version: u32 = 1;
pub const protocol_version: u32 = 1;
pub const snapshot_format_version: u32 = 1;
pub const asset_format_version: u32 = 2;

pub const BuildMetadata = struct {
    commit: []const u8,
    zig_version: []const u8,
    abi: u32,
    protocol: u32,
    snapshot_format: u32,
    asset_format: u32,
};
