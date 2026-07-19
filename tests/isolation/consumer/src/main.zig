const gravity = @import("gravity");

pub fn main() !void {
    if (gravity.abi_version != 1) return error.UnexpectedAbi;
}
