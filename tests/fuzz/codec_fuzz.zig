const std = @import("std");
const gravity = @import("gravity");
const codec = gravity.state.codec;

test "bounded malformed codec corpus never accepts partial input" {
    var bytes: [64]u8 = undefined;
    var seed: u32 = 0x4210_7a5d;
    const Context = struct {};
    const Visit = struct {
        fn visit(_: *Context, _: codec.Section) codec.Error!void {}
    }.visit;
    for (0..4_000) |_| {
        for (&bytes) |*byte| {
            seed = seed *% 1_664_525 +% 1_013_904_223;
            byte.* = @truncate(seed);
        }
        const length: usize = seed % (bytes.len + 1);
        var reader = codec.Reader.init(bytes[0..length]);
        var context = Context{};
        _ = codec.readSections(&reader, 1, Context, &context, Visit) catch continue;
        try reader.finish();
    }
}
