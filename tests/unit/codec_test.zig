const std = @import("std");
const gravity = @import("gravity");
const codec = gravity.state.codec;
const hash = gravity.state.hash;
const snapshot = gravity.state.snapshot;

test "canonical primitives and config size pass round trip" {
    const config = gravity.core.config.SimulationConfig.default;
    var size = codec.Writer.sizing();
    try codec.encodeConfig(&size, config);
    try std.testing.expectEqual(codec.config_encoded_size, size.written());
    var bytes: [codec.config_encoded_size]u8 = undefined;
    var writer = codec.Writer.init(&bytes);
    try codec.encodeConfig(&writer, config);
    try std.testing.expectEqual(size.written(), writer.written());
    var reader = codec.Reader.init(&bytes);
    const decoded = try codec.decodeConfig(&reader);
    try reader.finish();
    var again: [codec.config_encoded_size]u8 = undefined;
    var again_writer = codec.Writer.init(&again);
    try codec.encodeConfig(&again_writer, decoded);
    try std.testing.expectEqualSlices(u8, &bytes, &again);
    const expected = [_]u8{ 0xe5, 0x8a, 0xb8, 0xe8, 0x3e, 0xc6, 0x7a, 0x11, 0xac, 0xba, 0xbf, 0x35, 0xda, 0x59, 0x92, 0x2e, 0xc5, 0x45, 0x74, 0x17, 0x73, 0x7d, 0xcb, 0xf9, 0x85, 0x90, 0x54, 0x7d, 0x1c, 0x9a, 0x4b, 0x49 };
    try std.testing.expectEqualSlices(u8, &expected, &hash.oneShot256(.config, &bytes));
}

test "TLV rejects truncation ordering duplication and length bombs" {
    var payload = [_]u8{ 1, 2, 3 };
    var bytes: [32]u8 = undefined;
    var writer = codec.Writer.init(&bytes);
    try codec.writeHeader(&writer, 1, 2);
    try codec.writeSection(&writer, 2, &payload);
    try codec.writeSection(&writer, 3, &payload);
    try std.testing.expectError(error.DuplicateSection, codec.writeSection(&writer, 3, &payload));
    const Context = struct { calls: usize = 0 };
    const Visit = struct {
        fn visit(ctx: *Context, section: codec.Section) codec.Error!void {
            _ = section;
            ctx.calls += 1;
        }
    }.visit;
    var ctx = Context{};
    var reader = codec.Reader.init(bytes[0..writer.written()]);
    try codec.readSections(&reader, 1, Context, &ctx, Visit);
    try std.testing.expectEqual(@as(usize, 2), ctx.calls);
    var truncated = codec.Reader.init(bytes[0 .. writer.written() - 1]);
    try std.testing.expectError(error.EndOfInput, codec.readSections(&truncated, 1, Context, &ctx, Visit));
    var duplicate = bytes;
    duplicate[13] = 2;
    var duplicate_reader = codec.Reader.init(duplicate[0..writer.written()]);
    try std.testing.expectError(error.DuplicateSection, codec.readSections(&duplicate_reader, 1, Context, &ctx, Visit));
    var bomb = [_]u8{ 1, 0, 1, 0, 1, 0, 0, 0, 0xff, 0xff, 0xff, 0xff };
    var bomb_reader = codec.Reader.init(&bomb);
    try std.testing.expectError(error.SectionTooLarge, codec.readSections(&bomb_reader, 1, Context, &ctx, Visit));
    var required = [_]u8{ 1, 0, 1, 0, 1, 0x80, 0, 0, 0, 0 };
    var required_reader = codec.Reader.init(&required);
    try std.testing.expectError(error.UnknownRequiredSection, codec.readKnownSections(&required_reader, 1, &.{}, Context, &ctx, Visit));
}

test "bool enum-like bits and malformed config are rejected" {
    var bool_reader = codec.Reader.init(&[_]u8{2});
    try std.testing.expectError(error.InvalidBool, bool_reader.boolean());
    var data: [codec.config_encoded_size]u8 = undefined;
    var writer = codec.Writer.init(&data);
    try codec.encodeConfig(&writer, .default);
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    var reader = codec.Reader.init(&data);
    try std.testing.expectError(error.InvalidConfig, codec.decodeConfig(&reader));
}

test "domain hash streaming equals one shot and domains differ" {
    const input = "canonical payload";
    var sink = hash.Sink.init(.config);
    sink.update(input[0..4]);
    sink.update(input[4..]);
    const chunked = sink.final256();
    try std.testing.expectEqualSlices(u8, &hash.oneShot256(.config, input), &chunked);
    try std.testing.expect(!std.mem.eql(u8, &chunked, &hash.oneShot256(.state, input)));
    const short = hash.oneShot128(.config, input);
    try std.testing.expectEqualSlices(u8, short[0..], chunked[0..16]);
}

test "GRAVSNAP envelope round trips and rejects corrupt metadata" {
    const header = snapshot.Header{ .configuration = gravity.core.config.SimulationConfig.default, .asset_set = [_]u8{0xa5} ** 32 };
    var sizing = codec.Writer.sizing();
    try snapshot.writeHeader(&sizing, header);
    var bytes: [512]u8 = undefined;
    var writer = codec.Writer.init(&bytes);
    try snapshot.writeHeader(&writer, header);
    var reader = codec.Reader.init(bytes[0..writer.written()]);
    const decoded = try snapshot.readHeader(&reader);
    try reader.finish();
    try std.testing.expectEqualDeep(header, decoded);
    bytes[0] ^= 1;
    reader = codec.Reader.init(bytes[0..writer.written()]);
    try std.testing.expectError(error.InvalidMagic, snapshot.readHeader(&reader));
}

test "GRAVSNAP pipeline payload preserves fault and rejects invalid enum" {
    var state = gravity.dynamics.pipeline.State{ .tick = 9 };
    state.fault = .{ .tick = 8, .phase = .narrowphase, .object = 42, .code = .contact, .detail = .contact };
    var bytes: [64]u8 = undefined;
    var writer = codec.Writer.init(&bytes);
    try snapshot.writePipeline(&writer, state);
    var reader = codec.Reader.init(bytes[0..writer.written()]);
    const decoded = try snapshot.readPipeline(&reader);
    try reader.finish();
    try std.testing.expectEqualDeep(state.fault, decoded.fault);
    bytes[17] = 255;
    reader = codec.Reader.init(bytes[0..writer.written()]);
    try std.testing.expectError(error.InvalidEnum, snapshot.readPipeline(&reader));
}

test "GRAVSNAP pipeline snapshot uses required canonical section" {
    const header = snapshot.Header{ .configuration = gravity.core.config.SimulationConfig.default, .asset_set = [_]u8{7} ** 32 };
    const state = gravity.dynamics.pipeline.State{ .tick = 123 };
    var output: [512]u8 = undefined;
    var payload: [64]u8 = undefined;
    const bytes = try snapshot.encodePipelineSnapshot(header, state, &output, &payload);
    const decoded = try snapshot.decodePipelineSnapshot(bytes);
    try std.testing.expectEqualDeep(header, decoded.header);
    try std.testing.expectEqual(@as(u64, 123), decoded.state.tick);
}

test "GRAVSNAP pipeline and contacts sections restore atomically" {
    const header = snapshot.Header{ .configuration = gravity.core.config.SimulationConfig.default, .asset_set = [_]u8{9} ** 32 };
    var source_patches: [1]gravity.collision.contact_cache.Patch = undefined;
    const source = gravity.collision.contact_cache.Cache{ .patches = &source_patches };
    var output: [1024]u8 = undefined;
    var pipeline_payload: [64]u8 = undefined;
    var contacts_payload: [64]u8 = undefined;
    const bytes = try snapshot.encodePipelineContactsSnapshot(header, .{ .tick = 17 }, &source, &output, &pipeline_payload, &contacts_payload);
    var target_patches: [1]gravity.collision.contact_cache.Patch = undefined;
    var target = gravity.collision.contact_cache.Cache{ .patches = &target_patches };
    var stage: [1]gravity.collision.contact_cache.Patch = undefined;
    var scratch: [1]gravity.collision.contact_cache.Patch = undefined;
    const decoded = try snapshot.decodePipelineContactsSnapshot(bytes, &target, &stage, &scratch);
    try std.testing.expectEqual(@as(u64, 17), decoded.state.tick);
    try std.testing.expectEqual(@as(usize, 0), target.len);
}
