const std = @import("std");
const gravity = @import("gravity");

pub fn main(init: std.process.Init) !void {
    var args = try init.minimal.args.iterateAllocator(init.gpa);
    defer args.deinit();
    _ = args.next();
    const command = args.next() orelse return usage();
    if (std.mem.eql(u8, command, "source-check")) {
        const path = args.next() orelse return usage();
        if (args.next() != null) return usage();
        const bytes = try std.Io.Dir.cwd().readFileAlloc(init.io, path, init.gpa, .limited(16 * 1024 * 1024));
        defer init.gpa.free(bytes);
        try gravity.assets.source.validate(bytes, init.gpa);
        std.debug.print("valid canonical source\n", .{});
        return;
    }
    if (std.mem.eql(u8, command, "validate")) {
        const path = args.next() orelse return usage();
        if (args.next() != null) return usage();
        const bytes = try std.Io.Dir.cwd().readFileAlloc(init.io, path, init.gpa, .limited(16 * 1024 * 1024));
        defer init.gpa.free(bytes);
        const header = try gravity.geometry.baked.validateEncoded(bytes);
        const digest = gravity.state.hash.oneShot256(.asset, bytes);
        std.debug.print("valid kind={s} source_id={d} hash={x}\n", .{ @tagName(header.kind), header.source_id, digest });
        return;
    }
    if (std.mem.eql(u8, command, "manifest")) {
        const output_path = args.next() orelse return usage();
        var paths: std.array_list.Managed([]const u8) = .init(std.heap.page_allocator);
        defer paths.deinit();
        while (args.next()) |path| try paths.append(path);
        if (paths.items.len == 0) return usage();
        var inputs = try std.heap.page_allocator.alloc([]const u8, paths.items.len);
        defer std.heap.page_allocator.free(inputs);
        defer for (inputs) |input| if (input.len != 0) init.gpa.free(input);
        for (paths.items, 0..) |path, i| inputs[i] = try std.Io.Dir.cwd().readFileAlloc(init.io, path, init.gpa, .limited(16 * 1024 * 1024));
        const required = try gravity.assets.store.Store.memoryRequired(inputs);
        const memory = try std.heap.page_allocator.alignedAlloc(u8, .of(gravity.assets.store.Asset), required);
        defer std.heap.page_allocator.free(memory);
        const assets = try gravity.assets.store.Store.init(memory, inputs);
        const manifest = try std.heap.page_allocator.alloc(u8, 4 + assets.assets.len * 40);
        defer std.heap.page_allocator.free(manifest);
        const result = try gravity.assets.store.writeManifest(assets.assets, manifest);
        try std.Io.Dir.cwd().writeFile(init.io, .{ .sub_path = output_path, .data = result.bytes });
        std.debug.print("manifest assets={d} hash={x}\n", .{ assets.assets.len, result.content_hash });
        return;
    }
    return usage();
}

fn usage() error{InvalidArguments}!void {
    std.debug.print("usage: gravity-bake source-check <source.json> | validate <asset.tlv> | manifest <output> <asset.tlv>...\n", .{});
    return error.InvalidArguments;
}
