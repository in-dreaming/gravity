const std = @import("std");
const gravity = @import("gravity");

pub fn main(init: std.process.Init) !void {
    var args = try std.process.Args.Iterator.initAllocator(init.minimal.args, init.gpa);
    defer args.deinit();
    _ = args.next();
    const left_path = args.next() orelse return error.InvalidArguments;
    const right_path = args.next() orelse return error.InvalidArguments;
    if (args.next() != null) return error.InvalidArguments;
    const left = try readFile(init.io, left_path);
    defer std.heap.page_allocator.free(left);
    const right = try readFile(init.io, right_path);
    defer std.heap.page_allocator.free(right);
    if (try gravity.state.diff.first(left, right)) |difference| {
        std.debug.print("different: section={any} id={any} field={s} offset={d} left={any} right={any}\n", .{ difference.section, difference.id, @tagName(difference.field), difference.offset, difference.left, difference.right });
        return error.SnapshotsDiffer;
    }
    std.debug.print("equal\n", .{});
}
fn readFile(io: std.Io, path: []const u8) ![]u8 {
    var file = try std.Io.Dir.openFile(.cwd(), io, path, .{});
    defer file.close(io);
    const bytes = try std.heap.page_allocator.alloc(u8, @intCast(try file.length(io)));
    errdefer std.heap.page_allocator.free(bytes);
    var reader_buffer: [4096]u8 = undefined;
    var reader = file.reader(io, &reader_buffer);
    try reader.interface.readSliceAll(bytes);
    return bytes;
}
