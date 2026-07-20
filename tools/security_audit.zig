//! Task 25 build-graph, license and SBOM audit.
const std = @import("std");

fn read(init: std.process.Init, path: []const u8) ![]u8 {
    return std.Io.Dir.cwd().readFileAlloc(init.io, path, init.gpa, .limited(16 * 1024 * 1024));
}

pub fn main(init: std.process.Init) !void {
    var args = try init.minimal.args.iterateAllocator(init.gpa);
    defer args.deinit();
    _ = args.next();
    const spindle_gitlink = args.next() orelse return error.InvalidArguments;
    const spindle_checkout = args.next() orelse return error.InvalidArguments;
    if (args.next() != null or spindle_gitlink.len != 40 or spindle_checkout.len != 40) return error.InvalidArguments;
    if (!std.mem.eql(u8, spindle_gitlink, spindle_checkout)) return error.SpindleCheckoutDrift;

    const build = try read(init, "build.zig");
    defer init.gpa.free(build);
    if (std.mem.indexOf(u8, build, "third_party/spindle/src/root.zig") != null or std.mem.indexOf(u8, build, "spindle/src/zruntime/root.zig") != null) return error.ForbiddenSpindleAggregate;
    if (std.mem.indexOf(u8, build, "third_party/spindle/src/executor.zig") == null) return error.MissingExecutorOnlyEntry;

    var src = try std.Io.Dir.cwd().openDir(init.io, "src", .{ .iterate = true });
    defer src.close(init.io);
    var walker = try src.walk(init.gpa);
    defer walker.deinit();
    var spindle_sources: usize = 0;
    while (try walker.next(init.io)) |entry| {
        if (entry.kind != .file or !std.mem.endsWith(u8, entry.basename, ".zig")) continue;
        const bytes = try entry.dir.readFileAlloc(init.io, entry.basename, init.gpa, .limited(4 * 1024 * 1024));
        defer init.gpa.free(bytes);
        if (std.mem.indexOf(u8, bytes, "spindle") == null) continue;
        spindle_sources += 1;
        const allowed = std.mem.endsWith(u8, entry.path, "jobs/spindle_dispatcher.zig") or std.mem.endsWith(u8, entry.path, "jobs\\spindle_dispatcher.zig");
        if (!allowed) return error.SpindleImportEscapedAdapter;
        if (std.mem.indexOf(u8, bytes, "@import(\"spindle_executor\")") == null) return error.MissingNarrowImport;
        inline for ([_][]const u8{ "spindle.Runtime", "spindle.parallel", "TaskGraph", "spindle.ecs", "spindle.workflow", "sqlite", "archive" }) |forbidden| if (std.mem.indexOf(u8, bytes, forbidden) != null) return error.ForbiddenSpindleSubsystem;
    }
    if (spindle_sources != 1) return error.UnexpectedSpindleSourceCount;

    const notice = try read(init, "THIRD_PARTY_NOTICES.md");
    defer init.gpa.free(notice);
    const sbom = try read(init, "docs/security/sbom.spdx.json");
    defer init.gpa.free(sbom);
    const spindle_license = try read(init, "third_party/spindle/LICENSE");
    defer init.gpa.free(spindle_license);
    if (std.mem.indexOf(u8, notice, spindle_gitlink) == null or std.mem.indexOf(u8, sbom, spindle_gitlink) == null) return error.SpindlePinDrift;
    if (!std.mem.startsWith(u8, spindle_license, "MIT License")) return error.SpindleLicenseDrift;
    std.debug.print("security audit: executor-only graph, gitlink/checkout Spindle pin {s}, MIT license and SBOM verified\n", .{spindle_gitlink});
}
