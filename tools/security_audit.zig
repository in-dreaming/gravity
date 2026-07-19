//! Task 25 build-graph, license and SBOM audit.
const std = @import("std");

const spindle_commit = "6756fb2feecfa354a7ae42bca3af5d9bd66c7558";

fn read(init: std.process.Init, path: []const u8) ![]u8 {
    return std.Io.Dir.cwd().readFileAlloc(init.io, path, init.gpa, .limited(16 * 1024 * 1024));
}

pub fn main(init: std.process.Init) !void {
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
    if (std.mem.indexOf(u8, notice, spindle_commit) == null or std.mem.indexOf(u8, sbom, spindle_commit) == null) return error.SpindlePinDrift;
    if (!std.mem.startsWith(u8, spindle_license, "MIT License")) return error.SpindleLicenseDrift;
    std.debug.print("security audit: executor-only graph, Spindle pin {s}, MIT license and SBOM verified\n", .{spindle_commit});
}
