const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const dependency = b.dependency("gravity", .{ .target = target, .optimize = optimize });
    const executable = b.addExecutable(.{
        .name = "gravity-core-consumer",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
        }),
    });
    executable.root_module.addImport("gravity", dependency.module("gravity"));
    b.installArtifact(executable);
}
