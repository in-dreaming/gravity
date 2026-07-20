const std = @import("std");
const builtin = @import("builtin");

const required_zig_version = "0.16.0";
const package_version = "1.0.0";

pub fn build(b: *std.Build) void {
    if (!std.mem.eql(u8, builtin.zig_version_string, required_zig_version)) {
        std.debug.panic(
            "Gravity requires Zig {s}; found {s}",
            .{ required_zig_version, builtin.zig_version_string },
        );
    }

    // A baseline CPU is part of the deterministic build contract; host feature
    // discovery must never alter generated core code.
    const target = b.standardTargetOptions(.{
        .default_target = .{ .cpu_model = .baseline },
    });
    const optimize = b.standardOptimizeOption(.{});
    const metadata = addBuildMetadata(b);
    const package_module = b.addModule("gravity", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
        .link_libc = false,
        .link_libcpp = false,
    });
    package_module.addImport("build_options", metadata.createModule());
    package_module.addImport("gravity_abi_options", addAbiOptionsModule(b, target));
    const package_jobs = b.createModule(.{ .root_source_file = b.path("src/jobs/dispatcher.zig"), .target = target, .optimize = optimize });
    const package_host_jobs = b.createModule(.{ .root_source_file = b.path(if (target.result.cpu.arch == .wasm32) "src/jobs/wasm_serial.zig" else "src/jobs/host_dispatcher.zig"), .target = target, .optimize = optimize });
    if (target.result.cpu.arch != .wasm32) package_host_jobs.addImport("gravity_jobs", package_jobs);
    package_module.addImport("gravity_jobs", package_jobs);
    package_module.addImport("gravity_host_jobs", package_host_jobs);
    const static_library = b.addLibrary(.{
        .name = "gravity_static",
        .root_module = addAbiModule(b, target, optimize, metadata),
        .linkage = .static,
    });
    static_library.bundle_compiler_rt = true;
    b.installArtifact(static_library);
    static_library.installHeader(b.path("include/gravity.h"), "gravity.h");

    var shared_library: ?*std.Build.Step.Compile = null;
    if (target.result.cpu.arch != .wasm32) {
        const shared_artifact = b.addLibrary(.{
            .name = "gravity",
            .root_module = addAbiModule(b, target, optimize, metadata),
            .linkage = .dynamic,
        });
        b.installArtifact(shared_artifact);
        shared_library = shared_artifact;
    }

    const wasm_target = b.resolveTargetQuery(.{
        .cpu_arch = .wasm32,
        .os_tag = .freestanding,
        .abi = .none,
    });
    const wasm_module = addAbiModule(b, wasm_target, .ReleaseSmall, metadata);
    const wasm_library = b.addExecutable(.{ .name = "gravity", .root_module = wasm_module });
    wasm_library.entry = .disabled;
    wasm_library.rdynamic = true;
    const install_wasm = b.addInstallArtifact(wasm_library, .{});
    b.getInstallStep().dependOn(&install_wasm.step);

    const wasi_target = b.resolveTargetQuery(.{
        .cpu_arch = .wasm32,
        .os_tag = .wasi,
        .abi = .none,
    });

    const abi_artifacts = b.step("abi-artifacts", "Build Task 22 Windows, Linux, macOS, and WASM artifacts");
    const artifact_targets = .{
        .{ "windows-x86_64", std.Target.Query{ .cpu_arch = .x86_64, .os_tag = .windows, .abi = .gnu } },
        .{ "windows-aarch64", std.Target.Query{ .cpu_arch = .aarch64, .os_tag = .windows, .abi = .gnu } },
        .{ "linux-x86_64", std.Target.Query{ .cpu_arch = .x86_64, .os_tag = .linux, .abi = .musl } },
        .{ "linux-aarch64", std.Target.Query{ .cpu_arch = .aarch64, .os_tag = .linux, .abi = .musl } },
        .{ "macos-x86_64", std.Target.Query{ .cpu_arch = .x86_64, .os_tag = .macos } },
        .{ "macos-aarch64", std.Target.Query{ .cpu_arch = .aarch64, .os_tag = .macos } },
    };
    inline for (artifact_targets) |entry| {
        const artifact_target = b.resolveTargetQuery(entry[1]);
        const artifact_static = b.addLibrary(.{ .name = "gravity_static", .root_module = addAbiModule(b, artifact_target, .ReleaseSafe, metadata), .linkage = .static });
        artifact_static.bundle_compiler_rt = true;
        artifact_static.root_module.strip = true;
        const artifact_shared = b.addLibrary(.{ .name = "gravity", .root_module = addAbiModule(b, artifact_target, .ReleaseSafe, metadata), .linkage = .dynamic });
        artifact_shared.root_module.strip = true;
        abi_artifacts.dependOn(&b.addInstallArtifact(artifact_static, .{ .dest_dir = .{ .override = .{ .custom = b.fmt("abi/{s}/lib", .{entry[0]}) } } }).step);
        abi_artifacts.dependOn(&b.addInstallArtifact(artifact_shared, .{ .dest_dir = .{ .override = .{ .custom = b.fmt("abi/{s}/lib", .{entry[0]}) } } }).step);
    }
    abi_artifacts.dependOn(&install_wasm.step);
    const arm_linux_target = b.resolveTargetQuery(.{
        .cpu_arch = .aarch64,
        .os_tag = .linux,
        .abi = .musl,
    });

    const fmt_step = b.step("fmt", "Check Zig formatting");
    const fmt = b.addFmt(.{
        .paths = &.{ "build.zig", "src", "tests", "tools" },
        .check = true,
    });
    fmt_step.dependOn(&fmt.step);

    const test_step = b.step("test", "Run Debug unit tests");
    test_step.dependOn(&addSpindleTest(b, "spindle-integration-debug", target, .Debug, addSpindleModule(b, target, .Debug)).step);
    test_step.dependOn(&addJobsTest(b, "jobs-dispatcher-debug", target, .Debug, metadata, addSpindleModule(b, target, .Debug)).step);
    test_step.dependOn(&addZigTest(b, "unit-foundation-debug", target, .Debug, metadata, "tests/unit/foundation_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-fp-debug", target, .Debug, metadata, "tests/unit/fp_test.zig").step);
    test_step.dependOn(&addZigTest(b, "golden-fp-debug", target, .Debug, metadata, "tests/golden/fp_golden.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-geometry-debug", target, .Debug, metadata, "tests/unit/geometry_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-core-debug", target, .Debug, metadata, "tests/unit/core_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-codec-debug", target, .Debug, metadata, "tests/unit/codec_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-baked-geometry-debug", target, .Debug, metadata, "tests/unit/baked_geometry_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-assets-debug", target, .Debug, metadata, "tests/unit/asset_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-asset-source-debug", target, .Debug, metadata, "tests/unit/asset_source_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-runtime-shapes-debug", target, .Debug, metadata, "tests/unit/runtime_shapes_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-broadphase-debug", target, .Debug, metadata, "tests/unit/broadphase_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-analytic-debug", target, .Debug, metadata, "tests/unit/analytic_collision_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-gjk-debug", target, .Debug, metadata, "tests/unit/gjk_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-mesh-collision-debug", target, .Debug, metadata, "tests/unit/mesh_collision_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-contact-cache-debug", target, .Debug, metadata, "tests/unit/contact_cache_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-dynamics-debug", target, .Debug, metadata, "tests/unit/dynamics_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-constraints-debug", target, .Debug, metadata, "tests/unit/constraints_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-contact-solver-debug", target, .Debug, metadata, "tests/unit/contact_solver_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-joints-debug", target, .Debug, metadata, "tests/unit/joints_test.zig").step);
    test_step.dependOn(&addZigTest(b, "scenarios-joints-debug", target, .Debug, metadata, "tests/scenarios/joints_scenarios.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-queries-debug", target, .Debug, metadata, "tests/unit/queries_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-sleeping-debug", target, .Debug, metadata, "tests/unit/sleeping_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-ccd-debug", target, .Debug, metadata, "tests/unit/ccd_test.zig").step);
    test_step.dependOn(&addZigTest(b, "unit-pipeline-debug", target, .Debug, metadata, "tests/unit/pipeline_test.zig").step);

    // Keep Task 03's complete core suite explicitly bound to its optimization
    // mode.  Do not route an `-Doptimize` flag through the Debug-only `test`
    // convenience step: determinism must be exercised in all three modes.
    const core_all_modes = b.step("test-core-all-modes", "Run the complete Task 03 core suite in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| {
        const core_mode = b.step(b.fmt("test-core-{s}", .{@tagName(mode)}), b.fmt("Run the complete Task 03 core suite in {s}", .{@tagName(mode)}));
        core_mode.dependOn(&addZigTest(b, b.fmt("unit-core-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/core_test.zig").step);
        core_mode.dependOn(&addZigTest(b, b.fmt("unit-codec-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/codec_test.zig").step);
        core_all_modes.dependOn(core_mode);
    }

    const all_modes = b.step("test-all-modes", "Run unit tests in all supported optimization modes");
    all_modes.dependOn(core_all_modes);
    const spindle_all_modes = b.step("spindle-check-all-modes", "Validate Spindle integration in Debug, ReleaseSafe, and ReleaseFast");
    all_modes.dependOn(spindle_all_modes);
    const job_scaling = b.step("job-scaling", "Measure Task 23 serial and Spindle 1/2/4/8 World scaling with hash validation");
    job_scaling.dependOn(&addJobScaling(b, target, .ReleaseFast, metadata, addSpindleModule(b, target, .ReleaseFast)).step);
    const jobs_tsan = b.step("jobs-tsan", "Build, and on Linux run, the Task 23 dispatcher suite with ThreadSanitizer");
    const tsan_target = b.resolveTargetQuery(.{ .cpu_arch = .x86_64, .os_tag = .linux, .abi = .gnu, .cpu_model = .baseline });
    const tsan_test = addJobsTsanTest(b, tsan_target, metadata);
    if (builtin.os.tag == .linux) jobs_tsan.dependOn(&b.addRunArtifact(tsan_test).step) else jobs_tsan.dependOn(&tsan_test.step);
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| {
        spindle_all_modes.dependOn(&addSpindleTest(b, b.fmt("spindle-integration-{s}", .{@tagName(mode)}), target, mode, addSpindleModule(b, target, mode)).step);
        spindle_all_modes.dependOn(&addJobsTest(b, b.fmt("jobs-dispatcher-{s}", .{@tagName(mode)}), target, mode, metadata, addSpindleModule(b, target, mode)).step);
        all_modes.dependOn(&addZigTest(b, b.fmt("unit-foundation-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/foundation_test.zig").step);
        all_modes.dependOn(&addZigTest(b, b.fmt("unit-fp-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/fp_test.zig").step);
        all_modes.dependOn(&addZigTest(b, b.fmt("golden-fp-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/golden/fp_golden.zig").step);
        all_modes.dependOn(&addZigTest(b, b.fmt("unit-geometry-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/geometry_test.zig").step);
        all_modes.dependOn(&addZigTest(b, b.fmt("unit-baked-geometry-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/baked_geometry_test.zig").step);
        all_modes.dependOn(&addZigTest(b, b.fmt("unit-assets-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/asset_test.zig").step);
        all_modes.dependOn(&addZigTest(b, b.fmt("unit-asset-source-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/asset_source_test.zig").step);
        all_modes.dependOn(&addZigTest(b, b.fmt("unit-runtime-shapes-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/runtime_shapes_test.zig").step);
        all_modes.dependOn(&addZigTest(b, b.fmt("unit-broadphase-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/broadphase_test.zig").step);
    }

    const assets_all_modes = b.step("test-assets-all-modes", "Run Task 05 asset tests in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| {
        assets_all_modes.dependOn(&addZigTest(b, b.fmt("unit-assets-task05-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/asset_test.zig").step);
        assets_all_modes.dependOn(&addZigTest(b, b.fmt("unit-asset-source-task05-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/asset_source_test.zig").step);
    }

    const geometry_all_modes = b.step("test-geometry-all-modes", "Run Task 06 baked geometry tests in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| {
        geometry_all_modes.dependOn(&addZigTest(b, b.fmt("unit-baked-geometry-task06-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/baked_geometry_test.zig").step);
    }

    const runtime_shapes_all_modes = b.step("test-runtime-shapes-all-modes", "Run Task 07 runtime shape tests in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| {
        runtime_shapes_all_modes.dependOn(&addZigTest(b, b.fmt("unit-runtime-shapes-task07-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/runtime_shapes_test.zig").step);
    }
    const broadphase_all_modes = b.step("test-broadphase-all-modes", "Run Task 08 SAP broadphase tests in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| {
        broadphase_all_modes.dependOn(&addZigTest(b, b.fmt("unit-broadphase-task08-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/broadphase_test.zig").step);
    }
    const analytic_all_modes = b.step("test-analytic-all-modes", "Run Task 09 analytic collision tests in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| analytic_all_modes.dependOn(&addZigTest(b, b.fmt("unit-analytic-task09-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/analytic_collision_test.zig").step);
    const gjk_all_modes = b.step("test-gjk-all-modes", "Run Task 10 GJK tests in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| gjk_all_modes.dependOn(&addZigTest(b, b.fmt("unit-gjk-task10-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/gjk_test.zig").step);
    const mesh_collision_all_modes = b.step("test-mesh-collision-all-modes", "Run Task 11 mesh collision tests in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| mesh_collision_all_modes.dependOn(&addZigTest(b, b.fmt("unit-mesh-collision-task11-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/mesh_collision_test.zig").step);
    const contact_cache_all_modes = b.step("test-contact-cache-all-modes", "Run Task 12 contact cache tests in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| contact_cache_all_modes.dependOn(&addZigTest(b, b.fmt("unit-contact-cache-task12-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/contact_cache_test.zig").step);
    const dynamics_all_modes = b.step("test-dynamics-all-modes", "Run Task 13 dynamics tests in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| dynamics_all_modes.dependOn(&addZigTest(b, b.fmt("unit-dynamics-task13-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/dynamics_test.zig").step);
    const constraints_all_modes = b.step("test-constraints-all-modes", "Run Task 14 island and constraint row tests in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| constraints_all_modes.dependOn(&addZigTest(b, b.fmt("unit-constraints-task14-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/constraints_test.zig").step);
    const contact_solver_all_modes = b.step("test-contact-solver-all-modes", "Run Task 15 contact solver tests in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| contact_solver_all_modes.dependOn(&addZigTest(b, b.fmt("unit-contact-solver-task15-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/contact_solver_test.zig").step);
    const joints_all_modes = b.step("test-joints-all-modes", "Run Task 16 joint tests in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| joints_all_modes.dependOn(&addZigTest(b, b.fmt("unit-joints-task16-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/joints_test.zig").step);
    const joints_scenarios_all_modes = b.step("test-joints-scenarios-all-modes", "Run Task 16 joint scenarios in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| joints_scenarios_all_modes.dependOn(&addZigTest(b, b.fmt("scenarios-joints-task16-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/scenarios/joints_scenarios.zig").step);
    const queries_all_modes = b.step("test-queries-all-modes", "Run Task 17 query tests in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| queries_all_modes.dependOn(&addZigTest(b, b.fmt("unit-queries-task17-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/queries_test.zig").step);
    const sleeping_all_modes = b.step("test-sleeping-all-modes", "Run Task 18 sleeping tests in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| sleeping_all_modes.dependOn(&addZigTest(b, b.fmt("unit-sleeping-task18-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/sleeping_test.zig").step);
    const ccd_all_modes = b.step("test-ccd-all-modes", "Run Task 19 CCD tests in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| ccd_all_modes.dependOn(&addZigTest(b, b.fmt("unit-ccd-task19-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/ccd_test.zig").step);
    const pipeline_all_modes = b.step("test-pipeline-all-modes", "Run Task 20 World pipeline tests in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| pipeline_all_modes.dependOn(&addZigTest(b, b.fmt("unit-pipeline-task20-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/pipeline_test.zig").step);
    const pipeline_long_run = b.step("test-pipeline-long-run", "Run the Task 20 fixed World pipeline for one million ticks in ReleaseFast");
    const pipeline_long_run_test = addZigTest(b, "pipeline-long-run", target, .ReleaseFast, metadata, "tests/unit/pipeline_test.zig");
    pipeline_long_run_test.setEnvironmentVariable("GRAVITY_PIPELINE_LONG_RUN", "1");
    pipeline_long_run.dependOn(&pipeline_long_run_test.step);

    const snapshot_all_modes = b.step("test-snapshot-all-modes", "Run Task 21 snapshot, rollback, replay, and diff tests in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| {
        snapshot_all_modes.dependOn(&addZigTest(b, b.fmt("unit-snapshot-task21-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/unit/codec_test.zig").step);
        snapshot_all_modes.dependOn(&addZigTest(b, b.fmt("replay-cli-task21-{s}", .{@tagName(mode)}), target, mode, metadata, "tools/replay.zig").step);
    }

    const determinism = b.step("determinism", "Run deterministic foundation checks");
    determinism.dependOn(&addZigTest(b, "determinism", target, optimize, metadata, "tests/determinism/metadata_test.zig").step);

    const fuzz = b.step("fuzz", "Run bounded property probes over foundation metadata");
    const fuzz_core = b.step("fuzz-core", "Run bounded Task 03 memory and ID property probes");
    fuzz_core.dependOn(&addZigTest(b, "fuzz-core", target, optimize, metadata, "tests/fuzz/core_fuzz.zig").step);
    fuzz.dependOn(&addZigTest(b, "fuzz-foundation", target, optimize, metadata, "tests/fuzz/foundation_fuzz.zig").step);
    fuzz.dependOn(&addZigTest(b, "fuzz-fp", target, optimize, metadata, "tests/fuzz/fp_fuzz.zig").step);
    fuzz.dependOn(fuzz_core);
    fuzz.dependOn(&addZigTest(b, "fuzz-codec", target, optimize, metadata, "tests/fuzz/codec_fuzz.zig").step);
    fuzz.dependOn(&addZigTest(b, "fuzz-geometry", target, optimize, metadata, "tests/fuzz/geometry_fuzz.zig").step);
    fuzz.dependOn(&addZigTest(b, "fuzz-gjk", target, optimize, metadata, "tests/fuzz/gjk_fuzz.zig").step);
    fuzz.dependOn(&addZigTest(b, "fuzz-mesh", target, optimize, metadata, "tests/fuzz/mesh_fuzz.zig").step);
    fuzz.dependOn(&addZigTest(b, "fuzz-security", target, optimize, metadata, "tests/fuzz/security_fuzz.zig").step);
    fuzz.dependOn(&addZigTest(b, "fuzz-state-machine", target, optimize, metadata, "tests/fuzz/state_machine_fuzz.zig").step);
    const fuzz_all_modes = b.step("fuzz-all-modes", "Replay the complete bounded fuzz corpus in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| {
        inline for ([_][]const u8{ "foundation_fuzz.zig", "fp_fuzz.zig", "core_fuzz.zig", "codec_fuzz.zig", "geometry_fuzz.zig", "gjk_fuzz.zig", "mesh_fuzz.zig", "security_fuzz.zig", "state_machine_fuzz.zig" }) |source| {
            const name = source[0 .. source.len - 4];
            fuzz_all_modes.dependOn(&addZigTest(b, b.fmt("{s}-{s}", .{ name, @tagName(mode) }), target, mode, metadata, b.fmt("tests/fuzz/{s}", .{source})).step);
        }
    }
    const fuzz_minimize = b.step("fuzz-minimize", "Minimize a failing parser corpus while preserving its error class");
    const fuzz_minimize_run = addToolRun(b, "fuzz_minimize", target, .ReleaseSafe, metadata);
    if (b.args) |args| fuzz_minimize_run.addArgs(args);
    fuzz_minimize.dependOn(&fuzz_minimize_run.step);
    const fuzz_instrumented = b.step("fuzz-instrumented", "Build the Zig coverage-guided Task 25 parser harness");
    const parser_fuzz_artifact = addFuzzArtifact(b, "fuzz-parser-instrumented", target, .ReleaseSafe, metadata, "tests/fuzz/parser_coverage_fuzz.zig");
    fuzz_instrumented.dependOn(&b.addRunArtifact(parser_fuzz_artifact).step);

    const abi_test = b.step("abi-test", "Compile and run the C header compatibility test");
    abi_test.dependOn(&addAbiTest(b, target, optimize).step);
    abi_test.dependOn(&addZigTest(b, "abi-runtime", target, optimize, metadata, "tests/abi/abi_test.zig").step);
    abi_test.dependOn(&addAbiConsumer(b, "gravity-c11-consumer", "tests/abi/consumer.c", target, optimize, static_library).step);
    abi_test.dependOn(&addAbiConsumer(b, "gravity-cpp17-consumer", "tests/abi/consumer.cpp", target, optimize, static_library).step);
    const security_gate = b.step("security-gate", "Run Task 25 fuzz, ABI and Spindle lifecycle security gates");
    security_gate.dependOn(fuzz_all_modes);
    security_gate.dependOn(spindle_all_modes);
    security_gate.dependOn(abi_test);
    security_gate.dependOn(jobs_tsan);
    const security_audit_step = b.step("security-audit", "Audit Task 25 build graph, licenses and SBOM pinning");
    const security_audit = addToolRun(b, "security_audit", target, .ReleaseSafe, metadata);
    const spindle_gitlink = std.mem.trim(u8, b.run(&.{ "git", "rev-parse", "HEAD:third_party/spindle" }), " \t\r\n");
    const spindle_checkout = std.mem.trim(u8, b.run(&.{ "git", "-C", "third_party/spindle", "rev-parse", "HEAD" }), " \t\r\n");
    security_audit.addArg(spindle_gitlink);
    security_audit.addArg(spindle_checkout);
    security_audit_step.dependOn(&security_audit.step);
    security_gate.dependOn(security_audit_step);

    const abi_all_modes = b.step("test-abi-all-modes", "Run Task 22 ABI tests in Debug, ReleaseSafe, and ReleaseFast");
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| {
        abi_all_modes.dependOn(&addZigTest(b, b.fmt("abi-runtime-{s}", .{@tagName(mode)}), target, mode, metadata, "tests/abi/abi_test.zig").step);
    }

    const wasm_abi_smoke = b.step("abi-wasm-smoke", "Run the installed-style WASM C ABI consumer");
    const node_abi = b.addSystemCommand(&.{ "node", "tests/abi/wasm_consumer.mjs" });
    node_abi.addArtifactArg(wasm_library);
    wasm_abi_smoke.dependOn(&node_abi.step);

    if (target.result.os.tag == .windows) {
        const csharp_smoke = b.step("abi-csharp-smoke", "Run the C# P/Invoke consumer against the shared library");
        const dotnet = b.addSystemCommand(&.{ "dotnet", "run", "--project", "tests/abi/csharp/GravityAbiSmoke.csproj", "--artifacts-path", "zig-out/dotnet-artifacts", "--" });
        dotnet.addArtifactArg(shared_library.?);
        csharp_smoke.dependOn(&dotnet.step);
        abi_test.dependOn(&dotnet.step);
        const symbols = b.addSystemCommand(&.{ "pwsh", "-NoProfile", "-File", "tests/abi/check-symbols.ps1" });
        symbols.addArtifactArg(shared_library.?);
        symbols.addFileArg(b.path("tests/abi/abi-baseline-v1.json"));
        abi_test.dependOn(&symbols.step);
    }

    const benchmark = b.step("benchmark", "Run the foundation metadata benchmark");
    benchmark.dependOn(&addToolRun(b, "benchmark", target, optimize, metadata).step);

    const performance_small = b.step("performance-small", "Run the frozen Task 24 Small product benchmark");
    const small_run = addTask24Benchmark(b, target, .ReleaseFast, metadata);
    small_run.addArgs(&.{ "Small", "8", "2", "0" });
    performance_small.dependOn(&small_run.step);
    const performance_small_spindle = b.step("performance-small-spindle", "Enforce the Task 24 Small real-time rollback budget with Spindle");
    const small_spindle_run = addTask24Benchmark(b, target, .ReleaseFast, metadata);
    small_spindle_run.addArgs(&.{ "Small", "16", "4", "8", "gate" });
    performance_small_spindle.dependOn(&small_spindle_run.step);
    const performance_medium = b.step("performance-medium", "Run the frozen Task 24 Medium product benchmark");
    const medium_run = addTask24Benchmark(b, target, .ReleaseFast, metadata);
    medium_run.addArgs(&.{ "Medium", "8", "2", "0" });
    performance_medium.dependOn(&medium_run.step);
    const performance_medium_spindle = b.step("performance-medium-spindle", "Run Task 24 Medium with Spindle 8-worker hash validation");
    const medium_spindle_run = addTask24Benchmark(b, target, .ReleaseFast, metadata);
    medium_spindle_run.addArgs(&.{ "Medium", "8", "2", "8" });
    performance_medium_spindle.dependOn(&medium_spindle_run.step);
    const performance_scaling = b.step("performance-scaling", "Run Task 24 Medium and Stress worker scaling at 1/2/4/8 workers");
    var prior_scaling_run: ?*std.Build.Step = null;
    inline for ([_][]const u8{ "Medium", "Stress" }) |scene| {
        inline for ([_][]const u8{ "1", "2", "4", "8" }) |workers| {
            const run = addTask24Benchmark(b, target, .ReleaseFast, metadata);
            run.addArgs(&.{ scene, "2", "1", workers });
            if (prior_scaling_run) |prior| run.step.dependOn(prior);
            prior_scaling_run = &run.step;
        }
    }
    performance_scaling.dependOn(prior_scaling_run.?);
    const executor_overhead = b.step("performance-executor-overhead", "Measure Spindle submit/barrier/help-until/shutdown overhead and utilization");
    const executor_module = b.createModule(.{ .root_source_file = b.path("tools/task24_executor_benchmark.zig"), .target = target, .optimize = .ReleaseFast });
    executor_module.addImport("spindle_executor", addSpindleModule(b, target, .ReleaseFast));
    executor_overhead.dependOn(&b.addRunArtifact(b.addExecutable(.{ .name = "gravity-task24-executor-benchmark", .root_module = executor_module })).step);
    inline for ([_]struct { step: []const u8, scene: []const u8 }{ .{ .step = "performance-stress", .scene = "Stress" }, .{ .step = "performance-mesh-heavy", .scene = "MeshHeavy" }, .{ .step = "performance-joint-heavy", .scene = "JointHeavy" }, .{ .step = "performance-ccd", .scene = "CCD" } }) |entry| {
        const single = b.step(entry.step, b.fmt("Run the frozen Task 24 {s} product benchmark", .{entry.scene}));
        const run = addTask24Benchmark(b, target, .ReleaseFast, metadata);
        run.addArgs(&.{ entry.scene, "2", "1", "0" });
        single.dependOn(&run.step);
    }

    const performance_corpus = b.step("performance-corpus", "Run all six frozen Task 24 product benchmark scenes");
    var prior_performance_run: ?*std.Build.Step = null;
    inline for ([_][]const u8{ "Small", "Medium", "Stress", "MeshHeavy", "JointHeavy", "CCD" }) |scene| {
        const run = addTask24Benchmark(b, target, .ReleaseFast, metadata);
        run.addArgs(&.{ scene, "4", "1", "0" });
        if (prior_performance_run) |prior| run.step.dependOn(prior);
        prior_performance_run = &run.step;
    }
    performance_corpus.dependOn(prior_performance_run.?);
    const performance_gate = b.step("performance-gate", "Enforce Task 24 product budgets on a fixed reference runner");
    var prior_gate_run: ?*std.Build.Step = null;
    inline for ([_][]const u8{ "Small", "Medium", "Stress", "MeshHeavy", "JointHeavy", "CCD" }) |scene| {
        const run = addTask24Benchmark(b, target, .ReleaseFast, metadata);
        run.addArgs(&.{ scene, "64", "8", "8", "gate" });
        if (prior_gate_run) |prior| run.step.dependOn(prior);
        prior_gate_run = &run.step;
    }
    performance_gate.dependOn(prior_gate_run.?);
    const performance_ci = b.step("performance-ci", "Validate Task 24 schema and reject only significant regressions on shared CI");
    var prior_ci_run: ?*std.Build.Step = null;
    inline for ([_][]const u8{ "Small", "Medium" }) |scene| {
        const run = addTask24Benchmark(b, target, .ReleaseFast, metadata);
        run.addArgs(&.{ scene, "4", "2", "8", "ci" });
        if (prior_ci_run) |prior| run.step.dependOn(prior);
        prior_ci_run = &run.step;
    }
    performance_ci.dependOn(prior_ci_run.?);

    const wasm_validate = b.step("wasm-validate", "Run WASI golden vectors and benchmark with wasmtime");
    const wasm_golden = addZigTestArtifact(b, "golden-fp-wasi", wasi_target, .ReleaseSafe, metadata, "tests/golden/fp_golden.zig");
    const wasm_golden_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_golden_run.addArtifactArg(wasm_golden);
    wasm_validate.dependOn(&wasm_golden_run.step);
    const wasm_geometry = addZigTestArtifact(b, "unit-geometry-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/geometry_test.zig");
    const wasm_geometry_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_geometry_run.addArtifactArg(wasm_geometry);
    wasm_validate.dependOn(&wasm_geometry_run.step);
    const wasm_core = addZigTestArtifact(b, "unit-core-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/core_test.zig");
    const wasm_core_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_core_run.addArtifactArg(wasm_core);
    wasm_validate.dependOn(&wasm_core_run.step);
    const wasm_codec = addZigTestArtifact(b, "unit-codec-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/codec_test.zig");
    const wasm_codec_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_codec_run.addArtifactArg(wasm_codec);
    wasm_validate.dependOn(&wasm_codec_run.step);
    const wasm_replay_cli = addZigTestArtifact(b, "replay-cli-task21-wasi", wasi_target, .ReleaseSafe, metadata, "tools/replay.zig");
    const wasm_replay_cli_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_replay_cli_run.addArtifactArg(wasm_replay_cli);
    wasm_validate.dependOn(&wasm_replay_cli_run.step);
    const wasm_benchmark = addTool(b, "benchmark", wasi_target, .ReleaseSafe, metadata);
    const wasm_benchmark_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_benchmark_run.addArtifactArg(wasm_benchmark);
    wasm_validate.dependOn(&wasm_benchmark_run.step);
    const wasm_assets = addZigTestArtifact(b, "unit-assets-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/asset_test.zig");
    const wasm_assets_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_assets_run.addArtifactArg(wasm_assets);
    wasm_validate.dependOn(&wasm_assets_run.step);
    const wasm_baked = addZigTestArtifact(b, "unit-baked-geometry-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/baked_geometry_test.zig");
    const wasm_baked_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_baked_run.addArtifactArg(wasm_baked);
    wasm_validate.dependOn(&wasm_baked_run.step);
    const wasm_runtime_shapes = addZigTestArtifact(b, "unit-runtime-shapes-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/runtime_shapes_test.zig");
    const wasm_runtime_shapes_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_runtime_shapes_run.addArtifactArg(wasm_runtime_shapes);
    wasm_validate.dependOn(&wasm_runtime_shapes_run.step);
    const wasm_broadphase = addZigTestArtifact(b, "unit-broadphase-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/broadphase_test.zig");
    const wasm_broadphase_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_broadphase_run.addArtifactArg(wasm_broadphase);
    wasm_validate.dependOn(&wasm_broadphase_run.step);
    const wasm_analytic = addZigTestArtifact(b, "unit-analytic-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/analytic_collision_test.zig");
    const wasm_analytic_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_analytic_run.addArtifactArg(wasm_analytic);
    wasm_validate.dependOn(&wasm_analytic_run.step);
    const wasm_gjk = addZigTestArtifact(b, "unit-gjk-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/gjk_test.zig");
    const wasm_gjk_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_gjk_run.addArtifactArg(wasm_gjk);
    wasm_validate.dependOn(&wasm_gjk_run.step);
    const wasm_mesh_collision = addZigTestArtifact(b, "unit-mesh-collision-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/mesh_collision_test.zig");
    const wasm_mesh_collision_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_mesh_collision_run.addArtifactArg(wasm_mesh_collision);
    wasm_validate.dependOn(&wasm_mesh_collision_run.step);
    const wasm_contact_cache = addZigTestArtifact(b, "unit-contact-cache-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/contact_cache_test.zig");
    const wasm_contact_cache_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_contact_cache_run.addArtifactArg(wasm_contact_cache);
    wasm_validate.dependOn(&wasm_contact_cache_run.step);
    const wasm_dynamics = addZigTestArtifact(b, "unit-dynamics-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/dynamics_test.zig");
    const wasm_dynamics_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_dynamics_run.addArtifactArg(wasm_dynamics);
    wasm_validate.dependOn(&wasm_dynamics_run.step);
    const wasm_constraints = addZigTestArtifact(b, "unit-constraints-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/constraints_test.zig");
    const wasm_constraints_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_constraints_run.addArtifactArg(wasm_constraints);
    wasm_validate.dependOn(&wasm_constraints_run.step);
    const wasm_contact_solver = addZigTestArtifact(b, "unit-contact-solver-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/contact_solver_test.zig");
    const wasm_contact_solver_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_contact_solver_run.addArtifactArg(wasm_contact_solver);
    wasm_validate.dependOn(&wasm_contact_solver_run.step);
    const wasm_joints = addZigTestArtifact(b, "unit-joints-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/joints_test.zig");
    const wasm_joints_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_joints_run.addArtifactArg(wasm_joints);
    wasm_validate.dependOn(&wasm_joints_run.step);
    const wasm_joints_scenarios = addZigTestArtifact(b, "scenarios-joints-wasi", wasi_target, .ReleaseSafe, metadata, "tests/scenarios/joints_scenarios.zig");
    const wasm_joints_scenarios_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_joints_scenarios_run.addArtifactArg(wasm_joints_scenarios);
    wasm_validate.dependOn(&wasm_joints_scenarios_run.step);
    const wasm_queries = addZigTestArtifact(b, "unit-queries-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/queries_test.zig");
    const wasm_queries_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_queries_run.addArtifactArg(wasm_queries);
    wasm_validate.dependOn(&wasm_queries_run.step);
    const wasm_sleeping = addZigTestArtifact(b, "unit-sleeping-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/sleeping_test.zig");
    const wasm_sleeping_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_sleeping_run.addArtifactArg(wasm_sleeping);
    wasm_validate.dependOn(&wasm_sleeping_run.step);
    const wasm_ccd = addZigTestArtifact(b, "unit-ccd-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/ccd_test.zig");
    const wasm_ccd_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_ccd_run.addArtifactArg(wasm_ccd);
    wasm_validate.dependOn(&wasm_ccd_run.step);
    const wasm_pipeline = addZigTestArtifact(b, "unit-pipeline-wasi", wasi_target, .ReleaseSafe, metadata, "tests/unit/pipeline_test.zig");
    const wasm_pipeline_run = b.addSystemCommand(&.{ "wasmtime", "run" });
    wasm_pipeline_run.addArtifactArg(wasm_pipeline);
    wasm_validate.dependOn(&wasm_pipeline_run.step);
    const wasm_pipeline_long_run = b.step("test-pipeline-long-run-wasm", "Run the Task 20 one-million-tick pipeline hash gate under WASI");
    const wasm_pipeline_long = addZigTestArtifact(b, "unit-pipeline-long-wasi", wasi_target, .ReleaseFast, metadata, "tests/unit/pipeline_test.zig");
    const wasm_pipeline_long_exec = b.addSystemCommand(&.{ "wasmtime", "run", "--env", "GRAVITY_PIPELINE_LONG_RUN=1" });
    wasm_pipeline_long_exec.addArtifactArg(wasm_pipeline_long);
    wasm_pipeline_long_run.dependOn(&wasm_pipeline_long_exec.step);

    const arm_validate = b.step("arm-validate", "Build ARM64 Linux golden vectors and benchmark for qemu-aarch64");
    const arm_golden = addZigTestArtifact(b, "golden-fp-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/golden/fp_golden.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_golden, .{ .dest_sub_path = "gravity-fp-golden-aarch64" }).step);
    const arm_geometry = addZigTestArtifact(b, "unit-geometry-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/geometry_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_geometry, .{ .dest_sub_path = "gravity-geometry-aarch64" }).step);
    const arm_core = addZigTestArtifact(b, "unit-core-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/core_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_core, .{ .dest_sub_path = "gravity-core-aarch64" }).step);
    const arm_codec = addZigTestArtifact(b, "unit-codec-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/codec_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_codec, .{ .dest_sub_path = "gravity-codec-aarch64" }).step);
    const arm_benchmark = addTool(b, "benchmark", arm_linux_target, .ReleaseSafe, metadata);
    arm_validate.dependOn(&b.addInstallArtifact(arm_benchmark, .{ .dest_sub_path = "gravity-benchmark-aarch64" }).step);
    const arm_assets = addZigTestArtifact(b, "unit-assets-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/asset_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_assets, .{ .dest_sub_path = "gravity-assets-aarch64" }).step);
    const arm_baked = addZigTestArtifact(b, "unit-baked-geometry-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/baked_geometry_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_baked, .{ .dest_sub_path = "gravity-baked-geometry-aarch64" }).step);
    const arm_runtime_shapes = addZigTestArtifact(b, "unit-runtime-shapes-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/runtime_shapes_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_runtime_shapes, .{ .dest_sub_path = "gravity-runtime-shapes-aarch64" }).step);
    const arm_broadphase = addZigTestArtifact(b, "unit-broadphase-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/broadphase_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_broadphase, .{ .dest_sub_path = "gravity-broadphase-aarch64" }).step);
    const arm_analytic = addZigTestArtifact(b, "unit-analytic-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/analytic_collision_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_analytic, .{ .dest_sub_path = "gravity-analytic-aarch64" }).step);
    const arm_gjk = addZigTestArtifact(b, "unit-gjk-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/gjk_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_gjk, .{ .dest_sub_path = "gravity-gjk-aarch64" }).step);
    const arm_mesh_collision = addZigTestArtifact(b, "unit-mesh-collision-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/mesh_collision_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_mesh_collision, .{ .dest_sub_path = "gravity-mesh-collision-aarch64" }).step);
    const arm_contact_cache = addZigTestArtifact(b, "unit-contact-cache-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/contact_cache_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_contact_cache, .{ .dest_sub_path = "gravity-contact-cache-aarch64" }).step);
    const arm_dynamics = addZigTestArtifact(b, "unit-dynamics-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/dynamics_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_dynamics, .{ .dest_sub_path = "gravity-dynamics-aarch64" }).step);
    const arm_constraints = addZigTestArtifact(b, "unit-constraints-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/constraints_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_constraints, .{ .dest_sub_path = "gravity-constraints-aarch64" }).step);
    const arm_contact_solver = addZigTestArtifact(b, "unit-contact-solver-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/contact_solver_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_contact_solver, .{ .dest_sub_path = "gravity-contact-solver-aarch64" }).step);
    const arm_joints = addZigTestArtifact(b, "unit-joints-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/joints_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_joints, .{ .dest_sub_path = "gravity-joints-aarch64" }).step);
    const arm_joints_scenarios = addZigTestArtifact(b, "scenarios-joints-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/scenarios/joints_scenarios.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_joints_scenarios, .{ .dest_sub_path = "gravity-joints-scenarios-aarch64" }).step);
    const arm_queries = addZigTestArtifact(b, "unit-queries-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/queries_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_queries, .{ .dest_sub_path = "gravity-queries-aarch64" }).step);
    const arm_sleeping = addZigTestArtifact(b, "unit-sleeping-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/sleeping_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_sleeping, .{ .dest_sub_path = "gravity-sleeping-aarch64" }).step);
    const arm_ccd = addZigTestArtifact(b, "unit-ccd-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/ccd_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_ccd, .{ .dest_sub_path = "gravity-ccd-aarch64" }).step);
    const arm_pipeline = addZigTestArtifact(b, "unit-pipeline-aarch64", arm_linux_target, .ReleaseSafe, metadata, "tests/unit/pipeline_test.zig");
    arm_validate.dependOn(&b.addInstallArtifact(arm_pipeline, .{ .dest_sub_path = "gravity-pipeline-aarch64" }).step);

    const tools_step = b.step("tools", "Build foundation command-line tools");
    inline for ([_][]const u8{ "bake", "replay", "state_diff", "benchmark", "fuzz_minimize", "security_audit" }) |name| {
        const tool = addTool(b, name, target, optimize, metadata);
        tools_step.dependOn(&b.addInstallArtifact(tool, .{}).step);
    }

    const spindle_check = b.step("spindle-check", "Validate the pinned minimal Spindle executor profile");
    spindle_check.dependOn(&addSpindleTest(b, "spindle-integration", target, optimize, addSpindleModule(b, target, optimize)).step);

    const demo = b.step("demo", "Build and verify the Task 27 React/Three WASM demo and baked assets");
    const install_demo_wasm = b.addInstallArtifact(wasm_library, .{ .dest_sub_path = "demo-assets/gravity.wasm" });
    const demo_asset_tool = addTool(b, "demo_assets", b.graph.host, .ReleaseSafe, metadata);
    const generate_demo_assets = b.addRunArtifact(demo_asset_tool);
    const demo_hull = generate_demo_assets.addOutputFileArg("hull.grav");
    const demo_mesh = generate_demo_assets.addOutputFileArg("mesh.grav");
    const demo_height = generate_demo_assets.addOutputFileArg("height.grav");
    const demo_compound = generate_demo_assets.addOutputFileArg("compound.grav");
    const install_demo_hull = b.addInstallFile(demo_hull, "bin/demo-assets/hull.grav");
    const install_demo_mesh = b.addInstallFile(demo_mesh, "bin/demo-assets/mesh.grav");
    const install_demo_height = b.addInstallFile(demo_height, "bin/demo-assets/height.grav");
    const install_demo_compound = b.addInstallFile(demo_compound, "bin/demo-assets/compound.grav");
    const verify_demo_wasm = b.addSystemCommand(&.{ "node", "demo/web/scripts/verify-wasm.mjs" });
    verify_demo_wasm.addArtifactArg(wasm_library);
    verify_demo_wasm.step.dependOn(&install_demo_wasm.step);
    verify_demo_wasm.step.dependOn(&install_demo_hull.step);
    verify_demo_wasm.step.dependOn(&install_demo_mesh.step);
    verify_demo_wasm.step.dependOn(&install_demo_height.step);
    verify_demo_wasm.step.dependOn(&install_demo_compound.step);
    const abi_schema_check = b.addSystemCommand(&.{ "node", "demo/web/scripts/generate-abi.mjs", "--check" });
    abi_schema_check.step.dependOn(&verify_demo_wasm.step);
    const demo_install = b.addSystemCommand(&.{ "node", "demo/web/scripts/install.mjs" });
    demo_install.step.dependOn(&abi_schema_check.step);
    const vite_build = b.addSystemCommand(&.{ "pnpm", "--dir", "demo/web", "run", "build" });
    vite_build.setEnvironmentVariable("GRAVITY_INSTALL_PREFIX", b.pathFromRoot(b.getInstallPath(.prefix, "")));
    vite_build.step.dependOn(&demo_install.step);
    demo.dependOn(&vite_build.step);

    const demo_test = b.step("demo-test", "Run Task 27 ABI, case, rollback, lifecycle, DOM and screenshot tests");
    const playwright = b.addSystemCommand(&.{ "pnpm", "--dir", "demo/web", "run", "test" });
    playwright.step.dependOn(&vite_build.step);
    demo_test.dependOn(&playwright.step);

    const demo_isolation = b.step("demo-isolation", "Build an external-style Zig core consumer without frontend dependencies");
    const consumer = b.addSystemCommand(&.{ "zig", "build", "--build-file", "tests/isolation/consumer/build.zig", "--cache-dir", ".zig-cache/demo-consumer" });
    demo_isolation.dependOn(&consumer.step);

    const demo_run = b.step("demo-run", "Build and start the Task 27 local React/Three demo");
    const vite_run = b.addSystemCommand(&.{ "pnpm", "--dir", "demo/web", "run", "dev" });
    vite_run.step.dependOn(&vite_build.step);
    demo_run.dependOn(&vite_run.step);

    const release = b.step("release", "Build deterministic Task 28 source, native, WASM and Demo release packages");
    const release_package = b.addSystemCommand(&.{ "node", "tools/release.mjs", "--generate" });
    release_package.addArg(b.getInstallPath(.prefix, ""));
    release_package.addArg(package_version);
    release_package.addArg(std.mem.trim(u8, b.run(&.{ "git", "rev-parse", "HEAD" }), " \t\r\n"));
    release_package.step.dependOn(abi_artifacts);
    release_package.step.dependOn(&vite_build.step);
    release.dependOn(&release_package.step);

    const release_check = b.step("release-check", "Verify Task 28 release manifest and SHA-256 checksums");
    const verify_release = b.addSystemCommand(&.{ "node", "tools/release.mjs", "--verify" });
    verify_release.addArg(b.getInstallPath(.prefix, "release"));
    verify_release.step.dependOn(&release_package.step);
    release_check.dependOn(&verify_release.step);

    const qualification_audit = b.step("qualification-audit", "Audit Task 28 records, unfinished code, documents, versions, SBOM and CI matrix");
    const run_qualification_audit = b.addSystemCommand(&.{ "node", "tools/qualification_audit.mjs" });
    qualification_audit.dependOn(&run_qualification_audit.step);
    release_check.dependOn(qualification_audit);

    const qualification_native = b.step("qualification-native", "Run the complete Task 28 native mode, worker, long-run, fuzz, ABI, security and performance gate");
    inline for (.{ core_all_modes, assets_all_modes, geometry_all_modes, runtime_shapes_all_modes, broadphase_all_modes, analytic_all_modes, gjk_all_modes, mesh_collision_all_modes, contact_cache_all_modes, dynamics_all_modes, constraints_all_modes, contact_solver_all_modes, joints_all_modes, joints_scenarios_all_modes, queries_all_modes, sleeping_all_modes, ccd_all_modes, pipeline_all_modes, snapshot_all_modes, abi_all_modes }) |gate| qualification_native.dependOn(gate);
    qualification_native.dependOn(pipeline_long_run);
    qualification_native.dependOn(fuzz_all_modes);
    qualification_native.dependOn(job_scaling);
    qualification_native.dependOn(security_gate);
    qualification_native.dependOn(performance_ci);
    qualification_native.dependOn(determinism);
    qualification_native.dependOn(qualification_audit);

    const product_qualification = b.step("product-qualification", "Run the complete Task 28 local product qualification and release gate");
    product_qualification.dependOn(qualification_native);
    product_qualification.dependOn(wasm_validate);
    product_qualification.dependOn(wasm_pipeline_long_run);
    product_qualification.dependOn(wasm_abi_smoke);
    product_qualification.dependOn(demo_test);
    product_qualification.dependOn(release_check);
}

fn addBuildMetadata(b: *std.Build) *std.Build.Step.Options {
    const commit = std.mem.trim(u8, b.run(&.{ "git", "rev-parse", "HEAD" }), " \t\r\n");
    const options = b.addOptions();
    options.addOption([]const u8, "commit", commit);
    options.addOption([]const u8, "zig_version", required_zig_version);
    return options;
}

fn addCoreModule(
    b: *std.Build,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    metadata: *std.Build.Step.Options,
) *std.Build.Module {
    const module = b.createModule(.{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
        .link_libc = false,
        .link_libcpp = false,
    });
    module.addImport("build_options", metadata.createModule());
    module.addImport("gravity_abi_options", addAbiOptionsModule(b, target));
    const jobs = b.createModule(.{ .root_source_file = b.path("src/jobs/dispatcher.zig"), .target = target, .optimize = optimize });
    const host_jobs = b.createModule(.{ .root_source_file = b.path(if (target.result.cpu.arch == .wasm32) "src/jobs/wasm_serial.zig" else "src/jobs/host_dispatcher.zig"), .target = target, .optimize = optimize });
    if (target.result.cpu.arch != .wasm32) host_jobs.addImport("gravity_jobs", jobs);
    module.addImport("gravity_jobs", jobs);
    module.addImport("gravity_host_jobs", host_jobs);
    return module;
}

fn addAbiModule(
    b: *std.Build,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    metadata: *std.Build.Step.Options,
) *std.Build.Module {
    const module = b.createModule(.{
        .root_source_file = b.path("src/c_abi.zig"),
        .target = target,
        .optimize = optimize,
        .link_libc = false,
        .link_libcpp = false,
    });
    module.addImport("build_options", metadata.createModule());
    module.addImport("gravity_abi_options", addAbiOptionsModule(b, target));
    const jobs = b.createModule(.{ .root_source_file = b.path("src/jobs/dispatcher.zig"), .target = target, .optimize = optimize });
    const host_jobs = b.createModule(.{ .root_source_file = b.path(if (target.result.cpu.arch == .wasm32) "src/jobs/wasm_serial.zig" else "src/jobs/host_dispatcher.zig"), .target = target, .optimize = optimize });
    if (target.result.cpu.arch != .wasm32) host_jobs.addImport("gravity_jobs", jobs);
    module.addImport("gravity_jobs", jobs);
    module.addImport("gravity_host_jobs", host_jobs);
    return module;
}

fn addAbiOptionsModule(b: *std.Build, target: std.Build.ResolvedTarget) *std.Build.Module {
    const options = b.addOptions();
    options.addOption(bool, "serial_wasm", target.result.cpu.arch == .wasm32);
    return options.createModule();
}

fn addSpindleModule(
    b: *std.Build,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
) *std.Build.Module {
    const module = b.createModule(.{
        .root_source_file = b.path("third_party/spindle/src/executor.zig"),
        .target = target,
        .optimize = optimize,
    });
    return module;
}

fn addSpindleTest(
    b: *std.Build,
    name: []const u8,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    spindle: *std.Build.Module,
) *std.Build.Step.Run {
    const module = b.createModule(.{
        .root_source_file = b.path("tests/unit/spindle_integration_test.zig"),
        .target = target,
        .optimize = optimize,
    });
    module.addImport("spindle", spindle);
    return b.addRunArtifact(b.addTest(.{ .name = name, .root_module = module }));
}

fn addJobsTest(
    b: *std.Build,
    name: []const u8,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    metadata: *std.Build.Step.Options,
    spindle: *std.Build.Module,
) *std.Build.Step.Run {
    const jobs = b.createModule(.{ .root_source_file = b.path("src/jobs/dispatcher.zig"), .target = target, .optimize = optimize });
    const spindle_jobs = b.createModule(.{ .root_source_file = b.path("src/jobs/spindle_dispatcher.zig"), .target = target, .optimize = optimize });
    spindle_jobs.addImport("gravity_jobs", jobs);
    spindle_jobs.addImport("spindle_executor", spindle);
    const host_jobs = b.createModule(.{ .root_source_file = b.path("src/jobs/host_dispatcher.zig"), .target = target, .optimize = optimize });
    host_jobs.addImport("gravity_jobs", jobs);
    const gravity = b.createModule(.{ .root_source_file = b.path("src/root.zig"), .target = target, .optimize = optimize });
    gravity.addImport("gravity_abi_options", addAbiOptionsModule(b, target));
    gravity.addImport("build_options", metadata.createModule());
    gravity.addImport("gravity_jobs", jobs);
    gravity.addImport("gravity_host_jobs", host_jobs);
    const module = b.createModule(.{ .root_source_file = b.path("tests/unit/jobs_dispatcher_test.zig"), .target = target, .optimize = optimize });
    module.addImport("gravity_jobs", jobs);
    module.addImport("gravity_spindle_jobs", spindle_jobs);
    module.addImport("gravity_host_jobs", host_jobs);
    module.addImport("spindle_executor", spindle);
    module.addImport("gravity", gravity);
    return b.addRunArtifact(b.addTest(.{ .name = name, .root_module = module }));
}

fn addJobScaling(
    b: *std.Build,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    metadata: *std.Build.Step.Options,
    spindle: *std.Build.Module,
) *std.Build.Step.Run {
    const jobs = b.createModule(.{ .root_source_file = b.path("src/jobs/dispatcher.zig"), .target = target, .optimize = optimize });
    const spindle_jobs = b.createModule(.{ .root_source_file = b.path("src/jobs/spindle_dispatcher.zig"), .target = target, .optimize = optimize });
    spindle_jobs.addImport("gravity_jobs", jobs);
    spindle_jobs.addImport("spindle_executor", spindle);
    const host_jobs = b.createModule(.{ .root_source_file = b.path("src/jobs/host_dispatcher.zig"), .target = target, .optimize = optimize });
    host_jobs.addImport("gravity_jobs", jobs);
    const gravity = b.createModule(.{ .root_source_file = b.path("src/root.zig"), .target = target, .optimize = optimize });
    gravity.addImport("gravity_abi_options", addAbiOptionsModule(b, target));
    gravity.addImport("build_options", metadata.createModule());
    gravity.addImport("gravity_jobs", jobs);
    gravity.addImport("gravity_host_jobs", host_jobs);
    const module = b.createModule(.{ .root_source_file = b.path("tools/job_scaling.zig"), .target = target, .optimize = optimize });
    module.addImport("gravity_jobs", jobs);
    module.addImport("gravity_spindle_jobs", spindle_jobs);
    module.addImport("spindle_executor", spindle);
    module.addImport("gravity", gravity);
    return b.addRunArtifact(b.addExecutable(.{ .name = "gravity-job-scaling", .root_module = module }));
}

fn addJobsTsanTest(b: *std.Build, target: std.Build.ResolvedTarget, metadata: *std.Build.Step.Options) *std.Build.Step.Compile {
    const optimize: std.builtin.OptimizeMode = .ReleaseSafe;
    const jobs = b.createModule(.{ .root_source_file = b.path("src/jobs/dispatcher.zig"), .target = target, .optimize = optimize, .sanitize_thread = true });
    const spindle = b.createModule(.{ .root_source_file = b.path("third_party/spindle/src/executor.zig"), .target = target, .optimize = optimize, .sanitize_thread = true });
    const spindle_jobs = b.createModule(.{ .root_source_file = b.path("src/jobs/spindle_dispatcher.zig"), .target = target, .optimize = optimize, .sanitize_thread = true });
    spindle_jobs.addImport("gravity_jobs", jobs);
    spindle_jobs.addImport("spindle_executor", spindle);
    const host_jobs = b.createModule(.{ .root_source_file = b.path("src/jobs/host_dispatcher.zig"), .target = target, .optimize = optimize, .sanitize_thread = true });
    host_jobs.addImport("gravity_jobs", jobs);
    const gravity = b.createModule(.{ .root_source_file = b.path("src/root.zig"), .target = target, .optimize = optimize, .sanitize_thread = true });
    gravity.addImport("gravity_abi_options", addAbiOptionsModule(b, target));
    gravity.addImport("build_options", metadata.createModule());
    gravity.addImport("gravity_jobs", jobs);
    gravity.addImport("gravity_host_jobs", host_jobs);
    const module = b.createModule(.{ .root_source_file = b.path("tests/unit/jobs_dispatcher_test.zig"), .target = target, .optimize = optimize, .sanitize_thread = true });
    module.addImport("gravity_jobs", jobs);
    module.addImport("gravity_spindle_jobs", spindle_jobs);
    module.addImport("gravity_host_jobs", host_jobs);
    module.addImport("spindle_executor", spindle);
    module.addImport("gravity", gravity);
    return b.addTest(.{ .name = "jobs-dispatcher-tsan", .root_module = module });
}

fn addZigTest(
    b: *std.Build,
    name: []const u8,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    metadata: *std.Build.Step.Options,
    source: []const u8,
) *std.Build.Step.Run {
    return b.addRunArtifact(addZigTestArtifact(b, name, target, optimize, metadata, source));
}

fn addZigTestArtifact(
    b: *std.Build,
    name: []const u8,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    metadata: *std.Build.Step.Options,
    source: []const u8,
) *std.Build.Step.Compile {
    const test_module = b.createModule(.{
        .root_source_file = b.path(source),
        .target = target,
        .optimize = optimize,
    });
    test_module.addImport("gravity", addCoreModule(b, target, optimize, metadata));
    return b.addTest(.{ .name = name, .root_module = test_module });
}

fn addFuzzArtifact(
    b: *std.Build,
    name: []const u8,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    metadata: *std.Build.Step.Options,
    source: []const u8,
) *std.Build.Step.Compile {
    const test_module = b.createModule(.{
        .root_source_file = b.path(source),
        .target = target,
        .optimize = optimize,
    });
    test_module.addImport("gravity", addCoreModule(b, target, optimize, metadata));
    return b.addTest(.{ .name = name, .root_module = test_module });
}

fn addAbiTest(
    b: *std.Build,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
) *std.Build.Step.Run {
    const module = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .link_libc = true,
    });
    module.addCSourceFile(.{ .file = b.path("tests/abi/header_test.c") });
    module.addIncludePath(b.path("include"));
    const executable = b.addExecutable(.{
        .name = "gravity-abi-header-test",
        .root_module = module,
    });
    return b.addRunArtifact(executable);
}

fn addAbiConsumer(
    b: *std.Build,
    name: []const u8,
    source: []const u8,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    library: *std.Build.Step.Compile,
) *std.Build.Step.Run {
    const module = b.createModule(.{ .target = target, .optimize = optimize, .link_libc = true, .link_libcpp = std.mem.endsWith(u8, source, ".cpp") });
    module.addCSourceFile(.{ .file = b.path(source), .flags = if (std.mem.endsWith(u8, source, ".cpp")) &.{"-std=c++17"} else &.{"-std=c11"} });
    module.addIncludePath(b.path("include"));
    module.linkLibrary(library);
    const executable = b.addExecutable(.{ .name = name, .root_module = module });
    return b.addRunArtifact(executable);
}

fn addTool(
    b: *std.Build,
    name: []const u8,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    metadata: *std.Build.Step.Options,
) *std.Build.Step.Compile {
    const source = b.fmt("tools/{s}.zig", .{name});
    const tool_module = b.createModule(.{
        .root_source_file = b.path(source),
        .target = target,
        .optimize = optimize,
    });
    tool_module.addImport("gravity", addCoreModule(b, target, optimize, metadata));
    return b.addExecutable(.{ .name = b.fmt("gravity-{s}", .{name}), .root_module = tool_module });
}

fn addToolRun(
    b: *std.Build,
    name: []const u8,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    metadata: *std.Build.Step.Options,
) *std.Build.Step.Run {
    return b.addRunArtifact(addTool(b, name, target, optimize, metadata));
}

fn addTask24Benchmark(
    b: *std.Build,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    metadata: *std.Build.Step.Options,
) *std.Build.Step.Run {
    const jobs = b.createModule(.{ .root_source_file = b.path("src/jobs/dispatcher.zig"), .target = target, .optimize = optimize });
    const spindle = addSpindleModule(b, target, optimize);
    const spindle_jobs = b.createModule(.{ .root_source_file = b.path("src/jobs/spindle_dispatcher.zig"), .target = target, .optimize = optimize });
    spindle_jobs.addImport("gravity_jobs", jobs);
    spindle_jobs.addImport("spindle_executor", spindle);
    const host_jobs = b.createModule(.{ .root_source_file = b.path("src/jobs/host_dispatcher.zig"), .target = target, .optimize = optimize });
    host_jobs.addImport("gravity_jobs", jobs);
    const gravity = b.createModule(.{ .root_source_file = b.path("src/root.zig"), .target = target, .optimize = optimize });
    gravity.addImport("gravity_abi_options", addAbiOptionsModule(b, target));
    gravity.addImport("build_options", metadata.createModule());
    gravity.addImport("gravity_jobs", jobs);
    gravity.addImport("gravity_host_jobs", host_jobs);
    const module = b.createModule(.{ .root_source_file = b.path("tools/task24_benchmark.zig"), .target = target, .optimize = optimize });
    module.addImport("gravity", gravity);
    module.addImport("gravity_jobs", jobs);
    module.addImport("gravity_spindle_jobs", spindle_jobs);
    module.addImport("spindle_executor", spindle);
    return b.addRunArtifact(b.addExecutable(.{ .name = "gravity-task24-benchmark", .root_module = module }));
}
