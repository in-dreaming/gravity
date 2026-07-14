const std = @import("std");
const builtin = @import("builtin");

const required_zig_version = "0.16.0";

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
    const core_module = addCoreModule(b, target, optimize, metadata);

    const static_library = b.addLibrary(.{
        .name = "gravity",
        .root_module = core_module,
        .linkage = .static,
    });
    b.installArtifact(static_library);

    if (target.result.cpu.arch != .wasm32) {
        const shared_library = b.addLibrary(.{
            .name = "gravity",
            .root_module = core_module,
            .linkage = .dynamic,
        });
        b.installArtifact(shared_library);
    }

    const wasm_target = b.resolveTargetQuery(.{
        .cpu_arch = .wasm32,
        .os_tag = .freestanding,
        .abi = .none,
    });
    const wasm_module = addCoreModule(b, wasm_target, .ReleaseSmall, metadata);
    const wasm_library = b.addLibrary(.{
        .name = "gravity_foundation_wasm",
        .root_module = wasm_module,
        .linkage = .static,
    });
    const install_wasm = b.addInstallArtifact(wasm_library, .{});

    const wasi_target = b.resolveTargetQuery(.{
        .cpu_arch = .wasm32,
        .os_tag = .wasi,
        .abi = .none,
    });
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
    inline for ([_]std.builtin.OptimizeMode{ .Debug, .ReleaseSafe, .ReleaseFast }) |mode| {
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

    const abi_test = b.step("abi-test", "Compile and run the C header compatibility test");
    abi_test.dependOn(&addAbiTest(b, target, optimize).step);

    const benchmark = b.step("benchmark", "Run the foundation metadata benchmark");
    benchmark.dependOn(&addToolRun(b, "benchmark", target, optimize, metadata).step);

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
    inline for ([_][]const u8{ "bake", "replay", "state_diff", "benchmark" }) |name| {
        const tool = addTool(b, name, target, optimize, metadata);
        tools_step.dependOn(&b.addInstallArtifact(tool, .{}).step);
    }

    const demo = b.step("demo", "Build the core WASM artifact and verify frontend prerequisites");
    const node_check = b.addSystemCommand(&.{ "node", "--version" });
    node_check.step.dependOn(&install_wasm.step);
    const pnpm_check = b.addSystemCommand(&.{ "pnpm", "--version" });
    pnpm_check.step.dependOn(&node_check.step);
    demo.dependOn(&pnpm_check.step);

    const demo_run = b.step("demo-run", "Start the Task 26 frontend when it is available");
    const vite_run = b.addSystemCommand(&.{ "pnpm", "--dir", "demo/web", "run", "dev" });
    vite_run.step.dependOn(&pnpm_check.step);
    demo_run.dependOn(&vite_run.step);
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
    return module;
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
