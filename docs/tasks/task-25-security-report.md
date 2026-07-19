# Task 25 security qualification report

Date: 2026-07-19

## Delivered surface

- Coverage-guided and deterministic bounded fuzzing covers source asset JSON, baked TLV, GRAVSNAP, GRAVREPL, command streams, canonical decimals, C ABI commands/queries/pointers, fixed-point IDs, geometry, GJK/EPA, BVH, triangle/mesh pairs, World/joint/rollback state transitions, and the Gravity Spindle adapter lifecycle.
- Parser workspaces and iteration counts are fixed. Malformed snapshot mutation probes assert that rejected loads leave the canonical World hash unchanged.
- `fuzz-minimize` preserves the exact parser error class while removing chunks. The first saved regression is `tests/fuzz/corpus/asset-leading-decimal.json`.
- `security-audit` enforces the executor-only Spindle import, excludes aggregate Runtime/ECS/Workflow/SQLite/archive paths, and checks the gitlink commit against the MIT license, notices, and SPDX SBOM.
- `SECURITY.md`, `docs/security/threat-model.md`, `docs/security/fuzzing.md`, and `docs/security/sbom.spdx.json` define reporting, trust boundaries, entry limits, residual risk, and qualification commands.

## Defects found and fixed

1. Source asset validation accepted `.5` and `-.5`, although the canonical fixed-point parser rejects decimals without an integer component. Validation now matches the canonical parser and the minimized corpus freezes both forms.
2. Valid hostile snapshot/parser failures including trailing bytes, invalid ordering/enums, duplicate sections, and length overflow escaped the C ABI as `internal`. They now map to `corrupt_input`; ABI mutation tests reject any recurrence of `internal`.
3. The ABI dispatcher expectation still counted the pre-Task-24 partition plan. It now freezes the actual 18-batch plan.

## Frozen evidence

| Gate | Result |
| --- | --- |
| Linux Zig coverage-guided parser | 10,386,513 cumulative runs; 1,848 unique runs; 1320/11329 edges (11.65%); no crash/OOM/timeout |
| deterministic corpus, three modes | 56/56 build steps; 45/45 tests passed |
| parser minimizer | `asset-leading-decimal.json` reduced 59 to 56 bytes while preserving `InvalidDecimal` |
| ABI and language consumers | 15/15 steps passed, including C11, C++17, C# and symbol checks |
| executor-only dependency audit | Spindle `6756fb2feecfa354a7ae42bca3af5d9bd66c7558`, MIT and SBOM verified |

The final `security-gate` result below is recorded after running in a clean worktree with the pinned Spindle submodule. Linux executes ThreadSanitizer; Windows only cross-compiles that target because TSan runtime execution is host-specific.
