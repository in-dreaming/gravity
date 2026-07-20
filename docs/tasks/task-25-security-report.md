# Task 25 security qualification report

Date: 2026-07-20

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
| Linux Zig coverage-guided parser | 72 minutes; 102,942,390 runs; 5,684 unique runs; 1168/11329 edges (10.31%); no crash/OOM/timeout |
| deterministic corpus, three modes | current pin requalification: 83/83 build steps; 45/45 tests passed |
| parser minimizer | `asset-leading-decimal.json` reduced 59 to 56 bytes while preserving `InvalidDecimal` |
| ABI and language consumers | 15/15 steps passed, including C11, C++17, C# and symbol checks |
| clean-worktree `security-gate` | current pin requalification: 124/124 steps; 80/80 tests passed |
| Linux ThreadSanitizer execution | 4/4 steps; 9/9 dispatcher tests passed with no race report |
| executor-only dependency audit | Spindle `45aab8adf5f89500f6196b383265a5f9826312c2`, gitlink/checkout, MIT notice and SBOM verified |

The current-pin `security-gate` was recorded in an isolated clean worktree with the pinned Spindle submodule. Linux executes ThreadSanitizer from a native temporary Git checkout; Windows also cross-compiles the same target. The 72-minute coverage-guided parser evidence remains the frozen Task 25 campaign; the July 20 requalification replayed every bounded corpus in all three modes and did not substitute bounded replay for that campaign.
