# Zig module API

Downstream Zig builds can depend on the `gravity` module exposed by
`build.zig.zon`. The module exports fixed-point math, configuration and IDs,
baked assets, collision/query/dynamics primitives, canonical state tooling,
the logical jobs contract, build metadata, and the ABI implementation.

The Zig module is a source integration surface tied to exact package and Zig
versions; it is not a stable binary ABI. `include/gravity.h` remains the only
stable public ABI. Applications that need independent compiler or package
upgrades should consume the C ABI.

All public scalar state uses raw Q32.32 values. Callers own storage and scratch
capacity, must preserve AssetStore lifetime beyond Worlds, and must treat Tick
failure transactionally. Runtime float conversion, implicit allocation,
unordered collection iteration, or wall-clock delta time are outside the API
contract.

An external-style build fixture is maintained in
`tests/isolation/consumer`. `zig build demo-isolation` builds that fixture with
no Node or pnpm available and proves the package root does not depend on Demo
code.
