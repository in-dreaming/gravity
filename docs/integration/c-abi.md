# C ABI integration

Build and validate the installed surface:

```text
zig build
zig build abi-test
zig build test-abi-all-modes
zig build abi-wasm-smoke
zig build abi-csharp-smoke
zig build abi-artifacts
```

The default install contains `include/gravity.h`, a native `gravity_static`
static library, a
native shared library, and `bin/gravity.wasm`. `abi-artifacts` additionally
places ReleaseSafe Windows, Linux, and macOS libraries below `zig-out/abi/`.

The C11 and C++17 consumers are built and run directly by `abi-test`. The C#
consumer uses P/Invoke with an explicit shared-library resolver. The Node WASM
consumer grows linear memory, allocates opaque objects through the same caller-
memory API, saves/loads a snapshot, and compares its canonical state hash with
the Zig/C# reference vector.
