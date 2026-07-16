//! Compilation root for static/shared/WASM C ABI artifacts.
const implementation = @import("abi/root.zig");

comptime {
    // Importing the implementation from the `src` module root keeps all core
    // imports inside the package boundary and retains its `export` symbols.
    _ = implementation;
}
