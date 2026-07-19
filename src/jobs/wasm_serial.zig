//! Deliberately empty WASM job adapter.
//!
//! The wasm32-freestanding ABI keeps the stable dispatcher export but always
//! executes Gravity-owned batches serially. Host callbacks and native executor
//! types must not enter this module's build graph.
