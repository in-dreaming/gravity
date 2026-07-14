# Task 22：C ABI 与多平台产物

## 目的

通过稳定C ABI发布全部engine/asset/query/state/job能力，生成static/shared/WASM和真实消费者。

## 依赖

Tasks 05、17、20、21。

## 交付物

- `include/gravity.h`与Zig extern/export层；
- AssetStore/World caller-memory API；
- command/step/body state/event/query/snapshot/hash；
- job system描述预留并在Task23接入；
- Windows/Linux/macOS static/shared与WASM；
- C11/C++17/C# PInvoke/WASM smoke consumers；
- layout/symbol/ABI baseline。

## 详细实现架构

所有symbol `gravity_v1_`。Vec3=`3*i64`，Quat=`4*i64`，Mat3=`9*i64`，ID=u64。Struct含`struct_size`和reserved=0。数组pointer+u32/u64 count。输出caller buffer+required。AssetStore独立共享且比World长寿。

函数族必须覆盖versions/build info、asset memory/init/deinit/hash、world memory/init/deinit/tick/step/hash/error、body states/events、4类query、snapshot size/save/load。Command使用type/size/header/union并全量验证后commit。不得保存caller pointer。

## 实施步骤

1. 冻结完整header、错误码、所有权和扩展规则。
2. 双侧size/align/offset assertions。
3. 实现全部exports/conversion/validation。
4. 构建static/shared/WASM和symbol visibility。
5. 写独立installed consumers和ABI baseline。
6. 跑null/length/alignment/overflow/lifetime测试。

## 验证

- MSVC/Clang/GCC C11/C++17 consumer真实运行完整流程；
- C#/WASM smoke hash等于Zig；
- 少内存/错对齐/坏struct全量失败无部分修改；
- static/shared/WASM同replay；
- symbol仅allowlist，无panic/unwind跨ABI；
- sanitizer/valgrind可用目标无错误。

## 完成判定

外部消费者可用全部能力。仅header/version函数、缺query/snapshot或mock wrapper不算完成。

