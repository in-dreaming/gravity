# Task 22：C ABI 与多平台产物

## 目的

通过稳定C ABI发布全部engine/asset/query/state/job能力，生成static/shared/WASM和真实消费者。

## 依赖

Tasks 05、17、20、21。

## 交付物

- `include/gravity.h`与Zig extern/export层；
- AssetStore/World caller-memory API；
- command/step/body state/event/query/snapshot/hash；
- Gravity-owned synchronous batch-dispatch ABI；Task 22A 将 Spindle native
  backend 接到同一内部 contract，Spindle 类型不跨 C ABI；
- Windows/Linux/macOS static/shared与WASM；
- C11/C++17/C# PInvoke/WASM smoke consumers；
- layout/symbol/ABI baseline。

## 详细实现架构

所有symbol `gravity_v1_`。Vec3=`3*i64`，Quat=`4*i64`，Mat3=`9*i64`，ID=u64。Struct含`struct_size`和reserved=0。数组pointer+u32/u64 count。输出caller buffer+required。AssetStore独立共享且比World长寿。

函数族必须覆盖versions/build info、asset memory/init/deinit/hash、world memory/init/deinit/tick/step/hash/error、body states/events、4类query、snapshot size/save/load。Command使用type/size/header/union并全量验证后commit。不得保存caller pointer。

ABI 只冻结 Gravity 的批执行 contract：`dispatch_batch(user, job_count,
run_job, batch_context)` 按逻辑 job index 恰好执行一次，并在返回前完成整批。
descriptor、function/context pointer 只在调用期间借用，宿主不得保存；callback
禁止重入同一 World。ABI 不暴露 enqueue、wait、queue 或 task handle。

内建 Spindle executor 与宿主 callback 是可替换 backend；World 不保存
Spindle task、allocator、thread 或 queue 指针。宿主返回失败时，worker 结果
仍仅存在于预分配 staging，Gravity 不得发布部分 Tick。

## 实施步骤

1. 冻结完整header、错误码、所有权和扩展规则。
2. 双侧size/align/offset assertions。
3. 实现全部exports/conversion/validation和同步 batch-dispatch adapter。
4. 构建static/shared/WASM和symbol visibility。
5. 写独立installed consumers和ABI baseline。
6. 跑null/length/alignment/overflow/lifetime测试。

## 验证

- MSVC/Clang/GCC C11/C++17 consumer真实运行完整流程；
- C#/WASM smoke hash等于Zig；
- 少内存/错对齐/坏struct全量失败无部分修改；
- callback 少执行、重复执行、提前返回、返回错误与重入均被拒绝或使本 Tick
  无发布失败；descriptor 不得逃逸调用期；
- static/shared/WASM同replay；
- symbol仅allowlist，无panic/unwind跨ABI；
- sanitizer/valgrind可用目标无错误。

## 完成判定

外部消费者可用全部能力。仅header/version函数、缺query/snapshot或mock wrapper不算完成。
