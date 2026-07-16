# Task 22A：Spindle Executor 接入与已完成管线重构

## 目的

将固定版本 Spindle 接为 Gravity 的原生执行底座，同时保持物理状态、
canonical protocol、C ABI 和确定性规则完全由 Gravity 控制。

## 依赖

Tasks 20、21、22。

## 交付物

- `third_party/spindle` 固定 commit、许可证记录和最小 feature build gate；
- Gravity-owned `jobs.Dispatcher`/batch/range/barrier/error contract，与 Task 22
  的同步 `dispatch_batch` ABI 一一对应；
- serial、Spindle work-stealing 与 host synchronous batch adapters；Spindle
  FixedPool只作开销基线，DeterministicExecutor只作调度记录/复现工具；
- World初始化时预分配的Task/context/completion/fault slab及明确复用协议；
- Task 20 各 phase 的 ordered batch seam，数值 kernel 不改运算顺序；
- allocation/lifetime/shutdown/reentrancy audit；
- adapter fault、cancel、backpressure、worker-count matrix tests。

## 边界

Gravity 只导入 Spindle `src/executor.zig` / `spindle_executor` 窄入口。禁止
导入 aggregate Runtime、parallel helpers、Local Task Graph、ECS、Resource
Graph、Workflow、SQLite、archive、I/O 与 observability。Spindle 类型不进入
GRAVSNAP、GRAVREPL、state hash 或 C ABI。

Spindle 负责执行，Gravity 负责决定工作划分、逻辑 job index、staging
ownership、容量和结果顺序。任何实际 worker ID 影响输出布局/容量/失败、
atomic append、按完成顺序 merge、运行时地址/Spindle task ID 进入模拟结果
均为失败。

WorkStealingExecutor 只允许在 World 初始化或宿主层分配；Tick 内使用预分配
intrusive Task/context。每次复用必须执行 completion wait →
`waitQueueReleased` → `Task.reset` → 更新 generation/context，禁止仅看到
completed 就覆盖 Task。Spindle `parallel.forRange`、DeterministicExecutor 和
Local Task Graph 当前运行路径会动态分配，不进入 Gravity runtime Tick。

Dispatcher 的一次 batch 调用是同步 barrier。提交前必须验证 job count、
Spindle injection/local queue capacity、Task slab 和所有输出 staging 容量。
Kernel 只写逻辑 job-owned staging；任一 submit/backpressure/cancel/worker/host
callback 失败时不执行 canonical commit。禁止部分执行后回退 serial。

## 实施步骤

1. 冻结 submodule commit，并验证 `zig build spindle-check-all-modes`。
2. 定义 Gravity batch contract、逻辑 job-owned staging 与 serial oracle adapter。
3. 接入 Spindle WorkStealingExecutor 和 host synchronous batch adapter；建立
   FixedPool开销基线及DeterministicExecutor离线调度复现测试。
4. 将 Task 20 重构为 ordered work-list/batch seam，保持 serial 输出不变。
5. 接入 Task 22 `dispatch_batch`；验证错误时 Tick 无部分发布。
6. 运行 snapshot/replay/hash golden，证明重构前后逐位一致。

## 验证

- Debug/ReleaseSafe/ReleaseFast 下 adapter contract 相同；
- serial backend与1/2/4/8 worker、随机提交与逆序完成不改变golden；
- worker failure/backpressure/cancel 使当前 Tick 原子失败；
- rollback/replay 不保存 executor 内部状态，切换 backend 后结果相同；
- native 无泄漏、无悬空 Task、shutdown 可重复；WASM 使用 serial adapter；
- Tick 内 allocator probe 为 0，完成回调只写固定 range/slot；
- Task slab跨至少1M次batch复用，无`TaskQueued`、generation混淆、queue reference
  泄漏或shutdown后悬空Task；
- core 与 WASM 构建图不链接 Spindle platform threads。

## 完成判定

执行底座真实接入且旧 pipeline/replay golden 不变。仅能 import Spindle、
仅跑独立 smoke、或用 mutex 包住旧单线程实现不算完成。
