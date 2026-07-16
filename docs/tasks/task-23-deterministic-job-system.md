# Task 23：确定性 Job System

## 目的

实现product-ready必需的多线程并行，保证1/2/4/8 worker与单线程逐位一致。

## 依赖

Task 22A。

## 交付物

- 复用 Task 22/22A 已冻结的 `jobs.Dispatcher` 与同步 batch ABI，不新增第二套
  job descriptor、callback 或 executor abstraction；
- AABB、narrow pair、mesh primitive pair、独立island、query并行；
- 固定range/thread-local输出与stable merge；
- 内建随机/逆序test scheduler；
- race、TSAN可用测试与scaling报告。
- 复用 Task 21 replay corpus 的 native/WASM/1/2/4/8 worker hash 矩阵。

## 详细实现架构

主线程先生成ordered work list；worker只写由逻辑 job/input range 拥有的
固定slot或staging。实际worker ID不得影响布局、容量、overflow或merge。
Merge按input index/key。禁止atomic append、并行ID分配、共享impulse累加。
不同island并行，岛内顺序不变。

各phase必须冻结唯一输出方案：AABB按collider slot；narrow pair与mesh
primitive pair采用count→canonical prefix sum→fill；query按query index及稳定
hit compact；island solve只可直接写经证明互斥的dynamic body/joint/contact
cache集合，static/kinematic只读。任何无法证明写集合互斥的phase必须写staging。

执行器必须通过 Task 22A 的 Gravity adapter 使用 Spindle；物理 phase
不得直接依赖 Spindle 类型。Spindle work stealing 只影响执行时机，不影响
range ownership、merge key 或 fault publication。

C ABI batch callback只执行engine描述，调用期禁止重入或保存descriptor。
Worker/dispatch error发生时不执行phase commit，不回退到部分执行单线程。
WASM默认single worker但结果与native一致。

## 实施步骤

1. 为每个phase冻结input range、读写集、count/fill或direct-write证明、容量
   preflight与commit边界。
2. 通过Task 22A Dispatcher逐阶段并行并对单线程oracle。
3. 实现stable merge与Gravity-owned scheduler扰动；Spindle
   DeterministicExecutor记录仅用于复现时机，不作为正确性oracle。
4. 接入snapshot/rollback/C ABI。
5. race检测和1/2/4/8性能。

## 验证

- 每worker count逐Tick全section hash等于单线程；
- 随机延迟/逆序/work stealing不影响；
- 无data race；callback重入拒绝；
- worker failure无部分成功Tick；
- queue/backpressure/capacity在publish前确定失败，kernel staging不会成为World状态；
- rollback期间改变worker count仍一致；
- Medium/Stress有真实scaling报告。

## 完成判定

并行真实加速且逐位等于oracle。mutex包单线程、多世界并行或近似相同不算完成。
