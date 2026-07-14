# Task 23：确定性 Job System

## 目的

实现product-ready必需的多线程并行，保证1/2/4/8 worker与单线程逐位一致。

## 依赖

Tasks 20、21、22。

## 交付物

- engine job descriptor、C ABI enqueue/wait callbacks；
- AABB、narrow pair、mesh primitive pair、独立island、query并行；
- 固定range/thread-local输出与stable merge；
- 内建随机/逆序test scheduler；
- race、TSAN可用测试与scaling报告。

## 详细实现架构

主线程先生成ordered work list；worker只写固定slot/range或按worker index预分配buffer。Merge按input index/key。禁止atomic append、并行ID分配、共享impulse累加。不同island并行，岛内顺序不变。

C ABI job callbacks只执行engine描述，调用期禁止重入。Worker error使step fault，不回退到部分执行单线程。WASM默认single worker但结果与native一致。

## 实施步骤

1. 定义job ABI/lifetime/barrier/error。
2. 逐阶段并行并对单线程oracle。
3. 实现stable merge与scheduler扰动。
4. 接入snapshot/rollback/C ABI。
5. race检测和1/2/4/8性能。

## 验证

- 每worker count逐Tick全section hash等于单线程；
- 随机延迟/逆序/work stealing不影响；
- 无data race；callback重入拒绝；
- worker failure无部分成功Tick；
- rollback期间改变worker count仍一致；
- Medium/Stress有真实scaling报告。

## 完成判定

并行真实加速且逐位等于oracle。mutex包单线程、多世界并行或近似相同不算完成。
