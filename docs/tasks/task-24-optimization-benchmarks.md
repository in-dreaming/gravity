# Task 24：性能优化与基准

## 目的

在不改变single-thread golden的前提下优化内存、cache、排序、BVH、solver和rollback，达到product-ready预算。

## 依赖

Tasks 20、21、23。

## 交付物

- 固定Small/Medium/Stress/MeshHeavy/JointHeavy/CCD benchmark corpus；
- P50/P95/P99、memory、snapshot、rollback、worker scaling报告；
- SoA/cache/radix/BVH/solver优化；
- 性能回归CI阈值；
- flame/profile artifacts和优化ADR。
- Spindle work-stealing 的 submit、barrier、help-until、shutdown 开销与 worker
  利用率报告，并与serial adapter和FixedPool诊断基线对照；FixedPool不是第二套
  必须维护的production backend。

## 详细实现架构

先profile再优化。允许数据布局、批处理、cache和确定并行优化；任何改变运算顺序/结果的优化必须提升protocol并重新验收，默认禁止。SIMD只有证明各target结果逐位相同才启用。不可减少pair/contact/iteration/shape功能。

参考目标由专用硬件、OS电源模式、CPU affinity、warmup、样本数和噪声带共同
冻结；至少Medium 2,000 dynamic、5,000 contact、512 joints，Stress达到setup
默认容量的代表子集。8 Tick rollback单帧预算，Tick分配0。共享CI只验证schema
与显著回归；严格阈值只在固定runner执行。

## 实施步骤

1. 冻结机器、场景、指标和统计方法。
2. 分phase profile和memory accounting。
3. 逐项优化，每项跑完整determinism corpus。
4. 建立CI regression bands与噪声控制。
5. 输出native/WASM和worker scaling。
6. 单独归因 Gravity work partition、Spindle executor 与 stable merge 成本；
   不以更换 scheduler 掩盖负 scaling。

## 验证

- 优化前后全golden hash相同；
- 无功能/迭代降低；
- Tick分配0且峰值内存可解释；
- rollback/snapshot达到冻结预算；
- 1/2/4/8 worker均正确；Medium/Stress在冻结参考硬件达到明确效率/吞吐预算。
  Small、worker数超过物理核或低并行度phase允许有已解释开销，不要求单调加速；
- WASM demo典型case稳定实时。

## 完成判定

达到冻结产品预算且无确定性回归。仅microbenchmark、关闭功能或无CI门禁不算完成。
