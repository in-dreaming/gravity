# Task 17：3D 查询

## 目的

实现只读、确定的Ray、Convex Shape Cast及Point/Shape/AABB Overlap。

## 依赖

Tasks 08、09、10、11。

## 交付物

- 全shape ray cast；
- Sphere/Box/Capsule/ConvexHull/convex Compound shape cast；
- point/shape/AABB overlap；
- Any/Closest/All稳定模式；
- mesh/heightfield BVH查询；
- caller fixed buffer与required count。

## 详细实现架构

Hit key=`(fraction,ColliderId,childPath,primitiveId,feature)`。Any也是全序第一命中，不是遍历首个。Shape cast解析快速路径或最多32轮conservative advancement，超限non-converged。Compound caster只允许所有child凸且固定组合。

查询前后World canonical hash必须相同。Filter语义与simulation一致。All容量不足返回required且不把截断当成功。

## 实施步骤

1. 定义输入/Hit/filter/error。
2. 实现ray与overlap全shape。
3. 实现convex shape cast。
4. 集成BVH/compound和稳定排序。
5. 对比brute primitive oracle。

## 验证

- inside/tangent/parallel/zero-length/同fraction多hit；
- Any/Closest/All全序一致；
- mesh primitive和heightfield hole正确；
- query不改变World；
- capacity/non-convergence正确；
- native/WASM一致。

## 完成判定

查询精确且覆盖全部目标shape。Broad AABB hit、遍历首hit或JS侧计算不算完成。
