# Task 09：解析碰撞快速路径

## 目的

实现常用shape pair的高质量解析窄相与统一NarrowResult，为GJK提供可靠快速路径和oracle。

## 依赖

Task 07。

## 交付物

- point/segment/triangle/OBB距离原语；
- Sphere–Sphere、Sphere–Capsule、Capsule–Capsule、Sphere–Box、Capsule–Box、Box–Box SAT；
- normal/separation/witness/feature结果；
- 完全重合与平行退化tie-break；
- swap/transform/property/golden tests。

## 详细实现架构

normal A→B；separation正分离、0相切、负穿透。dispatch先按shape class和ColliderId规范A/B，再转换回ordered collider结果。完全重合优先center delta→relative velocity→ColliderId决定±X。Box–Box测试15个SAT轴，近平行cross轴按angular slop跳过，等深按axis class/index/ID全序。

Feature编码区分sphere、capsule side/end、box face/edge/vertex，不能依赖生成顺序。

## 实施步骤

1. 实现3D closest primitives。
2. 实现六类解析pair和15-axis SAT。
3. 实现dispatch/swap/feature。
4. 建立独立高精度case和退化corpus。
5. 与GJK任务后续做交叉oracle接口。

## 验证

- touching/overlap/separate/identical/parallel全覆盖；
- A/B swap normal相反且feature映射正确；
- rigid transform等变；
- Box SAT不漏cross-axis separation；
- fallback normal逐位稳定；
- native/WASM一致。

## 完成判定

所有快速路径真实返回witness与feature。仅bool或bounding shape近似不算完成。

