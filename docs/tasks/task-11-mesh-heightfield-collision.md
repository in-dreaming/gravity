# Task 11：Mesh、HeightField 碰撞

## 目的

实现convex–mesh、convex–heightfield和离散mesh–mesh，包含BVH遍历、triangle contact、邻接平滑与patch合并。

## 依赖

Tasks 06、07、10。

## 交付物

- ordered BVH traversal与primitive pair buffer；
- convex–triangle、triangle–triangle/SAT；
- mesh/heightfield/compound child traversal；
- welded normal、内部边ghost contact抑制；
- triangle patch聚合/最多4点per patch；
- dynamic mesh–mesh场景与brute oracle。

## 详细实现架构

BVH node pair traversal使用固定优先队列key，不按stack插入偶然顺序。primitive key包含AssetId/child/triangle ID。Mesh–mesh先BVH pair，再triangle–triangle；共面重叠有固定投影轴和裁剪全序。

相邻triangle contact按asset邻接和normal角阈值聚为surface patch，避免内部边重复冲量。HeightField hole跳过，tile/material进入feature。Dynamic mesh拓扑immutable，transform每Tick来自body。

## 实施步骤

1. 实现BVH single/pair traversal。
2. 实现convex–triangle和triangle–triangle。
3. 实现mesh/heightfield/compound dispatch。
4. 实现welded edge filtering与patch merge。
5. 对比小mesh全三角暴力oracle。
6. 建立dynamic mesh碰撞、共面、尖角、hole场景。

## 验证

- candidate/contacts与brute oracle集合一致；
- mesh内平面无ghost edge，真实sharp edge保留；
- mesh–mesh对A/B swap稳定；
- dynamic transform、sleep/wake前置数据正确；
- capacity超限fault不截断；
- native/WASM一致。

## 完成判定

全部mesh/heightfield离散pair真实支持。只做convex–mesh、禁止dynamic mesh或用triangle soup无邻接处理不算完成。
