# Task 10：GJK、EPA 与凸体接触流形

## 目的

实现任意凸体组合的距离、相交、穿透深度和最多4点稳定接触patch。

## 依赖

Tasks 06、07、09。

## 交付物

- Minkowski support/witness；
- GJK distance/intersection（32轮）；
- EPA polytope（64轮、256 faces）；
- reference/incident face clipping、patch reduction；
- ConvexHull/Box/Capsule/Sphere/convex Compound pair；
- degenerate corpus、oracle和cross-check。

## 详细实现架构

Support tie取最小stable vertex/feature ID。Simplex点排序与reduction固定；零search direction用Task09 fallback。GJK收敛基于raw progress阈值且固定上限。EPA closest face key=`(distance,normal key,face vertex IDs)`；horizon构建稳定排序，无hash iteration。

有face的凸体用face clipping生成patch；smooth形状用witness contact。最多4点按最深→最大三角面积→最大覆盖→feature key删减。达到上限未收敛必须fault。

## 实施步骤

1. 实现support与simplex barycentric/witness。
2. 实现GJK距离/相交及cache seed契约。
3. 实现EPA face/horizon/fixed pool。
4. 实现face clipping/patch reduction。
5. 集成解析/GJK一致性cross-check测试。
6. fuzz退化Hull、近平行face和极小间隙。

## 验证

- 与brute/high precision convex oracle分类一致；
- 解析pair与GJK witness在规定误差内且分类相同；
- 输入vertex/face存储地址不影响；
- non-convergence准确fault；
- patch≤4且稳定覆盖；
- native/WASM/mode一致。

## 完成判定

GJK/EPA/patch完整生产化。只返回EPA单点、随机horizon或超限返回最后近似不算完成。

