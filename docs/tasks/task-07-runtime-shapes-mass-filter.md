# Task 07：运行时形状、质量属性与过滤

## 目的

实现全部Shape的runtime view、Collider、质量组合、AABB/support/feature接口与碰撞过滤。

## 依赖

Tasks 02、03、05、06。

## 交付物

- Sphere/Box/Capsule/ConvexHull/Compound/TriangleMesh/HeightField runtime shape；
- Collider local transform、material、sensor、enabled、revision；
- local/world AABB、support map、face/edge/vertex/primitive访问；
- density/override质量组合和平行轴定理；
- layer/mask/group/body type/filter truth table。

## 详细实现架构

Primitive可inline，复杂shape引用AssetId。Dynamic TriangleMesh允许刚体运动；HeightField仅static/kinematic。Dynamic Compound可包含mesh，但质量必须有效。shape revision为未来可变shape预留，当前immutable asset revision固定；cache接口必须按revision失效，不能假设永远0。

过滤顺序：enabled→同body→group override→category/mask→body type→sensor只改变response类型。结果是ignore/overlap/contact。

## 实施步骤

1. 定义ShapeKind/runtime tagged view/child path/feature types。
2. 实现AABB/support/face访问与compound traversal。
3. 实现质量组合和override校验。
4. 实现Collider state/revision/material。
5. 实现纯函数filter。
6. 添加全部shape/body组合测试。

## 验证

- 每种shape边界、非法尺寸/asset/type组合拒绝；
- AABB包含所有几何；support与brute vertex oracle一致；
- 质量/惯量旋转和平移正确；
- filter对A/B交换对称；
- dynamic mesh与heightfield body限制准确；
- shape revision变化触发明确cache invalidation接口。

## 完成判定

所有shape有真实runtime能力。用bounding box/sphere代替复杂shape或忽略质量张量不算完成。

