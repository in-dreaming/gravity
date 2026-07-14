# Task 06：ConvexHull、Mesh、HeightField 与 BVH 烘焙

## 目的

实现复杂3D几何的确定离线构建、邻接、质量属性和只读加速结构。

## 依赖

Tasks 02、05。

## 交付物

- ConvexHull half-edge/face结构与验证；
- TriangleMesh welding、邻接、winding、welded normal、watertight检查；
- HeightField tile/min-max tree/hole/material；
- Compound与Mesh deterministic BVH；
- 体积、质心、完整对称惯量张量；
- BVH/质量/退化golden与oracle。

## 详细实现架构

Hull输入点排序后构建闭合凸多面体；共面/重复点按固定raw阈值处理，无法形成3D体积则失败。Mesh primitive ID来自canonical triangle order。BVH使用SAH整数代价与固定tie-break `(cost,axis,split,primitiveId)`；叶≤4 triangle。HeightField以固定tile order生成三角形和min/max tree。

Dynamic mesh自动质量只允许watertight、manifold、定向一致mesh；否则要求显式override。惯量计算全程wide accumulator，最后一次narrow。

## 实施步骤

1. 实现Hull拓扑/支持点/face访问。
2. 实现Mesh焊接、邻接、内部边信息和watertight验证。
3. 实现HeightField tiles/holes/material。
4. 实现稳定BVH build、serialize、traverse contract。
5. 实现体积/质心/惯量。
6. 建立独立brute-force和高精度oracle。

## 验证

- 输入点/triangle顺序规范后输出一致；
- 非流形、自交关键错误、退化Hull拒绝；
- BVH查询集合等于brute force且顺序稳定；
- dynamic mesh质量与高精度reference一致；
- HeightField hole/边界/材料正确；
- 三模式/native/WASM asset bytes相同。

## 完成判定

三类复杂geometry和BVH均完整。AABB占位Hull、无邻接mesh或非确定BVH不算完成。

