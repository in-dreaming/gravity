# Task 27：Three.js、React 控制面板与经典 Cases

## 目的

构造本地交互Demo，直观、可重复地演示引擎全部主要能力、确定性、回滚和性能。

## 依赖

Task 26。

## 交付物

- Three.js renderer、camera/light/debug draw；
- React控制面板、case选择、参数、pause/step/reset；
- diagnostics：Tick、hash、body/contact/joint/pair、phase timing、snapshot/rollback；
- 经典case集合和自动截图/行为测试；
- 本地使用文档。

## 详细实现架构

渲染只读取body transforms并做render interpolation，绝不反馈浮点transform到模拟。控制面板把参数通过canonical decimal/raw转换成Tick command。每个case定义固定Asset/Config/Command seed与expected关键hash/metric。

必须实现case：

1. Sphere/Box堆叠与金字塔；
2. 摩擦斜面与恢复系数球阵；
3. Newton cradle；
4. Distance/Ball/Hinge/Slider/Fixed/Cone-Twist展台含motor/limit/spring；
5. 3D ragdoll；
6. ConvexHull与Compound混合坍塌；
7. Dynamic TriangleMesh–Mesh离散碰撞；
8. HeightField地形；
9. CCD高速凸体对薄壁与运动Mesh target；
10. Ray/Shape Cast/Overlap可视化；
11. 2D planar DOF case；
12. Sleep/wake；
13. Snapshot/rollback：注入迟到输入并展示重演/hash；
14. Determinism：同输入双World并排hash；
15. Stress/worker scaling（WASM显示single，native数据可加载报告）。

## 实施步骤

1. 实现renderer/debug primitives和transform bridge。
2. 实现React状态模型，模拟状态不放React。
3. 逐个实现固定case与控制参数。
4. 实现diagnostics/hash/rollback timeline。
5. 加自动case加载、step、hash/assert、截图测试。
6. 优化UI resize/dispose/accessibility。

## 验证

- 15类case全部通过正式WASM API运行；
- reset得到相同hash；pause/single-step精确；
- rollback最终hash等于权威路径；
- renderer interpolation不改变World hash；
- case切换无WASM/Three资源泄漏；
- headless行为测试与关键截图通过；
- `zig build demo-run`是唯一启动入口。

## 完成判定

Demo完整展示系统能力且可诊断。静态动画、JS物理、只做少量case或无自动验证不算完成。

