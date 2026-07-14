# Task 15：3D 接触求解器

## 目的

实现3D Sequential Impulse/PGS接触求解、二维摩擦圆盘、恢复和split impulse。

## 依赖

Tasks 12、14。

## 交付物

- normal + tangent1/tangent2 rows；
- warm start与二维摩擦投影；
- 10轮速度、4轮位置求解；
- restitution、材质组合、split impulse；
- impulse回写cache；
- 堆叠、斜面、旋转摩擦和能量测试。

## 详细实现架构

每contact point构建normal row和2D tangent block。摩擦累计向量若长度超过`mu*normalImpulse`，按确定sqrt缩放回圆盘。Tangent basis使用Task12规则。Restitution只在接近速度超过阈值。Split impulse使用独立pseudo linear/angular velocities与最大线/角修正。

每轮先全部joint/DOF（Task16接入），再contact；本任务实现contact内部固定patch/point顺序。不得按收敛提前结束。

## 实施步骤

1. 构建3D contact effective mass与lever arm。
2. 实现warm/normal/tangent block PGS。
3. 实现restitution与friction combine。
4. 实现pseudo velocity位置/角修正。
5. 回写cache。
6. 建立100:1与更高配置边界、箱塔、斜面、旋转body场景。

## 验证

- normal impulse≥0；摩擦向量在圆盘内；
- split impulse不注入真实动能；
- resting stack/box pyramid在冻结稳定指标内；
- 旋转接触角impulse正确；
- warm start改善且不破坏确定性；
- native/WASM/mode一致。

## 完成判定

完整3D接触响应可用。只用一条摩擦轴、忽略angular Jacobian或直接改位置不算完成。

