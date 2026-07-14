# Task 19：CCD

## 目的

实现冻结范围内的确定性连续碰撞：凸caster对全部shape目标，防止高速穿透。

## 依赖

Tasks 08、10、11、17。

## 交付物

- `ccd_enabled` body/collider规则；
- swept broadphase与convex shape cast；
- 全World earliest TOI、TOI contact/solve/remaining time；
- TriangleMesh可作target不可作caster；无mesh–mesh CCD；
- TOI event、fault与高速场景。

## 详细实现架构

Caster：Sphere/Box/Capsule/ConvexHull/全部child凸的Compound。Target可为全部shape，包括dynamic TriangleMesh/HeightField。每子步取全局最小`(fraction,pair,child,primitive,feature)`，处理后重建受影响候选；最多8个TOI。Cast最多32轮，超限仍有命中则CcdNonConverged fault。

Mesh作为caster的`ccd_enabled`命令在预验证阶段拒绝。未标记高速体允许离散行为，责任明确。TOI产生的contact必须进入同Tickcache/event且不重复begin。

## 实施步骤

1. 定义sweep/eligibility/config validation。
2. 实现swept candidate和全局TOI queue。
3. 实现推进、TOI solve、remaining fraction。
4. 集成mesh target/cache/event/wake。
5. 建立bullet-thin wall、moving mesh target、多TOI/同TOI场景。

## 验证

- 凸bullet不穿任一target shape；
- dynamic mesh target运动正确计入relative sweep；
- mesh caster明确拒绝；
- 同TOI按全序；超限fault；
- 关闭CCD等于离散oracle；
- native/WASM/worker一致。

## 完成判定

约定范围全部连续检测。增加子步冒充CCD、静态mesh-only或超限返回近似不算完成。

