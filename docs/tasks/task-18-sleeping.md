# Task 18：确定性睡眠

## 目的

实现按岛、Tick计时、可快照的3D sleep/wake。

## 依赖

Tasks 14、15、16。

## 交付物

- awake flag/counter/wake reason；
- linear/angular阈值与整岛提交；
- command/joint/contact/kinematic唤醒传播；
- wake/sleep事件与snapshot visitor；
- 静止堆叠、岛合并/拆分测试。

## 详细实现架构

全部dynamic body速度平方低于阈值时岛counter同步递增，否则全岛归零；30 Tick后Tick末整岛sleep。睡眠时速度、force、torque归零，contact/joint impulse保留。Broad proxy保留以检测新响应接触。

非零force/impulse/setVelocity、kinematic target、新response contact、joint/motor/limit激活唤醒；sensor/query不唤醒。同Tick只发一次事件，最早reason记录。

## 实施步骤

1. 实现eligibility/counter/state。
2. 实现稳定wake graph propagation。
3. 接入event/cache/island。
4. 提供canonical visitor。
5. 建立阈值、旋转、mesh contact和warm wake场景。

## 验证

- 精确Tick入睡；无wall clock；
- 所有wake路径正确；sensor/query不唤醒；
- sleep后active workload下降；
- 关闭sleep与不含sleep oracle一致；
- save/load counter/flag完整；
- worker/native/WASM一致。

## 完成判定

真实节省求解且不漏唤醒。只跳过低速body或未快照counter不算完成。

