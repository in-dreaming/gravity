# Task 16：全部关节

## 目的

实现Distance、Ball-Socket、Hinge、Slider、Fixed、Cone-Twist及适用limit、motor、spring/damping。

## 依赖

Tasks 14、15。

## 交付物

- Joint pool/state/command lifecycle；
- 六类joint的3D Jacobian rows；
- linear/angular limits、motor、soft spring；
- canonical reference frame与singularity fallback；
- warm impulses与snapshot visitor；
- pendulum、slider、robot arm、ragdoll/cone-twist scenarios。

## 详细实现架构

Joint存local anchors/frames和canonical reference quaternion。Ball-Socket 3 linear rows；Hinge 3 linear+2 angular并加limit/motor；Slider锁5DOF并沿轴limit/motor；Fixed 6；Cone-Twist分swing cone和twist interval；Distance支持min/max/equal与spring。

轴/帧创建时确定正交基；近平行备用轴取绝对分量最小的world basis。Limit状态切换清对应impulse。Motor impulse受force/torque×dt限制。Body销毁按JointId销毁关联joint。

## 实施步骤

1. 定义joint描述/state/frame验证。
2. 实现六类row生成。
3. 实现limit/motor/spring与状态切换。
4. 接入固定求解顺序/warm回写。
5. 接入command/destroy/event/snapshot visitor。
6. 建立每类解析和组合场景。

## 验证

- 各joint保留/释放正确DOF；
- cone/twist边界和180°附近无随机轴；
- motor不超上限；spring频率/阻尼符合离散模型；
- joint chain/ragdoll稳定；
- destroy无悬空引用；
- native/WASM hash一致。

## 完成判定

六类关节、limit、motor、spring全部真实可用。锁transform或缺Cone-Twist奇异处理不算完成。

