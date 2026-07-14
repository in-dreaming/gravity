# Task 12：接触缓存与事件

## 目的

实现持久3D contact patch cache、warm impulse继承和全部有序contact/sensor事件。

## 依赖

Tasks 04、10、11。

## 交付物

- fixed-capacity patch/point cache；
- ManifoldKey/feature/revision匹配；
- normal与两维tangent impulse继承；
- begin/persist/end、sensor enter/stay/exit；
- canonical state visitor与hash；
- 多帧拓扑/mesh边界/销毁测试。

## 详细实现架构

新旧cache按完整ManifoldKey sorted merge。point按feature pair匹配；normal dot低于阈值或shape/material revision变化则整patch impulse清零。tangent basis每Tick由normal和稳定最小轴构造，旧二维impulse先转world tangent vector再投影到新basis，避免basis符号跳变。

Sensor保存overlap但不生成constraint。事件在merge完成后生成并按setup全序。容量不足使当前模拟phase fault，不丢浅contact。

## 实施步骤

1. 定义patch/point/cache revision与tangent basis。
2. 实现sorted merge、匹配、继承/清零。
3. 实现sensor/contact状态机。
4. 实现事件排序和canonical visitor。
5. 添加移动patch、triangle切换、销毁/filter/revision场景。

## 验证

- 稳定feature继承3D impulse；basis变化world摩擦向量连续；
- topology/revision/normal变化不错误继承；
- begin/persist/end和sensor事件无漏/重复；
- input manifold顺序扰动不改变cache/events；
- canonical round-trip准备完整；
- capacity失败不发布部分事件。

## 完成判定

Cache是完整未来状态，3D摩擦warm start正确。每帧清零、无序map或缺end事件不算完成。

