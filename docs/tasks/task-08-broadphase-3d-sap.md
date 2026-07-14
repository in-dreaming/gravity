# Task 08：3D SAP 广相

## 目的

实现每子步规范化重建的3D SAP，完整、稳定地产生Collider Pair。

## 依赖

Task 07。

## 交付物

- Proxy、X endpoints、active set、PairKey fixed buffers；
- world/fat/swept AABB；
- X sweep + Y/Z复核 + filter；
- stable sort/unique/capacity fault；
- O(n²) oracle、压力、确定性benchmark。

## 详细实现架构

按ColliderId生成root proxy。Endpoint key=`(x_raw,start-before-end,ColliderId)`。Active set按ColliderId有序。候选通过Y/Z相切overlap和Task07 filter后规范为`(min,max)`。Compound只发root proxy，child在narrow处理。

每子步重建，snapshot不保存endpoint。Pair先写scratch，完整sort/unique/capacity成功后发布；不能让narrow消费前缀。

## 实施步骤

1. 实现proxy和速度swept AABB。
2. 实现endpoint radix/active set。
3. 集成Y/Z/filter和PairKey。
4. 实现事务publish/counters。
5. 对比随机brute force并跑8,192 body压力。

## 验证

- candidate集合与brute force完全相同；
- 相切、同endpoint、generation复用、disable/filter变化无ghost pair；
- 输入/地址/worker扰动不改变pair bytes；
- capacity少1明确fault；
- Tick零分配；
- native/WASM/mode一致。

## 完成判定

完整、稳定、可重建的3D pair stream。动态树替换、截断或无序active set不算完成。

