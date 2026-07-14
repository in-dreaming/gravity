# Task 21：Snapshot、Rollback、Replay 与 Diff

## 目的

实现完整逻辑状态保存/恢复、120 Tick回滚、输入重演、分层hash和字段diff，证明L4。

## 依赖

Tasks 04、12、16、20。

## 交付物

- GRAVSNAP/GRAVREPL格式；
- size/save/two-pass atomic load/derived rebuild；
- full snapshot ring与input history；
- replay CLI/mismatch二分；
- world/body/collider/contact/joint/events hash；
- section→ID→field diff；
- decoder fuzz和随机rollback。

## 详细实现架构

保存setup全部逻辑状态，包括full inertia/quaternion/DOF/joint/contact/sleep/generation/fault。Asset只保存ID与asset-set hash。SAP/BVH traversal/island/rows不保存。

Load第一遍验证magic/version/hash/容量/引用/canonical quaternion且不修改World；第二遍确定拷贝后重建派生状态。Replay=初始snapshot+逐Tick canonical commands+expected hashes。初版全量ring，不用未验证delta。

## 实施步骤

1. 建立state field inventory与新增字段漏序列化检查。
2. 实现snapshot format/two-pass load/rebuild。
3. 实现ring/input history/resimulation。
4. 实现replay runner/二分/diff。
5. fuzz decoder和随机1～120 Tick rollback。

## 验证

- 每Tick save/load→step等于连续路径；
- 100k随机rollback无mismatch；
- contact/joint/sleep/CCD/mesh/ID复用时恢复正确；
- corrupt/wrong asset/config/protocol拒绝且World不变；
- encode/decode/encode相同；
- field翻转被精确diff；
- native/WASM/worker replay一致。

## 完成判定

完整状态L4通过。只保存body transform、丢cache或仅最终帧相同不算完成。

