# Task 05：资产格式与烘焙器

## 目的

实现canonical source schema、binary asset、AssetStore和CLI，为所有复杂3D shape提供确定只读数据。

## 依赖

Tasks 02、03、04。

## 交付物

- asset format/loader/validator；
- `tools/bake.zig` CLI；
- Sphere/Box/Capsule/ConvexHull/TriangleMesh/HeightField/Compound/Material schema；
- asset content hash、manifest、asset-set hash；
- caller-memory GravityAssetStore；
- golden/corrupt/repeat bake tests。

## 详细实现架构

Source是canonical JSON：实数必须十进制字符串，ID/count为integer。Baker统一单位、winding、稳定source ID和child order，调用Task 06几何烘焙。Binary采用Task04 TLV；AssetStore先完整验证再复制到调用方内存，source blob随后可释放。

Material运行时不可原地修改，只能切换MaterialId。Compound无环、深度≤8、直接child≤256。所有错误有稳定诊断码与source path，不静默修复。

## 实施步骤

1. 冻结JSON/binary schema和版本。
2. 实现primitive/material/compound canonicalization。
3. 接入Task06 hull/mesh/heightfield输出。
4. 实现manifest/AssetStore memory requirements/init/hash。
5. 实现CLI严格退出码与报告。
6. 建立合法、退化、恶意和容量corpus。

## 验证

- 重烘焙逐字节相同；
- source顺序扰动经stable ID规范后结果相同；
- cycle、非法real、重复ID、容量/深度越界拒绝；
- 一字节损坏被发现；
- AssetStore失败不半初始化，可多World共享；
- native/WASM loader得到同一hash。

## 完成判定

所有runtime复杂shape只能来自已验证AssetStore；runtime float/拓扑修复/占位资产不算完成。
