# Task 04：Canonical Codec 与 BLAKE3

## 目的

实现与 Zig 内存布局无关的little-endian codec、严格解码和统一domain-separated BLAKE3。

## 依赖

Task 03。

## 交付物

- bounded reader/writer、size-only pass、TLV sections；
- primitive/Fp/Vec3/Quat/ID/config canonical encoding；
- Hash128/Hash256 streaming sinks和setup domain tags；
- malformed corpus、round-trip、fuzz、跨平台golden。

## 详细实现架构

禁止 memcpy struct/padding/tagged union。bool仅0/1，enum用固定整数，section必须按ID升序且不重复。writer size pass与实际pass共享field visitor。Hash128取BLAKE3 digest前16字节；payload integrity可用32字节。

Decoder对长度加乘、容量、递归深度和未知required section严格检查，不分配攻击者指定大小。失败不产生部分对象。

## 实施步骤

1. 定义codec error和版本化section规则。
2. 实现bounded reader/writer/size sink。
3. 实现基础类型和config visitor。
4. 封装BLAKE3 domain API。
5. 构建golden与decoder fuzz入口。

## 验证

- encode→decode→encode逐字节相同；
- size pass与written精确一致；
- truncation、重复/倒序、非法bool/enum、长度炸弹全部拒绝；
- chunked hash等于one-shot；
- host endian/usize/layout不影响；
- Debug/ReleaseSafe/ReleaseFast/native/WASM相同。

## 完成判定

Codec/hash可作为所有持久格式唯一基础。raw memory hash或happy-path serializer不算完成。

