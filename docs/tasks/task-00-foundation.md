# Task 00：工程、工具链与可复现构建

## 目的

建立 Zig 0.16.0 product-ready 工程、模块边界、构建图和 CI 基础。不得实现空物理 API冒充进度。

## 依赖

无。

## 交付物

- `build.zig`、`build.zig.zon`、`.zigversion` 和构建期精确版本检查；
- `src/root.zig`、`src/version.zig` 与 setup 规定的目录；
- core/static/shared/WASM/tools/tests/demo 的真实 build step 框架；
- `fmt`、`test`、`test-all-modes`、`determinism`、`abi-test`、`fuzz`、`benchmark`、`demo`、`demo-run` step；
- Linux x86-64 基础 CI 与开发文档；
- build metadata：commit、Zig、ABI、protocol、snapshot、asset format version。

## 详细实现架构

Core 默认不链接 libc。target/optimize 来自标准 build options，CPU feature 必须显式固定。测试从真实 root module 导入。Demo 是单向依赖 core 的独立构建子图；普通 `zig build` 和其他仓库模块引用不得解析 pnpm/React/Three.js。

尚无实现的后续 step 只能运行实际存在的测试集合，不能固定返回成功、生成空 library API 或空 WASM。生成物写 `zig-out`/cache，不写回源码树，明确生成并入库的数学表除外。

## 实施步骤

1. 建立目录、root module 和依赖边界检查。
2. 双重锁定 Zig 0.16.0。
3. 建立各 artifact/test/tool/demo build function。
4. 让 demo build 仅在显式 step 时检查 Node/pnpm。
5. 增加三 optimize mode smoke test和无 libc core 检查。
6. 编写 `CONTRIBUTING.md`，列出唯一命令与 hermetic toolchain。

## 验证

- `zig fmt --check`；
- Debug/ReleaseSafe/ReleaseFast `zig build test`；
- 非 0.16.0 明确失败；
- core 不隐式链接 libc/libm；
- 普通构建不访问 demo/node_modules/network；
- 从不同工作目录结果相同；
- 无空函数、mock artifact 或固定成功脚本。

## 完成判定

所有入口真实执行已有代码/测试，模块隔离生效。仅创建目录或空构建目标不算完成。

