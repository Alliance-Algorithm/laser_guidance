# 开发说明

## 目录职责

- `include/`
  - 对外头文件。当前采用扁平布局，只放稳定公共接口。
- `src/`
  - 核心实现，按 `core / vision / capture / streaming / tracking / io / guidance` 分模块。
- `tools/`
  - 手工运行入口、硬件 bring-up 与离线工具，不承担底层算法实现本身。
- `tests/`
  - 自动化验证目标，文件名统一为 `*_test.cpp`。
- `config/`
  - 默认运行、比赛、标定、采集配置。
- `models/`
  - `.onnx` / `.engine` 与电压映射模型文件。
- `test_data/`
  - 样本回放与固定测试资源。

## 当前目录拆分原则

当前已经有明确的模块拆分，不要再额外引入新的“大而泛”的中间层，例如：

- 全局 service locator
- 统一 runtime manager 大总管
- 面向未来外部总线的 bridge 抽象

如果后续出现下面任一情况，再考虑继续拆层：

- 单个模块内部出现两个以上长期并存的实现
- 多个入口共享了难以维护的大段运行时编排逻辑
- 某个数据结构同时承担配置、算法状态、外部协议三种职责

## 推荐改动顺序

当你要扩展功能时，优先级建议是：

1. 先判断需求属于 `runtime`、`guidance`、`vision`、`tracking` 还是 `bridge`
2. 优先扩展 typed `RuntimeCommand` / `RuntimeSnapshot` / internal helper，而不是继续扩 `TargetObservation` 或 `tool_*`
3. 扩展 `tests/` 自动验证，尽量覆盖真实 helper/real implementation，而不是复制逻辑进测试
4. 扩展 `tools/` 人工验证路径，但保持入口薄，不新增长期存在的工具私有主循环
5. 最后才考虑新增更细的目录或兼容层

这样可以避免一开始就把结构拆复杂。

## 文档维护规则

每次项目结构发生变化时，至少同步更新：

- `README.md`
- `docs/AGENTS.md`
- `docs/architecture.md`

如果运行方式、入口脚本或数据流发生变化，还必须同步补：

- 新的数据流图
- 新的输入输出说明
- 新的运行入口说明

## 当前完成标准

当前阶段的完成标准不是平台接入或体系集成，而是：

- 相机稳定运行
- pipeline 能稳定处理正常图像和异常输入
- 自动测试与人工入口都存在
- 核心逻辑保持 standalone，可独立构建和运行

## 当前模型接入约束

- ONNX Runtime 依赖当前必须保持为可选构建，不要把默认构建改成强依赖。
- 在没有真实模型契约前，不要提前把 `mean/std`、`NMS`、`class_names` 等模型专属配置写死进 public config。
- `model` 后端当前允许扩展内部 `ModelRuntime` / `ModelAdapter`，但不要把这些内部类型提升到 public API。
- 数据集生成相关能力当前应保持在 internal / tools 层，不要把 session/export manifest 结构提升到 public API。
- 本仓库只负责数据集生成和模型接入，不负责本地训练逻辑；训练流程默认放在外部平台。
