# Docs

当前文档按三个层次组织：

- `architecture.md`
  - 说明当前最小骨架的真实数据流、目标关系和边界。
- `development.md`
  - 说明目录职责、代码演进规则和开发时的判断原则。
- `dataset_collection.md`
  - 说明如何用采集卡录原始视频会话，并直接上传或按需离线抽帧生成待标注数据集。
- `hardware_ft4222_dac8568.md`
  - 说明 `FT4222 USB-to-SPI` 到 `DAC8568 ±10V` 模块的供电、接线、SPI 时序与最小联调流程。
- `runtime_operations.md` / `runtime-metrics.md`
  - 说明 freshness runtime 的运行约束、日志指标与运行模式。

当前文档默认面向 standalone 使用。

推荐阅读顺序：

1. `README.md`
2. `architecture.md`
3. `hardware_ft4222_dac8568.md`
4. `dataset_collection.md`
5. `development.md`
