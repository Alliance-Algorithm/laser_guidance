# TodoList

## 已完成

- [x] `CompetitionRuntime` 收口为 `main` / `preview`
- [x] `tool_guidance` 脱离共享 runtime，改为 `GuidanceOpsApp`
- [x] `ControlLoop` 改为直接组合具体组件
- [x] `CaptureDevice` 收口为具体入口
- [x] `PerceptionRunner` 直接管理 ONNX / TensorRT backend
- [x] `ModelAdapter` 改为 free-function 适配层
- [x] ws30 / lidar 接入删除
- [x] 主文档与示例配置同步更新
- [x] `RuntimeCommand` 改为 `std::variant` closed sum type，工厂在 `runtime_command::` namespace
- [x] `Error` / `ErrorKind` 分类错误域 + `format_error`，public 控制路径使用 `std::expected<T, Error>`
- [x] 迁移 capture/guidance/FT4222 内部错误返回至 `Error` 域
- [x] `LatestValue` 非抛出 shutdown（`pop` 返回 `std::optional<T>`）
- [x] `PerceptionRunner` 改用 `std::jthread`
- [x] Hik backend 类型迁出 `capture_device.hpp` 至 `src/capture/hik_backend.{hpp,cpp}`
- [x] `laser_guidance_support` 使用显式 CMake 源文件列表

## 后续可做

- [ ] 增加更多 `CompetitionRuntime(main / preview)` 验收测试
- [ ] 增加 Hik backend 的硬件侧回归验证
- [ ] 检查可选构建矩阵下的编译覆盖率
