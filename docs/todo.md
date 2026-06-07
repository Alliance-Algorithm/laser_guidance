# 已完成工作记录

## 相机标定工具

- [x] 实现 `examples/camera_calibration.cpp`（~400 行，C++ OpenCV 棋盘格标定）
  - 复用 `V4l2Capture` 采集管线，OpenCV highgui 键盘交互
  - 采集 / 标定 / 保存 YAML / 加载 YAML / 去畸变预览 / 重置样本
  - 输出 `config/camera_calib.yaml`（YAML 格式）
- [x] P0 数据结构语义修复
  - 拆分 `CalibSample`（去重复 `object_points`）
  - `ChessboardDetection` 替代 `std::pair<bool, vector>`
  - `CalibrationUiStatus` 收束 overlay 参数
- [x] P1 提取 `CalibSession` 类，消除 `main()` 里的 `goto`
- [x] P2 load/save 改为 `std::expected<void/CalibResult, std::string>`，对齐仓库 V4l2Capture 风格

## CMake 构建

- [x] 新增 `example_camera_calibration` 到 CMakeLists.txt 构建和安装
- [x] 新增 OpenCV `calib3d` component
- [x] 精简 CMakeLists.txt：`file(GLOB)` 库源文件、list 驱动 example/test 循环、global property 自动记录安装 target、install 目录 foreach

## V4L2 采集链路

- [x] 新增 `bgr24` 像素格式（`V4l2PixelFormat::bgr24`，fourcc `BGR3`）
- [x] `V4l2Capture::open()` 加预热丢帧（8 帧），解决 UGREEN 开流前几帧 empty buffer
- [x] `V4l2Capture::read_frame()` 改为安全读帧 + 重试 10 次，捕获 `cv::Exception`

## 预览与采集配置

- [x] `example_v4l2_preview` 改为原生预览（不再走 Pipeline 检测和 overlay）
- [x] `config/capture_red_20m.yaml` pixel_format 固定 `mjpeg`
- [x] `ugreen-visualcamera.sh` auto 模式优先 YUYV（修 MJPG 优先导致 ffmpeg corrupted data）

## 相机硬件配置

- [x] 确定采集链路：Z50 Micro HDMI → UGREEN 95348 → /dev/video2
- [x] 确认 Z50 HDMI 输出上限：1080p60 Clean HDMI
- [x] 推荐曝光参数：M 档，f/5.6，1/500s，ISO 800，MF@20m
- [x] 推荐焦段：140mm（等效 210mm）

## 标定参数

- [x] 内参矩阵：从 Python tkinter 标定工具采的棋盘格数据（cx=993, cy=486, fx=13055）
- [x] 畸变系数：沿用 Python 标定结果（k2 偏大，14mm 长焦端四角可接受）
- [x] 已写入 `config/camera_calib.yaml`

## 已知待解决

- [ ] C++ 标定工具 cy 偏（画面底部采集不均），需重新采 15~25 张均匀覆盖全画面
- [ ] 棋盘格物理尺寸确认（当前 15mm 代码写死）
- [ ] `k2=33` 畸变系数偏大，后续用正确覆盖的棋盘格重标后应降低
