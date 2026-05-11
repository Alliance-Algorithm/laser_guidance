# 总目标

使用相机视觉引导激光振镜，使激光实时命中移动无人机搭载的特征靶。

硬件控制链路：Host PC → USB-to-SPI Bridge → SPI-to-DAC 驱动板 → 模拟电压 → 振镜 X/Y 角度。

## 当前阶段

引导最小闭环已打通，外参标定工具就绪，进入精度调优阶段：

- `V4L2/UVC` 取图 ✅
- 相机标定（内参+畸变） ✅
- YOLO26 ONNX 端到端推理 ✅（3 class：purple/red/blue）
- 实时检测可视化（框+置信度+准星） ✅
- 采集卡配置（UGREEN 1080p@60 MJPG） ✅
- EKF tracker（常加速度模型，像素平滑+预测） ✅
- FT4222 → DAC8568 振镜 SPI 控制 ✅
- 单目测距（靶物理尺寸 + bbox 尺度，72×50mm） ✅
- 相机→振镜外参（平移已知：t_x=-92.5, t_y=30, t_z=100mm） ✅
- 旋转外参在线标定（WASD 对准 → 空格记录 → solver 求解） ✅
- `purple confirmed` 命中样本自动采集（`hit_calib_records.csv`）✅
- 矩形面扫描（持续 raster，扩大覆盖面积） ✅
- 振镜解析运动学（含镜间距 + Euler 旋转） ✅
- 线性角度→电压映射（±30° ↔ ±5V） ✅
- EKF-A 集成：瞄准用 EKF 预测中心，测距用 bbox ✅

## 物理外参（已确定，不动）

| 参数 | 值 | 含义 |
|---|---|---|
| t_x_mm | -92.5 | 相机在激光右边 92.5mm |
| t_y_mm | 30.0 | 相机在激光上面 30mm |
| t_z_mm | 100.0 | 相机在激光后面 100mm |
| r_x_deg | 待求解 | 旋转（pitch） |
| r_y_deg | 待求解 | 旋转（yaw） |
| r_z_deg | 待求解 | 旋转（roll） |

## 标定工作流

```bash
# 1. 采集数据
./build/tool_guidance config/calib_guidance.yaml
# WASD 微调激光对准靶 → 空格记录 → 换角度重复 → 生成 calib_records.csv

# 2. 求解旋转外参
./build/tool_calib_solve calib_records.csv config/calib_guidance.yaml
# 输出优化后的 r_x_deg, r_y_deg, r_z_deg

# 或直接使用命中样本
./build/tool_calib_solve hit_calib_records.csv config/default.yaml

# 3. 写入 default.yaml，正常跟踪
```

## 后续阶段

1. 振镜非线性标定（角度→电压多项式/LUT）
2. 超前预测补偿（固定时延，如 +10ms）
3. 激光落点视觉闭环（外环修正）
