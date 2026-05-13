# 总目标

使用相机视觉引导激光振镜，使激光实时命中移动无人机搭载的特征靶。

硬件控制链路：Host PC → USB-to-SPI Bridge → SPI-to-DAC 驱动板 → 模拟电压 → 振镜 X/Y 角度。

## 当前阶段

引导最小闭环已打通，direct_voltage 为当前主线，SPI 推到 30MHz：

- `V4L2/UVC` 取图 ✅
- 相机标定（内参+畸变） ✅
- YOLO26 ONNX/TensorRT 端到端推理 ✅（3 class：purple/red/blue）
- 实时检测可视化（框+置信度+准星） ✅
- 采集卡配置（UGREEN 1080p@60 MJPG） ✅
- EKF tracker（常加速度模型）+ lookahead 40ms 超前预测 ✅
- FT4222 → DAC8568 振镜 SPI 控制（30MHz SCK）✅
- 单目测距（靶物理尺寸 + bbox 尺度，72×50mm） ✅
- 相机→振镜外参（平移已知：t_x=-92.5, t_y=30, t_z=100mm） ✅
- Direct voltage 视觉→电压映射（poly3 v6，1894 样本，6 轮迭代训练）✅
- LUT 电压映射实验模型（vision_voltage_lut_v1.yaml）✅
- 矩形面扫描（持续 raster，0.06°×0.08°，10×10 grid）✅
- 旋转外参在线标定（WASD 对准 → 空格记录 → solver 求解） ✅
- `purple confirmed` 命中样本自动采集 ✅

## 标定数据路径

所有 CSV 位于 `test_data/calib/`：
- `voltage_records.csv` — 累计电压标定数据（多轮追加，6 批）
- `voltage_records_v1.csv` — 第一批基线（873 样本）
- `voltage_records_merged_v*.csv` — 各版本合并去重数据集
- `geometry_calib_records.csv` — 几何外参标定记录
- `geometry_hit_calib_records.csv` — Purple 命中自动采集

## Direct Voltage 训练流

```bash
# 标定采集
./build/tool_guidance config/direct_voltage_calib.yaml
# WASD 对准 → 空格记录 → test_data/calib/voltage_records.csv

# 训练 poly3
python scripts/train_voltage_poly.py \
  --input test_data/calib/voltage_records_merged_v6.csv \
  --output models/vision_voltage_poly_v7.yaml

# 训练 LUT
python scripts/train_voltage_lut.py \
  --input test_data/calib/voltage_records_merged_v6.csv \
  --output models/vision_voltage_lut_v2.yaml
```

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
# WASD 微调激光对准靶 → 空格记录 → 换角度重复 → 生成 geometry_calib_records.csv

# 2. 求解旋转外参
./build/tool_calib_solve geometry_calib_records.csv config/calib_guidance.yaml
# 输出优化后的 r_x_deg, r_y_deg, r_z_deg

# 或直接使用命中样本
./build/tool_calib_solve geometry_hit_calib_records.csv config/default.yaml

# 3. 写入 default.yaml，正常跟踪
```

## 后续阶段

1. 振镜非线性标定（角度→电压多项式/LUT）
2. 超前预测补偿（固定时延，如 +10ms）
3. 激光落点视觉闭环（外环修正）
