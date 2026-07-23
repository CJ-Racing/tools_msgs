# 双目鱼眼联合标定

## 概述

本工具链完成双目鱼眼相机的联合标定，输出内参 K/D 和外参 R/T，供 `position.py` 双目三角化使用。

**技术路线**：`cv2.fisheye.calibrate` (单目) → 归一化去畸变 → `cv2.stereoCalibrate` (双目 R/T)

> OpenCV 4.2 的 `fisheye.stereoCalibrate` 存在 assertion bug，本方案通过先去畸变再用标准 stereoCalibrate 规避。

## 目录结构

```
scripts/
├── capture_stereo_pairs.py   # Step 1: 采集同步图像对
├── stereo_calibrate.py       # Step 2: 联合标定
├── verify_calibration.py     # Step 3: 验证标定结果
├── images/                   # 采集的图像对
│   ├── left_000.png
│   ├── right_000.png
│   └── ...
└── output/                   # 标定输出
    ├── stereo_calib.npz      # NumPy 格式 (程序加载用)
    └── stereo_calib.yaml     # YAML 格式 (人类可读)
```

## 标定流程

### Step 1: 采集图像对

**前置条件**: ROS 节点运行中，双目相机发布图像话题。

```bash
# 默认话题
rosrun stereo_calib capture_stereo_pairs.py

# 自定义话题
rosrun stereo_calib capture_stereo_pairs.py \
    _left_topic:=/hik_camera/left/image \
    _right_topic:=/hik_camera/right/image
```

**操作**:
- `空格` 保存一对图像
- `q` 退出

**采集要求**:
- **数量**: ≥ 25 对（建议 30-40 对，允许自动剔除几张）
- **覆盖**: 标定板放置在画面不同位置（中心、四角、边缘）
- **角度**: 包含正面、倾斜 ≤30° 的多种姿态
- **距离**: 0.5m - 2.0m 范围内变换
- **要求**: 标定板在左右图中**都完全可见**

### Step 2: 联合标定

```bash
cd scripts/

# 当前标定板: 8×6 内角点 (9×7 方格), 70mm 方格
python3 stereo_calibrate.py --pattern 8 6 --square 0.070

# 如果更换标定板 (例: 11×8 内角点, 30mm 方格):
python3 stereo_calibrate.py --pattern 11 8 --square 0.030

# 使用其他目录的图像:
python3 stereo_calibrate.py --pattern 8 6 --square 0.070 --image-dir ./new_images
```

**参数说明**:

| 参数 | 含义 | 示例 |
|------|------|------|
| `--pattern COLS ROWS` | 棋盘格**内角点**数 | 9×7 方格板 → `--pattern 8 6` |
| `--square` | 方格边长 (米) | 70mm → `0.070` |
| `--image-dir` | 图像目录 | 默认 `./images` |
| `--output-dir` | 输出目录 | 默认 `./output` |
| `--show` | 显示角点检测结果 | 调试用 |

**质量标准**:

| 指标 | 优秀 | 可用 | 需重采 |
|------|------|------|--------|
| 重投影误差 | < 0.5 px | < 1.0 px | > 1.5 px |
| 光轴夹角 | 60-70° | 50-80° | 其他 |
| 基线长度 | 与物理安装一致 | ±20% | 偏差大 |

### Step 3: 验证

```bash
# 查看校正后极线对齐
python3 verify_calibration.py --index 0

# 保存校正图
python3 verify_calibration.py --index 5 --save
```

**判断标准**: 校正后左右图的同一物体应落在同一水平线上。

## 应用标定结果

标定成功后，将 `output/stereo_calib.npz` 中的参数更新到 `position.py`:

```python
# position.py __init__ 中硬编码标定值:
self.K_left = np.array([...])       # 从 npz['K_left'] 复制
self.D_left = np.array([...])       # 从 npz['D_left'] 复制
self.K_right = np.array([...])      # 从 npz['K_right'] 复制
self.D_right = np.array([...])      # 从 npz['D_right'] 复制
self.R_stereo = np.array([...])     # 从 npz['R_stereo'] 复制
self.T_stereo = np.array([...])     # 从 npz['T_stereo'] 复制
```

## 重新标定场景

以下情况需要重新标定:
1. 相机被物理碰动或重新安装
2. 双目匹配负深度比例突然升高
3. 更换镜头或调整焦距
4. 更换标定板规格

## 注意事项

- 标定板的 `--pattern` 是**内角点数**，不是方格数（例如 9×7 方格 → 8×6 内角点）
- 鱼眼畸变模型为 equidistant（4 系数 k1-k4），不是标准针孔的 5/8 系数模型
- 脚本会自动剔除条件数差的图像（最多剔除 1/3），无需手动筛选
