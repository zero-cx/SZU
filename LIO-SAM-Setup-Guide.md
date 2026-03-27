# LIO-SAM 完整运行方案

## 📋 系统环境检查清单

| 组件 | 版本 | 状态 |
|------|------|------|
| ROS | Noetic | ✅ 已安装 |
| GTSAM | 4.0.3 | ✅ 已安装 |
| PCL | 1.10 | ✅ 已安装 |
| OpenCV | 4.2.0 | ✅ 已安装 |
| OpenMP | - | ✅ 已安装 |

## 🚀 快速开始（自动脚本）

```bash
# 1. 进入工作空间
cd /home/szu/test/demo04_ws

# 2. 运行自动设置脚本
bash setup_lio_sam.sh

# 3. 编译
catkin_make

# 4. 配置环境
source devel/setup.bash

# 5. 运行
roslaunch lio_sam run.launch
```

## 📁 项目结构

```
demo04_ws/
├── src/
│   └── lio_sam/              # 功能包（已重命名）
│       ├── src/
│       │   ├── imageProjection.cpp
│       │   ├── featureExtraction.cpp
│       │   ├── mapOptimization.cpp
│       │   └── imuPreintegration.cpp
│       ├── include/
│       │   └── utility.h
│       ├── config/
│       │   └── params.yaml
│       ├── launch/
│       │   └── run.launch
│       ├── msg/
│       └── srv/
├── build/                    # 编译生成
├── devel/                    # 开发环境
└── LIO-SAM-Setup-Guide.md    # 本指南
```

## 🔧 节点说明

| 节点名 | 功能 | 输入 | 输出 |
|--------|------|------|------|
| lio_sam_imageProjection | 点云去畸变 | 原始点云+IMU | 去畸变点云 |
| lio_sam_featureExtraction | 特征提取 | 去畸变点云 | 角点+面点 |
| lio_sam_mapOptimization | 地图优化 | 特征点+GPS | 位姿+地图 |
| lio_sam_imuPreintegration | IMU预积分 | IMU数据 | 高频里程计 |

## 📊 数据准备

### 支持的数据集

1. **KITTI** (推荐测试)
2. **VLP-16 数据集**
3. **Livox 数据集**
4. **Ouster 数据集**

### 录制自己的数据

```bash
# 录制数据包
rosbag record -o lio_sam_data.bag \
    /velodyne_points \
    /imu/data \
    /gps/fix
```

## ⚙️ 参数配置

编辑 `src/lio_sam/config/params.yaml`：

```yaml
# 关键参数
sensor: "velodyne"          # 传感器类型: velodyne/ouster/livox
N_SCAN: 16                  # 激光线数
Horizon_SCAN: 1800          # 水平分辨率

# IMU 配置
imuAccNoise: 0.01
imuGyrNoise: 0.001
imuGravity: 9.80511

# 外参标定 (Lidar -> IMU)
extrinsicTrans: [0.0, 0.0, 0.0]
extrinsicRot: [1, 0, 0, 0, 1, 0, 0, 0, 1]
```

## 🐛 常见问题

### 1. CMake 找不到 GTSAM
```bash
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

### 2. 点云 topic 不匹配
修改 `params.yaml` 中的 `pointCloudTopic`

### 3. IMU 坐标系错误
检查 `extrinsicRot` 和 `extrinsicRPY` 参数

### 4. 内存不足
```bash
# 限制线程数
rosparam set /lio_sam/numberOfCores 2
```

## 📈 性能优化

| 参数 | 默认值 | 建议 |
|------|--------|------|
| numberOfCores | 4 | 根据 CPU 调整 |
| mappingProcessInterval | 0.15 | 降低提高频率 |
| surroundingKeyframeSearchRadius | 50.0 | 减小加快搜索 |

## 📝 代码修改记录

### 已修复问题
1. ✅ 包名规范化 `LIO-SAM-master` → `lio_sam`
2. ✅ 拼写错误修复 `tolerance`
3. ✅ 异常捕获改为引用传递
4. ✅ 头文件命名空间清理

## 🎯 验证步骤

```bash
# 1. 检查节点启动
rosnode list | grep lio_sam

# 2. 查看话题
rostopic list | grep lio_sam

# 3. 可视化
rviz -d src/lio_sam/launch/include/config/rviz.rviz
```
