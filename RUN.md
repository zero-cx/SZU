# 🚀 LIO-SAM 运行指南

## ✅ 编译状态

```
编译时间: 2025-03-27
编译状态: ✅ 成功
生成文件: 4 个可执行文件
```

## 📁 可执行文件

| 节点 | 大小 | 功能 |
|------|------|------|
| lio_sam_imageProjection | 19MB | 点云去畸变 |
| lio_sam_featureExtraction | 17MB | 特征提取 |
| lio_sam_mapOptmization | 65MB | 地图优化 |
| lio_sam_imuPreintegration | 34MB | IMU预积分 |

## 🏃 快速运行

### 1. 加载环境

```bash
cd /home/szu/test/demo04_ws
source devel/setup.bash
```

### 2. 启动 LIO-SAM

```bash
roslaunch lio_sam run.launch
```

### 3. 可视化

```bash
# 新终端
rviz -d src/lio_sam/launch/include/config/rviz.rviz
```

### 4. 播放数据包

```bash
# 新终端
rosbag play your_data.bag
```

## 🔧 核心修改记录

### 代码修复

| 问题 | 修复方案 | 状态 |
|------|----------|------|
| OpenCV 头文件 | `opencv/cv.h` → `opencv2/opencv.hpp` | ✅ |
| C++ 标准 | `c++11` → `c++14` | ✅ |
| FLANN 序列化 | 添加序列化特化模板 | ✅ |
| 拼写错误 | `tolerance` 修正 | ✅ |
| 异常捕获 | 改为引用捕获 | ✅ |
| 命名空间 | 头文件清理 using namespace | ✅ |
| 包名 | `LIO-SAM-master` → `lio_sam` | ✅ |

### CMakeLists.txt 修改

```cmake
set(CMAKE_CXX_FLAGS "-std=c++14")
add_compile_options(-Wno-deprecated -Wno-unused-variable)
```

### utility.h 修改

```cpp
// OpenCV 4.x 兼容
#include <opencv2/opencv.hpp>

// FLANN 序列化兼容
namespace flann { namespace serialization { 
    template <typename T> struct Serializer;
    template <typename K, typename V> struct Serializer<std::unordered_map<K,V>> {
        template <typename InputArchive> static void load(InputArchive&, std::unordered_map<K,V>&) {}
        template <typename OutputArchive> static void save(OutputArchive&, const std::unordered_map<K,V>&) {}
    };
}}
```

## 📝 配置参数

编辑 `src/lio_sam/config/params.yaml`：

```yaml
# 传感器类型
sensor: "velodyne"  # 可选: velodyne, ouster, livox

# 激光雷达参数
N_SCAN: 16              # 线数
Horizon_SCAN: 1800      # 水平分辨率

# Topic 配置
pointCloudTopic: "points_raw"
imuTopic: "imu_correct"
odomTopic: "odometry/imu"
gpsTopic: "odometry/gps"

# IMU 噪声参数
imuAccNoise: 0.01
imuGyrNoise: 0.001
imuGravity: 9.80511

# 外参 (Lidar -> IMU)
extrinsicTrans: [0.0, 0.0, 0.0]
extrinsicRot: [1, 0, 0, 0, 1, 0, 0, 0, 1]
extrinsicRPY: [1, 0, 0, 0, 1, 0, 0, 0, 1]
```

## 🐛 故障排除

### 1. CMake 找不到 GTSAM

```bash
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

### 2. 编译内存不足

```bash
catkin_make -j1  # 单线程编译
```

### 3. 运行时库错误

```bash
# 添加库路径
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

### 4. 点云不显示

- 检查 topic 名称是否匹配
- 检查 frame_id 设置
- 确认时间戳同步

## 📊 话题列表

```
/lio_sam/deskew/cloud_deskewed       # 去畸变点云
/lio_sam/feature/cloud_corner        # 角点特征
/lio_sam/feature/cloud_surface       # 面点特征
/lio_sam/mapping/odometry            # 位姿输出
/lio_sam/mapping/path                # 轨迹
/lio_sam/mapping/map_global          # 全局地图
```

## 🎯 下一步

1. **准备数据**: 使用 rosbag 录制或下载公开数据集
2. **标定外参**: 使用 lio_sam 的 IMU 调试工具
3. **调整参数**: 根据传感器型号修改配置
4. **运行测试**: `roslaunch lio_sam run.launch`

---

**环境**: ROS Noetic + Ubuntu 20.04  
**状态**: ✅ 就绪运行
