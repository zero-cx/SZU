#!/bin/bash
# LIO-SAM 启动脚本

echo "========================================"
echo "  LIO-SAM 启动器"
echo "========================================"

WORKSPACE="/home/szu/test/demo04_ws"
cd $WORKSPACE

# 检查编译
if [ ! -d "$WORKSPACE/devel" ]; then
    echo "错误: 未找到编译结果，请先运行 catkin_make"
    exit 1
fi

# 加载环境
echo "[1/4] 加载 ROS 环境..."
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# 检查依赖
echo "[2/4] 检查依赖..."
if rospack find robot_localization > /dev/null 2>&1; then
    echo "  ✓ robot_localization 已安装"
    NAVSAT_AVAILABLE=true
else
    echo "  ⚠ robot_localization 未安装 (GPS功能已自动禁用)"
    echo "    如需GPS功能，运行: sudo apt install ros-noetic-robot-localization"
    NAVSAT_AVAILABLE=false
fi

# 检查节点是否存在
echo "[3/4] 检查可执行文件..."
for node in imageProjection featureExtraction mapOptmization imuPreintegration; do
    if [ -f "$WORKSPACE/devel/lib/lio_sam/lio_sam_$node" ]; then
        echo "  ✓ $node"
    else
        echo "  ✗ $node 缺失"
        exit 1
    fi
done

# 启动
echo "[4/4] 启动 LIO-SAM..."
echo ""
echo "提示:"
echo "  - 按 Ctrl+C 停止"
echo "  - 新终端播放数据: rosbag play xxx.bag"
echo "  - 启用GPS功能: roslaunch lio_sam run.launch use_navsat:=true"
echo ""

sleep 1
if [ "$NAVSAT_AVAILABLE" = true ]; then
    roslaunch lio_sam run.launch
else
    roslaunch lio_sam run.launch use_navsat:=false
fi
