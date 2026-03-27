#!/bin/bash
# 快速测试脚本 - 验证 LIO-SAM 基本功能

echo "========================================"
echo "  LIO-SAM 快速测试"
echo "========================================"

WORKSPACE="/home/szu/test/demo04_ws"
cd $WORKSPACE

# 加载环境
source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/*/setup.bash
source devel/setup.bash

echo ""
echo "【1/4】检查节点】"
rosnode list 2>/dev/null | grep lio_sam || echo "节点未启动（正常，还未运行）"

echo ""
echo "【2/4】检查话题】"
rostopic list 2>/dev/null | grep lio_sam | head -10 || echo "话题未创建"

echo ""
echo "【3/4】检查参数】"
rosparam list 2>/dev/null | grep lio_sam | head -5 || echo "参数未加载"

echo ""
echo "【4/4】启动测试（5秒后自动关闭）】"
echo "启动 roslaunch lio_sam run.launch..."
timeout 5 roslaunch lio_sam run.launch 2>&1 | tail -20 || true

echo ""
echo "========================================"
echo "测试完成！"
echo ""
echo "正常运行命令:"
echo "  roslaunch lio_sam run.launch"
