#!/bin/bash
# LIO-SAM 自动设置脚本
# 作者: Code Review Assistant
# 日期: 2025-03-26

set -e  # 遇到错误立即退出

echo "========================================"
echo "  LIO-SAM 自动设置脚本"
echo "========================================"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 工作空间路径
WORKSPACE="/home/szu/test/demo04_ws"

# 检查 ROS 环境
echo -e "${YELLOW}[1/6] 检查 ROS 环境...${NC}"
if [ -z "$ROS_DISTRO" ]; then
    if [ -f /opt/ros/noetic/setup.bash ]; then
        source /opt/ros/noetic/setup.bash
        echo -e "${GREEN}✓ ROS Noetic 环境已加载${NC}"
    else
        echo -e "${RED}✗ 未找到 ROS 环境${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}✓ ROS $ROS_DISTRO 已激活${NC}"
fi

# 检查依赖
echo -e "${YELLOW}[2/6] 检查依赖...${NC}"
MISSING_DEPS=""

# 检查 GTSAM
if ! dpkg -l | grep -q libgtsam-dev; then
    MISSING_DEPS="$MISSING_DEPS libgtsam-dev"
fi

# 检查 PCL
if ! dpkg -l | grep -q libpcl-dev; then
    MISSING_DEPS="$MISSING_DEPS libpcl-dev"
fi

# 检查 OpenCV
if ! dpkg -l | grep -q libopencv-dev; then
    MISSING_DEPS="$MISSING_DEPS libopencv-dev"
fi

if [ -n "$MISSING_DEPS" ]; then
    echo -e "${YELLOW}⚠ 缺少依赖，尝试自动安装:$MISSING_DEPS${NC}"
    sudo apt-get update
    sudo apt-get install -y $MISSING_DEPS || {
        echo -e "${RED}✗ 依赖安装失败${NC}"
        exit 1
    }
else
    echo -e "${GREEN}✓ 所有依赖已安装${NC}"
fi

# 清理旧编译（可选）
echo -e "${YELLOW}[3/6] 清理旧编译文件...${NC}"
cd $WORKSPACE
if [ -d build ] || [ -d devel ]; then
    read -p "是否清理旧编译文件？(y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf build devel
        echo -e "${GREEN}✓ 已清理${NC}"
    fi
else
    echo -e "${GREEN}✓ 无需清理${NC}"
fi

# 初始化工作空间
echo -e "${YELLOW}[4/6] 初始化工作空间...${NC}"
if [ ! -f src/CMakeLists.txt ]; then
    catkin_init_workspace src || true
fi
echo -e "${GREEN}✓ 工作空间已初始化${NC}"

# 安装 ROS 依赖
echo -e "${YELLOW}[5/6] 安装 ROS 依赖...${NC}"
cd $WORKSPACE
if command -v rosdep &> /dev/null; then
    rosdep install --from-paths src --ignore-src -y || {
        echo -e "${YELLOW}⚠ rosdep 安装部分失败，继续编译...${NC}"
    }
else
    echo -e "${YELLOW}⚠ rosdep 未安装，跳过${NC}"
fi

# 编译
echo -e "${YELLOW}[6/6] 编译项目...${NC}"
cd $WORKSPACE
catkin_make -j$(nproc) 2>&1 | tee build.log

# 检查编译结果
if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  ✓ 编译成功！${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo "使用步骤:"
    echo "  1. source devel/setup.bash"
    echo "  2. roslaunch lio_sam run.launch"
    echo ""
    echo "可执行文件:"
    ls -1 devel/lib/lio_sam/
else
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}  ✗ 编译失败${NC}"
    echo -e "${RED}========================================${NC}"
    echo ""
    echo "查看 build.log 了解详细信息"
    echo "常见问题:"
    echo "  - GTSAM 版本不兼容"
    echo "  - 内存不足（尝试 catkin_make -j1）"
    exit 1
fi
