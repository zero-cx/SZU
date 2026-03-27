#!/bin/bash
# 环境检查脚本

echo "========================================"
echo "  LIO-SAM 环境检查"
echo "========================================"
echo ""

# 颜色
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

check_cmd() {
    if command -v $1 &> /dev/null; then
        echo -e "${GREEN}✓${NC} $1: $(command -v $1)"
        return 0
    else
        echo -e "${RED}✗${NC} $1: 未安装"
        return 1
    fi
}

check_pkg() {
    if dpkg -l | grep -q "^ii  $1"; then
        local ver=$(dpkg -l | grep "^ii  $1" | head -1 | awk '{print $3}')
        echo -e "${GREEN}✓${NC} $1: $ver"
        return 0
    else
        echo -e "${RED}✗${NC} $1: 未安装"
        return 1
    fi
}

echo "【ROS 环境】"
if [ -n "$ROS_DISTRO" ]; then
    echo -e "${GREEN}✓${NC} ROS_DISTRO: $ROS_DISTRO"
else
    echo -e "${RED}✗${NC} ROS 环境未激活"
fi
check_cmd roscore
check_cmd catkin_make
echo ""

echo "【核心依赖】"
check_pkg libgtsam-dev
check_pkg libpcl-dev
check_pkg libopencv-dev
check_pkg libboost-all-dev
echo ""

echo "【ROS 包】"
check_pkg ros-$ROS_DISTRO-tf
check_pkg ros-$ROS_DISTRO-cv-bridge
check_pkg ros-$ROS_DISTRO-pcl-conversions
echo ""

echo "【项目结构】"
WORKSPACE="/home/szu/test/demo04_ws"
if [ -d "$WORKSPACE/src/lio_sam" ]; then
    echo -e "${GREEN}✓${NC} 功能包: lio_sam"
    ls -1 $WORKSPACE/src/lio_sam/src/
else
    echo -e "${RED}✗${NC} 功能包不存在"
fi
echo ""

echo "【编译状态】"
if [ -d "$WORKSPACE/devel" ]; then
    echo -e "${GREEN}✓${NC} 已编译"
    if [ -d "$WORKSPACE/devel/lib/lio_sam" ]; then
        echo "可执行文件:"
        ls -1 $WORKSPACE/devel/lib/lio_sam/ | sed 's/^/  - /'
    fi
else
    echo -e "${YELLOW}⚠${NC} 未编译"
fi
echo ""

echo "========================================"
