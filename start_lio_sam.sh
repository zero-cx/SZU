#!/bin/bash
set -euo pipefail

# LIO-SAM 一键启动脚本（可选相机/rosbag）
# 运行模式：
#   RUN_MODE=pure_lio    -> 纯 LIO-SAM
#   RUN_MODE=detect_only -> 仅二维码检测（关闭融合因子）
#   RUN_MODE=qr_fusion   -> 二维码检测 + 融合

WORKSPACE="${WORKSPACE:-/home/szu/test/demo04_ws}"
INPUT_MODE="${INPUT_MODE:-camera}"      # camera | bag | none
USE_NAVSAT="${USE_NAVSAT:-auto}"        # auto | true | false
BAG_FILE="${BAG_FILE:-}"
WAIT_TIMEOUT="${WAIT_TIMEOUT:-20}"

RUN_MODE="${RUN_MODE:-qr_fusion}"
QR_IMAGE_TOPIC="${QR_IMAGE_TOPIC:-/camera/color/image_raw}"
QR_CAMERA_INFO_TOPIC="${QR_CAMERA_INFO_TOPIC:-/camera/color/camera_info}"
QR_CAMERA_FRAME="${QR_CAMERA_FRAME:-camera_color_optical_frame}"
QR_MARKER_SIZE="${QR_MARKER_SIZE:-0.14}"
QR_CONFIDENCE_THRESHOLD="${QR_CONFIDENCE_THRESHOLD:-0.55}"
QR_PUBLISH_IMAGE="${QR_PUBLISH_IMAGE:-false}"
QR_DICTIONARY_NAME="${QR_DICTIONARY_NAME:-DICT_6X6_250}"
QR_POSE_TOPIC="${QR_POSE_TOPIC:-/qr_detection/pose}"
QR_DETECTION_TOPIC="${QR_DETECTION_TOPIC:-/qr_detection/detection}"
QR_DEBUG_IMAGE_TOPIC="${QR_DEBUG_IMAGE_TOPIC:-/qr_detection/image}"

# 兼容历史变量 USE_QR
USE_QR="${USE_QR:-}"
if [ -n "$USE_QR" ]; then
    if [ "$USE_QR" = "true" ] && [ "$RUN_MODE" = "pure_lio" ]; then
        RUN_MODE="qr_fusion"
    elif [ "$USE_QR" = "false" ]; then
        RUN_MODE="pure_lio"
    fi
fi

ENABLE_COLOR="${ENABLE_COLOR:-true}"
ALIGN_DEPTH="${ALIGN_DEPTH:-true}"
COLOR_WIDTH="${COLOR_WIDTH:-1280}"
COLOR_HEIGHT="${COLOR_HEIGHT:-720}"
COLOR_FPS="${COLOR_FPS:-30}"

CAMERA_PID=""
BAG_PID=""

cleanup() {
    local code=$?
    trap - INT TERM EXIT
    if [ -n "${BAG_PID}" ] && kill -0 "${BAG_PID}" 2>/dev/null; then
        kill "${BAG_PID}" 2>/dev/null || true
    fi
    if [ -n "${CAMERA_PID}" ] && kill -0 "${CAMERA_PID}" 2>/dev/null; then
        kill "${CAMERA_PID}" 2>/dev/null || true
    fi
    exit $code
}
trap cleanup INT TERM EXIT

wait_topic_has_publisher() {
    local topic="$1"
    local timeout_sec="$2"
    local elapsed=0
    while [ "$elapsed" -lt "$timeout_sec" ]; do
        if rostopic info "$topic" 2>/dev/null | grep -q "Publishers:" && \
           ! rostopic info "$topic" 2>/dev/null | grep -q "Publishers: None"; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    return 1
}

echo "========================================"
echo "  LIO-SAM 一键启动器"
echo "========================================"

echo "[0/5] 检查工作空间..."
cd "$WORKSPACE"
if [ ! -d "$WORKSPACE/devel" ]; then
    echo "错误: 未找到编译结果，请先运行 catkin_make"
    exit 1
fi

echo "[1/5] 加载 ROS 环境..."
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"

echo "[2/5] 检查依赖与可执行文件..."
rospack find lio_sam >/dev/null
for node in imageProjection featureExtraction mapOptmization imuPreintegration qrDetection; do
    if [ -f "$WORKSPACE/devel/lib/lio_sam/lio_sam_${node}" ]; then
        echo "  ✓ ${node}"
    else
        echo "  ✗ ${node} 缺失"
        exit 1
    fi
done

if [ "$USE_NAVSAT" = "auto" ]; then
    if rospack find robot_localization >/dev/null 2>&1; then
        USE_NAVSAT="true"
        echo "  ✓ robot_localization 已安装，自动启用 use_navsat"
    else
        USE_NAVSAT="false"
        echo "  ⚠ robot_localization 未安装，自动关闭 use_navsat"
    fi
fi

case "$RUN_MODE" in
    pure_lio)
        USE_QR_DETECTION="false"
        USE_QR_FACTOR="false"
        ;;
    detect_only)
        USE_QR_DETECTION="true"
        USE_QR_FACTOR="false"
        ;;
    qr_fusion)
        USE_QR_DETECTION="true"
        USE_QR_FACTOR="true"
        ;;
    *)
        echo "错误: RUN_MODE 必须是 pure_lio | detect_only | qr_fusion"
        exit 1
        ;;
esac

echo "[3/5] 启动输入源（模式: $INPUT_MODE）..."
case "$INPUT_MODE" in
    camera)
        if ! rospack find realsense2_camera >/dev/null 2>&1; then
            echo "错误: 未找到 realsense2_camera 包。"
            echo "可选方案: INPUT_MODE=bag 或 INPUT_MODE=none"
            exit 1
        fi

        roslaunch realsense2_camera rs_camera.launch \
            enable_color:="$ENABLE_COLOR" \
            align_depth:="$ALIGN_DEPTH" \
            color_width:="$COLOR_WIDTH" \
            color_height:="$COLOR_HEIGHT" \
            color_fps:="$COLOR_FPS" \
            >/tmp/realsense_launch.log 2>&1 &
        CAMERA_PID=$!

        echo "  - 等待 $QR_IMAGE_TOPIC 发布者..."
        if ! wait_topic_has_publisher "$QR_IMAGE_TOPIC" "$WAIT_TIMEOUT"; then
            echo "错误: 等待 $QR_IMAGE_TOPIC 超时"
            echo "请检查 /tmp/realsense_launch.log"
            exit 1
        fi

        echo "  - 等待 $QR_CAMERA_INFO_TOPIC 发布者..."
        if ! wait_topic_has_publisher "$QR_CAMERA_INFO_TOPIC" "$WAIT_TIMEOUT"; then
            echo "错误: 等待 $QR_CAMERA_INFO_TOPIC 超时"
            echo "请检查 /tmp/realsense_launch.log"
            exit 1
        fi
        echo "  ✓ 相机话题已就绪"
        ;;

    bag)
        if [ -z "$BAG_FILE" ]; then
            echo "错误: INPUT_MODE=bag 时必须提供 BAG_FILE"
            exit 1
        fi
        if [ ! -f "$BAG_FILE" ]; then
            echo "错误: BAG_FILE 不存在: $BAG_FILE"
            exit 1
        fi

        rosparam set use_sim_time true
        rosbag play "$BAG_FILE" --clock >/tmp/rosbag_play.log 2>&1 &
        BAG_PID=$!
        echo "  ✓ rosbag 已启动: $BAG_FILE"
        ;;

    none)
        echo "  ⚠ 跳过输入源启动，请确保外部节点会发布 Lidar/IMU/相机话题"
        ;;

    *)
        echo "错误: INPUT_MODE 必须是 camera | bag | none"
        exit 1
        ;;
esac

echo "[4/5] 设置模式参数..."
rosparam set /lio_sam/useQRFactor "$USE_QR_FACTOR"
echo "  RUN_MODE: $RUN_MODE"
echo "  use_qr_detection: $USE_QR_DETECTION"
echo "  /lio_sam/useQRFactor: $USE_QR_FACTOR"

echo "[5/5] 运行主程序（按 Ctrl+C 停止）"
roslaunch lio_sam run.launch \
    use_qr_detection:="$USE_QR_DETECTION" \
    use_navsat:="$USE_NAVSAT" \
    qr_marker_size:="$QR_MARKER_SIZE" \
    qr_confidence_threshold:="$QR_CONFIDENCE_THRESHOLD" \
    qr_publish_image:="$QR_PUBLISH_IMAGE" \
    qr_image_topic:="$QR_IMAGE_TOPIC" \
    qr_camera_info_topic:="$QR_CAMERA_INFO_TOPIC" \
    qr_pose_topic:="$QR_POSE_TOPIC" \
    qr_detection_topic:="$QR_DETECTION_TOPIC" \
    qr_debug_image_topic:="$QR_DEBUG_IMAGE_TOPIC" \
    qr_camera_frame:="$QR_CAMERA_FRAME" \
    qr_dictionary_name:="$QR_DICTIONARY_NAME"
