#!/bin/bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/szu/test/demo04_ws}"
QR_IMAGE_TOPIC="${QR_IMAGE_TOPIC:-/camera/color/image_raw}"
QR_CAMERA_INFO_TOPIC="${QR_CAMERA_INFO_TOPIC:-/camera/color/camera_info}"
QR_DETECTION_TOPIC="${QR_DETECTION_TOPIC:-/qr_detection/detection}"
QR_DEBUG_IMAGE_TOPIC="${QR_DEBUG_IMAGE_TOPIC:-/qr_detection/image}"
QR_PUBLISH_IMAGE="${QR_PUBLISH_IMAGE:-false}"
TOPIC_TIMEOUT="${TOPIC_TIMEOUT:-8}"

echo "========================================"
echo "  LIO-SAM 检测链路快速预检"
echo "========================================"

cd "$WORKSPACE"
source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/*/setup.bash
source "$WORKSPACE/devel/setup.bash"

pass=0
fail=0

ok() {
    echo "[PASS] $1"
    pass=$((pass + 1))
}

ng() {
    echo "[FAIL] $1"
    fail=$((fail + 1))
}

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

echo "\n[1/5] 检查自定义消息"
if rosmsg show lio_sam/qr_detection >/dev/null 2>&1; then
    ok "lio_sam/qr_detection 可识别"
else
    ng "lio_sam/qr_detection 不可识别（请检查编译与 source）"
fi

echo "\n[2/5] 检查图像 topic: $QR_IMAGE_TOPIC"
if wait_topic_has_publisher "$QR_IMAGE_TOPIC" "$TOPIC_TIMEOUT"; then
    ok "图像 topic 有发布者"
else
    ng "图像 topic 无发布者"
fi

echo "\n[3/5] 检查 camera_info topic: $QR_CAMERA_INFO_TOPIC"
if wait_topic_has_publisher "$QR_CAMERA_INFO_TOPIC" "$TOPIC_TIMEOUT"; then
    ok "camera_info topic 有发布者"
else
    ng "camera_info topic 无发布者"
fi

echo "\n[4/5] 检查检测输出 topic: $QR_DETECTION_TOPIC"
if wait_topic_has_publisher "$QR_DETECTION_TOPIC" "$TOPIC_TIMEOUT"; then
    ok "检测输出 topic 有发布者"
else
    ng "检测输出 topic 无发布者（请确认已启动 qrDetection 节点）"
fi

echo "\n[5/5] 检查调试图 topic（仅调试模式）"
if [ "$QR_PUBLISH_IMAGE" = "true" ]; then
    if wait_topic_has_publisher "$QR_DEBUG_IMAGE_TOPIC" "$TOPIC_TIMEOUT"; then
        ok "调试图 topic 有发布者"
    else
        ng "调试图 topic 无发布者（QR_PUBLISH_IMAGE=true）"
    fi
else
    echo "[SKIP] QR_PUBLISH_IMAGE=false，跳过调试图 topic 检查"
fi

echo "\n========================================"
echo "预检完成: PASS=$pass FAIL=$fail"
if [ "$fail" -gt 0 ]; then
    exit 1
fi
exit 0
