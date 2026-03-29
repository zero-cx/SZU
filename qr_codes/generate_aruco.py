#!/usr/bin/env python3
"""
生成 ArUco 标记用于打印
打印尺寸建议：15cm x 15cm（实际二维码边长 14cm，留白边）
"""
import cv2
import os

# 使用 6x6 字典，ID 0-249 可用
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# 创建输出目录
os.makedirs("markers", exist_ok=True)

# 生成 4 个标记（你可以打印多个贴在不同位置）
marker_ids = [0, 1, 2, 3]
marker_size_px = 400  # 图片分辨率

for marker_id in marker_ids:
    img = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size_px)
    filename = f"markers/marker_{marker_id}.png"
    cv2.imwrite(filename, img)
    print(f"已生成: {filename}")

print("\n打印建议：")
print("- 在 Word/PPT 中插入图片，设置尺寸为 14cm x 14cm")
print("- 用硬纸板或亚克力板固定，确保平整")
print("- 贴到墙面或支架上，高度与相机平齐")
