cmake_minimum_required(VERSION 3.0)
project(qr_test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 OpenCV
find_package(OpenCV 4 REQUIRED)

# 检查是否包含 aruco 模块
if("${OpenCV_LIBS}" MATCHES "aruco")
    message(STATUS "OpenCV aruco module found in OpenCV_LIBS")
else()
    message(STATUS "OpenCV aruco may be in contrib, trying to find opencv_aruco...")
    find_library(OPENCV_ARUCO_LIB opencv_aruco PATHS ${OpenCV_LIB_DIR} NO_DEFAULT_PATH)
    if(OPENCV_ARUCO_LIB)
        list(APPEND OpenCV_LIBS ${OPENCV_ARUCO_LIB})
        message(STATUS "Found opencv_aruco: ${OPENCV_ARUCO_LIB}")
    endif()
endif()

include_directories(${OpenCV_INCLUDE_DIRS})

# 编译检测程序
add_executable(test_qr test_qr.cpp)
target_link_libraries(test_qr ${OpenCV_LIBS})

# 编译简化版（无GUI，保存结果到文件）
add_executable(test_qr_simple test_qr_simple.cpp)
target_link_libraries(test_qr_simple ${OpenCV_LIBS})

# 可选：如果使用 RealSense SDK，取消下面注释
# find_package(realsense2 REQUIRED)
# target_link_libraries(test_qr ${realsense2_LIBRARY})

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV libs: ${OpenCV_LIBS}")
