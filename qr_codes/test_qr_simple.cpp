/**
 * QR码检测测试程序 (无GUI版本，保存结果到文件)
 * 
 * 编译: cd build && cmake .. && make
 * 运行: ./test_qr_simple
 * 
 * 输出: input.jpg - 原始图像, result.jpg - 检测结果
 */

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <iomanip>

using namespace cv;
using namespace std;

// ==================== 用户参数 ====================
const double FX = 905.0, FY = 905.0;  // 焦距
const double CX = 640.0, CY = 360.0;  // 光心
const double MARKER_SIZE = 0.14;      // 二维码边长(米)
const int CAMERA_ID = 2;              // 设备号
const int FRAME_WIDTH = 1280;         // 分辨率
const int FRAME_HEIGHT = 720;
// =================================================

int main(int argc, char** argv)
{
    // 打开相机
    VideoCapture cap(CAMERA_ID);
    if (!cap.isOpened()) {
        cerr << "错误：无法打开相机 /dev/video" << CAMERA_ID << endl;
        return -1;
    }

    cap.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

    // 丢弃前几帧（相机启动需要热身）
    Mat frame;
    for (int i = 0; i < 5; i++) {
        cap.read(frame);
    }

    // 读取一帧
    if (!cap.read(frame)) {
        cerr << "读取帧失败" << endl;
        return -1;
    }

    cout << "成功捕获图像: " << frame.cols << "x" << frame.rows << endl;
    imwrite("input.jpg", frame);
    cout << "已保存: input.jpg" << endl;

    // 准备检测
    Mat gray;
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
    Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();

    vector<vector<Point2f>> corners;
    vector<int> ids;
    aruco::detectMarkers(gray, dictionary, corners, ids, parameters);

    // 相机参数
    Mat cameraMatrix = (Mat_<double>(3,3) << FX, 0, CX, 0, FY, CY, 0, 0, 1);
    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);

    if (!ids.empty()) {
        cout << "检测到 " << ids.size() << " 个标记:" << endl;

        // 绘制检测边框
        aruco::drawDetectedMarkers(frame, corners, ids);

        // 估计位姿
        vector<Vec3d> rvecs, tvecs;
        aruco::estimatePoseSingleMarkers(corners, MARKER_SIZE, cameraMatrix, distCoeffs, rvecs, tvecs);

        for (size_t i = 0; i < ids.size(); i++) {
            drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.05);

            double distance = norm(tvecs[i]);
            
            // 标注信息
            string text = "ID:" + to_string(ids[i]) + " Dist:" + to_string(int(distance*100)) + "cm";
            putText(frame, text, Point(corners[i][0].x, corners[i][0].y - 10),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);

            cout << "  [ID " << ids[i] << "] 距离: " << fixed << setprecision(3) 
                 << distance << "m, 位置: [" << tvecs[i][0] << ", " 
                 << tvecs[i][1] << ", " << tvecs[i][2] << "]" << endl;
        }

        imwrite("result.jpg", frame);
        cout << "检测结果已保存: result.jpg" << endl;
    } else {
        cout << "未检测到任何标记" << endl;
        imwrite("result.jpg", frame);
        cout << "原图已保存: result.jpg (无检测结果)" << endl;
    }

    cap.release();
    return 0;
}
