/**
 * QR码检测与位姿估计测试程序 (Intel RealSense D435)
 * 
 * 编译: mkdir build && cd build && cmake .. && make
 * 运行: ./test_qr
 * 
 * 按 'q' 退出，按 's' 保存当前帧
 */

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <iomanip>

using namespace cv;
using namespace std;

// ==================== 用户可修改参数 ====================
// 相机参数 (D435 1280x720 近似值，建议用kalibr或ros标定)
const double FX = 905.0;  // 焦距x
const double FY = 905.0;  // 焦距y
const double CX = 640.0;  // 光心x (1280/2)
const double CY = 360.0;  // 光心y (720/2)

// 畸变系数 (D435 出厂近似值，可设为0先做测试)
const double K1 = 0.0, K2 = 0.0, P1 = 0.0, P2 = 0.0, K3 = 0.0;

// 二维码物理尺寸 (米)，打印时测量实际边长填写
const double MARKER_SIZE = 0.14;  // 14cm = 0.14m

// 相机设备号 (D435 RGB 通常是 /dev/video2，如果不行试 0 或 4)
const int CAMERA_ID = 2;
// ======================================================

int main(int argc, char** argv)
{
    cout << "程序启动..." << endl;
    cout << "尝试打开相机 /dev/video" << CAMERA_ID << endl;
    
    // 打开相机
    VideoCapture cap(CAMERA_ID);
    if (!cap.isOpened()) {
        cerr << "错误：无法打开相机 /dev/video" << CAMERA_ID << endl;
        cerr << "尝试: ls /dev/video* 查看可用设备" << endl;
        return -1;
    }
    cout << "相机已打开!" << endl;

    // 设置 D435 分辨率 (可选，降低分辨率可提高帧率)
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(CAP_PROP_FPS, 30);

    cout << "相机已启动，分辨率: " 
         << cap.get(CAP_PROP_FRAME_WIDTH) << "x" 
         << cap.get(CAP_PROP_FRAME_HEIGHT) << endl;

    // 相机内参矩阵
    Mat cameraMatrix = (Mat_<double>(3,3) << 
        FX, 0,  CX,
        0,  FY, CY,
        0,  0,  1);

    // 畸变向量
    Mat distCoeffs = (Mat_<double>(5,1) << K1, K2, P1, P2, K3);

    // ArUco 字典和检测参数
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
    Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();

    Mat frame, gray;
    int frame_count = 0;

    cout << "\n操作说明:" << endl;
    cout << "  q - 退出程序" << endl;
    cout << "  s - 保存当前帧到 screenshot.jpg" << endl;
    cout << "\n等待检测二维码..." << endl;

    while (true) {
        if (!cap.read(frame)) {
            cerr << "帧读取失败" << endl;
            break;
        }
        frame_count++;

        // 转换为灰度图
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // 检测标记
        vector<vector<Point2f>> corners;
        vector<int> ids;
        aruco::detectMarkers(gray, dictionary, corners, ids, parameters);

        // 如果检测到标记
        if (!ids.empty()) {
            // 绘制检测到的标记边框
            aruco::drawDetectedMarkers(frame, corners, ids);

            // 估计位姿 (rvec: 旋转向量, tvec: 平移向量)
            vector<Vec3d> rvecs, tvecs;
            aruco::estimatePoseSingleMarkers(corners, MARKER_SIZE, 
                                              cameraMatrix, distCoeffs, 
                                              rvecs, tvecs);

            // 绘制坐标轴并输出信息
            for (size_t i = 0; i < ids.size(); i++) {
                drawFrameAxes(frame, cameraMatrix, distCoeffs, 
                              rvecs[i], tvecs[i], 0.05);

                // 计算距离
                double distance = norm(tvecs[i]);
                
                // 转换为欧拉角（便于理解）
                Mat R;
                Rodrigues(rvecs[i], R);
                double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) + 
                                R.at<double>(1,0) * R.at<double>(1,0));
                double yaw = atan2(R.at<double>(2,1), R.at<double>(2,2)) * 180.0 / CV_PI;
                double pitch = atan2(-R.at<double>(2,0), sy) * 180.0 / CV_PI;
                double roll = atan2(R.at<double>(1,0), R.at<double>(0,0)) * 180.0 / CV_PI;

                // 在图像上显示信息
                string text = "ID:" + to_string(ids[i]) + 
                             " D:" + to_string(int(distance*100)) + "cm";
                putText(frame, text, Point(corners[i][0].x, corners[i][0].y - 10),
                        FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);

                // 控制台输出（每30帧输出一次，避免刷屏）
                if (frame_count % 30 == 0) {
                    cout << fixed << setprecision(2);
                    cout << "[ID " << ids[i] << "] ";
                    cout << "距离: " << distance << "m, ";
                    cout << "位置: [" << tvecs[i][0] << ", " 
                         << tvecs[i][1] << ", " << tvecs[i][2] << "], ";
                    cout << "角度: [yaw:" << yaw << ", pitch:" << pitch 
                         << ", roll:" << roll << "]" << endl;
                }
            }
        }

        // 显示状态信息
        string fps_text = "Frame: " + to_string(frame_count);
        putText(frame, fps_text, Point(10, 30), 
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);

        // 显示图像
        imshow("D435 QR Code Detection", frame);

        char key = (char)waitKey(1);
        if (key == 'q') {
            break;
        } else if (key == 's') {
            imwrite("screenshot.jpg", frame);
            cout << "截图已保存: screenshot.jpg" << endl;
        }
    }

    cap.release();
    destroyAllWindows();
    cout << "程序已退出" << endl;
    return 0;
}
