/**
 * QR码检测 ROS 节点 (简化版)
 * 
 * 订阅: /camera/color/image_raw
 *       /camera/color/camera_info
 * 发布: /qr_detection/pose
 *       /qr_detection/image
 */

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

class QRDetection
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Subscriber subImage;
    ros::Subscriber subCameraInfo;
    ros::Publisher pubPose;
    ros::Publisher pubImage;
    
    Ptr<aruco::Dictionary> dictionary;
    Ptr<aruco::DetectorParameters> detectorParams;
    
    Mat cameraMatrix;
    Mat distCoeffs;
    bool cameraInfoReceived;
    
    double markerSize;
    bool publishImage;
    
public:
    QRDetection() : pnh("~"), cameraInfoReceived(false)
    {
        // 读取参数
        pnh.param<double>("marker_size", markerSize, 0.14);
        pnh.param<bool>("publish_image", publishImage, true);
        
        ROS_INFO("QR Detection Node Started");
        ROS_INFO("Marker size: %.2f m", markerSize);
        
        // 初始化 ArUco
        dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
        detectorParams = aruco::DetectorParameters::create();
        
        // 调低检测阈值，提高低光照下的检测率
        detectorParams->adaptiveThreshWinSizeMin = 3;
        detectorParams->adaptiveThreshWinSizeMax = 23;
        detectorParams->adaptiveThreshWinSizeStep = 10;
        detectorParams->adaptiveThreshConstant = 7;  // 默认7，低光照可降到5-6
        detectorParams->minMarkerPerimeterRate = 0.03;  // 最小轮廓占比
        detectorParams->polygonalApproxAccuracyRate = 0.03;  // 多边形逼近精度
        
        ROS_INFO("ArUco detector parameters tuned for low-light conditions");
        
        // 订阅相机图像
        subImage = nh.subscribe("/camera/color/image_raw", 1, &QRDetection::imageHandler, this);
        subCameraInfo = nh.subscribe("/camera/color/camera_info", 1, &QRDetection::cameraInfoHandler, this);
        
        // 发布检测结果
        pubPose = nh.advertise<geometry_msgs::PoseStamped>("/qr_detection/pose", 1);
        pubImage = nh.advertise<sensor_msgs::Image>("/qr_detection/image", 1);
        
        ROS_INFO("Waiting for camera info...");
    }
    
    void cameraInfoHandler(const sensor_msgs::CameraInfoConstPtr& msg)
    {
        if (cameraInfoReceived) return;
        
        ROS_INFO("Received camera info");
        
        cameraMatrix = Mat::zeros(3, 3, CV_64F);
        cameraMatrix.at<double>(0, 0) = msg->K[0];
        cameraMatrix.at<double>(0, 2) = msg->K[2];
        cameraMatrix.at<double>(1, 1) = msg->K[4];
        cameraMatrix.at<double>(1, 2) = msg->K[5];
        cameraMatrix.at<double>(2, 2) = 1.0;
        
        distCoeffs = Mat::zeros(5, 1, CV_64F);
        for (size_t i = 0; i < 5 && i < msg->D.size(); i++) {
            distCoeffs.at<double>(i, 0) = msg->D[i];
        }
        
        cameraInfoReceived = true;
        ROS_INFO("Camera info: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                 msg->K[0], msg->K[4], msg->K[2], msg->K[5]);
    }
    
    void imageHandler(const sensor_msgs::ImageConstPtr& msg)
    {
        if (!cameraInfoReceived) {
            ROS_WARN_THROTTLE(5.0, "Waiting for camera info...");
            return;
        }
        
        static int frame_count = 0;
        frame_count++;
        
        if (frame_count % 30 == 0) {
            ROS_INFO("Processing frame %d", frame_count);
        }
        
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        Mat frame = cv_ptr->image;
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        
        vector<vector<Point2f>> corners;
        vector<int> ids;
        aruco::detectMarkers(gray, dictionary, corners, ids, detectorParams);
        
        if (frame_count % 30 == 0) {
            ROS_INFO("Frame %d: detected %zu markers", frame_count, ids.size());
        }
        
        if (!ids.empty()) {
            vector<Vec3d> rvecs, tvecs;
            aruco::estimatePoseSingleMarkers(corners, markerSize, cameraMatrix, distCoeffs, rvecs, tvecs);
            
            for (size_t i = 0; i < ids.size(); i++) {
                // ============= 动态可信度评估 =============
                double distance = norm(tvecs[i]);
                
                // 1. 距离评分（越远越不可靠）
                double distanceScore = min(1.0, max(0.0, 1.0 - (distance - 1.0) / 4.0));  // 1m内1.0，5m外0.0
                
                // 2. 图像质量评分（基于对比度）
                Rect qrROI = boundingRect(corners[i]);
                Mat qrRegion = gray(qrROI);
                Scalar meanVal, stdDev;
                meanStdDev(qrRegion, meanVal, stdDev);
                double contrastScore = min(1.0, max(0.0, stdDev.val[0] / 50.0));  // 对比度越高越可靠
                
                // 3. 视角评分（正对最可靠）- 旋转向量模可能异常，暂时禁用
                // double viewAngle = norm(rvecs[i]) * 180.0 / CV_PI;  // 转换为角度
                // double viewScore = min(1.0, max(0.0, 1.0 - viewAngle / 45.0));  // 45°外为0
                
                // 综合可信度（0-1）- 去掉viewScore
                double confidence = distanceScore * contrastScore;
                
                // 调试：打印各个评分
                ROS_INFO_THROTTLE(1.0, "QR ID %d: dist=%.2fm, distScore=%.2f, contrastScore=%.2f, confidence=%.2f", 
                                  ids[i], distance, distanceScore, contrastScore, confidence);
                // ===========================================
                
                // 仅发布高可信度的检测结果（阈值从0.3降到0.1）
                if (confidence > 0.5) {
                    geometry_msgs::PoseStamped poseMsg;
                    poseMsg.header = msg->header;
                    poseMsg.header.frame_id = "camera_color_optical_frame";
                    
                    poseMsg.pose.position.x = tvecs[i][0];
                    poseMsg.pose.position.y = tvecs[i][1];
                    poseMsg.pose.position.z = tvecs[i][2];
                    
                    // 将可信度存储在 orientation.w 字段（四元数不用）
                    poseMsg.pose.orientation.w = confidence;
                    
                    pubPose.publish(poseMsg);
                    
                    double distance = norm(tvecs[i]);
                    ROS_INFO_THROTTLE(1.0, "QR ID %d detected: distance=%.2f m, confidence=%.2f", ids[i], distance, confidence);
                }
                
                if (publishImage) {
                    aruco::drawDetectedMarkers(frame, corners, ids);
                    drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
                    
                    string text = "ID:" + to_string(ids[i]) + " Dist:" + to_string(int(distance*100)) + "cm";
                    putText(frame, text, Point(corners[i][0].x, corners[i][0].y - 10),
                            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
                }
            }
        }
        
        if (publishImage) {
            sensor_msgs::ImagePtr outMsg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
            pubImage.publish(outMsg);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam_qrDetection");
    
    QRDetection qrDetection;
    
    ROS_INFO("\033[1;32m----> QR Detection Started.\033[0m");
    
    ros::spin();
    
    return 0;
}
