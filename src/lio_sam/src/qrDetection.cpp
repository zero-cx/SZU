#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>

#include <lio_sam/qr_detection.h>

#include <algorithm>
#include <cctype>
#include <string>
#include <unordered_map>

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
    ros::Publisher pubDetection;
    ros::Publisher pubImage;

    Ptr<aruco::Dictionary> dictionary;
    Ptr<aruco::DetectorParameters> detectorParams;

    Mat cameraMatrix;
    Mat distCoeffs;
    bool cameraInfoReceived;

    string imageTopic;
    string cameraInfoTopic;
    string poseTopic;
    string detectionTopic;
    string debugImageTopic;
    string cameraFrame;
    string dictionaryName;

    double markerSize;
    bool publishImage;
    double minConfidence;
    int adaptiveThreshWinSizeMin;
    int adaptiveThreshWinSizeMax;
    int adaptiveThreshWinSizeStep;
    double adaptiveThreshConstant;
    double minMarkerPerimeterRate;
    double polygonalApproxAccuracyRate;
    double debugAxisLength;

    string toUpper(string s)
    {
        transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        return s;
    }

    Ptr<aruco::Dictionary> createDictionaryFromName(const string& name)
    {
        static const unordered_map<string, aruco::PREDEFINED_DICTIONARY_NAME> kDictMap = {
            {"DICT_4X4_50", aruco::DICT_4X4_50},
            {"DICT_4X4_100", aruco::DICT_4X4_100},
            {"DICT_4X4_250", aruco::DICT_4X4_250},
            {"DICT_4X4_1000", aruco::DICT_4X4_1000},
            {"DICT_5X5_50", aruco::DICT_5X5_50},
            {"DICT_5X5_100", aruco::DICT_5X5_100},
            {"DICT_5X5_250", aruco::DICT_5X5_250},
            {"DICT_5X5_1000", aruco::DICT_5X5_1000},
            {"DICT_6X6_50", aruco::DICT_6X6_50},
            {"DICT_6X6_100", aruco::DICT_6X6_100},
            {"DICT_6X6_250", aruco::DICT_6X6_250},
            {"DICT_6X6_1000", aruco::DICT_6X6_1000},
            {"DICT_7X7_50", aruco::DICT_7X7_50},
            {"DICT_7X7_100", aruco::DICT_7X7_100},
            {"DICT_7X7_250", aruco::DICT_7X7_250},
            {"DICT_7X7_1000", aruco::DICT_7X7_1000},
            {"DICT_ARUCO_ORIGINAL", aruco::DICT_ARUCO_ORIGINAL},
            {"DICT_APRILTAG_16H5", aruco::DICT_APRILTAG_16h5},
            {"DICT_APRILTAG_25H9", aruco::DICT_APRILTAG_25h9},
            {"DICT_APRILTAG_36H10", aruco::DICT_APRILTAG_36h10},
            {"DICT_APRILTAG_36H11", aruco::DICT_APRILTAG_36h11}
        };

        const string key = toUpper(name);
        const auto it = kDictMap.find(key);
        if (it == kDictMap.end()) {
            ROS_WARN("Unknown dictionary_name '%s', fallback to DICT_6X6_250", name.c_str());
            return aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
        }

        return aruco::getPredefinedDictionary(it->second);
    }

public:
    QRDetection() : pnh("~"), cameraInfoReceived(false)
    {
        pnh.param<string>("image_topic", imageTopic, "/camera/color/image_raw");
        pnh.param<string>("camera_info_topic", cameraInfoTopic, "/camera/color/camera_info");
        pnh.param<string>("pose_topic", poseTopic, "/qr_detection/pose");
        pnh.param<string>("detection_topic", detectionTopic, "/qr_detection/detection");
        pnh.param<string>("debug_image_topic", debugImageTopic, "/qr_detection/image");
        pnh.param<string>("camera_frame", cameraFrame, "camera_color_optical_frame");
        pnh.param<string>("dictionary_name", dictionaryName, "DICT_6X6_250");

        pnh.param<double>("marker_size", markerSize, 0.14);
        pnh.param<bool>("publish_image", publishImage, true);
        pnh.param<double>("confidence_threshold", minConfidence, 0.5);

        pnh.param<int>("adaptive_thresh_win_size_min", adaptiveThreshWinSizeMin, 3);
        pnh.param<int>("adaptive_thresh_win_size_max", adaptiveThreshWinSizeMax, 23);
        pnh.param<int>("adaptive_thresh_win_size_step", adaptiveThreshWinSizeStep, 10);
        pnh.param<double>("adaptive_thresh_constant", adaptiveThreshConstant, 7.0);
        pnh.param<double>("min_marker_perimeter_rate", minMarkerPerimeterRate, 0.03);
        pnh.param<double>("polygonal_approx_accuracy_rate", polygonalApproxAccuracyRate, 0.03);
        pnh.param<double>("debug_axis_length", debugAxisLength, 0.1);

        ROS_INFO("QR Detection Node Started");
        ROS_INFO("image_topic: %s", imageTopic.c_str());
        ROS_INFO("camera_info_topic: %s", cameraInfoTopic.c_str());
        ROS_INFO("pose_topic: %s", poseTopic.c_str());
        ROS_INFO("detection_topic: %s", detectionTopic.c_str());
        ROS_INFO("debug_image_topic: %s", debugImageTopic.c_str());
        ROS_INFO("camera_frame: %s", cameraFrame.c_str());
        ROS_INFO("dictionary_name: %s", dictionaryName.c_str());
        ROS_INFO("marker_size: %.3f m, confidence_threshold: %.2f", markerSize, minConfidence);

        dictionary = createDictionaryFromName(dictionaryName);
        detectorParams = aruco::DetectorParameters::create();
        detectorParams->adaptiveThreshWinSizeMin = adaptiveThreshWinSizeMin;
        detectorParams->adaptiveThreshWinSizeMax = adaptiveThreshWinSizeMax;
        detectorParams->adaptiveThreshWinSizeStep = adaptiveThreshWinSizeStep;
        detectorParams->adaptiveThreshConstant = adaptiveThreshConstant;
        detectorParams->minMarkerPerimeterRate = minMarkerPerimeterRate;
        detectorParams->polygonalApproxAccuracyRate = polygonalApproxAccuracyRate;

        subImage = nh.subscribe(imageTopic, 1, &QRDetection::imageHandler, this);
        subCameraInfo = nh.subscribe(cameraInfoTopic, 1, &QRDetection::cameraInfoHandler, this);

        pubPose = nh.advertise<geometry_msgs::PoseStamped>(poseTopic, 10);
        pubDetection = nh.advertise<lio_sam::qr_detection>(detectionTopic, 10);
        pubImage = nh.advertise<sensor_msgs::Image>(debugImageTopic, 1);

        ROS_INFO("Waiting for camera info from %s ...", cameraInfoTopic.c_str());
    }

    void cameraInfoHandler(const sensor_msgs::CameraInfoConstPtr& msg)
    {
        if (cameraInfoReceived) return;

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
        ROS_INFO("Camera info ready: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", msg->K[0], msg->K[4], msg->K[2], msg->K[5]);
    }

    void imageHandler(const sensor_msgs::ImageConstPtr& msg)
    {
        if (!cameraInfoReceived) {
            ROS_WARN_THROTTLE(5.0, "No camera_info yet on %s", cameraInfoTopic.c_str());
            return;
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

        if (ids.empty()) {
            ROS_WARN_THROTTLE(2.0, "No marker detected from %s", imageTopic.c_str());
        }

        vector<Vec3d> rvecs, tvecs;
        if (!ids.empty()) {
            aruco::estimatePoseSingleMarkers(corners, markerSize, cameraMatrix, distCoeffs, rvecs, tvecs);
            if (publishImage) {
                aruco::drawDetectedMarkers(frame, corners, ids);
            }

            for (size_t i = 0; i < ids.size(); i++) {
                const double distance = norm(tvecs[i]);
                const double distanceScore = min(1.0, max(0.0, 1.0 - (distance - 1.0) / 4.0));

                Rect qrROI = boundingRect(corners[i]);
                qrROI &= Rect(0, 0, gray.cols, gray.rows);
                if (qrROI.width <= 0 || qrROI.height <= 0) {
                    continue;
                }

                Mat qrRegion = gray(qrROI);
                Scalar meanVal, stdDev;
                meanStdDev(qrRegion, meanVal, stdDev);
                const double contrastScore = min(1.0, max(0.0, stdDev.val[0] / 50.0));
                const double confidence = distanceScore * contrastScore;

                if (confidence < minConfidence) {
                    ROS_WARN_THROTTLE(2.0, "Reject marker id=%d, confidence=%.2f < threshold=%.2f", ids[i], confidence, minConfidence);
                    continue;
                }

                cv::Mat rotMat;
                cv::Rodrigues(rvecs[i], rotMat);
                tf::Matrix3x3 tfRot(
                    rotMat.at<double>(0, 0), rotMat.at<double>(0, 1), rotMat.at<double>(0, 2),
                    rotMat.at<double>(1, 0), rotMat.at<double>(1, 1), rotMat.at<double>(1, 2),
                    rotMat.at<double>(2, 0), rotMat.at<double>(2, 1), rotMat.at<double>(2, 2));
                tf::Quaternion q;
                tfRot.getRotation(q);

                geometry_msgs::PoseStamped poseMsg;
                poseMsg.header = msg->header;
                poseMsg.header.frame_id = cameraFrame;
                poseMsg.pose.position.x = tvecs[i][0];
                poseMsg.pose.position.y = tvecs[i][1];
                poseMsg.pose.position.z = tvecs[i][2];
                poseMsg.pose.orientation.x = q.x();
                poseMsg.pose.orientation.y = q.y();
                poseMsg.pose.orientation.z = q.z();
                poseMsg.pose.orientation.w = confidence;
                pubPose.publish(poseMsg);

                lio_sam::qr_detection detMsg;
                detMsg.header = poseMsg.header;
                detMsg.marker_id = ids[i];
                detMsg.pose.position = poseMsg.pose.position;
                detMsg.pose.orientation.x = q.x();
                detMsg.pose.orientation.y = q.y();
                detMsg.pose.orientation.z = q.z();
                detMsg.pose.orientation.w = q.w();
                detMsg.confidence = confidence;
                detMsg.distance = distance;
                pubDetection.publish(detMsg);

                ROS_INFO_THROTTLE(1.0, "Publish marker id=%d dist=%.2fm confidence=%.2f", ids[i], distance, confidence);

                if (publishImage) {
                    drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], debugAxisLength);
                    const string text = "ID:" + to_string(ids[i]) + " Dist:" + to_string(static_cast<int>(distance * 100)) + "cm";
                    putText(frame, text, Point(corners[i][0].x, corners[i][0].y - 10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
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
