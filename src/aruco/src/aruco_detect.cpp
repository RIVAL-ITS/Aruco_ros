#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std;
using namespace cv;

class ArucoDetectNode : public rclcpp::Node {
public:
    ArucoDetectNode() : Node("aruco_detect_node") {
        //declare parameter
        this->declare_parameter("marker_size_cm",5.0);
        this->declare_parameter("camera_frame","camera_link");
        this->declare_parameter("tf_to", "base_link");
        this->declare_parameter("path_cam", "/dev/v4l/by-id/usb-BC-231018-A_XWF_1080P_PC_Camera-video-index0");

        //get param
        marker_size_cm = this->get_parameter("marker_size_cm").as_double();
        camera_frame = this->get_parameter("camera_frame").as_string();
        tf_to = this->get_parameter("tf_to").as_string();
        path_cam = this->get_parameter("path_cam").as_string();

        //Debug
        RCLCPP_INFO(this->get_logger(),"marker_size_cm: %f", marker_size_cm);
        RCLCPP_INFO(this->get_logger(),"camera_frame: %s", camera_frame.c_str());
        RCLCPP_INFO(this->get_logger(),"tf_to: %s", tf_to.c_str());
        RCLCPP_INFO(this->get_logger(),"path_cam: %s", path_cam.c_str());

        // --- Kamera ---
        cap.open(path_cam, cv::CAP_V4L2);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Tidak dapat membuka kamera!");
            return;
        }

        // --- Kalibrasi ---
        fs = FileStorage("calibration.yaml", FileStorage::READ);
        if (!fs.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Gagal membuka file kalibrasi!");
            return;
        }

        fs["cameraMatrix"] >> cameraMatrix;
        fs["distCoeffs"] >> distCoeffs;
        fs.release();

        if (cameraMatrix.empty() || distCoeffs.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter kalibrasi kosong!");
            return;
        }

        // --- ArUco Dictionary ---
        dictionary = aruco::getPredefinedDictionary(aruco::DICT_5X5_100);
        markerLength = marker_size_cm; // cm

        float m = markerLength / 2.0f;
        objectPoints = {
            {-m,  m, 0},
            { m,  m, 0},
            { m, -m, 0},
            {-m, -m, 0}
        };

        // --- TF Setup ---
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // --- Timer ---
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&ArucoDetectNode::image_callback, this)
        );
    }

    ~ArucoDetectNode() {
        cap.release();
        destroyAllWindows();
    }

private:
    VideoCapture cap;
    FileStorage fs;
    Mat cameraMatrix, distCoeffs;
    Ptr<aruco::Dictionary> dictionary;
    vector<Point3f> objectPoints;
    float markerLength;
    rclcpp::TimerBase::SharedPtr timer_;

    //parameter variabel
    float marker_size_cm = 5.0;
    std::string camera_frame = "camera_link";
    std::string tf_to = "base_link";
    std::string path_cam = "/dev/v4l/by-id/usb-BC-231018-A_XWF_1080P_PC_Camera-video-index0";

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void image_callback() {
        Mat frame;
        cap >> frame;
        if (frame.empty()) return;

        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners;
        aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);

        if (!markerIds.empty()) {
            aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

            for (size_t i = 0; i < markerIds.size(); ++i) {
                Vec3d rvec, tvec;
                bool success = solvePnP(objectPoints, markerCorners[i], cameraMatrix, distCoeffs, rvec, tvec);

                if (success) {
                    // --- Broadcast TF camera_link -> aruco_marker ---
                    cv::Mat rotationMatrix;
                    cv::Rodrigues(rvec, rotationMatrix);

                    tf2::Matrix3x3 tf_rot(
                        rotationMatrix.at<double>(0,0), rotationMatrix.at<double>(0,1), rotationMatrix.at<double>(0,2),
                        rotationMatrix.at<double>(1,0), rotationMatrix.at<double>(1,1), rotationMatrix.at<double>(1,2),
                        rotationMatrix.at<double>(2,0), rotationMatrix.at<double>(2,1), rotationMatrix.at<double>(2,2)
                    );

                    tf2::Quaternion q;
                    tf_rot.getRotation(q);

                    geometry_msgs::msg::TransformStamped t;
                    t.header.stamp = this->get_clock()->now();
                    t.header.frame_id = camera_frame;
                    t.child_frame_id = "aruco_marker_" + std::to_string(markerIds[i]);
                    t.transform.translation.x = tvec[0] / 100.0;
                    t.transform.translation.y = tvec[1] / 100.0;
                    t.transform.translation.z = tvec[2] / 100.0;
                    t.transform.rotation.x = q.x();
                    t.transform.rotation.y = q.y();
                    t.transform.rotation.z = q.z();
                    t.transform.rotation.w = q.w();
                    tf_broadcaster_->sendTransform(t);

                    // --- Dapatkan transform base_link -> aruco_marker ---
                    try {
                        auto transform = tf_buffer_->lookupTransform(
                            tf_to,
                            t.child_frame_id,
                            tf2::TimePointZero
                        );

                        RCLCPP_INFO(this->get_logger(),
                            "Marker %d wrt %s: (%.2f, %.2f, %.2f)",
                            markerIds[i],
                            tf_to.c_str(),
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z
                        );
                    } catch (tf2::TransformException &ex) {
                        RCLCPP_WARN(this->get_logger(), "Transform gagal: %s", ex.what());
                    }

                    drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, markerLength);
                }
            }
        }
        imshow("Aruco Detection", frame);
        waitKey(1);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoDetectNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
