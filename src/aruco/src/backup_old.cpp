#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std;
using namespace cv;

class ArucoDetectNode : public rclcpp::Node {
public:
    ArucoDetectNode() : Node("aruco_detect_node") {

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    
        // Buka kamera
        cap.open("/dev/v4l/by-id/usb-BC-231018-A_XWF_1080P_PC_Camera-video-index0", cv::CAP_V4L2);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Tidak dapat membuka kamera!");
            return;
        }

        // Load kalibrasi kamera
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

        // Inisialisasi dictionary ArUco
        dictionary = aruco::getPredefinedDictionary(aruco::DICT_5X5_100);

        // Panjang sisi marker (dalam cm atau meter, sesuaikan dengan objectPoints)
        markerLength = 5.3f;

        // Object points (urutan: top-left, top-right, bottom-right, bottom-left)
        float m = markerLength / 2.0f;
        objectPoints = {
            {-m,  m, 0},
            { m,  m, 0},
            { m, -m, 0},
            {-m, -m, 0}
        };

        // Buat timer callback
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
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void image_callback() {
        Mat frame;
        cap >> frame;

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Frame kosong dari kamera!");
            return;
        }

        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners;
        aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);

        if (!markerIds.empty()) {
            aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

            for (size_t i = 0; i < markerIds.size(); ++i) {
                // Hitung pusat marker
                Point2f center = (markerCorners[i][0] + markerCorners[i][1] +
                                  markerCorners[i][2] + markerCorners[i][3]) / 4;
                cout << "Marker ID: " << markerIds[i] << " | Center: " << center << endl;

                // Estimasi pose dengan solvePnP
                Vec3d rvec, tvec;
                bool success = solvePnP(objectPoints, markerCorners[i], cameraMatrix, distCoeffs, rvec, tvec);

                if (success) {
                    drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, markerLength);

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
                    t.header.frame_id = "camera_link";
                    t.child_frame_id = "aruco_marker_" + std::to_string(markerIds[i]);

                    t.transform.translation.x = tvec[0];
                    t.transform.translation.y = tvec[1];
                    t.transform.translation.z = tvec[2];
                    t.transform.rotation.x = q.x();
                    t.transform.rotation.y = q.y();
                    t.transform.rotation.z = q.z();
                    t.transform.rotation.w = q.w();

                    tf_broadcaster_->sendTransform(t);
                }
                else {
                    RCLCPP_WARN(this->get_logger(), "Gagal solvePnP untuk marker ID %d", markerIds[i]);
                }
            }
        } else {
            cout << "Tidak ada marker terdeteksi." << endl;
        }

        imshow("ArUco Marker Detection", frame);
        waitKey(1); // untuk update window
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoDetectNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
