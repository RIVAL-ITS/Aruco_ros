#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "rclcpp/rclcpp.hpp"

using namespace std;
using namespace cv;

class ArucoDetectNode : public rclcpp::Node {
public:
    ArucoDetectNode() : Node("aruco_detect_node") {
        // Buka kamera
        cap.open("/dev/v4l/by-id/usb-BC-231018-A_XWF_1080P_PC_Camera-video-index0", cv::CAP_V4L2);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Tidak dapat membuka kamera!");
            return;
        }

        // Load kalibrasi kamera
        fs = FileStorage("/home/ichbinwil/Documents/Aruco_ros/calibration.yaml", FileStorage::READ);
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
                    cout << "Pose (tvec) marker ID " << markerIds[i]
                         << ": [" << tvec[0] << ", " << tvec[1] << ", " << tvec[2] << "]" << endl;
                } else {
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
