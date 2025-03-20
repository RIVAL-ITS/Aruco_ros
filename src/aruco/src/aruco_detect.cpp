#include <iostream>
#include <vector>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include "rclcpp/rclcpp.hpp"

using namespace std;
using namespace cv;

//TODO Variabel PnP
/*std::vector<cv::Point3f> objectPoints = {
    {-r, -r, 0},   // Kiri atas (dibalik)
    {r, -r, 0},    // Kanan atas (dibalik)
    {r, r, 0},     // Kanan bawah (dibalik)
    {-r, r, 0}     // Kiri bawah (dibalik)
};
*/

vector<cv::Point3f> objectPoints = {
  {-7.5,-7.5,0},
  {7.5,-7.5,0},
  {7.5,7.5,0},
  {-7.5,7.5,0},
};

class ArucoDetectNode : public rclcpp::Node {
  public:
  ArucoDetectNode():Node("aruco_degtec_node"){
    timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&ArucoDetectNode::image_callback, this));


    cap.open("/dev/v4l/by-id/usb-BC-231018-A_XWF_1080P_PC_Camera-video-index0",cv::CAP_V4L2);
    
    if (!cap.isOpened()) {
      std::cerr << "Error: Tidak dapat membuka kamera!\n";
      return;
    }

    fs = cv::FileStorage("/home/ichbinwil/Opencv_ROS/calibration.yaml", cv::FileStorage::READ);


    if (!fs.isOpened()) {
        std::cerr << "Error opening calibration file!" << std::endl;
        return;
    }
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();

    if (cameraMatrix.empty() || distCoeffs.empty()) {
        std::cerr << "Error: Calibration parameters are empty!" << std::endl;
        return;
    }

    // Pilih dictionary ArUco yang sesuai
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);


  }

  ~ArucoDetectNode(){

    // Tutup kamera
    cap.release();
    cv::destroyAllWindows();

  }

  private:

  cv::Mat cameraMatrix, distCoeffs;
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  cv::VideoCapture cap;
  cv::FileStorage fs;
  rclcpp::TimerBase::SharedPtr timer_;


  void image_callback(){
    cv::Mat frame;
    cap >> frame;  // Ambil frame dari kamera
    if (frame.empty()) {
        std::cerr << "Error: Frame kosong!\n";
        return;
    }

    // Deteksi marker ArUco
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);

    // Gambar hasil deteksi
    if (!markerIds.empty()) {
        cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

        cv::Point2f center;

        // Cetak posisi piksel dari setiap marker
        for (size_t i = 0; i < markerIds.size(); ++i) {
            std::cout << "Marker ID: " << markerIds[i] << std::endl;
            for (size_t j = 0; j < markerCorners[i].size(); ++j) {
                // std::cout << "Corner " << j << ": " << markerCorners[i][j] << std::endl;
                center = (markerCorners[i][0] + markerCorners[i][1] + markerCorners[i][2] + markerCorners[i][3]) / 4;
            }
            std::cout << "Center: " << center << endl;
        }

        // Estimasi pose
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 15, cameraMatrix, distCoeffs, rvecs, tvecs);
        for (size_t i = 0; i < markerIds.size(); i++) {
            cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 15);

            float x_cm = tvecs[i][0];
            float y_cm = tvecs[i][1];
            float z_cm = tvecs[i][2];

            std::cout << "Translation vector for marker ID " << markerIds[i] << ": [" << x_cm << ", " << y_cm << ", " << z_cm << "]" << std::endl;
        }

    }else {
        std::cout << "No markers detected." << std::endl;
    }

    // Tampilkan hasil di window
    cv::imshow("ArUco Marker Detection", frame);
    cv::waitKey(1);

  }

};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoDetectNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
