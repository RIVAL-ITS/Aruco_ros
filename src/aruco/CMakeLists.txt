cmake_minimum_required(VERSION 3.8)
project(aruco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(OpenCV REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui aruco)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

ament_export_dependencies(rclcpp OpenCV tf2_ros tf2 geometry_msgs tf2_geometry_msgs)

add_executable(aruco_detect src/aruco_detect.cpp)
target_include_directories(aruco_detect PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_compile_features(aruco_detect PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(aruco_detect ${OpenCV_LIBS})

ament_target_dependencies(aruco_detect 
  "rclcpp" 
  "OpenCV"
  "tf2_ros"
  "tf2"
  "geometry_msgs"
  "tf2_geometry_msgs"
)


install(TARGETS 
  aruco_detect
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
