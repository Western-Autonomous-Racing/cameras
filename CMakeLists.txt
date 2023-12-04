# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.5)
# Set the project name
project(camera_imu)

# set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Include OpenCV headers
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    /usr/include
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# Add the source files
set(CAMERA_SOURCES utils/test_camera.cpp src/Camera.cpp)

# Create the executables
add_library(camera SHARED ${CAMERA_SOURCES})

# Link the OpenCV libraries to your executables
target_link_libraries(camera ${OpenCV_LIBS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

set(CAMERAIMU_SOURCES src/CameraIMU_main.cpp src/CameraIMU_node.cpp)

add_executable(CameraIMUNode ${CAMERAIMU_SOURCES})

ament_target_dependencies(CameraIMUNode rclcpp sensor_msgs std_msgs cv_bridge builtin_interfaces)

target_link_libraries(CameraIMUNode camera ${OpenCV_LIBS} MPU6050)

ament_package()