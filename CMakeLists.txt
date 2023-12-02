# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the project name
project(camera_imu)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include OpenCV headers
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    /usr/include
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# Add the source files
set(CAMERA_SOURCES utils/test_camera.cpp src/camera.cpp)

# Create the executables
add_library(camera SHARED ${CAMERA_SOURCES})

# Link the OpenCV libraries to your executables
target_link_libraries(camera ${OpenCV_LIBS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

set(NODE_SOURCES utils/test_node.cpp src/camera_imu_node.cpp)

add_executable(test_node ${NODE_SOURCES})

target_link_libraries(test_node camera ${INCLUDE_DIRECTORIES})

ament_package()