# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.5)
# Set the project name
project(camera-imu)

# set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(realsense2 REQUIRED)

# Include OpenCV headers
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    /usr/include
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# Add the source files
set(RGBCAMERA_SOURCES utils/test_camera.cpp src/RGBCamera.cpp)

set(STEREOCAMERA_SOURCES utils/test_stereo.cpp src/StereoCamera.cpp)

# Create the executables
add_library(rgbcamera SHARED ${RGBCAMERA_SOURCES})
add_library(stereocamera SHARED ${STEREOCAMERA_SOURCES})


# Link the OpenCV libraries to your executables
target_link_libraries(rgbcamera ${OpenCV_LIBS})
target_link_libraries(stereocamera ${OpenCV_LIBS} ${realsense2_LIBRARY})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

set(CAMERAIMU_SOURCES src/CameraIMU_main.cpp src/CameraIMU_node.cpp)

add_executable(CameraIMUNode ${CAMERAIMU_SOURCES})

ament_target_dependencies(CameraIMUNode rclcpp sensor_msgs std_msgs cv_bridge builtin_interfaces)

target_link_libraries(CameraIMUNode rgbcamera ${OpenCV_LIBS} MPU6050 stereocamera)

install(TARGETS CameraIMUNode DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS py_src/imu_node.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/)

ament_package()