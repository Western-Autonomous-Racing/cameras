# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.0)

# Set the project name
project(camera_imu)

# FILEPATH: /home/war/war-projects/cameras/CMakeLists.txt
# BEGIN: find_opencv
# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include OpenCV headers
include_directories(${OpenCV_INCLUDE_DIRS})
# END: find_opencv

# Add the include directory
include_directories(include)

# Add the source files
set(SOURCES src/main.cpp src/camera.cpp)

# Create the executable
add_executable(camera_imu ${SOURCES})

target_link_libraries(camera_imu ${OpenCV_LIBS})
