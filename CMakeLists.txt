# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.0)

# Set the project name
project(camera_imu)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include OpenCV headers
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(/usr/include)

# Add the include directories
include_directories(include)

# Add the source files
set(CAMERA_SOURCES src/main.cpp src/camera.cpp)

# Create the executables
add_executable(camera ${CAMERA_SOURCES})

# Link the OpenCV libraries to your executables
target_link_libraries(camera ${OpenCV_LIBS})