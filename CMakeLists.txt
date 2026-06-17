set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_minimum_required(VERSION 3.28)

project(VisionSystem LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

add_executable(vision_system)

target_sources(vision_system PRIVATE
    src/main.cpp
    src/network.cpp
    src/pose_estimation.cpp
    src/video_stream.cpp
)

target_include_directories(vision_system PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
