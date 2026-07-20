set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_minimum_required(VERSION 3.28)

project(VisionSystem LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

add_executable(
    VisionSystem
    src/main.cpp
    src/network.cpp
    src/pose_estimation.cpp
    src/video_stream.cpp
)

target_link_libraries(VisionSystem PRIVATE 
    ${OpenCV_LIBS} 
    apriltag 
    Eigen3::Eigen
)

target_compile_options(VisionSystem PRIVATE 
    $<$<CONFIG:Debug>:-Og -Wall>
)

target_sources(VisionSystem PRIVATE)

target_include_directories(VisionSystem PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
