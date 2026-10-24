cmake_minimum_required(VERSION 3.28)
project(VisionSystem LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to an optimized build if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# WPILib is found via -DCMAKE_PREFIX_PATH=$HOME/wpilib-install
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(ntcore REQUIRED)   # pulls in wpinet and wpiutil transitively

add_executable(VisionSystem
    src/main.cpp
    src/network.cpp
    src/pose_estimation.cpp
    src/video_stream.cpp
)

#target_include_directories(VisionSystem PRIVATE
#    ${CMAKE_CURRENT_SOURCE_DIR}/src
#)

target_link_libraries(VisionSystem PRIVATE
    ${OpenCV_LIBS}
    apriltag
    Eigen3::Eigen
    ntcore
)

target_compile_options(VisionSystem PRIVATE
    $<$<CONFIG:Debug>:-Og -Wall>
)