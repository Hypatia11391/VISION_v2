#set(CMAKE_CXX_EXTENSIONS OFF)

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
    ntcore
    wpinet
    wpiutil
)

target_compile_options(VisionSystem PRIVATE 
    $<$<CONFIG:Debug>:-Og -Wall>
)

target_sources(VisionSystem PRIVATE)

set(NATIVE "$ENV{VIRTUAL_ENV}/lib/python3.13/site-packages/native")

target_include_directories(VisionSystem PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    "${NATIVE}/ntcore/include"
    "${NATIVE}/wpiutil/include"
    "${NATIVE}/wpinet/include"
    #"${NATIVE}/wpimath/include"
    #"${NATIVE}/wpilib/include"
    #"${NATIVE}/wpihal/include"
)

# Add Library Search Paths (-L)
target_link_directories(VisionSystem PRIVATE
    "${NATIVE}/ntcore/lib"
    "${NATIVE}/wpinet/lib"
    "${NATIVE}/wpiutil/lib"
)

# Handle RPATHs (-Wl,-rpath,...) so the executable finds the shared libs at runtime
set_target_properties(VisionSystem PROPERTIES
    BUILD_RPATH "${NATIVE}/ntcore/lib;${NATIVE}/wpinet/lib;${NATIVE}/wpiutil/lib"
    INSTALL_RPATH "${NATIVE}/ntcore/lib;${NATIVE}/wpinet/lib;${NATIVE}/wpiutil/lib"
)
