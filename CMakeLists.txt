cmake_minimum_required(VERSION 3.8)
project(StereoDepthSensor)
find_package(OpenCV REQUIRED)

link_directories(/usr/local/lib/ )

set(CMAKE_CXX_STANDARD 11)

include_directories(include)

set(SOURCE_FILES
        src/CalibrateCamera.cpp
        src/CalibrationBoard.cpp
        src/Camera.cpp
        src/CamSys.cpp
        src/StereoSystem.cpp
        src/ToolBox.cpp src/ShowImgGui.cpp include/ShowImgGui.h)

add_executable(StereoDepthSensor ${SOURCE_FILES})
target_link_libraries (StereoDepthSensor MVSDK  ${OpenCV_LIBS} python2.7)
