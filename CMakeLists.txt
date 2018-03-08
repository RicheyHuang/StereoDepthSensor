cmake_minimum_required(VERSION 3.8)
project(StereoDepthSensor)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

link_directories(/usr/local/lib/ ${PCL_LIBRARY_DIRS})

set(CMAKE_CXX_STANDARD 11)

include_directories(${PCL_INCLUDE_DIRS} include)
add_definitions(${PCL_DEFINITIONS})

set(SOURCE_FILES
        src/CalibrateCamera.cpp
        src/CalibrationBoard.cpp
        src/Camera.cpp
        src/CamSys.cpp
        src/StereoSystem.cpp
        src/ToolBox.cpp src/ShowImgGui.cpp include/ShowImgGui.h)

add_executable(StereoDepthSensor ${SOURCE_FILES})
target_link_libraries (StereoDepthSensor MVSDK  ${OpenCV_LIBS} ${PCL_LIBRARIES} python2.7)


