cmake_minimum_required(VERSION 3.0.0)

# 项目信息
project(Tianmou_SLAM VERSION 0.1.0)

# 查找 ROS 依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# C++ 标准设置
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Build type 设置
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

# 编译选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -std=c++14")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# 查找 OpenCV
find_package(OpenCV 4.6 REQUIRED)
find_package(PCL REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV >= 4.2 not found.")
endif()

message("OPENCV VERSION:")
message(${OpenCV_VERSION})

# 查找 Eigen3 和 Pangolin
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# RealSense SDK 支持
find_package(realsense2)
message(STATUS "TIANMOUC_SDK_DIR: $ENV{TIANMOUC_SDK_DIR}")

# catkin 包配置
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS OpenCV Eigen3 Pangolin realsense2
)

# 包含路径
include_directories(
    ${catkin_INCLUDE_DIRS}  # 添加 ROS 的头文件目录
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/CameraModels
    ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    $ENV{TIANMOUC_SDK_DIR}/usb/inc
    $ENV{TIANMOUC_SDK_DIR}/usb/api
    $ENV{TIANMOUC_SDK_DIR}/inc
)

add_definitions(${PCL_DEFINITIONS})

add_compile_definitions(USB_SDK)

# 外部库路径
link_directories("cyusb/lib")

# 编译项目的主库
add_library(${PROJECT_NAME} SHARED
    src/System.cc
    src/Tracking.cc
    src/LocalMapping.cc
    src/LoopClosing.cc
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/FrameDrawer.cc
    src/Converter.cc
    src/MapPoint.cc
    src/KeyFrame.cc
    src/Atlas.cc
    src/Map.cc
    src/MapDrawer.cc
    src/Optimizer.cc
    src/Frame.cc
    src/KeyFrameDatabase.cc
    src/Sim3Solver.cc
    src/Viewer.cc
    src/ImuTypes.cc
    src/G2oTypes.cc
    src/CameraModels/Pinhole.cpp
    src/CameraModels/KannalaBrandt8.cpp
    src/OptimizableTypes.cpp
    src/MLPnPsolver.cpp
    src/GeometricTools.cc
    src/TwoViewReconstruction.cc
    src/Config.cc
    src/Settings.cc
    src/camera_data_server.cpp
    src/dual_camera.cpp
    include/System.h
    include/Tracking.h
    include/LocalMapping.h
    include/LoopClosing.h
    include/ORBextractor.h
    include/ORBmatcher.h
    include/FrameDrawer.h
    include/Converter.h
    include/MapPoint.h
    include/KeyFrame.h
    include/Atlas.h
    include/Map.h
    include/MapDrawer.h
    include/Optimizer.h
    include/Frame.h
    include/KeyFrameDatabase.h
    include/Sim3Solver.h
    include/Viewer.h
    include/ImuTypes.h
    include/G2oTypes.h
    include/CameraModels/GeometricCamera.h
    include/CameraModels/Pinhole.h
    include/CameraModels/KannalaBrandt8.h
    include/OptimizableTypes.h
    include/MLPnPsolver.h
    include/GeometricTools.h
    include/TwoViewReconstruction.h
    include/SerializationUtils.h
    include/Config.h
    include/Settings.h
    include/camera_data_server.hpp
    include/dual_camera.hpp
)

# 链接库文件
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}  # ROS 依赖库
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${PCL_LIBRARY_DIRS}
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
    -lboost_serialization
    -lcrypto
)

# RealSense SDK 支持
if(realsense2_FOUND)
    include_directories(${realsense_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
endif()

# 编译 g2o 库
add_subdirectory(Thirdparty/g2o)

# 设置输出路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Stereo)

# 添加主程序 stereo_tianmou
add_executable(stereo_tianmou
    Examples_old/Stereo/stereo_tianmou.cc
    $ENV{TIANMOUC_SDK_DIR}/src/tmc_proc.cpp
    $ENV{TIANMOUC_SDK_DIR}/usb/src/new_dataRecevier.cpp
    $ENV{TIANMOUC_SDK_DIR}/usb/src/camera.cpp
    $ENV{TIANMOUC_SDK_DIR}/usb/src/lyn_cam.cpp
    $ENV{TIANMOUC_SDK_DIR}/usb/src/tools.cpp
    $ENV{TIANMOUC_SDK_DIR}/src/isp.cpp
)

# 链接主程序所需库
target_link_libraries(stereo_tianmou
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}  # ROS 依赖库
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${PCL_LIBRARIES}
    libcyusb.so
    pthread
    -lusb-1.0
)

