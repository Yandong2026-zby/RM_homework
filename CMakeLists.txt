cmake_minimum_required(VERSION 3.10)
project(MvCameraDemo)

set(CMAKE_CXX_STANDARD 17)

# 海康MVS SDK头文件和库路径
include_directories(/opt/MVS/include)
link_directories(/opt/MVS/lib/64)

# 加载OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# main: 离线图片上的装甲板识别 + PnP 调试
add_executable(main main.cpp)
target_link_libraries(main MvCameraControl ${OpenCV_LIBS})

# grab: 接工业相机实时取流，用于标定和后续在线识别
add_executable(grab camera_grab.cpp)
target_link_libraries(grab MvCameraControl ${OpenCV_LIBS})

# capture: 实时识别装甲板并拍图，攒标定/训练用的数据集
add_executable(capture capture_armor.cpp)
target_link_libraries(capture MvCameraControl ${OpenCV_LIBS})
add_executable(calibration calibration.cpp)
target_link_libraries(calibration ${OpenCV_LIBS})