cmake_minimum_required(VERSION 3.10)
project(MvCameraDemo)

set(CMAKE_CXX_STANDARD 17)

# 海康MVS SDK头文件和库路径
include_directories(/opt/MVS/include)
link_directories(/opt/MVS/lib/64)

# 加载OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 指定：用main.cpp生成名叫main的可执行程序
add_executable(main main.cpp)

# 链接海康相机库 + OpenCV库
target_link_libraries(main MvCameraControl ${OpenCV_LIBS})
