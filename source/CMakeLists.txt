#输入最低版本要求，一般是这个，反正我没改
cmake_minimum_required(VERSION 2.8)

#C11标准，现在一般都用这个
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

#输入项目名字，也就是文件夹的名字
project(myrm)

#找到opencv包，改成自己的版本就行，注意大小写为OpenCV
find_package(OpenCV REQUIRED)
find_package(Threads)

#Release模式编译更快，也可以使用Debug模式试一下
if(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
endif()

#添加头文件搜索路径
include_directories(${OpenCV_INCLUDE_DIRS} ../header)

#第一个参数是项目文件夹名字，第二个参数是要跑的.cpp文件
add_executable( myrm main.cpp ArmorDetector.cpp SivirDetector.cpp)

#链接库，第一个参数是项目文件夹名字，第二个参数是链接的库，我照抄网上的，我也不知为啥是这个库
target_link_libraries( myrm ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries( myrm /usr/lib/libgxiapi.so)
target_link_libraries( myrm /usr/lib/libdximageproc.so)
