#1.cmake verson，指定cmake版本 
# cmake_minimum_required(VERSION 3.2)
cmake_minimum_required(VERSION 2.8)

#2.project name，指定项目的名称，一般和项目的文件夹名称对应
project(ground_segmention)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2")
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        sensor_msgs
        roscpp
        cv_bridge
        tf
        )

#3.head file path，头文件目录
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        )

link_directories(
        ${PCL_LIBRARY_DIRS}
        )

# file(GLOB_RECURSE cuda_srcs src/*.cu)
# cuda_add_library(cucodes SHARED ${cuda_srcs})
# target_link_libraries(cucodes cuda cublas cudart cudnn)

#6.add executable file，添加要编译的可执行文件
add_executable(${PROJECT_NAME} src/main.cpp)

#7.add link library，添加可执行文件所需要的库，比如我们用到了libm.so（命名规则：lib+name+.so），就添加该库的名称
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
