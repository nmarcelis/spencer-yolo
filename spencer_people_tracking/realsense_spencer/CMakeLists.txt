cmake_minimum_required(VERSION 2.8.3)
project(realsense_spencer)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )
