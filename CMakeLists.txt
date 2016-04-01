cmake_minimum_required(VERSION 2.8)

project(rgbd-odometry)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(OpenCV REQUIRED)

file(GLOB TARGET_SRC
    src/*.cpp
)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(rgbd-odometry ${TARGET_SRC})
target_link_libraries (rgbd-odometry ${OpenCV_LIBS})
