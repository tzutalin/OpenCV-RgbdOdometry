cmake_minimum_required(VERSION 2.8)

project(rgbd-odometry)

#c++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#opencv
find_package(OpenCV 3.1)
if (OpenCV_FOUND)
	message("Find your opencv succesfully")
else()
	message("Cannot find your opencv, please specifiy your opencv path")
	set(OpenCV_INCLUDE_DIRS /home/darrenl/library/opencv/install/include)
	set(OpenCV_LIBS opencv_core opencv_highgui opencv_imgproc opencv_imgcodecs opencv_calib3d)
	link_directories(/home/darrenl/library/opencv/install/lib)
endif()

file(GLOB TARGET_SRC
    src/*.cpp
    src/opencv_odometry/*.cpp
)

include_directories(${OpenCV_INCLUDE_DIRS} include)

add_executable(rgbd-odometry ${TARGET_SRC})
target_link_libraries (rgbd-odometry ${OpenCV_LIBS})
