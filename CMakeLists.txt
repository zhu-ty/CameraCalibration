# CMake entry point.
# Note that this CMakeLists.txt in fully independent from the one in the parent directory
# (since that's the goal of this tutorial !)

# For an introduction to CMake, see
# http://www.cmake.org/cmake/help/cmake_tutorial.html (at least steps 1 and 2)

cmake_minimum_required (VERSION 3.10)
set(name "CameraClibration")
project ( ${name} )

# cmake module
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# c++ version
set (CMAKE_CXX_STANDARD 11)
add_definitions(-D"WIN32_LEAN_AND_MEAN") 
add_definitions(-D"ASIO_STANDALONE")

set (WIN_LIB_PATH "E:/libs")

#opencv 
if ( WIN32 )
	set(OpenCV_DIR ${WIN_LIB_PATH}/OpenCV/x64/vc14/lib)
	set(OpenCV_CONFIG_PATH ${WIN_LIB_PATH}/OpenCV)
	#set(WIN_HEADER_PATH "${PROJECT_SOURCE_DIR}/winheadfile")
    #include_directories(${WIN_HEADER_PATH})
endif()

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
	

set(MY_HEADER_FILES
	SingleCameraCalibration.h
	StereoCameraCalibration.h
	INIReader.h
	SysUtil.hpp
)

set(MY_SOURCE_FILES 
	main.cpp
	SingleCameraCalibration.cpp
	StereoCameraCalibration.cpp
)
include_directories(
	${PROJECT_SOURCE_DIR}	
)

add_executable(${name}
	${MY_HEADER_FILES}
	${MY_SOURCE_FILES}
	main.cpp
	)

target_link_libraries(${name}
	${OpenCV_LIBS}
	)
