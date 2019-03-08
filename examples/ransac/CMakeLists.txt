cmake_minimum_required(VERSION 2.8)
project(ransac)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 ") # -march=native
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -std=c++11 -g")
if( NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
    message("Build types:" "Release")
endif()

# 设置查找Findxxx.cmake文件的路径
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package(GLib REQUIRED )      # GLib
if(NOT GLib_FOUND)
    # set(GLib_DIR "/home/gcj/GitHub/usr/lib/cmake/DLib/")
    #find_package(GLib REQUIRED)
    message(FATAL_ERROR "GLib is not found!")
endif(NOT GLib_FOUND)

find_package(Pangolin REQUIRED)   # Pangolin
if(NOT Pangolin_FOUND)
    message(FATAL_ERROR "Pangolin is not found!")
endif()

find_package(glog 0.3.5 REQUIRED)       #glog
if(NOT glog_FOUND)
    message(FATAL_ERROR "glog is not found!")
endif()

find_package(PCL 1.9 REQUIRED )         #COMPONENTS common io
add_definitions(${PCL_DEFINITIONS})
find_package(Eigen3 REQUIRED)           #Eigen3


include_directories(${GLib_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_executable(circle_test circle_test.cc)
add_executable(plane_test plane_test.cc)

target_link_libraries(circle_test   ${Pangolin_LIBRARIES} ${PCL_LIBRARIES} glog::glog ${GLib_LIBRARIES})
target_link_libraries(plane_test    ${Pangolin_LIBRARIES} ${PCL_LIBRARIES} glog::glog ${GLib_LIBRARIES})
