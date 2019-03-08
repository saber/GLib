cmake_minimum_required(VERSION 2.8)
# 1、cmake 在执行这里时，CMake 会自动检测编译器，默认是 c/c++ 的。会自动检测当前所属平台。如果是 windows vs的，会把 MSVC 置为 true
#    如果是类 UNIX 平台后面会自动设置下面的 CMAKE_COMPILER_IS_GUNCC CMAKE_COMPILER_IS_GUNXX 变量为 true
#    用于平台检测。
# 2、还会做一些变量的赋值，比如 PROJECT_SOURCE_DIR PROJECT_VERSION_MAJOR 等等
project(GLib)

# 一些编译选项，可以在终端中直接使用 cmake -Dvar=value 来改变！！！，此时这里给定了默认值！全部都是 cache 变量
option(EXPORT_BUILD_DIR "Export build directory using CMake (enables external use without install)." ON) # 默认不安装，只导出
option(BUILD_utils  "Build tuils (basic c++ functions)." ON)
option(BUILD_math "Build math(requires utils)." ON)

# 默认 Debug 状态，会详细输出一些 log
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  #每次强制设置成 cache 变量。此时可以 unset 前面的普通变量(相同名字的)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo") # 设置 CMAKE_BUILD_TYPE cache 变量会包含其他属性值，然后可以通过 get_property()
endif()

# 根据 CMake 自动检测的平台，执行相关代码！
if(MSVC)
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]") # 检测 vs win 平台 -w 选项是否设置，如果设置了话，全部置位最高 -w4
    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}") # 这个表示将 CMAKE_CXX_FLAGS 中的"/w[0-4]"替换成"/w4"。表示最高警告
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -pedantic")
  # set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 ")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
endif()

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# 源文件集
set(GLib_SRCS)
if(BUILD_utils)
    set(GLib_SRCS
        src/utils/Random.cc
        ${GLib_SRCS}  # 注意这里，之前就错了！
    )
endif(BUILD_utils)

if(BUILD_math)
    set(GLib_SRCS
        src/math/ransac/ransac.cc
        src/math/ransac/ransac_app.cc
        ${GLib_SRCS}
    )
endif(BUILD_math)

set(LIB_SHARED "SHARED")
if(WIN32)
  set(LIB_SHARED "STATIC")
endif(WIN32)

# 设置查找 Findxxx.cmake 文件的路径
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
find_package(Eigen3 REQUIRED)           #Eigen3
find_package(glog 0.3.5 REQUIRED)       #glog
if(NOT glog_FOUND)
    message(FATAL_ERROR "glog is not found!")
endif()

#设置依赖库
set(THIRD_PARTY_LIBS
    glog::glog
)

# 设置本工程库的输出路径
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# 本工程头文件路径
set(GLib_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/include/math/
    ${PROJECT_SOURCE_DIR}/include/utils/
)

include_directories(
    ${GLib_INCLUDE_DIRS}
    ${glog_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)
add_library(${PROJECT_NAME} ${LIB_SHARED} ${GLib_SRCS}) # libGLib.so
target_link_libraries(${PROJECT_NAME} ${THIRD_PARTY_LIBS})

# 安装方法 #2: 将 GLib/build 目录放在 local CMake registry.因此不需要安装
# registry (~/.cmake/packages on *nix & OS X).
if(EXPORT_BUILD_DIR)
  # 配置导出文件
  configure_file(cmake/GLibConfig.cmake.in
  "${PROJECT_BINARY_DIR}/GLibConfig.cmake" @ONLY)
  export(PACKAGE ${PROJECT_NAME}) # GLib
else(EXPORT_BUILD_DIR)
    # 配置安装文件
    configure_file(cmake/GLibInstall.cmake.in
    "${PROJECT_BINARY_DIR}/GLibConfig.cmake" @ONLY)
endif(EXPORT_BUILD_DIR)


# 安装库 默认不添加 lib 前缀
install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
if(BUILD_utils)
  install(DIRECTORY include/utils DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
endif()
if(BUILD_math)
  install(DIRECTORY include/math  DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
endif()

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/GLibConfig.cmake"
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})
install(FILES "${PROJECT_BINARY_DIR}/GLibConfig.cmake"
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/DLib/) # 实际上在 usr/local/lib/cmake usr/lib/cmake 都是默认的搜索路径。
