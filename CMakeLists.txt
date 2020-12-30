cmake_minimum_required(VERSION 3.10)
project(MMS LANGUAGES C CXX)

# ==============================================================================
# H2O version
# ==============================================================================
file(STRINGS "src/version.h" _H2O_VERSION_HPP_CONTENTS REGEX "#define H2O_VERSION_")
foreach(v MAJOR MINOR REVISION)
  if("${_H2O_VERSION_HPP_CONTENTS}" MATCHES "#define H2O_VERSION_${v} ([0-9]+)")
    set(H2O_VERSION_${v} "${CMAKE_MATCH_1}")
  else()
    message(FATAL_ERROR "Failed to retrieve the H2O version the source code. Missing H2O_VERSION_${v}.")
  endif()
endforeach()
set(H2O_VERSION ${H2O_VERSION_MAJOR}.${H2O_VERSION_MINOR}.${H2O_VERSION_REVISION})

set(H2O_DATA_DIR "${CMAKE_CURRENT_LIST_DIR}/data")

option(H2O_BUILD_SHARED "" ON)
option(H2O_BUILD_TEST "" ON)
option(H2O_BUILD_DOG "" ON)
option(H2O_BUILD_SERIAL "" OFF)

set(PROJECT_VERSION
    "${H2O_VERSION_MAJOR}.${H2O_VERSION_MINOR}.${H2O_VERSION_PATCH}")
set(PROJECT_VERSION_MAJOR ${H2O_VERSION_MAJOR})
set(PROJECT_VERSION_MINOR ${H2O_VERSION_MINOR})
set(PROJECT_VERSION_PATCH ${H2O_VERSION_PATCH})

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(BUILD_SHARED_LIBS ${H2O_BUILD_SHARED})
if(BUILD_SHARED_LIBS AND MSVC)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif(BUILD_SHARED_LIBS AND MSVC)

if(MSVC)
  set(
    CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} /bigobj /MP /wd4005 /wd4996 /wd4819 /wd4251 /wd4267 /wd4244 /wd4275 /wd4530 /wd4334 /wd4018"
    )
  add_definitions(-D_ENABLE_EXTENDED_ALIGNED_STORAGE -DNOMINMAX)
  add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
  add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
endif(MSVC)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  set(CMAKE_CXX_FLAGS 
    "${CMAKE_CXX_FLAGS} -Wno-write-strings -Wno-unused-result -Wno-ignored-attributes"
  )
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  
  set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
  set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)
endif()

if(${H2O_BUILD_DOG})
    add_definitions(-DH2O_BUILD_DOG)
endif(${H2O_BUILD_DOG})

if(${H2O_BUILD_SERIAL})
    add_definitions(-DH2O_BUILD_SERIAL)
endif(${H2O_BUILD_SERIAL})

# OpenCL::OpenCL
find_package(OpenCL REQUIRED)

# EIGEN3_INCLUDE_DIR
find_package(Eigen3 REQUIRED)

find_package(Qt5 REQUIRED
             COMPONENTS Core
                        Gui
                        Widgets
                        LinguistTools
                        Network
                        Charts)
include(Qt5LinguistToolsMacrosNoInclude) # 添加创建翻译文件的函数

# OpenCV_LIBS
find_package(OpenCV
             REQUIRED
             calib3d
             core
             features2d
             flann
             highgui
             imgcodecs
             imgproc
             videoio
             ximgproc
             xphoto
             stitching)

add_definitions(-DCGAL_HEADER_ONLY)
find_package(CGAL REQUIRED)

# Boost_LIBRARIES Boost::c
# set(Boost_USE_STATIC_LIBS ON)
# add_definitions(-DBOOST_ALL_NO_LIB)
find_package(Boost REQUIRED filesystem locale)

# GDAL_LIBRARY
find_package(GDAL REQUIRED)

# ${PROJ4_LIBRARIES}
find_package(PROJ4 REQUIRED)

# glog::glog
find_package(Glog REQUIRED)

# find_package(LAPACK REQUIRED)

# ${CERES_LIBRARIES}
find_package(Ceres REQUIRED)

# IPP_LIBRARIES or IPP::module
find_package(IPP REQUIRED)

# OpenGL::GL OpenGL::GLU
find_package(OpenGL REQUIRED)

# GeographicLib_LIBRARIES
find_package(GeographicLib REQUIRED)

# TIFF::TIFF
find_package(TIFF REQUIRED)

# Laszip::Laszip
find_package(Laszip REQUIRED)

# OpenMP::OpenMP_<lang>
find_package(OpenMP REQUIRED)

# ${BLAS_LIBRARIES}
# find_package(BLAS REQUIRED)

# pdalcpp pdal_util
find_package(PDAL REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR}
                    ${CMAKE_CURRENT_LIST_DIR}/depends
                    ${CMAKE_CURRENT_SOURCE_DIR}/src
                    ${IPP_INCLUDE_DIRS}
                    ${GDAL_INCLUDE_DIR})

include(CMakeHelper)

add_subdirectory(depends)
add_subdirectory(src)

if(H2O_BUILD_TEST)
  add_subdirectory(tests)
endif(H2O_BUILD_TEST)
