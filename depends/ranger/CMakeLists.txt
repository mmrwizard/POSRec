## ======================================================================================##
## Check for C++11. For GCC this is >=4.7
## ======================================================================================##
include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# if(COMPILER_SUPPORTS_CXX11)
#   message("Compiler with C++11 support found.")
# else()
#   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler, i.e. gcc >= 4.7 or Clang >= 3.0.")
# endif()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

## ======================================================================================##
## Subdirectories and source files
## ======================================================================================##

file(GLOB SOURCES *.cpp *.h)

## ======================================================================================##
## Executable
## ======================================================================================##
add_library(ranger STATIC ${SOURCES})

