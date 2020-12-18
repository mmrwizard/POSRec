file(GLOB SPLINTER_SRC "*.h" "*.cpp")
include_directories(${CMAKE_CURRENT_LIST_DIR})
add_library(splinter STATIC ${SPLINTER_SRC})
