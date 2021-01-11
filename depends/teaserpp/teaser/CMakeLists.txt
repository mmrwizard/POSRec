file(GLOB SOURCES "*.h" "*.cpp" "*.cc")
add_library(teaserpp STATIC ${SOURCES})
target_link_libraries(teaserpp
        PUBLIC Eigen3::Eigen
        PRIVATE pmc
        )
target_include_directories(teaserpp PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        )

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(teaserpp PRIVATE OpenMP::OpenMP_CXX)
endif()

add_library(teaserpp::teaserpp ALIAS teaserpp)
