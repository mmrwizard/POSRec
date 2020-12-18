file(GLOB SOURCES "*.h" "*.cpp")
file(GLOB AM_SOURCES "../../depends/AdaptiveMultigrid/*.h"
     "../../depends/AdaptiveMultigrid/*.inl")
list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_LIST_DIR}/interpolation.cpp")
file(GLOB INTERPOLATION "interpolation*")

source_group(Poisson FILES ${AM_SOURCES})
h2o_add_library(h2o_poisson 
    SOURCES ${SOURCES} ${AM_SOURCES}
    PRIVATE_LINKS OpenCL::OpenCL ${OpenCV_LIBS} OpenMP::OpenMP_CXX h2o_base h2o_polygon)

h2o_add_library(h2o_poisson_interpolation
    SOURCES ${INTERPOLATION} ${AM_SOURCES}
    PRIVATE_LINKS OpenCL::OpenCL ${OpenCV_LIBS} OpenMP::OpenMP_CXX h2o_base h2o_polygon)
