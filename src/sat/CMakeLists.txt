file(GLOB SOURCES "*.cpp" "*.h")
h2o_add_library(h2o_sat SOURCES ${SOURCES}
 PUBLIC_LINKS h2o_base 
 PRIVATE_LINKS ${CERES_LIBRARIES} ${LAPACK_LIBRARIES} ${GeographicLib_LIBRARIES}
)
