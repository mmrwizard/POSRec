file(GLOB SOURCES "*.cpp" "*.h")
file(GLOB SOURCES_SPLINE "spline/*.*")

source_group(Spline FILES ${SOURCES_SPLINE})

h2o_add_library(h2o_mms SOURCES "${SOURCES_SPLINE};${SOURCES}"
 PUBLIC_LINKS h2o_base 
 PRIVATE_LINKS ${CERES_LIBRARIES}
)
