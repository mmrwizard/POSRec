file(GLOB SOURCES "*.h" "*.cpp" "*.hpp")
h2o_add_library(h2o_polygon SOURCES ${SOURCES} PUBLIC_LINKS h2o_base PRIVATE_LINKS h2o_ogl Qt5::Core Qt5::Gui ${GDAL_LIBRARY})

