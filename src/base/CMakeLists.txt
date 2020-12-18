file(GLOB SOURCES "*.h" "*.hpp" "*.cpp")

h2o_add_library(h2o_base
  SOURCES ${SOURCES}
  PUBLIC_LINKS OpenMP::OpenMP_CXX glog::glog ${OpenCV_LIBS} Qt5::Core Qt5::Gui
  PRIVATE_LINKS aes ${Boost_LIBRARIES} ${GDAL_LIBRARY} ${PROJ4_LIBRARIES} TIFF::TIFF Laszip::Laszip ${IPP_LIBRARIES} Qt5::Widgets Qt5::Network OpenGL::GL OpenGL::GLU
)
