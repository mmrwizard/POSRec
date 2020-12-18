file(GLOB OGL_SRC "*.h" "*.cpp")
h2o_add_library(h2o_ogl SOURCES ${OGL_SRC} PUBLIC_LINKS h2o_base PRIVATE_LINKS Qt5::Core Qt5::Gui)
