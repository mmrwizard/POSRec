file(GLOB SOURCES "*.h" "*.cpp")
h2o_add_software(MMSProj SOURCES ${SOURCES}
                LINKS h2o_mms h2o_ogl ${GDAL_LIBRARY} Qt5::Core Qt5::Gui Qt5::Widgets Laszip::Laszip
                INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}
)
