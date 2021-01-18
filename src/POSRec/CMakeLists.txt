file(GLOB SOURCES "*.h" "*.cpp")

h2o_add_software(MMSRec SOURCES ${SOURCES}
                LINKS h2o_mms ${GDAL_LIBRARY} Qt5::Core Qt5::Gui Qt5::Widgets
                INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}
)
