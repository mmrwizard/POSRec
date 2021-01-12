h2o_add_software(MMSLiDARCalibrate SOURCES main.cpp
                LINKS h2o_mms ${GDAL_LIBRARY} Qt5::Core Qt5::Gui Qt5::Widgets
                INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}
)

h2o_add_software(MMSLiDARCalibrateIcp SOURCES main_icp.cpp
                LINKS h2o_mms ${GDAL_LIBRARY} Qt5::Core Qt5::Gui Qt5::Widgets
                INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}
)
