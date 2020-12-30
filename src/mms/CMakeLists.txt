file(GLOB SOURCES "*.cpp" "*.h")
file(GLOB SOURCES_SPLINE "spline/*.*")

source_group(Spline FILES ${SOURCES_SPLINE})

if(MSVC)
h2o_add_library(h2o_mms SOURCES "${SOURCES_SPLINE};${SOURCES}"
 PUBLIC_LINKS h2o_base 
 PRIVATE_LINKS ${CERES_LIBRARIES} ${GeographicLib_LIBRARIES} Laszip::Laszip hsapi
)
# copy qm files to position
get_target_property(DLL_FILES hsapi IMPORTED_LOCATION)
message(STATUS "${DLL_FILES}")

foreach(FILE IN LISTS DLL_FILES)
        add_custom_command(
        TARGET h2o_mms POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${FILE}"
                "$<TARGET_FILE_DIR:h2o_mms>/"
        COMMENT "Copying HSRawApi\n")
endforeach()

else()

h2o_add_library(h2o_mms SOURCES "${SOURCES_SPLINE};${SOURCES}"
 PUBLIC_LINKS h2o_base 
 PRIVATE_LINKS ${CERES_LIBRARIES} ${GeographicLib_LIBRARIES} Laszip::Laszip HS::RAW HS::RAWCORE HS::ALGORITHM
)

add_custom_command(
        TARGET h2o_mms POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:HS::RAW>"
                "$<TARGET_FILE_DIR:h2o_mms>/"
        COMMENT "Copying HSRawApi\n")

add_custom_command(
        TARGET h2o_mms POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:HS::RAWCORE>"
                "$<TARGET_FILE_DIR:h2o_mms>/"
        COMMENT "Copying HSRawApi\n")
add_custom_command(
        TARGET h2o_mms POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:HS::ALGORITHM>"
                "$<TARGET_FILE_DIR:h2o_mms>/"
        COMMENT "Copying HSRawApi\n")
endif()




