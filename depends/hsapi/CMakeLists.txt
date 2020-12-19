add_library(hsapi SHARED IMPORTED GLOBAL)
set_property(TARGET hsapi APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_property(TARGET hsapi APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)

file(GLOB HSFILES "bin/*.dll")

set_target_properties(
    hsapi
    PROPERTIES IMPORTED_IMPLIB
               ${CMAKE_CURRENT_LIST_DIR}/lib/HsRaw.lib
               IMPORTED_LOCATION
               "${HSFILES}"
               INTERFACE_INCLUDE_DIRECTORIES
               ${CMAKE_CURRENT_LIST_DIR}/include
               IMPORTED_GLOBAL TRUE
               INTERFACE_COMPILE_DEFINITIONS "_HAS_STD_BYTE=0")

