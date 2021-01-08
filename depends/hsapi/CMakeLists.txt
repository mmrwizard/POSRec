
if(MSVC)

add_library(HS::RAW SHARED IMPORTED GLOBAL)
set_property(TARGET HS::RAW APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_property(TARGET HS::RAW APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)

file(GLOB HSFILES "bin/*.dll")
set_target_properties(
    HS::RAW
    PROPERTIES IMPORTED_IMPLIB
               ${CMAKE_CURRENT_LIST_DIR}/lib/HsRaw.lib
               IMPORTED_LOCATION
               "${HSFILES}"
               INTERFACE_INCLUDE_DIRECTORIES
               ${CMAKE_CURRENT_LIST_DIR}/include
               IMPORTED_GLOBAL TRUE
               INTERFACE_COMPILE_DEFINITIONS "_HAS_STD_BYTE=0")

install(FILES "${HSFILES}" DESTINATION bin)

else()

set(HSAPIS "")
list(APPEND HSAPIS "RAW")
list(APPEND HSAPIS "RAWCORE")
list(APPEND HSAPIS "ALGORITHM")

foreach(hsapi ${HSAPIS})
    add_library("HS::${hsapi}" SHARED IMPORTED)
    set_property(TARGET "HS::${hsapi}"  APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
    set_property(TARGET "HS::${hsapi}"  APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
endforeach()

set_target_properties(
    HS::RAW
    PROPERTIES IMPORTED_IMPLIB
               ${CMAKE_CURRENT_LIST_DIR}/lib/libHsRaw.so
               IMPORTED_LOCATION
               ${CMAKE_CURRENT_LIST_DIR}/lib/libHsRaw.so
               INTERFACE_INCLUDE_DIRECTORIES
               ${CMAKE_CURRENT_LIST_DIR}/include_linux
               IMPORTED_GLOBAL TRUE
               IMPORTED_NO_SONAME TRUE)
set_target_properties(
    HS::RAWCORE
    PROPERTIES IMPORTED_IMPLIB
                ${CMAKE_CURRENT_LIST_DIR}/lib/libHsRawCore.so
                IMPORTED_LOCATION
                ${CMAKE_CURRENT_LIST_DIR}/lib/libHsRawCore.so
                INTERFACE_INCLUDE_DIRECTORIES
                ${CMAKE_CURRENT_LIST_DIR}/include_linux
                IMPORTED_GLOBAL TRUE
                IMPORTED_NO_SONAME TRUE)
set_target_properties(
    HS::ALGORITHM
    PROPERTIES IMPORTED_IMPLIB
                ${CMAKE_CURRENT_LIST_DIR}/lib/libHsAlgorithm.so
                IMPORTED_LOCATION
                ${CMAKE_CURRENT_LIST_DIR}/lib/libHsAlgorithm.so
                INTERFACE_INCLUDE_DIRECTORIES
                ${CMAKE_CURRENT_LIST_DIR}/include_linux
                IMPORTED_GLOBAL TRUE
                IMPORTED_NO_SONAME TRUE)

install(FILES "$<TARGET_FILE:HS::RAW>;$<TARGET_FILE:HS::RAWCORE>;$<TARGET_FILE:HS::ALGORITHM>"
    DESTINATION lib  
)

endif()




