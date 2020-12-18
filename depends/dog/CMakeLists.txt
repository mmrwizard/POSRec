add_library(dog SHARED IMPORTED GLOBAL)
set_property(TARGET dog APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_property(TARGET dog APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(
    dog
    PROPERTIES IMPORTED_IMPLIB
               ${CMAKE_CURRENT_LIST_DIR}/lib/hasp_windows_x64_35148.lib
               IMPORTED_LOCATION
               ${CMAKE_CURRENT_LIST_DIR}/bin/dog_windows_x64_3150280.dll
               INTERFACE_INCLUDE_DIRECTORIES
               ${CMAKE_CURRENT_LIST_DIR}/include
               INTERFACE_COMPILE_DEFINITIONS
               "H2O_BUILD_FLOAT")

