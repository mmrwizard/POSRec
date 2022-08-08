#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "liblas" for configuration "Debug"
set_property(TARGET liblas APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(liblas PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/debug/lib/liblas.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_DEBUG "${_IMPORT_PREFIX}/debug/lib/tiffd.lib;${_IMPORT_PREFIX}/debug/lib/lzmad.lib;${_IMPORT_PREFIX}/debug/lib/jpegd.lib;${_IMPORT_PREFIX}/debug/lib/zlibd.lib;geotiff_library;${_IMPORT_PREFIX}/debug/lib/boost_program_options-vc140-mt-gd.lib;${_IMPORT_PREFIX}/debug/lib/boost_thread-vc140-mt-gd.lib;${_IMPORT_PREFIX}/debug/lib/boost_system-vc140-mt-gd.lib;${_IMPORT_PREFIX}/debug/lib/boost_iostreams-vc140-mt-gd.lib;${_IMPORT_PREFIX}/debug/lib/boost_filesystem-vc140-mt-gd.lib;${_IMPORT_PREFIX}/debug/lib/boost_chrono-vc140-mt-gd.lib;${_IMPORT_PREFIX}/debug/lib/boost_date_time-vc140-mt-gd.lib;${_IMPORT_PREFIX}/debug/lib/boost_atomic-vc140-mt-gd.lib;${_IMPORT_PREFIX}/debug/lib/boost_regex-vc140-mt-gd.lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/debug/bin/liblas.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS liblas )
list(APPEND _IMPORT_CHECK_FILES_FOR_liblas "${_IMPORT_PREFIX}/debug/lib/liblas.lib" "${_IMPORT_PREFIX}/debug/bin/liblas.dll" )

# Import target "liblas_c" for configuration "Debug"
set_property(TARGET liblas_c APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(liblas_c PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/debug/lib/liblas_c.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_DEBUG "liblas"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/debug/bin/liblas_c.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS liblas_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_liblas_c "${_IMPORT_PREFIX}/debug/lib/liblas_c.lib" "${_IMPORT_PREFIX}/debug/bin/liblas_c.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
