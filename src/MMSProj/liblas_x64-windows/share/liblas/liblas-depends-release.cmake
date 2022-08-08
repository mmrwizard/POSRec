#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "liblas" for configuration "Release"
set_property(TARGET liblas APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(liblas PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/liblas.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "${_IMPORT_PREFIX}/lib/tiff.lib;${_IMPORT_PREFIX}/lib/lzma.lib;${_IMPORT_PREFIX}/lib/jpeg.lib;${_IMPORT_PREFIX}/lib/zlib.lib;geotiff_library;${_IMPORT_PREFIX}/lib/boost_program_options-vc140-mt.lib;${_IMPORT_PREFIX}/lib/boost_thread-vc140-mt.lib;${_IMPORT_PREFIX}/lib/boost_system-vc140-mt.lib;${_IMPORT_PREFIX}/lib/boost_iostreams-vc140-mt.lib;${_IMPORT_PREFIX}/lib/boost_filesystem-vc140-mt.lib;${_IMPORT_PREFIX}/lib/boost_chrono-vc140-mt.lib;${_IMPORT_PREFIX}/lib/boost_date_time-vc140-mt.lib;${_IMPORT_PREFIX}/lib/boost_atomic-vc140-mt.lib;${_IMPORT_PREFIX}/lib/boost_regex-vc140-mt.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/liblas.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS liblas )
list(APPEND _IMPORT_CHECK_FILES_FOR_liblas "${_IMPORT_PREFIX}/lib/liblas.lib" "${_IMPORT_PREFIX}/bin/liblas.dll" )

# Import target "liblas_c" for configuration "Release"
set_property(TARGET liblas_c APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(liblas_c PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/liblas_c.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "liblas"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/liblas_c.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS liblas_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_liblas_c "${_IMPORT_PREFIX}/lib/liblas_c.lib" "${_IMPORT_PREFIX}/bin/liblas_c.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
