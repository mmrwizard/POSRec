# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#.rst:
# FindPROJ4
# --------
#
# Find the PROJ4 library (libPROJ4).
#
# Imported targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the following :prop_tgt:`IMPORTED` targets:
#
# ``PROJ4::PROJ4``
#   The PROJ4 library, if found.
#
# Result variables
# ^^^^^^^^^^^^^^^^
#
# This module will set the following variables in your project:
#
# ``PROJ4_FOUND``
#   true if the PROJ4 headers and libraries were found
# ``PROJ4_INCLUDE_DIR``
#   the directory containing the PROJ4 headers
# ``PROJ4_INCLUDE_DIRS``
#   the directory containing the PROJ4 headers
# ``PROJ4_LIBRARIES``
#   PROJ4 libraries to be linked
#
# Cache variables
# ^^^^^^^^^^^^^^^
#
# The following cache variables may also be set:
#
# ``PROJ4_INCLUDE_DIR``
#   the directory containing the PROJ4 headers
# ``PROJ4_LIBRARY``
#   the path to the PROJ4 library

find_path(PROJ4_INCLUDE_DIR "proj.h")

set(PROJ4_NAMES ${PROJ4_NAMES} "proj" "proj_6_0")
foreach(name ${PROJ4_NAMES})
  list(APPEND PROJ4_NAMES_DEBUG "${name}" "${name}d")
endforeach()

if(NOT PROJ4_LIBRARY)
  find_library(PROJ4_LIBRARY_RELEASE NAMES ${PROJ4_NAMES})
  find_library(PROJ4_LIBRARY_DEBUG NAMES ${PROJ4_NAMES_DEBUG})
  include(SelectLibraryConfigurations)
  select_library_configurations(PROJ4)
  mark_as_advanced(PROJ4_LIBRARY_RELEASE PROJ4_LIBRARY_DEBUG)
endif()
unset(PROJ4_NAMES)
unset(PROJ4_NAMES_DEBUG)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PROJ4
                                  REQUIRED_VARS PROJ4_LIBRARY PROJ4_INCLUDE_DIR
                                  VERSION_VAR PROJ4_VERSION_STRING)

if(PROJ4_FOUND)
  set(PROJ4_LIBRARIES ${PROJ4_LIBRARY})
  set(PROJ4_INCLUDE_DIRS "${PROJ4_INCLUDE_DIR}")

  if(NOT TARGET PROJ4::PROJ4)
    add_library(PROJ4::PROJ4 UNKNOWN IMPORTED)
    if(PROJ4_INCLUDE_DIRS)
      set_target_properties(PROJ4::PROJ4 PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${PROJ4_INCLUDE_DIRS}")
    endif()
    if(EXISTS "${PROJ4_LIBRARY}")
      set_target_properties(PROJ4::PROJ4 PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "C"
        IMPORTED_LOCATION "${PROJ4_LIBRARY}")
    endif()
    if(EXISTS "${PROJ4_LIBRARY_RELEASE}")
      set_property(TARGET PROJ4::PROJ4 APPEND PROPERTY
        IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties(PROJ4::PROJ4 PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
        IMPORTED_LOCATION_RELEASE "${PROJ4_LIBRARY_RELEASE}")
    endif()
    if(EXISTS "${PROJ4_LIBRARY_DEBUG}")
      set_property(TARGET PROJ4::PROJ4 APPEND PROPERTY
        IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(PROJ4::PROJ4 PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "C"
        IMPORTED_LOCATION_DEBUG "${PROJ4_LIBRARY_DEBUG}")
    endif()
  endif()
endif()

mark_as_advanced(PROJ4_INCLUDE_DIR PROJ4_LIBRARY)
