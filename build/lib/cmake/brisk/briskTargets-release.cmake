#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "brisk::agast" for configuration "Release"
set_property(TARGET brisk::agast APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(brisk::agast PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libagast.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS brisk::agast )
list(APPEND _IMPORT_CHECK_FILES_FOR_brisk::agast "${_IMPORT_PREFIX}/lib/libagast.a" )

# Import target "brisk::brisk" for configuration "Release"
set_property(TARGET brisk::brisk APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(brisk::brisk PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libbrisk.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS brisk::brisk )
list(APPEND _IMPORT_CHECK_FILES_FOR_brisk::brisk "${_IMPORT_PREFIX}/lib/libbrisk.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
