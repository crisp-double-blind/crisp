#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "crisp::crisp" for configuration "Release"
set_property(TARGET crisp::crisp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(crisp::crisp PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/crisp.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/crisp.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS crisp::crisp )
list(APPEND _IMPORT_CHECK_FILES_FOR_crisp::crisp "${_IMPORT_PREFIX}/lib/crisp.lib" "${_IMPORT_PREFIX}/bin/crisp.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
