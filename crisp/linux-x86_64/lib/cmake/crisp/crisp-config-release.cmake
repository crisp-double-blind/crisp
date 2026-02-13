#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "crisp::crisp" for configuration "Release"
set_property(TARGET crisp::crisp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(crisp::crisp PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcrisp.so"
  IMPORTED_SONAME_RELEASE "libcrisp.so"
  )

list(APPEND _cmake_import_check_targets crisp::crisp )
list(APPEND _cmake_import_check_files_for_crisp::crisp "${_IMPORT_PREFIX}/lib/libcrisp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
