include(CMakeFindDependencyMacro)

find_dependency(Eigen3 REQUIRED)
find_dependency(spdlog REQUIRED)

get_filename_component(CONFIG_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(CRISP_LIB_DIR "${CONFIG_DIR}" DIRECTORY)
get_filename_component(CRISP_ROOT_DIR "${CRISP_LIB_DIR}" DIRECTORY)

set(crisp_INCLUDE_DIRS "${CRISP_ROOT_DIR}/include")

if(NOT TARGET crisp::crisp)
  add_library(crisp::crisp SHARED IMPORTED)
  set_target_properties(crisp::crisp PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${crisp_INCLUDE_DIRS}"
    IMPORTED_LOCATION "${CRISP_LIB_DIR}/crisp.dll"
    IMPORTED_IMPLIB "${CRISP_LIB_DIR}/crisp.lib"
    INTERFACE_LINK_LIBRARIES "Eigen3::Eigen;spdlog::spdlog"
    INTERFACE_COMPILE_FEATURES cxx_std_20
  )
endif()
