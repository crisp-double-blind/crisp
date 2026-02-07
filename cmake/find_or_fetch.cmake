if(COMMAND find_or_fetch)
  return()
endif()

function(find_or_fetch)
  include(FetchContent)

  set(options HEADER_ONLY FIX_NO_CONFIG_SUFFIX)
  set(one_value_args LIBRARY_NAME LIBRARY_VERSION GIT_REPO GIT_TAG TARGET_NAME)
  set(multi_value_args CMAKE_ARGS)

  cmake_parse_arguments(
    ARGS "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN}
  )
  string(TOLOWER ${ARGS_LIBRARY_NAME} library_name)

  FetchContent_Declare(
    ${ARGS_LIBRARY_NAME}
    GIT_REPOSITORY "${ARGS_GIT_REPO}"
    GIT_TAG "${ARGS_GIT_TAG}"
    CMAKE_ARGS ${ARGS_CMAKE_ARGS}
  )
  set(${library_name}_install_dir "${FETCHCONTENT_BASE_DIR}/${library_name}")

  if(NOT MSVC OR ARGS_HEADER_ONLY)
    find_package(${ARGS_LIBRARY_NAME} ${ARGS_LIBRARY_VERSION} QUIET)

    if(NOT ${ARGS_LIBRARY_NAME}_FOUND)
      FetchContent_Populate(${ARGS_LIBRARY_NAME})
      execute_process(
        COMMAND ${CMAKE_COMMAND}
        -G "${CMAKE_GENERATOR}" "${${library_name}_SOURCE_DIR}"
        -DCMAKE_INSTALL_PREFIX=${${library_name}_install_dir}
        -DCMAKE_BUILD_TYPE=Release
        ${ARGS_CMAKE_ARGS}
        WORKING_DIRECTORY "${${library_name}_BINARY_DIR}"
      )
      execute_process(
        COMMAND ${CMAKE_COMMAND}
        --build . --target install --config Release
        WORKING_DIRECTORY "${${library_name}_BINARY_DIR}"
      )
      find_package(
        ${ARGS_LIBRARY_NAME} REQUIRED
        PATHS "${${library_name}_install_dir}" NO_DEFAULT_PATH
      )
    endif()

  elseif(NOT ARGS_FIX_NO_CONFIG_SUFFIX)
    find_package(${ARGS_LIBRARY_NAME} ${ARGS_LIBRARY_VERSION} QUIET)

    if(NOT ${ARGS_LIBRARY_NAME}_FOUND)
      FetchContent_Populate(${ARGS_LIBRARY_NAME})

      foreach(build_type IN ITEMS Debug Release)
        file(MAKE_DIRECTORY "${${library_name}_BINARY_DIR}/${build_type}")
        execute_process(
          COMMAND ${CMAKE_COMMAND}
          -G "${CMAKE_GENERATOR}" "${${library_name}_SOURCE_DIR}"
          -DCMAKE_INSTALL_PREFIX=${${library_name}_install_dir}
          -DCMAKE_BUILD_TYPE=${build_type}
          ${ARGS_CMAKE_ARGS}
          WORKING_DIRECTORY "${${library_name}_BINARY_DIR}/${build_type}"
        )
        execute_process(
          COMMAND ${CMAKE_COMMAND}
          --build . --target install --config ${build_type}
          WORKING_DIRECTORY "${${library_name}_BINARY_DIR}/${build_type}"
        )
      endforeach()

      find_package(
        ${ARGS_LIBRARY_NAME} REQUIRED
        PATHS "${${library_name}_install_dir}" NO_DEFAULT_PATH
      )
    endif()

    get_target_property(
      ${library_name}_release ${ARGS_TARGET_NAME} IMPORTED_LOCATION_RELEASE
    )
    set_target_properties(
      ${ARGS_TARGET_NAME} PROPERTIES
      IMPORTED_LOCATION_MINSIZEREL "${${library_name}_release}"
      IMPORTED_LOCATION_RELWITHDEBINFO "${${library_name}_release}"
    )

  else()
    find_package(
      ${ARGS_LIBRARY_NAME} QUIET
      PATHS "${${library_name}_install_dir}/Debug" NO_DEFAULT_PATH
    )

    if(NOT ${ARGS_LIBRARY_NAME}_FOUND)
      FetchContent_Populate(${ARGS_LIBRARY_NAME})

      foreach(build_type IN ITEMS Debug Release)
        file(MAKE_DIRECTORY "${${library_name}_BINARY_DIR}/${build_type}")
        execute_process(
          COMMAND ${CMAKE_COMMAND}
          -G "${CMAKE_GENERATOR}" "${${library_name}_SOURCE_DIR}"
          -DCMAKE_INSTALL_PREFIX=${${library_name}_install_dir}/${build_type}
          -DCMAKE_BUILD_TYPE=${build_type}
          ${ARGS_CMAKE_ARGS}
          WORKING_DIRECTORY "${${library_name}_BINARY_DIR}/${build_type}"
        )
        execute_process(
          COMMAND ${CMAKE_COMMAND}
          --build . --target install --config ${build_type}
          WORKING_DIRECTORY "${${library_name}_BINARY_DIR}/${build_type}"
        )
      endforeach()

      find_package(
        ${ARGS_LIBRARY_NAME} REQUIRED
        PATHS "${${library_name}_install_dir}/Debug" NO_DEFAULT_PATH
      )
    endif()

    get_target_property(
      ${library_name}_debug ${ARGS_TARGET_NAME} IMPORTED_LOCATION_DEBUG
    )
    file(RELATIVE_PATH rel_path
      "${${library_name}_install_dir}/Debug" "${${library_name}_debug}"
    )
    set(${library_name}_release
      "${${library_name}_install_dir}/Release/${rel_path}"
    )
    set_target_properties(
      ${ARGS_TARGET_NAME} PROPERTIES
      IMPORTED_LOCATION_RELEASE "${${library_name}_release}"
      IMPORTED_LOCATION_MINSIZEREL "${${library_name}_release}"
      IMPORTED_LOCATION_RELWITHDEBINFO "${${library_name}_release}"
    )
  endif()
endfunction()
