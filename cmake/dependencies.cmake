include(${CMAKE_CURRENT_LIST_DIR}/find_or_fetch.cmake)

find_or_fetch(
  LIBRARY_NAME Eigen3 LIBRARY_VERSION 3.4.0
  GIT_REPO https://gitlab.com/libeigen/eigen.git GIT_TAG 3.4.0
  CMAKE_ARGS "-DEIGEN_BUILD_DOC=OFF" "-DBUILD_TESTING=OFF"
  HEADER_ONLY
)

find_or_fetch(
  LIBRARY_NAME spdlog LIBRARY_VERSION 1.15.2
  GIT_REPO https://github.com/gabime/spdlog.git GIT_TAG v1.15.2
  CMAKE_ARGS
  "-DSPDLOG_BUILD_EXAMPLE=OFF" "${spdlog_fmt_external_opt}"
  "-DCMAKE_POSITION_INDEPENDENT_CODE=ON"
  TARGET_NAME spdlog::spdlog
)

find_or_fetch(
  LIBRARY_NAME glfw3 LIBRARY_VERSION 3.4
  GIT_REPO https://github.com/glfw/glfw.git GIT_TAG 3.4
  CMAKE_ARGS
  "-DGLFW_BUILD_EXAMPLES=OFF" "-DGLFW_BUILD_TESTS=OFF"
  "-DGLFW_BUILD_DOCS=OFF" "-DGLFW_BUILD_WAYLAND=OFF"
  FIX_NO_CONFIG_SUFFIX TARGET_NAME glfw
)
