cmake_minimum_required(VERSION 3.10)
project(crisp_double_blind)

include(cmake/dependencies.cmake)

find_package(crisp REQUIRED PATHS "${CMAKE_CURRENT_SOURCE_DIR}/crisp")

add_subdirectory(sample)
add_subdirectory(evaluations)
