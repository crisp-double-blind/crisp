file(GLOB cases ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

foreach(case ${cases})
  get_filename_component(case_name ${case} NAME)
  string(REPLACE ".cpp" "" test_name ${case_name})
  add_executable(${test_name} ${case})
  target_link_libraries(${test_name} PRIVATE crisp::crisp)
endforeach()
