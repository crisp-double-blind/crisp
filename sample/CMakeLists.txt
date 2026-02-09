add_executable(app app.cpp)
target_link_libraries(app PRIVATE crisp::crisp)

add_executable(basic basic.cpp)
target_link_libraries(basic PRIVATE crisp::crisp glfw)

if(WIN32)
  include(../cmake/copy_dll.cmake)
  copy_dll(app)
  copy_dll(basic)
endif()
