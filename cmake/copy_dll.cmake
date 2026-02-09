function(copy_dll exe_target)
  add_custom_command(TARGET ${exe_target} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:crisp::crisp>
    $<TARGET_FILE_DIR:${exe_target}>
  )
endfunction()
