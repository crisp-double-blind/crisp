target_sources(
  crisp PRIVATE
  arena.hpp
  constants.hpp
  data.hpp
  heap.hpp
  crisp.hpp
  macro.hpp
  model.hpp
  render.hpp
  user.hpp
  xmacro.hpp
)
target_sources(crisp-app PRIVATE app.hpp)

add_subdirectory(arena)
