#include <iostream>

#include <GLFW/glfw3.h>

#include <crisp/crisp.hpp>

std::unique_ptr<crisp::camera_t> g_cam;
std::unique_ptr<crisp::scene_t> g_scn;
crisp::mouse_t g_mouse;

void on_framebuffer_size(GLFWwindow* window, int width, int height) {
  g_scn->viewport.width = width;
  g_scn->viewport.height = height;
}

void on_mouse_button(GLFWwindow* window, int button, int action, int mods) {
  bool press = (action == GLFW_PRESS);
  g_mouse.left_btn = (button == GLFW_MOUSE_BUTTON_LEFT) ? press : false;
  g_mouse.right_btn = (button == GLFW_MOUSE_BUTTON_RIGHT) ? press : false;
  g_mouse.scroll_btn = (button == GLFW_MOUSE_BUTTON_MIDDLE) ? press : false;
}

void on_cursor_pos(GLFWwindow* window, double x, double y) {
  g_mouse.dx = static_cast<float>(x - g_mouse.x) / g_scn->viewport.width;
  g_mouse.dy = static_cast<float>(y - g_mouse.y) / g_scn->viewport.height;
  g_mouse.x = static_cast<int>(x);
  g_mouse.y = static_cast<int>(y);
  if (g_mouse.left_btn || g_mouse.right_btn || g_mouse.scroll_btn) {
    crisp::move_camera(*g_cam, g_mouse);
  }
}

void on_scroll(GLFWwindow* window, double xoffset, double yoffset) {
  g_mouse.sx = static_cast<float>(xoffset) / g_scn->viewport.width;
  g_mouse.sy = static_cast<float>(yoffset) / g_scn->viewport.height;
  crisp::move_camera(*g_cam, g_mouse);
}

int main(int argc, char const* argv[]) {
  if (argc != 2) {
    std::cerr << "binary model not given";
    return 1;
  }

  auto m = crisp::load_model(argv[1]);
  auto d = crisp::make_data(*m);

  if (!glfwInit()) {
    std::cerr << "failed to initialize GLFW" << std::endl;
    return 1;
  }

  glfwWindowHint(GLFW_SAMPLES, 4);
  GLFWwindow* window = glfwCreateWindow(800, 600, "basic", nullptr, nullptr);
  if (!window) {
    std::cerr << "failed to create GLFW window" << std::endl;
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  g_cam = crisp::load_camera(*m);
  g_scn = crisp::make_scene(*m);

  glfwGetFramebufferSize(
    window, &g_scn->viewport.width, &g_scn->viewport.height);
  glfwSetFramebufferSizeCallback(window, on_framebuffer_size);
  glfwSetMouseButtonCallback(window, on_mouse_button);
  glfwSetCursorPosCallback(window, on_cursor_pos);
  glfwSetScrollCallback(window, on_scroll);

  while (!glfwWindowShouldClose(window)) {
    float time = static_cast<float>(glfwGetTime());

    crisp::step(*m, *d);

    crisp::update_scene(*m, *g_cam, *g_scn);
    crisp::render_scene(*m, *d, *g_scn);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
