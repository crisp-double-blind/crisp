#include <crisp/crisp.hpp>

int main(int argc, char const* argv[]) {
  auto app = crisp::make_app(argc == 2 ? argv[1] : "");

  app->init();
  while (app->isOpen()) {
    app->render();
  }

  return 0;
}
