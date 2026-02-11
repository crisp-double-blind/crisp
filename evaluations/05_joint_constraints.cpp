#include <crisp/crisp.hpp>

int main() {
  auto model = crisp::make_model();
  model->addRobot(
    {.path = "assets/door/door.urdf",
     .root_pos = {0, 0, 1},
     .root_quat = Eigen::Quaternionr(0.7071, 0.7071, 0, 0)},
    true);
  model->cfg().vis.cam.target = {0.0f, 0.0f, 0.7f};
  model->cfg().vis.cam.distance = 3.85f;
  model->cfg().vis.cam.azimuth = 120.0f;
  model->cfg().vis.cam.elevation = -20.0f;
  model->cfg().vis.cam.fovy = 45.0f;
  model->cfg().vis.gnd.grid_size = 1.0f;
  model->getGround()->cfg().width = 20;
  model->getGround()->cfg().height = 20;
  model->addLight(
    {.type = crisp::light_e::directional,
     .enabled = true,
     .cast_shadow = true,
     .pos = {1.0f, -1.0f, 1.0f},
     .dir = {-1.0f, 1.0f, -1.0f},
     .ambient = {0.3f, 0.3f, 0.3f},
     .diffuse = {0.8f, 0.8f, 0.8f},
     .specular = {1.0f, 1.0f, 1.0f},
     .attenuation = {1.0f, 0.0f, 0.0f},
     .cutoff = 0.0f,
     .exponent = 0.0f});
  model->cfg().opt.gravity.setZero();
  model->cfg().opt.solver_max_iter = 128;
  model->cfg().opt.solver = crisp::solver_e::sub_admm;

  model->addActuator(
    {.type = crisp::actuator_e::force, .jointid = 1, .gain = {0, 1000, 100}});

  auto m = model->compile();
  m->act.u0().setZero();

  auto app = crisp::make_app(std::move(m), false);

  app->setControl([&](auto const& m, auto const& d, auto u, auto udot) {
    u[0] = std::min(d.world.time * 0.01, 0.5);
  });

  app->init();
  while (app->isOpen()) {
    app->render();
  }

  return 0;
}
