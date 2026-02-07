#include <crisp/crisp.hpp>

#include <fstream>

int main() {
  auto model = crisp::make_model();
  model->cfg().vis.gnd.grid_size = 1.0f;
  model->getGround()->cfg().width = 20;
  model->getGround()->cfg().height = 20;
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

  model->addActuator(
    {.type = crisp::actuator_e::force, .jointid = 1, .gain = {0, 1000, 100}});

  auto m = model->compile();

  m->act.u0().setZero();

  // m->state.q0() << 0.046743, 0.000024, 0.436035, 0.000006, 0.723991,
  // -0.000028,
  //   0.689809, -0.000793, -1.236725, 0.000940, -0.029036, 0.031640, -2.363523,
  //   -2.774024, -0.872700, 0.023604, -0.029250, 0.031007, -2.363334,
  //   -2.774084, -0.872700, 0.024274;

  m->opt.gravity.setZero();
  m->opt.solver_max_iter = 128;
  m->opt.jacobian = crisp::jacobian_e::dense;
  m->opt.solver = crisp::solver_e::sub_admm;

  // m->opt.solver_max_interval = 100;
  // m->opt.canal_beta_fric_init = 1e+1;
  // m->opt.canal_beta_fric_max = 1e+4;
  // m->opt.solver_max_inner = 30;
  // m->opt.canal_beta_init = 10;
  // m->opt.canal_beta_max = 1000;
  // m->opt.warmstart = false;
  // m->opt.solver_min_grad_inner = 0;

  auto app = crisp::make_app(std::move(m), false);

  app->setControl([&](auto const& m, auto const& d, auto u, auto udot) {
    u[0] = std::min(d.world.time * 0.01, 0.5);
  });

  app->init();
  while (app->isOpen()) {
    app->render();
    if (app->data().world.time > 10.0) break;
  }

  return 0;
}
