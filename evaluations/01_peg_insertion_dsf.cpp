#include <crisp/crisp.hpp>

#include <numbers>

int main() {
  auto model = crisp::make_model();

  double eps = 100e-6;
  double radius = 0.025 - eps / 2;
  double height = 0.10;
  double mass = 0.01;
  double theta = 0.5 / 180.0 * std::numbers::pi;
  double y_offset = -std::sin(theta) * height / 2;

  double mu = 0.01;
  {
    auto& body = model->addBody({.pos = {0.0, 0.0, height / 2}});
    auto& geom = body.addGeom({.mu = mu});
    Eigen::Matrix2Xr vert(2, 4);
    vert << 0.0075, -0.0075, -0.0075, 0.0075,  //
      height / 2, height / 2, -height / 2, -height / 2;
    geom.createTDSF({.a = 0.0325, .b = 0.0325}, {.vert = vert}, 500);
    geom.createMaterial(
      {.rgba = crisp::oklch(0.55f, 0.015f, 255),
       .specular = {0.25f, 0.25f, 0.25f},
       .shininess = 32});
  }
  {
    auto& body = model->addBody(
      {.pos = {0.0, y_offset, 1.5 * height},
       .quat =
         Eigen::Quaterniond(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitX())),
       .mass = mass,
       .inertia =
         {mass * (3 * radius * radius + height * height) / 12.0,
          mass * (3 * radius * radius + height * height) / 12.0,
          mass * radius * radius / 2, 0, 0, 0}},
      false);
    auto& geom = body.addGeom({.mu = mu});
    Eigen::Matrix2Xr vert(2, 4);
    vert << radius, radius, -radius, -radius,  //
      height / 2, -height / 2, height / 2, -height / 2;
    geom.createDSF({.vert = vert}, 500, 0.0001);
    geom.createMaterial(
      {.rgba = crisp::oklch(0.75f, 0.020f, 255),
       .specular = {0.45f, 0.45f, 0.45f},
       .shininess = 64});
  }
  model->cfg().vis.cam.target = {0.0f, 0.0f, 0.05f};
  model->cfg().vis.cam.distance = 0.5f;
  model->cfg().vis.cam.azimuth = 0.0f;
  model->cfg().vis.cam.elevation = -30.0f;
  model->addLight(
    {.type = crisp::light_e::directional,
     .enabled = true,
     .cast_shadow = true,
     .pos = {-1.0f, -2.0f, 3.0f},
     .dir = {1.0f, 2.0f, -3.0f},
     .ambient = {0.3f, 0.3f, 0.3f},
     .diffuse = {0.8f, 0.8f, 0.8f},
     .specular = {1.0f, 1.0f, 1.0f},
     .attenuation = {1.0f, 0.0f, 0.0f},
     .cutoff = 0.0f,
     .exponent = 0.0f});
  model->cfg().opt.erp = 0.01;
  model->cfg().opt.dt = 2e-3;
  model->cfg().opt.solver = crisp::solver_e::canal;
  model->cfg().vis.bg.setOnes();

  auto app = crisp::make_app(model->compile());

  app->init();
  while (app->isOpen()) {
    app->render();
  }

  return 0;
}
