#include <crisp/crisp.hpp>

#include "utils/math.hpp"

int main() {
  constexpr double mass_ratio = 10;

  constexpr double box_mass = 1;
  constexpr double box_length = 0.1;
  constexpr double pos_offset = 0.05;
  constexpr double rot_offset = 0;  // deg
  constexpr double mu = 5;

  auto model = crisp::make_model();
  model->getGround()->cfg().mu = mu;
  {
    double mass = box_mass / mass_ratio;
    double inertia = mass * 1e-3;
    auto& body = model->addBody(
      {.pos = {0.0, 0.0, box_length * 0.5},
       .mass = mass,
       .inertia = {inertia, inertia, inertia, 0, 0, 0}},
      false);
    auto& geom = body.addGeom(
      {.type = crisp::geometry_e::box,
       .size = crisp::make_box_size(box_length, box_length, box_length),
       .mu = mu});
    geom.createMaterial({.rgba = crisp::oklch(0.8f, 0.1f, 250)});
  }
  {
    double mass = box_mass;
    double inertia = mass * 1e-3;
    auto& body = model->addBody(
      {.pos = {0.0, 0.0, box_length * 1.5},
       .mass = mass,
       .inertia = {inertia, inertia, inertia, 0, 0, 0}},
      false);
    auto& geom = body.addGeom(
      {.type = crisp::geometry_e::box,
       .size = crisp::make_box_size(box_length, box_length, box_length),
       .mu = mu});
    geom.createMaterial({.rgba = crisp::oklch(0.6f, 0.1f, 250)});
  }
  {
    double mass = box_mass * mass_ratio;
    double inertia = mass * 1e-3;
    auto& body = model->addBody(
      {.pos = {0.0, 0.0, box_length * 2.5 + pos_offset},
       .quat = utils::aa2quat(rot_offset, {1.0, 0.0, 0.0}),
       .mass = mass,
       .inertia = {inertia, inertia, inertia, 0, 0, 0}},
      false);
    auto& geom = body.addGeom(
      {.type = crisp::geometry_e::box,
       .size = crisp::make_box_size(box_length, box_length, box_length),
       .mu = mu});
    geom.createMaterial({.rgba = crisp::oklch(0.4f, 0.1f, 250)});
  }
  model->cfg().vis.cam.target = {0.0f, 0.0f, 0.15f};
  model->cfg().vis.cam.distance = 0.5f;
  model->cfg().vis.cam.azimuth = -15.0f;
  model->cfg().vis.cam.elevation = -15.0f;
  model->addLight(
    {.type = crisp::light_e::directional,
     .enabled = true,
     .cast_shadow = true,
     .pos = {-2.0f, 1.0f, 3.0f},
     .dir = {2.0f, -1.0f, -3.0f},
     .ambient = {0.3f, 0.3f, 0.3f},
     .diffuse = {0.8f, 0.8f, 0.8f},
     .specular = {1.0f, 1.0f, 1.0f},
     .attenuation = {1.0f, 0.0f, 0.0f},
     .cutoff = 0.0f,
     .exponent = 0.0f});
  model->cfg().opt.dt = 1e-3;
  model->cfg().opt.erp = 1e-3;
  model->cfg().opt.solver = crisp::solver_e::canal;

  auto app = crisp::make_app(model->compile());

  app->init();
  while (app->isOpen()) {
    app->render();
  }

  return 0;
}
