#include <crisp/crisp.hpp>

#include <fstream>

#include "utils/math.hpp"

int main() {
  constexpr double slope_deg = 30;
  constexpr double cone_deg = 25;

  constexpr double cube_mass = 1;
  constexpr double cube_length = 0.1;
  constexpr double pos = -2;
  constexpr double offset = 0.1;

  auto model = crisp::make_model();
  {
    auto& body =
      model->addBody({.quat = utils::aa2quat(slope_deg, {1.0, 0.0, 0.0})});
    auto& geom = body.addGeom(
      {.type = crisp::geometry_e::plane,
       .size = crisp::make_plane_size(1, 10),
       .mu = utils::tand(cone_deg)});
    geom.createMaterial({.rgba = {crisp::oklch(0.9f, 0.02f, 270)}});
  }
  {
    double inertia = cube_mass * 1e-3;
    auto& body = model->addBody(
      {.pos =
         {0,
          -pos * utils::cosd(slope_deg) -
            (cube_length / 2 + offset) * utils::sind(slope_deg),
          -pos * utils::sind(slope_deg) +
            (cube_length / 2 + offset) * utils::cosd(slope_deg)},
       .quat = utils::aa2quat(slope_deg, {1.0, 0.0, 0.0}),
       .mass = cube_mass,
       .inertia = {inertia, inertia, inertia, 0, 0, 0}},
      false);
    auto& geom = body.addGeom(
      {.type = crisp::geometry_e::box,
       .size = crisp::make_box_size(cube_length, cube_length, cube_length),
       .mu = utils::tand(cone_deg)});
    geom.createMaterial({.rgba = crisp::oklch(0.8f, 0.1f, 150)});
  }
  model->cfg().vis.cam.target = {0.0f, 0.3f, 0.3f};
  model->cfg().vis.cam.distance = 1.5f;
  model->cfg().vis.cam.azimuth = 45.0f;
  model->cfg().vis.cam.elevation = -10.0f;
  model->cfg().vis.gnd.grid_size = 1.0f;
  model->cfg().vis.bg.setOnes();
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
  // model->cfg().opt.dt = 1e-3;
  model->cfg().opt.margin[2] = 0.1;
  model->cfg().opt.solver = crisp::solver_e::canal;

  auto m = model->compile();

  auto logger = std::ofstream(".log/crisp_stick_canal.csv");
  logger << "time,x,y,z\n";
  auto app = crisp::make_app(std::move(m));
  app->addPostStepHook([&](crisp::model_t const& m, crisp::data_t const& d) {
    logger << d.world.time << "," << d.body.pos[1][0] << "," << d.body.pos[1][1]
           << "," << d.body.pos[1][2] << "\n";
  });

  app->init();
  while (app->isOpen()) {
    app->render();
    if (app->data().world.time > 10.0) break;
  }

  return 0;
}
