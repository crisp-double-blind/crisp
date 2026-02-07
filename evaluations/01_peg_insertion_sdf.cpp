#include <crisp/crisp.hpp>

#include <numbers>
#include <algorithm>

int main() {
  crisp::register_sdf(
    [](Eigen::Vector3r const& x, Eigen::VectorXr const& p) {
      double inner_r = p[0];
      double outer_r = p[1];
      double half_h = p[2] / 2;

      double center_r = (inner_r + outer_r) / 2;
      double half_w = (outer_r - inner_r) / 2;

      double u = std::sqrt(x[0] * x[0] + x[1] * x[1]);
      double qx = u - center_r;
      double qy = x[2];

      double phi = 0.0;
      Eigen::Vector3r grad;
      grad.setZero();
      if (std::abs(qx) > half_w && std::abs(qy) > half_h) {
        phi = std::sqrt(
          (std::abs(qx) - half_w) * (std::abs(qx) - half_w) +
          (std::abs(qy) - half_h) * (std::abs(qy) - half_h));
        grad[0] = (qx - (qx > 0 ? half_w : -half_w)) * (x[0] / u) / phi;
        grad[1] = (qx - (qx > 0 ? half_w : -half_w)) * (x[1] / u) / phi;
        grad[2] = (qy - (qy > 0 ? half_h : -half_h)) / phi;
      } else {
        phi = std::max(
          std::max(std::abs(qx) - half_w, std::abs(qy) - half_h),
          std::max(-std::abs(qx) - half_w, -std::abs(qy) - half_h));
        if (std::abs(qx) - half_w >= std::abs(qy) - half_h) {
          grad[0] = (qx > 0 ? 1 : -1) * (x[0] / u);
          grad[1] = (qx > 0 ? 1 : -1) * (x[1] / u);
          grad[2] = 0;
        } else {
          grad[0] = 0;
          grad[1] = 0;
          grad[2] = (qy > 0 ? 1 : -1);
        }
      }

      return crisp::signed_distance_t {phi, grad};
    },
    [](Eigen::Matrix3r const&, Eigen::VectorXr const& p) {
      double margin = 0.01;
      return (Eigen::Vector6r() << -(p[1] + margin), -(p[1] + margin),
              -(p[2] / 2 + margin), p[1] + margin, p[1] + margin,
              p[2] / 2 + margin)
        .finished();
    },
    3);

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
    geom.createSDF(
      {.type = 0,
       .param =
         (Eigen::VectorXr(3) << radius + eps / 2, 0.040, height).finished()});
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
    auto& geom = body.addGeom(
      {.type = crisp::geometry_e::cylinder,
       .size = crisp::make_cylinder_size(radius, height),
       .mu = mu});
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
