#include <crisp/crisp.hpp>

#include <numbers>

inline double PI = std::numbers::pi;

double fract(double x) {
  return x - std::floor(x);
}
double sdf_subtraction(double a, double b) {
  return std::max(a, -b);
}
double sdf_intersection(double a, double b) {
  return std::max(a, b);
}
double sdf_union(double a, double b) {
  return std::min(a, b);
}

double bolt_distance(Eigen::Vector3d const& p, double radius_attr) {
  constexpr double screw = 12.0;
  double radius = std::sqrt(p.x() * p.x() + p.y() * p.y()) - radius_attr;
  double sqrt12 = std::sqrt(2.0) / 2.0;

  double azimuth = std::atan2(p.y(), p.x());
  double triangle = std::abs(fract(p.z() * screw - azimuth / PI / 2.0) - 0.5);
  double thread = (radius - triangle / screw) * sqrt12;

  double bolt = sdf_subtraction(thread, 0.5 - std::abs(p.z() + 0.5));
  double cone = (p.z() - radius) * sqrt12;

  bolt = sdf_subtraction(bolt, cone + 1.0 * sqrt12);

  double k = 6.0 / PI / 2.0;
  double ang = -std::floor(std::atan2(p.y(), p.x()) * k + 0.5) / k;

  double s0 = std::sin(ang);
  double s1 = std::sin(ang + PI * 0.5);

  double rx = s1 * p.x() - s0 * p.y();
  double ry = s0 * p.x() + s1 * p.y();
  double head = rx - 0.5;

  head = sdf_intersection(head, std::abs(p.z() + 0.25) - 0.25);
  head = sdf_intersection(head, (p.z() + radius - 0.22) * sqrt12);

  return sdf_union(bolt, head);
}

Eigen::Vector3d bolt_gradient(Eigen::Vector3d const& p, double radius_attr) {
  constexpr double eps = 1e-8;
  double d0 = bolt_distance(p, radius_attr);

  Eigen::Vector3d px = p;
  px.x() += eps;
  Eigen::Vector3d py = p;
  py.y() += eps;
  Eigen::Vector3d pz = p;
  pz.z() += eps;

  double dx = (bolt_distance(px, radius_attr) - d0) / eps;
  double dy = (bolt_distance(py, radius_attr) - d0) / eps;
  double dz = (bolt_distance(pz, radius_attr) - d0) / eps;

  return Eigen::Vector3d(dx, dy, dz);
}

double nut_distance(Eigen::Vector3d const& p, double radius_attr) {
  constexpr double screw = 12.0;
  double radius2 = std::sqrt(p.x() * p.x() + p.y() * p.y()) - radius_attr;
  double sqrt12 = std::sqrt(2.0) / 2.0;

  double azimuth = std::atan2(p.y(), p.x());
  double triangle = std::abs(fract(p.z() * screw - azimuth / PI / 2.0) - 0.5);
  double thread2 = (radius2 - triangle / screw) * sqrt12;

  double cone2 = (p.z() - radius2) * sqrt12;

  double hole = sdf_subtraction(thread2, cone2 + 0.5 * sqrt12);
  hole = sdf_union(hole, -cone2 - 0.05 * sqrt12);

  double k = 6.0 / PI / 2.0;
  double ang = -std::floor(std::atan2(p.y(), p.x()) * k + 0.5) / k;

  double s0 = std::sin(ang);
  double s1 = std::sin(ang + PI * 0.5);

  double rx = s1 * p.x() - s0 * p.y();
  double ry = s0 * p.x() + s1 * p.y();

  double head = rx - 0.5;

  head = sdf_intersection(head, std::abs(p.z() + 0.25) - 0.25);
  head = sdf_intersection(head, (p.z() + radius2 - 0.22) * sqrt12);

  return sdf_subtraction(head, hole);
}

Eigen::Vector3d nut_gradient(Eigen::Vector3d const& p, double radius_attr) {
  constexpr double eps = 1e-8;
  double d0 = nut_distance(p, radius_attr);

  Eigen::Vector3d px = p;
  px.x() += eps;
  Eigen::Vector3d py = p;
  py.y() += eps;
  Eigen::Vector3d pz = p;
  pz.z() += eps;

  double dx = (nut_distance(px, radius_attr) - d0) / eps;
  double dy = (nut_distance(py, radius_attr) - d0) / eps;
  double dz = (nut_distance(pz, radius_attr) - d0) / eps;

  return Eigen::Vector3d(dx, dy, dz);
}

int main() {
  int bolt_sdf = crisp::register_sdf(
    [&](Eigen::Vector3r const& x, Eigen::VectorXr const& p) {
      double radius_attr = static_cast<double>(p[0]);
      Eigen::Vector3d xd = x.template cast<double>();

      double dist = bolt_distance(xd, radius_attr);
      Eigen::Vector3d g = bolt_gradient(xd, radius_attr);

      double gn = g.norm();
      Eigen::Vector3d n = Eigen::Vector3d::Zero();
      if (gn > 1e-12) {
        n = g / gn;
      }

      Eigen::Vector3r grad =
        n.template cast<typename Eigen::Vector3r::Scalar>();

      return crisp::signed_distance_t {
        static_cast<typename Eigen::Vector3r::Scalar>(dist), grad};
    },
    [](Eigen::Matrix3r const&, Eigen::VectorXr const&) {
      return Eigen::Vector6r {-0.5, -0.6, -1.0, 0.5, 0.6, 1.0};
    },
    1);

  int nut_sdf = crisp::register_sdf(
    [&](Eigen::Vector3r const& x, Eigen::VectorXr const& p) {
      double radius_attr = static_cast<double>(p[0]);
      Eigen::Vector3d xd = x.template cast<double>();

      double dist = nut_distance(xd, radius_attr);
      Eigen::Vector3d g = nut_gradient(xd, radius_attr);

      double gn = g.norm();
      Eigen::Vector3d n = Eigen::Vector3d::Zero();
      if (gn > 1e-12) {
        n = g / gn;
      }

      Eigen::Vector3r grad =
        n.template cast<typename Eigen::Vector3r::Scalar>();

      return crisp::signed_distance_t {
        static_cast<typename Eigen::Vector3r::Scalar>(dist), grad};
    },
    [](Eigen::Matrix3r const&, Eigen::VectorXr const&) {
      return Eigen::Vector6r {-0.5, -0.5774, -0.5, 0.5, 0.5774, 0.5};
    },
    1);

  auto model = crisp::make_model();
  {
    auto& body = model->addBody(
      {.pos = {0, 0, 0},
       .quat = {0, 1, 0, 0},
       .mass = 461.829,
       .inertia = {61.52395774, 38.72903532, 38.71283906, 0, 0, 0}},
      true);
    auto& geom = body.addGeom({.mu = 0.0});
    geom.createSDF(
      {.type = bolt_sdf, .param = (Eigen::VectorXr(1) << 0.26).finished()});
    geom.createMaterial(
      {.rgba = crisp::oklch(0.74f, 0.02f, 255),
       .emission = {0.0f, 0.0f, 0.0f},
       .specular = {0.45f, 0.45f, 0.45f},
       .shininess = 64});
  }
  {
    auto& body = model->addBody(
      {.pos = {-0.0012496, 0.00329058, 0.830362},
       .quat = {-0.000212626, 0.999996, -0.00200453, 0.00185878},
       .mass = 572.926,
       .inertia = {70.53488747, 70.51978074, 64.78383615, 0, 0, 0}},
      false);
    auto& geom = body.addGeom({.visual = true, .mu = 0.0});
    constexpr double tol = 1000e-6;
    geom.createSDF(
      {.type = nut_sdf,
       .param = (Eigen::VectorXr(1) << 0.26 + tol / 2).finished()});
    geom.createMaterial(
      {.rgba = crisp::oklch(0.54f, 0.015f, 255),
       .emission = {0.0f, 0.0f, 0.0f},
       .specular = {0.35f, 0.35f, 0.35f},
       .shininess = 48});
  }
  model->cfg().vis.cam.target = {0.0f, 0.0f, 0.5f};
  model->cfg().vis.cam.distance = 5;
  model->cfg().vis.cam.azimuth = 0;
  model->cfg().vis.cam.elevation = -30;
  model->cfg().vis.cam.fovy = 60;
  model->cfg().vis.gnd.grid_size *= 10;
  model->cfg().vis.gnd.line_width *= 10;
  model->getGround()->cfg().width = 100;
  model->getGround()->cfg().height = 100;
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
  model->cfg().opt.solver_max_iter = 1000;
  model->cfg().opt.solver = crisp::solver_e::canal;
  model->cfg().ncon_max = 200;

  auto app = crisp::make_app(model->compile());

  app->init();
  while (app->isOpen()) {
    app->render();
  }

  return 0;
}
