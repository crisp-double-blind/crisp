#include <crisp/crisp.hpp>

#include <fstream>
#include <numbers>

double Fract(double x) {
  return x - std::floor(x);
}
double Subtraction(double a, double b) {
  return std::max(a, -b);
}
double Intersection(double a, double b) {
  return std::max(a, b);
}
double Union(double a, double b) {
  return std::min(a, b);
}

double bolt_distance(const Eigen::Vector3d& p, double radius_attr) {
  constexpr double screw = 12.0;
  const double radius = std::sqrt(p.x() * p.x() + p.y() * p.y()) - radius_attr;
  const double sqrt12 = std::sqrt(2.0) / 2.0;

  const double azimuth = std::atan2(p.y(), p.x());
  const double triangle =
    std::abs(Fract(p.z() * screw - azimuth / std::numbers::pi / 2.0) - 0.5);
  const double thread = (radius - triangle / screw) * sqrt12;

  double bolt = Subtraction(thread, 0.5 - std::abs(p.z() + 0.5));
  const double cone = (p.z() - radius) * sqrt12;

  bolt = Subtraction(bolt, cone + 1.0 * sqrt12);

  const double k = 6.0 / std::numbers::pi / 2.0;
  const double ang = -std::floor(std::atan2(p.y(), p.x()) * k + 0.5) / k;

  const double s0 = std::sin(ang);
  const double s1 = std::sin(ang + std::numbers::pi * 0.5);

  const double rx = s1 * p.x() - s0 * p.y();
  const double ry = s0 * p.x() + s1 * p.y();
  double head = rx - 0.5;

  head = Intersection(head, std::abs(p.z() + 0.25) - 0.25);
  head = Intersection(head, (p.z() + radius - 0.22) * sqrt12);

  return Union(bolt, head);
}

Eigen::Vector3d bolt_gradient_fd(const Eigen::Vector3d& p, double radius_attr) {
  const double eps = 1e-8;
  const double d0 = bolt_distance(p, radius_attr);

  Eigen::Vector3d px = p;
  px.x() += eps;
  Eigen::Vector3d py = p;
  py.y() += eps;
  Eigen::Vector3d pz = p;
  pz.z() += eps;

  const double dx = (bolt_distance(px, radius_attr) - d0) / eps;
  const double dy = (bolt_distance(py, radius_attr) - d0) / eps;
  const double dz = (bolt_distance(pz, radius_attr) - d0) / eps;

  return Eigen::Vector3d(dx, dy, dz);
}

double nut_distance(const Eigen::Vector3d& p, double radius_attr) {
  constexpr double screw = 12.0;
  const double radius2 = std::sqrt(p.x() * p.x() + p.y() * p.y()) - radius_attr;
  const double sqrt12 = std::sqrt(2.0) / 2.0;

  const double azimuth = std::atan2(p.y(), p.x());
  const double triangle =
    std::abs(Fract(p.z() * screw - azimuth / std::numbers::pi / 2.0) - 0.5);
  const double thread2 = (radius2 - triangle / screw) * sqrt12;

  const double cone2 = (p.z() - radius2) * sqrt12;

  double hole = Subtraction(thread2, cone2 + 0.5 * sqrt12);
  hole = Union(hole, -cone2 - 0.05 * sqrt12);

  const double k = 6.0 / std::numbers::pi / 2.0;
  const double ang = -std::floor(std::atan2(p.y(), p.x()) * k + 0.5) / k;

  const double s0 = std::sin(ang);
  const double s1 = std::sin(ang + std::numbers::pi * 0.5);

  const double rx = s1 * p.x() - s0 * p.y();
  const double ry = s0 * p.x() + s1 * p.y();

  double head = rx - 0.5;

  head = Intersection(head, std::abs(p.z() + 0.25) - 0.25);
  head = Intersection(head, (p.z() + radius2 - 0.22) * sqrt12);

  return Subtraction(head, hole);
}

Eigen::Vector3d nut_gradient_fd(const Eigen::Vector3d& p, double radius_attr) {
  const double eps = 1e-8;
  const double d0 = nut_distance(p, radius_attr);

  Eigen::Vector3d px = p;
  px.x() += eps;
  Eigen::Vector3d py = p;
  py.y() += eps;
  Eigen::Vector3d pz = p;
  pz.z() += eps;

  const double dx = (nut_distance(px, radius_attr) - d0) / eps;
  const double dy = (nut_distance(py, radius_attr) - d0) / eps;
  const double dz = (nut_distance(pz, radius_attr) - d0) / eps;

  return Eigen::Vector3d(dx, dy, dz);
}

int main() {
  // crisp::__logger__->set_level(spdlog::level::debug);
  auto model = crisp::make_model();

  int bolt_sdf_type = crisp::register_sdf(
    [&](Eigen::Vector3r const& x, Eigen::VectorXr const& p) {
      const double radius_attr = static_cast<double>(p[0]);
      Eigen::Vector3d xd = x.template cast<double>();

      const double dist = bolt_distance(xd, radius_attr);
      Eigen::Vector3d g = bolt_gradient_fd(xd, radius_attr);

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
      // return Eigen::Vector6r {-0.5, -0.5731, -1.0, 0.5, 0.5731, 1.0};
      return Eigen::Vector6r {-0.5, -0.6, -1.0, 0.5, 0.6, 1.0};
    },
    1);

  int nut_sdf_type = crisp::register_sdf(
    [&](Eigen::Vector3r const& x, Eigen::VectorXr const& p) {
      const double radius_attr = static_cast<double>(p[0]);
      Eigen::Vector3d xd = x.template cast<double>();

      const double dist = nut_distance(xd, radius_attr);
      Eigen::Vector3d g = nut_gradient_fd(xd, radius_attr);

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

  {
    auto& body = model->addBody(
      {.pos = {0, 0, 0},
       .quat = {0, 1, 0, 0},
       .mass = 461.829,
       .inertia = {61.52395774, 38.72903532, 38.71283906, 0, 0, 0}},
      true);
    auto& geom = body.addGeom({.mu = 0.0});
    geom.createSDF(
      {.type = bolt_sdf_type,
       .param = (Eigen::VectorXr(1) << 0.26).finished()});
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

    // geom.createMesh({.path = "assets/Nut_blender/nut_500.obj"});
    double tol = 1000e-6;
    geom.createSDF(
      {.type = nut_sdf_type,
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
  model->cfg().vis.bg.setZero();

  model->cfg().ncon_max = 200;

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

  model->getGround()->cfg().width = 100;
  model->getGround()->cfg().height = 100;
  model->cfg().vis.gnd.grid_size *= 10;
  model->cfg().vis.gnd.line_width *= 10;
  model->cfg().vis.bg.setOnes();

  auto m = model->compile();

  m->opt.solver_max_iter = 1000;
  m->opt.solver = crisp::solver_e::sub_admm;

  auto app = crisp::make_app(std::move(m), false);

  // auto logger = std::ofstream("crisp_boltnut_sdf_subadmm_500_.csv");
  // logger << "time,x,y,z,contact_id,dist,penetration,px,py,pz,nx,ny,nz\n";
  // app->addPostStepHook([&](crisp::model_t const& m, crisp::data_t const& d) {
  //   double d_max = 0;
  //   for (int i = 0; i < d.size.ncon; ++i) {
  //     d_max = std::min(d_max, d.con.feature[i].gap);
  //     logger << d.world.time << "," << d.body.pos[1][0] << ","
  //            << d.body.pos[1][1] << "," << d.body.pos[1][2] << "," << i <<
  //            ","
  //            << d.con.feature[i].gap << ","
  //            << (d.con.feature[i].gap < 0 ? -d.con.feature[i].gap : 0)
  //            << ",0,0,0,0,0,0\n";
  //   }
  //   crisp::__logger__->info("nut penetration: {:.6f}", d_max * 1e6);
  // });

  app->init();
  while (app->isOpen()) {
    app->render();
    // if (app->data().body.pos[1][2] < 0.6 || app->data().world.time > 30)
    // break;
  }
  return 0;
}
