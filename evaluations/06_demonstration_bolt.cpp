#include <crisp/crisp.hpp>

#include <fstream>
#include <numbers>
#include <sstream>

inline double PI = std::numbers::pi;

std::vector<Eigen::VectorXr> txt2traj(std::string const& c, int length) {
  std::vector<Eigen::VectorXr> data;
  Eigen::VectorXr tmp(length);
  std::ifstream file;
  std::string line;
  file.open(c.c_str());
  while (std::getline(file, line)) {
    tmp.setZero();
    std::stringstream ss(line);
    std::string cell;
    int count = 0;
    while (std::getline(ss, cell, ',')) {
      tmp[count++] = std::stod(cell);
    }
    data.push_back(tmp);
  }
  file.close();
  return data;
}

std::vector<Eigen::Vector3r> xyz = {
  Eigen::Vector3r(0, 0, 0.333),
  Eigen::Vector3r(0, 0, 0),
  Eigen::Vector3r(0, -0.316, 0),
  Eigen::Vector3r(0.0825, 0, 0),
  Eigen::Vector3r(-0.0825, 0.384, 0),
  Eigen::Vector3r(0, 0, 0),
  Eigen::Vector3r(0.088, 0, 0),
  Eigen::Vector3r(0, 0, 0.107),
  Eigen::Vector3r(0, 0, 0)};

std::vector<Eigen::Vector3r> rpy = {
  Eigen::Vector3r(0, 0, 0),       Eigen::Vector3r(-PI / 2, 0, 0),
  Eigen::Vector3r(PI / 2, 0, 0),  Eigen::Vector3r(PI / 2, 0, 0),
  Eigen::Vector3r(-PI / 2, 0, 0), Eigen::Vector3r(PI / 2, 0, 0),
  Eigen::Vector3r(PI / 2, 0, 0),  Eigen::Vector3r(0, 0, 0),
  Eigen::Vector3r(0, 0, 0)};

std::vector<int> ax = {3, 3, 3, 3, 3, 3, 3, 0, 0};

Eigen::Matrix3r rot(double th, char axis) {
  Eigen::Matrix3r R;
  if (axis == 'x') {
    R << 1, 0, 0, 0, cos(th), -sin(th), 0, sin(th), cos(th);
  } else if (axis == 'y') {
    R << cos(th), 0, sin(th), 0, 1, 0, -sin(th), 0, cos(th);
  } else {
    R << cos(th), -sin(th), 0, sin(th), cos(th), 0, 0, 0, 1;
  }
  return R;
}

Eigen::Vector3r unskew(Eigen::Matrix3r const& S) {
  return Eigen::Vector3r(S(2, 1), S(0, 2), S(1, 0));
}

Eigen::Matrix4r get_rot(
  Eigen::Vector3r const& rpy, Eigen::Vector3r const& xyz) {
  Eigen::Matrix3r R = rot(rpy[2], 'z') * rot(rpy[1], 'y') * rot(rpy[0], 'x');
  Eigen::Matrix4r SE3 = Eigen::Matrix4r::Identity();
  SE3.block<3, 3>(0, 0) = R;
  SE3.block<3, 1>(0, 3) = xyz;
  return SE3;
}

Eigen::Matrix4r get_rot(double th, int ax) {
  Eigen::Matrix4r SE3_matrix = Eigen::Matrix4r::Identity();
  if (ax == 1) {
    SE3_matrix.block<3, 3>(0, 0) = rot(th, 'x');
  } else if (ax == 2) {
    SE3_matrix.block<3, 3>(0, 0) = rot(th, 'y');
  } else if (ax == 3) {
    SE3_matrix.block<3, 3>(0, 0) = rot(th, 'z');
  }
  return SE3_matrix;
}

Eigen::Matrix4r get_trans(Eigen::Vector3r const& pos) {
  Eigen::Matrix4r SE3 = Eigen::Matrix4r::Identity();
  SE3.block<3, 1>(0, 3) = pos;
  return SE3;
}

Eigen::Matrix4r fk_ee(
  Eigen::VectorXr const& q, Eigen::Matrix3r const& R_base,
  Eigen::Vector3r const& p_base) {
  std::vector<double> qt = {q(0), q(1), q(2), q(3), q(4), q(5), q(6), 0, 0};

  Eigen::Matrix4r SE3 = Eigen::Matrix4r::Identity();
  SE3.block<3, 3>(0, 0) = R_base;
  SE3.block<3, 1>(0, 3) = p_base;

  for (int i = 0; i < 9; ++i) {
    SE3 = SE3 * get_rot(rpy[i], xyz[i]) * get_rot(qt[i], ax[i]);
  }

  return SE3;
}

Eigen::MatrixXr jaco_ee(
  Eigen::VectorXr const& q, Eigen::Matrix3r const& R_base,
  Eigen::Vector3r const& p_base) {
  std::vector<double> qt = {q(0), q(1), q(2), q(3), q(4), q(5), q(6), 0, 0};

  Eigen::Matrix4r SE3 = Eigen::Matrix4r::Identity();
  SE3.block<3, 3>(0, 0) = R_base;
  SE3.block<3, 1>(0, 3) = p_base;

  std::vector<Eigen::Matrix4r> T;
  T.push_back(SE3);

  for (int i = 0; i < 9; ++i) {
    SE3 = T[i] * get_rot(rpy[i], xyz[i]) * get_rot(qt[i], ax[i]);
    T.push_back(SE3);
  }

  Eigen::MatrixXr J(6, 7);
  J.setZero();
  for (int col = 0; col < 7; ++col) {
    Eigen::Vector3r tmp = T[col + 1].block<3, 1>(0, ax[col] - 1);
    Eigen::Vector3r tmp2 = SE3.block<3, 1>(0, 3) - T[col + 1].block<3, 1>(0, 3);
    tmp = tmp.cross(tmp2);
    J.block<3, 1>(0, col) = tmp;
    J.block<3, 1>(3, col) = T[col + 1].block<3, 1>(0, ax[col] - 1);
  }

  return J;
}

void ik_ee(
  Eigen::VectorXr& q, Eigen::Matrix3r const& R_base,
  Eigen::Vector3r const& p_base, Eigen::Vector3r const& p_t,
  Eigen::Matrix3r const& R_t, Eigen::VectorXr const& q_l,
  Eigen::VectorXr const& q_u, Eigen::VectorXr const& Kdiag) {
  Eigen::Matrix4r T;
  Eigen::MatrixXr J(6, 7);
  Eigen::Matrix3r R_e;
  Eigen::Vector3r p_e;
  Eigen::VectorXr dx(6);
  Eigen::VectorXr dq(q.size());

  for (int i = 0; i < 300; ++i) {
    T = fk_ee(q, R_base, p_base);
    J = jaco_ee(q, R_base, p_base);
    R_e = T.block<3, 3>(0, 0);
    p_e = T.block<3, 1>(0, 3);

    dx.segment(0, 3) = p_e - p_t;
    dx.segment(3, 3) =
      R_e * unskew(0.5 * (R_t.transpose() * R_e - R_e.transpose() * R_t));

    dq = J.transpose() * Kdiag.asDiagonal() * dx;

    q = q - dq / 100;

    for (int j = 0; j < q.size(); ++j) {
      q[j] = std::max(std::min(q[j], q_u[j]), q_l[j]);
    }
  }
}

void my_control(
  Eigen::Ref<Eigen::VectorXr> act_in, Eigen::VectorXr const& q_cur, double time,
  Eigen::VectorXr const& q0, Eigen::Matrix3r const& R_base,
  Eigen::Vector3r const& p_base, Eigen::Vector3r const& p_t,
  Eigen::Matrix3r const& R_t, Eigen::VectorXr const& q_l,
  Eigen::VectorXr const& q_u) {
  act_in.setZero();

  if (time < 16) {
    Eigen::VectorXr Kdiag(6);
    Kdiag << 10, 10, 10, 10, 10, 10;

    Eigen::VectorXr q = q_cur.head<7>();
    ik_ee(q, R_base, p_base, p_t, R_t, q_l, q_u, Kdiag);
    act_in.head<7>() = q0 + (q - q0) * std::min(time / 15, 1.0);
  }

  else {
    Eigen::VectorXr Kdiag(6);
    Kdiag << 1, 1, 10, 10, 10, 10;

    Eigen::Matrix4r T = fk_ee(q_cur, R_base, p_base);
    Eigen::Vector3r p_t2 = T.block(0, 3, 3, 1) - Eigen::Vector3r(0, 0, 0.002);
    p_t2[0] = p_t[0];
    p_t2[1] = p_t[1];

    Eigen::Matrix3r R_t2 = R_t;

    Eigen::VectorXr q = q_cur.head<7>();

    ik_ee(q, R_base, p_base, p_t2, R_t2, q_l, q_u, Kdiag);
    act_in.head<7>() = q;
  }

  if (time > 17) {
    act_in(7) = q_cur[7] + PI / 6;
    if (q_cur[7] >= 4 * PI) {
      act_in(7) = q_cur[7];
    }
  }
}

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
  constexpr double screw = 1.0 / 0.005;
  double radius = std::sqrt(p.x() * p.x() + p.y() * p.y()) - radius_attr;
  double sqrt12 = std::sqrt(2.0) / 2.0;

  double azimuth = std::atan2(p.y(), p.x());
  double triangle = std::abs(fract(p.z() * screw - azimuth / PI / 2.0) - 0.5);
  double thread = (radius - triangle / screw) * sqrt12;

  double bolt = sdf_subtraction(thread, 0.02 - std::abs(p.z() + 0.02));
  double cone = (p.z() - radius) * sqrt12;

  bolt = sdf_subtraction(bolt, cone + 1.0 * sqrt12);

  double k = 6.0 / PI / 2.0;
  double ang = -std::floor(std::atan2(p.y(), p.x()) * k + 0.5) / k;

  double s0 = std::sin(ang);
  double s1 = std::sin(ang + PI * 0.5);

  double rx = s1 * p.x() - s0 * p.y();
  double ry = s0 * p.x() + s1 * p.y();
  double head = rx - 0.028;

  head = sdf_intersection(head, std::abs(p.z() + 0.0125) - 0.008);
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

int main() {
  int bolt_sdf_type = crisp::register_sdf(
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
      return Eigen::Vector6r {-0.035, -0.035, -0.05, 0.035, 0.035, 0.05};
    },
    1);

  auto model = crisp::make_model();
  model->addRobot(
    {.path = "assets/panda/panda_hebi_demo.urdf"});
  {
    auto& body = model->addBody(
      {.pos = {0.5, -0.025, 0.015 - 0.011},
       .quat = {0, 1, 0, 0},
       .mass = 461.829,
       .inertia = {61.52395774, 38.72903532, 38.71283906, 0, 0, 0}},
      true);
    auto& geom = body.addGeom({.visual = true});
    double tol = 2000e-6;
    geom.createSDF(
      {.type = bolt_sdf_type,
       .param =
         (Eigen::VectorXr(1) << 41.86565 / 1000 / 2 - tol / 2).finished()});
    geom.createMaterial(
      {.rgba = crisp::oklch(0.74f, 0.02f, 255),
       .emission = {0.0f, 0.0f, 0.0f},
       .specular = {0.45f, 0.45f, 0.45f},
       .shininess = 64});
  }
  {
    // table
    auto& body = model->addBody({.pos = {0.5, -0.025, 0}}, true);
    auto& geom = body.addGeom(
      {.type = crisp::geometry_e::box,
       .size = crisp::make_box_size(0.1, 0.1, 0.022),
       .mu = 0.1});
    geom.createMaterial(
      {.rgba = crisp::oklch(0.80f, 0.008f, 90),
       .emission = {0.0f, 0.0f, 0.0f},
       .specular = {0.04f, 0.04f, 0.04f},
       .shininess = 16});
  }
  model->cfg().vis.cam.target = {0.3f, 0.0f, 0.3f};
  model->cfg().vis.cam.azimuth = 140.0f;
  model->cfg().vis.cam.elevation = 0.0f;
  model->cfg().vis.cam.fovy = 45.0f;
  model->addLight(
    {.type = crisp::light_e::directional,
     .enabled = true,
     .cast_shadow = true,
     .pos = {3.0f, -2.0f, 1.0f},
     .dir = {-3.0f, 2.0f, -1.0f},
     .ambient = {0.1f, 0.1f, 0.1f},
     .diffuse = {0.4f, 0.4f, 0.4f},
     .specular = {0.5f, 0.5f, 0.5f},
     .attenuation = {1.0f, 0.0f, 0.0f},
     .cutoff = 0.0f,
     .exponent = 0.0f});
  model->addLight(
    {.type = crisp::light_e::directional,
     .enabled = true,
     .cast_shadow = true,
     .pos = {1.0f, -2.0f, 3.0f},
     .dir = {-1.0f, 2.0f, -3.0f},
     .ambient = {0.0f, 0.0f, 0.0f},
     .diffuse = {0.8f, 0.8f, 0.8f},
     .specular = {0.3f, 0.3f, 0.3f},
     .attenuation = {1.0f, 0.0f, 0.0f},
     .cutoff = 0.0f,
     .exponent = 0.0f});
  model->cfg().opt.gravity.setZero();
  model->cfg().opt.solver_max_iter = 128;
  model->cfg().opt.solver = crisp::solver_e::canal;

  for (int i = 0; i < 7; ++i) {
    model->addActuator(
      {.type = crisp::actuator_e::position,
       .jointid = i + 1,
       .gain = {0, 1000, 100}});
  }
  model->addActuator(
    {.type = crisp::actuator_e::position, .jointid = 9, .gain = {0, 50, 5}});

  auto m = model->compile();
  Eigen::VectorXr q0(8);
  q0 << 0.00000, -0.06313, 0.00000, -2.18291, -0.00000, 2.12035, -0.00000, 0;
  m->state.q0() = q0;
  m->act.u0() << 0, 0, 0, 0, 0, 0, 0, 0;

  auto app = crisp::make_app(std::move(m));

  auto u_traj = txt2traj("assets/boltnut/boltnut_q_13.csv", 8);
  app->data().act.u() = u_traj[0];

  Eigen::VectorXr q_l(8);
  q_l << -PI, -PI, -PI, -PI, -PI, -PI, -1e+10, -1e+10;
  Eigen::VectorXr q_u(8);
  q_u << PI, PI, PI, PI, PI, PI, 1e+10, 1e+10;

  app->setControl([&](auto const& m, auto const& d, auto u, auto udot) {
    my_control(
      u, d.state.q(), d.world.time, q0, Eigen::Matrix3r::Identity(),
      Eigen::Vector3r(0, 0, 0.011),
      Eigen::Vector3r(0.5 + 0.00, -0.025 + 0.00, 0.015 + 0.155), rot(PI, 'x'),
      q_l, q_u);
  });

  app->init();
  while (app->isOpen()) {
    app->render();
  }

  return 0;
}
