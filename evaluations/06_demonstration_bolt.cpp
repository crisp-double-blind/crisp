#include <crisp/crisp.hpp>

#include <fstream>
#include <sstream>
#include <iostream>

std::vector<Eigen::VectorXr> txt2traj(std::string c, int length) {
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

Eigen::MatrixXr txt2mat(std::string c, int row, int col) {
  Eigen::MatrixXr data(row, col);
  std::ifstream file;
  file.open(c.c_str());
  for (int i = 0; i < row; i++) {
    for (int j = 0; j < col; j++) {
      file >> data(i, j);
    }
  }
  file.close();
  return data;
}

Eigen::VectorXr txt2vec(std::string c, int length) {
  Eigen::VectorXr data(length);
  std::ifstream file;
  file.open(c.c_str());
  for (int i = 0; i < length; i++) {
    file >> data(i);
  }
  file.close();
  return data;
}

double tanh_diff(double x) {
  double cosh = 0.5 * (std::exp(x) + std::exp(-x));
  return 1.0 / (cosh * cosh);
}

crisp::signed_distance_t NN_sdf(
  Eigen::Vector3r p, const Eigen::MatrixXr& W0, const Eigen::MatrixXr& W1,
  const Eigen::MatrixXr& W2, const Eigen::VectorXr& W3,
  const Eigen::VectorXr& b0, const Eigen::VectorXr& b1,
  const Eigen::VectorXr& b2, double b3, const Eigen::VectorXr& input_coeff,
  const Eigen::VectorXr& output_coeff) {
  p(0) = input_coeff(0) * p(0) + input_coeff(3);
  p(1) = input_coeff(1) * p(1) + input_coeff(4);
  p(2) = input_coeff(2) * p(2) + input_coeff(5);
  Eigen::VectorXr a1 = W0 * p + b0;
  Eigen::VectorXr a2 = W1 * a1.cwiseMax(0.0) + b1;
  Eigen::VectorXr a3 = W2 * a2.cwiseMax(0.0) + b2;
  double a4 = W3.dot(a3.cwiseMax(0.0)) + b3;

  Eigen::VectorXr dh4 = W3 * tanh_diff(a4);
  Eigen::VectorXr dh3 = W2.transpose() *
    a3.binaryExpr(dh4, [](double xi, double yi) { return xi > 0 ? yi : 0.0; });
  Eigen::VectorXr dh2 = W1.transpose() *
    a2.binaryExpr(dh3, [](double xi, double yi) { return xi > 0 ? yi : 0.0; });
  Eigen::VectorXr dh1 = W0.transpose() *
    a1.binaryExpr(dh2, [](double xi, double yi) { return xi > 0 ? yi : 0.0; });

  Eigen::Vector3r grad;
  grad(0) = (dh1(0) / output_coeff(0)) * input_coeff(0);
  grad(1) = (dh1(1) / output_coeff(0)) * input_coeff(1);
  grad(2) = (dh1(2) / output_coeff(0)) * input_coeff(2);
  grad.normalize();

  return crisp::signed_distance_t {
    (tanh(a4) - output_coeff(1)) / output_coeff(0), grad};
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
// Eigen::Vector3r(0, 0, 0.09615) };

double PI = 3.1415;
std::vector<Eigen::Vector3r> rpy = {
  Eigen::Vector3r(0, 0, 0),       Eigen::Vector3r(-PI / 2, 0, 0),
  Eigen::Vector3r(PI / 2, 0, 0),  Eigen::Vector3r(PI / 2, 0, 0),
  Eigen::Vector3r(-PI / 2, 0, 0), Eigen::Vector3r(PI / 2, 0, 0),
  Eigen::Vector3r(PI / 2, 0, 0),  Eigen::Vector3r(0, 0, 0),
  Eigen::Vector3r(0, 0, 0)};

std::vector<int> ax = {3, 3, 3, 3, 3, 3, 3, 0, 0};

Eigen::Matrix3r Rot(const double& th, const char& axis) {
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

Eigen::Vector3r unskew(const Eigen::Matrix3r& S) {
  return Eigen::Vector3r(S(2, 1), S(0, 2), S(1, 0));
}

Eigen::Matrix4r get_rot(
  const Eigen::Vector3r& _rpy, const Eigen::Vector3r& _xyz) {
  Eigen::Matrix3r R = Rot(_rpy[2], 'z') * Rot(_rpy[1], 'y') * Rot(_rpy[0], 'x');
  Eigen::Matrix4r SE3 = Eigen::Matrix4r::Identity();
  SE3.block(0, 0, 3, 3) = R;
  SE3.block(0, 3, 3, 1) = _xyz;
  return SE3;
}

Eigen::Matrix4r SE3_rot(const double th, const int ax) {
  Eigen::Matrix4r SE3_matrix = Eigen::Matrix4r::Identity();
  if (ax == 1) {
    SE3_matrix.block(0, 0, 3, 3) = Rot(th, 'x');
  } else if (ax == 2) {
    SE3_matrix.block(0, 0, 3, 3) = Rot(th, 'y');
  } else if (ax == 3) {
    SE3_matrix.block(0, 0, 3, 3) = Rot(th, 'z');
  }
  return SE3_matrix;
}

Eigen::Matrix4r get_trans(const Eigen::Vector3r& pos) {
  Eigen::Matrix4r SE3 = Eigen::Matrix4r::Identity();
  SE3.block(0, 3, 3, 1) = pos;
  return SE3;
}

Eigen::Matrix4r fk_ee(
  const Eigen::VectorXr& q, const Eigen::Matrix3r& R_base,
  const Eigen::Vector3r& p_base) {
  std::vector<double> qt = {q(0), q(1), q(2), q(3), q(4), q(5), q(6), 0, 0};

  Eigen::Matrix4r SE3 = Eigen::Matrix4r::Identity();
  SE3.block(0, 0, 3, 3) = R_base;
  SE3.block(0, 3, 3, 1) = p_base;

  for (int i = 0; i < 9; ++i) {
    SE3 = SE3 * get_rot(rpy[i], xyz[i]) * SE3_rot(qt[i], ax[i]);
  }

  return SE3;
}

Eigen::MatrixXr jaco_ee(
  const Eigen::VectorXr& q, const Eigen::Matrix3r& R_base,
  const Eigen::Vector3r& p_base) {
  std::vector<double> qt = {q(0), q(1), q(2), q(3), q(4), q(5), q(6), 0, 0};

  Eigen::Matrix4r SE3 = Eigen::Matrix4r::Identity();
  SE3.block(0, 0, 3, 3) = R_base;
  SE3.block(0, 3, 3, 1) = p_base;

  std::vector<Eigen::Matrix4r> T;
  T.push_back(SE3);

  for (int i = 0; i < 9; ++i) {
    SE3 = T[i] * get_rot(rpy[i], xyz[i]) * SE3_rot(qt[i], ax[i]);
    T.push_back(SE3);
  }

  Eigen::MatrixXr J(6, 7);
  J.setZero();
  for (int col = 0; col < 7; ++col) {
    Eigen::Vector3r tmp = T[col + 1].block(0, ax[col] - 1, 3, 1);
    Eigen::Vector3r tmp2 = SE3.block(0, 3, 3, 1) - T[col + 1].block(0, 3, 3, 1);
    tmp = tmp.cross(tmp2);
    J.block(0, col, 3, 1) = tmp;
    J.block(3, col, 3, 1) = T[col + 1].block(0, ax[col] - 1, 3, 1);
  }

  return J;
}

void ik_ee(
  Eigen::VectorXr& q, const Eigen::Matrix3r& R_base,
  const Eigen::Vector3r& p_base, const Eigen::Vector3r& p_t,
  const Eigen::Matrix3r& R_t, const Eigen::VectorXr& q_l,
  const Eigen::VectorXr& q_u, const Eigen::VectorXr& Kdiag) {
  Eigen::Matrix4r T;
  Eigen::MatrixXr J(6, 7);
  Eigen::Matrix3r R_e;
  Eigen::Vector3r p_e;
  Eigen::VectorXr dx(6);

  Eigen::VectorXr dq(q.size());
  // Eigen::LLT<Eigen::MatrixXr> solver;

  Eigen::MatrixXr K(6, 6);
  K = Kdiag.asDiagonal();

  for (int i = 0; i < 300; ++i) {
    T = fk_ee(q, R_base, p_base);
    J = jaco_ee(q, R_base, p_base);
    R_e = T.block(0, 0, 3, 3);
    p_e = T.block(0, 3, 3, 1);

    dx.segment(0, 3) = p_e - p_t;
    dx.segment(3, 3) =
      R_e * unskew(0.5 * (R_t.transpose() * R_e - R_e.transpose() * R_t));

    // JJT = J * J.transpose() + lam * lam * Eigen::MatrixXr::Identity(6, 6);
    // solver.compute(JJT);

    dq = J.transpose() * K * dx;
    // solver.compute(J);
    // dq = solver.solve(K * dx);

    q = q - dq / 100;
    // COUT_VECTOR(p_t);
    // COUT_VECTOR(p_e);
    // COUT_VECTOR(dq);

    for (int j = 0; j < q.size(); ++j) {
      q[j] = std::max(std::min(q[j], q_u[j]), q_l[j]);
    }
  }

  // COUT_VECTOR(p_t);
  // COUT_VECTOR(p_e);
  // COUT_SCALAR((p_t - p_e).norm());
}

void my_control(
  Eigen::Ref<Eigen::VectorXr> act_in, const Eigen::VectorXr& q_cur, double time,
  const Eigen::VectorXr& q0, const Eigen::Matrix3r& R_base,
  const Eigen::Vector3r& p_base, const Eigen::Vector3r& p_t,
  const Eigen::Matrix3r& R_t, const Eigen::VectorXr& q_l,
  const Eigen::VectorXr& q_u) {
  act_in.setZero();

  if (time < 16) {
    Eigen::VectorXr Kdiag(6);
    Kdiag << 10, 10, 10, 10, 10, 10;

    Eigen::VectorXr q = q_cur.segment(0, 7);
    // Eigen::VectorXr q = sd->q;
    ik_ee(q, R_base, p_base, p_t, R_t, q_l, q_u, Kdiag);
    act_in.segment(0, 7) = q0 + (q - q0) * std::min(time / 15, 1.0);
  } else {
    Eigen::VectorXr Kdiag(6);
    Kdiag << 1, 1, 10, 10, 10, 10;

    // Eigen::Vector3r p_t2 = p_t +
    //   (Eigen::Vector3r(p_t[0], p_t[1], 0.06 + 0.052 - 0.0) - p_t) *
    //     std::min((time - 16) / 15, 1.0);

    Eigen::Matrix4r T = fk_ee(q_cur, R_base, p_base);
    Eigen::Vector3r p_t2 = T.block(0, 3, 3, 1) - Eigen::Vector3r(0, 0, 0.003);
    p_t2[0] = p_t[0];
    p_t2[1] = p_t[1];

    // Eigen::Matrix3r R_t2 = INROL::Rot(-0.8 * (sd->time - 35) / 5, 'z') * R_t;
    Eigen::Matrix3r R_t2 = R_t;
    // Eigen::VectorXr q = sd->q;

    Eigen::VectorXr q = q_cur.segment(0, 7);

    ik_ee(q, R_base, p_base, p_t2, R_t2, q_l, q_u, Kdiag);
    act_in.segment(0, 7) = q;
  }

  if (time > 17) {
    // act_in(7) = (4 * (time - 17) / 1);
    act_in(7) = q_cur[7] + PI / 6;
    if (q_cur[7] >= 4 * PI) {
      act_in(7) = q_cur[7];
    }
  }
}

int main() {
  // crisp::__logger__->set_level(spdlog::level::debug);

  Eigen::MatrixXr W0 = txt2mat("assets/boltnut/W0.txt", 64, 3);
  Eigen::MatrixXr W1 = txt2mat("assets/boltnut/W1.txt", 64, 64);
  Eigen::MatrixXr W2 = txt2mat("assets/boltnut/W2.txt", 64, 64);
  Eigen::VectorXr W3 = txt2vec("assets/boltnut/W3.txt", 64);
  Eigen::VectorXr b0 = txt2vec("assets/boltnut/b0.txt", 64);
  Eigen::VectorXr b1 = txt2vec("assets/boltnut/b1.txt", 64);
  Eigen::VectorXr b2 = txt2vec("assets/boltnut/b2.txt", 64);
  Eigen::VectorXr b3_vec = txt2vec("assets/boltnut/b3.txt", 1);
  double b3 = b3_vec(0);
  Eigen::VectorXr input_coeff = txt2vec("assets/boltnut/input_coeff.txt", 6);
  Eigen::VectorXr output_coeff = txt2vec("assets/boltnut/output_coeff.txt", 2);

  int sdf_type = crisp::register_sdf(
    [&](Eigen::Vector3r const& x, Eigen::VectorXr const&) {
      return NN_sdf(
        x, W0, W1, W2, W3, b0, b1, b2, b3, input_coeff, output_coeff);
    },
    [](Eigen::Matrix3r const&, Eigen::VectorXr const&) {
      return Eigen::Vector6r {-0.025, -0.025, 0.019, 0.025, 0.025, 0.041};
    },
    0);

  auto model = crisp::make_model();
  model->addRobot(
    {.path = "assets/panda/panda_hebi.urdf", .root_pos = {0, 0, 0.011}});
  {
    auto& body = model->addBody({.pos = {0.5, -0.025, 0.015}});
    auto& geom_r =
      body.addGeom({.visual = true, .con_type = 0, .con_affinity = 0});
    geom_r.createMesh(
      {.path = "assets/boltnut/M48_P5_bolt.obj", .scale = {1, 1, 1}});
    auto& geom_c = body.addGeom({.visual = false});
    geom_c.createSDF({.type = sdf_type});
  }
  model->cfg().vis.cam.target = {0.3f, 0.0f, 0.25f};
  model->cfg().vis.cam.azimuth = 140.0f;
  model->cfg().vis.cam.elevation = -10.0f;
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
  for (int i = 0; i < 7; ++i) {
    model->addActuator(
      {.type = crisp::actuator_e::position,
       .jointid = i + 1,
       .gain = {0, 2000, 200}});
  }
  model->addActuator(
    {.type = crisp::actuator_e::position, .jointid = 9, .gain = {0, 50, 5}});

  auto m = model->compile();
  Eigen::VectorXr q0(8);
  q0 << 0.00000, -0.06313, 0.00000, -2.18291, -0.00000, 2.12035, -0.00000, 0;
  m->state.q0() = q0;
  m->act.u0() << 0, 0, 0, 0, 0, 0, 0, 0;

  Eigen::VectorXr q_l(8);
  q_l << -PI, -PI, -PI, -PI, -PI, -PI, -1e+10, -1e+10;
  Eigen::VectorXr q_u(8);
  q_u << PI, PI, PI, PI, PI, PI, 1e+10, 1e+10;

  // m->opt.gravity.setZero();

  m->opt.solver_max_iter = 128;
  m->opt.jacobian = crisp::jacobian_e::sparse;
  // m->opt.solver = crisp::solver_e::canal;
  m->opt.solver = crisp::solver_e::sub_admm;

  auto app = crisp::make_app(std::move(m), false);

  auto u_traj = txt2traj("assets/boltnut/boltnut_q_13.csv", 8);
  app->data().act.u() = u_traj[0];

  app->setControl([&](auto const& m, auto const& d, auto u, auto udot) {
    my_control(
      u, d.state.q(), d.world.time, q0, Eigen::Matrix3r::Identity(),
      Eigen::Vector3r(0, 0, 0.011), Eigen::Vector3r(0.5, -0.025, 0.015 + 0.145),
      Rot(PI, 'x'), q_l, q_u);
  });

  bool trigger = true;
  app->init();
  while (app->isOpen()) {
    app->render();
  }

  return 0;
}
