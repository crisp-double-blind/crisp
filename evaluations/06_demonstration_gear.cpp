#include <crisp/crisp.hpp>

#include "utils/math.hpp"

inline double PI = std::numbers::pi;

double sdf_intersection(double a, double b) {
  return std::max(a, b);
}
double sdf_subtraction(double a, double b) {
  return std::max(a, -b);
}
double sdf_circle(double rho, double r) {
  return rho - r;
}

double sdf_smooth_union(double a, double b, double k) {
  double h = std::min(std::max(0.5 + 0.5 * (b - a) / k, 0.0), 1.0);
  return b * (1. - h) + a * h - k * h * (1. - h);
}
double sdf_smooth_intersection(double a, double b, double k) {
  return sdf_subtraction(
    sdf_intersection(a, b),
    sdf_smooth_union(sdf_subtraction(a, b), sdf_subtraction(b, a), k));
}

double mod(double x, double y) {
  return x - y * std::floor(x / y);
}

double distance_2d(
  Eigen::Vector3r const& p, double alpha = 0, double D = 2.8, double N = 25,
  double innerD = -1) {
  // see https://www.shadertoy.com/view/3lG3WR
  double psi = 3.096e-5 * N * N - 6.557e-3 * N + 0.551;  // pressure angle
  double R = D / 2.0;
  /* The Pitch Circle Diameter is the diameter of a circle which by a pure
   * rolling action would transmit the same motion as the actual gear wheel. It
   * should be noted that in the case of wheels which connect non-parallel
   * shafts, the pitch circle diameter is different for each cross section of
   * the wheel normal to the axis of rotation.
   */

  double rho = std::sqrt(p[0] * p[0] + p[1] * p[1]);
  double Pd = N / D;  // Diametral Pitch: teeth per unit length of diameter
  double P = PI / Pd;  // Circular Pitch: the length of arc round the pitch
                       // circle between corresponding points on adjacent teeth.
  double a = 1.0 / Pd;  // Addendum: radial length of a tooth from the pitch
                        // circle to the tip of the tooth.

  double Do = D + 2.0 * a;  // Outside Diameter
  double Ro = Do / 2.0;

  double h = 2.2 / Pd;

  double innerR = Ro - h - 0.14 * D;
  if (innerD >= 0.0) {
    innerR = innerD / 2.0;
  }

  // Early exit
  if (innerR - rho > 0.0) return innerR - rho;

  // Early exit
  if (Ro - rho < -0.2) return rho - Ro;

  double Db = D * std::cos(psi);  // Base Diameter
  double Rb = Db / 2.0;

  double fi = std::atan2(p[1], p[0]) + alpha;
  double alphaStride = P / R;

  double invAlpha = std::acos(Rb / R);
  double invPhi = std::tan(invAlpha) - invAlpha;

  double shift = alphaStride / 2.0 - 2.0 * invPhi;

  double fia = mod(fi + shift / 2.0, alphaStride) - shift / 2.0;
  double fib = mod(-fi - shift + shift / 2.0, alphaStride) - shift / 2.0;

  double dista = -1.0e6;
  double distb = -1.0e6;

  if (Rb < rho) {
    double acos_rbRho = std::acos(Rb / rho);

    double thetaa = fia + acos_rbRho;
    double thetab = fib + acos_rbRho;

    double ta = std::sqrt(rho * rho - Rb * Rb);

    // https://math.stackexchange.com/questions/1266689/distance-from-a-point-to-the-involute-of-a-circle
    dista = ta - Rb * thetaa;
    distb = ta - Rb * thetab;
  }

  double gearOuter = sdf_circle(rho, Ro);
  double gearLowBase = sdf_circle(rho, Ro - h);
  double crownBase = sdf_circle(rho, innerR);
  double cogs = sdf_intersection(dista, distb);
  double baseWalls =
    sdf_intersection(fia - (alphaStride - shift), fib - (alphaStride - shift));

  cogs = sdf_intersection(baseWalls, cogs);
  cogs = sdf_smooth_intersection(gearOuter, cogs, 0.0035 * D);
  cogs = sdf_smooth_union(gearLowBase, cogs, Rb - Ro + h);
  cogs = sdf_subtraction(cogs, crownBase);

  return cogs;
}

double extrusion(Eigen::Vector3r const& p, double sdf_2d, double h) {
  double w[2] = {sdf_2d, std::abs(p[2]) - h};
  double w_abs[2] = {std::max(w[0], 0.0), std::max(w[1], 0.0)};
  return std::min(std::max(w[0], w[1]), 0.) +
    std::sqrt(w_abs[0] * w_abs[0] + w_abs[1] * w_abs[1]);
}

double distance(
  Eigen::Vector3r const& p, double alpha = 0, double D = 2.8, double N = 25,
  double thickness = 0.2, double innerD = -1) {
  return extrusion(p, distance_2d(p, alpha, D, N, innerD), thickness / 2.);
}

crisp::signed_distance_t gear_sdf(
  Eigen::Vector3r const& p, double alpha = 0, double D = 2.8, double N = 25,
  double thickness = 0.2, double innerD = -1) {
  double dist = distance(p, alpha, D, N, thickness, innerD);

  double eps = 1e-8;

  Eigen::Vector3r pointX = Eigen::Vector3r(p[0] + eps, p[1], p[2]);
  double distX = distance(pointX, alpha, D, N, thickness, innerD);

  Eigen::Vector3r pointY = Eigen::Vector3r(p[0], p[1] + eps, p[2]);
  double distY = distance(pointY, alpha, D, N, thickness, innerD);

  Eigen::Vector3r pointZ = Eigen::Vector3r(p[0], p[1], p[2] + eps);
  double distZ = distance(pointZ, alpha, D, N, thickness, innerD);

  Eigen::Vector3r grad;
  grad[0] = (distX - dist) / eps;
  grad[1] = (distY - dist) / eps;
  grad[2] = (distZ - dist) / eps;
  if (grad.norm() == 0) {
    grad.setZero();
  } else {
    grad /= grad.norm();
  }

  return {dist, grad};
}

void my_control(
  Eigen::Ref<Eigen::VectorXr> act_in, double time, Eigen::VectorXr const& q0,
  Eigen::VectorXr const& qt1, Eigen::VectorXr const& qt2,
  Eigen::VectorXr const& qt3, Eigen::VectorXr const& qt4,
  Eigen::VectorXr const& qt5) {
  act_in.setZero();
  act_in[9] = utils::deg2rad(-7.2);

  if (time < 5) {
    act_in.segment(0, 7) = q0;
    act_in[7] = 0.03;
    act_in[8] = 0.03;
  } else if (time < 7) {
    act_in.segment(0, 7) = q0 + (qt1 - q0) * std::min((time - 5) / 1.5, 1.0);
    act_in[7] = 0.03;
    act_in[8] = 0.03;
  } else if (time < 8.5) {
    act_in.segment(0, 7) = qt1;
    act_in[7] = 0.0;
    act_in[8] = 0.0;
  } else if (time < 10.5) {
    act_in.segment(0, 7) =
      qt1 + (qt2 - qt1) * std::min((time - 8.5) / 1.5, 1.0);
    act_in[7] = 0.0;
    act_in[8] = 0.0;
  } else if (time < 12.5) {
    act_in.segment(0, 7) =
      qt2 + (qt3 - qt2) * std::min((time - 10.5) / 1.5, 1.0);
    act_in[7] = 0.0;
    act_in[8] = 0.0;
  } else if (time < 16) {
    act_in.segment(0, 7) = qt3 + (qt4 - qt3) * std::min((time - 12.5) / 3, 1.0);
    act_in[7] = 0.0;
    act_in[8] = 0.0;
  } else if (time < 16.2) {
    act_in.segment(0, 7) = qt4;
    act_in[7] = 0.03;
    act_in[8] = 0.03;
  } else if (time < 18) {
    act_in.segment(0, 7) =
      qt4 + (qt5 - qt4) * std::min((time - 16.2) / 0.5, 1.0);
    act_in[7] = 0.03;
    act_in[8] = 0.03;
  } else {
    act_in.segment(0, 7) = qt5;
    act_in[7] = 0.03;
    act_in[8] = 0.03;
    act_in[9] = std::min(-(time - 18) * PI * 0.7 - 7.2 / 180 * PI, 200 * PI);
  }
}

int main() {
  int gear1_sdf_type = crisp::register_sdf(
    [](Eigen::Vector3r const& x, Eigen::VectorXr const&) {
      return gear_sdf(x, 0, 0.1, 25, 0.01, 0.02);
    },
    [](Eigen::Matrix3r const&, Eigen::VectorXr const&) {
      return Eigen::Vector6r {-0.06, -0.06, -0.06, 0.06, 0.06, 0.06};
    },
    0);

  int gear2_sdf_type = crisp::register_sdf(
    [](Eigen::Vector3r const& x, Eigen::VectorXr const&) {
      return gear_sdf(x, 0, 0.2, 50, 0.01, 0.02);
    },
    [](Eigen::Matrix3r const&, Eigen::VectorXr const&) {
      return Eigen::Vector6r {-0.11, -0.11, -0.11, 0.11, 0.11, 0.11};
    },
    0);

  auto mat_table = crisp::material_t {
    .rgba = crisp::oklch(0.80f, 0.008f, 90),
    .emission = {0.0f, 0.0f, 0.0f},
    .specular = {0.04f, 0.04f, 0.04f},
    .shininess = 16};
  auto mat_gear = crisp::material_t {
    .rgba = crisp::oklch(0.68f, 0.015f, 250),
    .emission = {0.0f, 0.0f, 0.0f},
    .specular = {0.55f, 0.55f, 0.55f},
    .shininess = 110};
  auto mat_shaft = crisp::material_t {
    .rgba = crisp::oklch(0.62f, 0.008f, 250),
    .emission = {0.0f, 0.0f, 0.0f},
    .specular = {0.85f, 0.85f, 0.85f},
    .shininess = 240};
  auto mat_bar = crisp::material_t {
    .rgba = crisp::oklch(0.64f, 0.07f, 200),
    .emission = {0.0f, 0.008f, 0.01f},
    .specular = {0.22f, 0.22f, 0.22f},
    .shininess = 70};

  auto model = crisp::make_model();
  model->addRobot({.path = "assets/panda/franka_gripper2.urdf"});
  {
    // actuator axis
    double eps = 2e-3;  // diameter tolerance
    auto& body = model->addBody({.pos = {0.6, 0.0 + 0.0025, 0.4 + 0.02}}, true);
    auto& geom = body.addGeom(
      {.type = crisp::geometry_e::cylinder,
       .size = crisp::make_cylinder_size(0.01 - eps / 2, 0.04),
       .mu = 0.1});
    geom.createMaterial(mat_shaft);

    // actuator gear
    auto& body_g = body.addChild(
      {.pos = {0.0, 0.0, -0.02 + 0.0075},
       .quat = utils::aa2quat(-7.2, {0.0, 0.0, 1.0}),
       .mass = 0.1,
       .inertia = {6.33e-5, 6.33e-5, 1.25e-4, 0, 0, 0}},
      {.type = crisp::joint_e::revolute, .axis = {0, 0, 1}});

    auto& geom_v =
      body_g.addGeom({.visual = true, .con_type = 0, .con_affinity = 0});
    geom_v.createMesh({.path = "assets/gear/gear1.obj", .scale = {1, 1, 1}});
    geom_v.createMaterial(mat_gear);

    auto& geom_c = body_g.addGeom(
      {.visual = false, .con_type = 2, .con_affinity = 1, .mu = 0.1});
    geom_c.createSDF({.type = gear1_sdf_type});
  }

  {
    // target gear
    auto& body = model->addBody(
      {.pos = {0.52, 0.13, 0.415},
       .quat = utils::aa2quat(3.6, {0.0, 0.0, 1.0}),
       .mass = 0.1,
       .inertia = {6.33e-5, 6.33e-5, 1.25e-4, 0, 0, 0}},
      false);

    auto& geom_v =
      body.addGeom({.visual = true, .con_type = 0, .con_affinity = 0});
    geom_v.createMesh({.path = "assets/gear/gear1.obj", .scale = {1, 1, 1}});
    geom_v.createMaterial(mat_gear);

    auto& geom_c = body.addGeom({.visual = false, .mu = 0.1});
    geom_c.createSDF({.type = gear1_sdf_type});
  }

  {
    // end axis
    double dist = 0.15 + 0.0035;
    double eps = 2e-3;  // diameter tolerance
    auto& body = model->addBody(
      {.pos = {0.6 + dist * 0.6, -dist * 0.8 - 0.1, 0.4 + 0.02}}, true);
    auto& geom = body.addGeom(
      {.type = crisp::geometry_e::cylinder,
       .size = crisp::make_cylinder_size(0.01 - eps / 2, 0.04),
       .mu = 0.1});
    geom.createMaterial(mat_shaft);

    // end gear
    auto& body_g = body.addChild(
      {.pos = {0.0, 0.0, -0.02 + 0.0075},
       .quat = utils::aa2quat(-3.6, {0.0, 0.0, 1.0}),
       .mass = 0.1,
       .inertia = {6.33e-5, 6.33e-5, 1.25e-4, 0, 0, 0}},
      {.type = crisp::joint_e::revolute, .axis = {0, 0, 1}});

    auto& geom_v =
      body_g.addGeom({.visual = true, .con_type = 0, .con_affinity = 0});
    geom_v.createMesh({.path = "assets/gear/gear2.obj", .scale = {1, 1, 1}});
    geom_v.createMaterial(mat_gear);

    auto& geom_c = body_g.addGeom(
      {.visual = false, .con_type = 2, .con_affinity = 1, .mu = 0.1});
    geom_c.createSDF({.type = gear2_sdf_type});

    // bar
    auto& body_b = body_g.addChild(
      {.pos = {0.05, 0.0, 0.005 + 0.005},
       .quat = utils::aa2quat(0.0, {0.0, 0.0, 1.0})},
      {.type = crisp::joint_e::fixed});
    auto& geom_b = body_b.addGeom(
      {.type = crisp::geometry_e::box,
       .size = crisp::make_box_size(0.06, 0.01, 0.01),
       .con_type = 0,
       .con_affinity = 0});
    geom_b.createMaterial(mat_bar);
  }

  {
    // table
    auto& body = model->addBody({.pos = {0.65, -0.1, 0.2}}, true);
    auto& geom = body.addGeom(
      {.type = crisp::geometry_e::box,
       .size = crisp::make_box_size(0.3, 0.6, 0.4),
       .con_type = 2,
       .con_affinity = 1,
       .mu = 0.1});
    geom.createMaterial(mat_table);
  }

  {
    // target axis
    double eps = 2e-3;  // diameter tolerance
    auto& body = model->addBody({.pos = {0.6, -0.1, 0.4 + 0.045}}, true);
    auto& geom = body.addGeom(
      {.type = crisp::geometry_e::cylinder,
       .size = crisp::make_cylinder_size(0.01 - eps / 2, 0.09),
       .mu = 0.1});
    geom.createMaterial(mat_shaft);
  }

  model->cfg().vis.cam.target = {0.0f, -0.1f, 0.3f};
  model->cfg().vis.cam.distance = 1.3f;
  model->cfg().vis.cam.azimuth = 180.0f;
  model->cfg().vis.cam.elevation = -20.0f;
  model->cfg().vis.cam.fovy = 45;
  model->addLight(
    {.type = crisp::light_e::directional,
     .enabled = true,
     .cast_shadow = true,
     .pos = {1.0f, 2.0f, 3.0f},
     .dir = {-1.0f, -2.0f, -3.0f},
     .ambient = {0.3f, 0.3f, 0.3f},
     .diffuse = {0.8f, 0.8f, 0.8f},
     .specular = {1.0f, 1.0f, 1.0f},
     .attenuation = {1.0f, 0.0f, 0.0f},
     .cutoff = 0.0f,
     .exponent = 0.0f});
  model->cfg().opt.erp = 0.01;
  model->cfg().opt.solver = crisp::solver_e::sub_admm;
  model->cfg().opt.solver_max_iter = 500;
  model->cfg().opt.warmstart = false;
  model->cfg().ncon_max = 1000;

  for (int i = 0; i < 10; ++i) {
    if (i == 7) continue;
    model->addActuator(
      {.type = crisp::actuator_e::position,
       .jointid = i + 1,
       .gain = {0, 2000, 200}});
  }
  model->addActuator(
    {.type = crisp::actuator_e::position, .jointid = 12, .gain = {0, 200, 20}});

  auto m = model->compile();

  Eigen::VectorXr q0(7);
  q0 << 0.237437218809148, -0.728100541383945, 0.152045154951825,
    -2.84004714848913, -2.50025875468802, 2.53220910200815, 1.12892685814029;

  Eigen::VectorXr qt1(7);  // grasp
  qt1 << 0.154708501746311, -0.475086394774886, 0.170785881762325,
    -2.65377761255647, -2.62474446054682, 2.48615482037952, 1.22905250974781;

  Eigen::VectorXr qt2(7);  // up
  qt2 << 0.162210625946555, -0.567553088641004, 0.110049457486990,
    -2.51908385312721, -2.51521311434482, 2.69733246185543, 1.05312349510900;

  Eigen::VectorXr qt3(7);  // above axis
  qt3 << -0.0464290944589402, -0.354641595486542, -0.105781528445334,
    -2.20656949757837, 2.65025762720998, 2.82903401303429, -1.13689539278797;

  Eigen::VectorXr qt4(7);  // down
  qt4 << -0.110475738780582, -0.344620900047932, -0.0728888625453880,
    -2.29301943029251, 2.68164932928831, 2.72878024701990, -1.16982148156211;

  Eigen::VectorXr qt5(7);  // second front
  qt5 << -0.302424535322473, -0.608112038269523, 0.0518181670160084,
    -2.69542055794137, 2.65400282348492, 2.56127271483118, -1.12278357320995;

  m->state.q0().head<9>() << q0, 0, 0;
  m->act.u0().head<9>() << q0, 0, 0;

  m->geom.mu[13] = m->geom.mu[17] = 5;

  auto app = crisp::make_app(std::move(m));

  app->setControl([&](auto const& m, auto const& d, auto u, auto udot) {
    my_control(u, d.world.time, q0, qt1, qt2, qt3, qt4, qt5);
  });

  app->init();
  while (app->isOpen()) {
    app->render();
  }

  return 0;
}
