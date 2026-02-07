#pragma once

#include <crisp/model.hpp>

#include <functional>
#include <memory>
#include <string_view>

namespace crisp {
class Body;
class Joint;
class Geometry;
class Ground;
class Material;
class Texture;

struct robot_t {
  path_t path;
  Eigen::Vector3r root_pos = Eigen::Vector3r::Zero();
  Eigen::Quaternionr root_quat = Eigen::Quaternionr::Identity();
};

struct light_t {
  light_e type;
  bool enabled;
  bool cast_shadow;
  Eigen::Vector3f pos;
  Eigen::Vector3f dir;
  Eigen::Vector3f ambient;
  Eigen::Vector3f diffuse;
  Eigen::Vector3f specular;
  Eigen::Vector3f attenuation;
  float cutoff;
  float exponent;
};

struct actuator_t {
  actuator_e type;
  int jointid;
  int local_dof;
  Eigen::Vector3r gain;
};

struct body_t {
  std::string name;
  Eigen::Vector3r pos = Eigen::Vector3r::Zero();
  Eigen::Quaternionr quat = Eigen::Quaternionr::Identity();

  Eigen::Vector3r ipos = Eigen::Vector3r::Zero();
  Eigen::Quaternionr iquat = Eigen::Quaternionr::Identity();
  real_t mass;
  inertia_t inertia;

  bool explicit_inertial;
};

struct joint_t {
  std::string name;
  joint_e type;
  Eigen::Vector3r axis = {1, 0, 0};

  real_t damping = 0;
  real_t friction = 0;

  bool limited;
  Eigen::Vector2r range;
};

struct geometry_t {
  std::string name;
  geometry_e type = geometry_e::undefined;
  Eigen::Vector3r pos = Eigen::Vector3r::Zero();
  Eigen::Quaternionr quat = Eigen::Quaternionr::Identity();
  Eigen::Vector4r size = Eigen::Vector4r::Zero();

  bool visual = true;
  uint8_t con_type = 1;
  uint8_t con_affinity = 0xFF;
  real_t mu = 1;
  real_t k = 0;
};
inline Eigen::Vector4r make_plane_size(real_t width, real_t height) {
  return {width, height, 0, 0};
}
inline Eigen::Vector4r make_sphere_size(real_t radius) {
  return {radius, 0, 0, 0};
}
inline Eigen::Vector4r make_capsule_size(real_t radius, real_t length) {
  return {radius, length / 2, 0, 0};
}
inline Eigen::Vector4r make_cylinder_size(real_t radius, real_t length) {
  return {radius, length / 2, 0, 0};
}
inline Eigen::Vector4r make_box_size(real_t lx, real_t ly, real_t lz) {
  return {lx / 2, ly / 2, lz / 2, 0};
}

struct body_camera_t {
  std::string name;
  Eigen::Vector3f pos = Eigen::Vector3f::Zero();
  Eigen::Quaternionf quat = Eigen::Quaternionf::Identity();
  float fovy = 60.0f;
  float near = 0.01f;
  float far = 30.0f;
  bool overlay = true;
  int width = 640;
  int height = 480;
};

struct convex_t {
  std::string name;
  path_t path;
  Eigen::Matrix3Xr vert;
};

struct mesh_t {
  std::string name;
  path_t path;
  Eigen::Vector3r scale = Eigen::Vector3r::Ones();
};

struct sdf_t {
  std::string name;
  int type;
  Eigen::VectorXr param;
};

struct convex2d_t {
  std::string name;
  Eigen::Matrix2Xr vert;
};

struct ellipse_t {
  real_t a;
  real_t b;
};

struct material_t {
  std::string name;
  Eigen::Vector4f rgba;
  Eigen::Vector3f emission = {0.0f, 0.0f, 0.0f};
  Eigen::Vector3f specular = {0.1f, 0.1f, 0.1f};
  float shininess = 32.0f;
};
Eigen::Vector4f oklch(
  float lightness, float chroma, float hue, float alpha = 1.0f);

struct texture_t {
  std::string name;
  path_t path;
};

struct ground_t {
  real_t width = 10;
  real_t height = 10;
  real_t mu = 1;
  real_t k = 0;
};

class Model {
public:
  struct config_t {
    model_t::option_t opt = {
      .dt = real_t(0.01),
      .gravity = {real_t(0), real_t(0), real_t(-9.81)},
      .margin = {real_t(0.01), real_t(0.01), real_t(0.01)},
      .erp = real_t(0.1),
      .friction = friction_e::arithmetic,
      .jacobian = jacobian_e::adaptive,
      .solver = solver_e::pgs,
      .solver_max_iter = 100,
      .solver_max_inner = 30,
      .solver_max_interval = 20,
      .solver_max_linesearch = 50,
      .max_value = 1e+15,
      .warmstart = true,
      .solver_outer_earlystop = true,
      .solver_min_res = 1e-10,
      .solver_min_rel_res = 1e-4,
      .solver_min_grad_inner = 1e-6,
      .solver_min_grad_linesearch = 1e-16,
      .solver_update_beta = true,
      .canal_beta_init = 1e+4,
      .canal_beta_max = 1e+6,
      .canal_beta_lim_init = 1e+4,
      .canal_beta_lim_max = 1e+6,
      .canal_beta_fric_init = 1e+3,
      .canal_beta_fric_max = 1e+6,
      .admm_beta_init = 1e-1,
      .admm_beta_min = 1e-4,
      .admm_beta_max = 1e+4,
      .admm_beta_lim_init = 1e-4,
      .admm_beta_lim_min = 1e-4,
      .admm_beta_lim_max = 1e+4,
      .admm_beta_fric_init = 1e-4,
      .admm_beta_fric_min = 1e-4,
      .admm_beta_fric_max = 1e+4,
      .admm_gamma = 1,
      .admm_update_iter_max = 100,
      .admm_update_w = 1,
      .admm_update_w_lim = 100,
      .admm_update_w_fric = 100};

    model_t::visual_t vis = {
      .cam =
        {.target = {0.0f, 0.0f, 0.0f},
         .distance = 1.0f,
         .azimuth = 0.0f,
         .elevation = -10.0f,
         .fovy = 60.0f,
         .near = 0.01f,
         .far = 100.0f},
      .gnd =
        {.grid_size = 0.5f,
         .line_width = 0.003f,
         .fade_inner = 2.0f,
         .fade_outer = 1.0f,
         .reflectance = 0.1f},
      .bg = {0.8f, 0.9f, 1.0f}};

    int ncon_max = 100;
    std::size_t nstack = 0;
  };

protected:
  config_t cfg_;

public:
  Model(config_t const& cfg): cfg_(cfg) {}
  virtual ~Model() = default;

  auto& cfg() const { return cfg_; }
  auto& cfg() { return cfg_; }

  virtual Ground* getGround() const = 0;

  virtual Body& addBody(body_t const& body, bool fixed = true) = 0;
  virtual bool addRobot(robot_t const& robot, bool fixed = true) = 0;

  virtual void addLight(light_t const& light) = 0;
  virtual void addActuator(actuator_t const& act) = 0;

  virtual std::unique_ptr<model_t> compile() = 0;
};  // namespace crisp

std::unique_ptr<Model> make_model(Model::config_t const& cfg = {});

class Body {
protected:
  body_t cfg_;

public:
  Body(body_t const& cfg): cfg_(cfg) {}
  virtual ~Body() = default;

  auto& cfg() const { return cfg_; }
  auto& cfg() { return cfg_; }

  virtual Joint& getJoint() const = 0;

  virtual Body& addChild(body_t const& body, joint_t const& joint) = 0;
  virtual Geometry& addGeom(geometry_t const& geom) = 0;
  virtual void addCamera(body_camera_t const& cam) = 0;
};

class Joint {
protected:
  joint_t cfg_;

public:
  Joint(joint_t const& cfg): cfg_(cfg) {}
  virtual ~Joint() = default;

  auto& cfg() const { return cfg_; }
  auto& cfg() { return cfg_; }
};

class Geometry {
protected:
  geometry_t cfg_;

public:
  Geometry(geometry_t const& cfg): cfg_(cfg) {}
  virtual ~Geometry() = default;

  auto& cfg() const { return cfg_; }
  auto& cfg() { return cfg_; }

  virtual void createConvex(convex_t const& cvx) = 0;
  virtual void createMesh(mesh_t const& mesh) = 0;
  virtual void createSDF(sdf_t const& sdf) = 0;
  virtual void createDSF(convex_t const& cvx, real_t sharp) = 0;
  virtual void createDSF(convex2d_t const& cvx, real_t sharp, real_t eps_z) = 0;
  virtual void createTDSF(
    convex2d_t const& cvx1, real_t sharp1,  //
    convex2d_t const& cvx2, real_t sharp2) = 0;
  virtual void createTDSF(
    convex2d_t const& cvx1, real_t sharp1, ellipse_t const& ell2) = 0;
  virtual void createTDSF(
    ellipse_t const& ell1, convex2d_t const& cvx2, real_t sharp2) = 0;
  virtual void createTDSF(ellipse_t const& ell1, ellipse_t const& ell2) = 0;

  virtual Material& createMaterial(material_t const& mat) = 0;
};

class Material {
protected:
  material_t cfg_;

public:
  Material(material_t const& cfg): cfg_(cfg) {}
  virtual ~Material() = default;

  auto& cfg() const { return cfg_; }
  auto& cfg() { return cfg_; }

  virtual void createTexture(texture_t const& tex_cfg) = 0;
};

class Texture {
protected:
  texture_t cfg_;

public:
  Texture(texture_t const& cfg): cfg_(cfg) {}
  virtual ~Texture() = default;

  auto& cfg() const { return cfg_; }
  auto& cfg() { return cfg_; }
};

class Ground {
protected:
  ground_t cfg_;

public:
  Ground(ground_t const& cfg): cfg_(cfg) {}
  virtual ~Ground() = default;

  auto& cfg() const { return cfg_; }
  auto& cfg() { return cfg_; }

  virtual Material& getBaseMaterial() const = 0;
  virtual Material& getLineMaterial() const = 0;
};

using uri_resolver_t = std::function<path_t(std::string_view)>;
void register_uri_resolver(std::string_view scheme, uri_resolver_t&& resolver);
}  // namespace crisp
