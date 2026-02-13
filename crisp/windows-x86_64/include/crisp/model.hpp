/*
 * CRISP: Contact-Rich Simulation Platform
 * Copyright (c) 2026 The CRISP authors.
 * Licensed for academic and non-commercial use only.
 * See the LICENSE file for details.
 */

#pragma once  // clang-format off

#include <crisp/arena.hpp>
#include <crisp/export.hpp>
#include <crisp/heap.hpp>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace crisp {
enum class friction_e {
  arithmetic, geometric, harmonic, min, max
};
enum class jacobian_e {
  dense, sparse, adaptive
};
enum class solver_e {
  pgs, canal, sub_admm
};
enum class joint_e {
  fixed, floating, revolute, prismatic
};
enum class geometry_e {
  plane, sphere, capsule, box, cylinder, convex, sdf, mesh, dsf, tdsf, _end,
  dsf_vert = dsf | 0x10,
  dsf_axis = dsf | 0x20,
  tdsf_vv = tdsf | 0x10,
  tdsf_ve = tdsf | 0x20,
  tdsf_ev = tdsf | 0x30,
  tdsf_ee = tdsf | 0x40,
  undefined = -1
};
enum class light_e {
  directional, point, spot
};
enum class actuator_e {
  position, force
};

struct inertia_t { real_t ixx, iyy, izz, ixy, ixz, iyz; };

struct model_t {
  using Vec3r = Eigen::Vector3r;
  using Vec3f = Eigen::Vector3f;

  struct size_t {
    int nq;
    int nv;
    int nu;

    int nbody;
    int ndamp;
    int nfric;
    int nlim;
    int ngeom;
    int nmesh;
    int ncvx;
    int ncvx2d;
    int nsdf;
    int nmat;
    int ntex;
    int ncam;
    int nlight;
    int ndsf;
    int ntdsf;
    int nnames;
    int npaths;

    int nM;
    int nD;
    int ncon_max;

    std::size_t nbump;
    std::size_t nstack;
  } size;

  struct option_t {
    real_t     dt;
    Vec3r      gravity;

    Vec3r      margin;
    real_t     erp;
    friction_e friction;

    jacobian_e jacobian;
    solver_e   solver;
    int        solver_max_iter;
    int        solver_max_inner;
    int        solver_max_interval;
    int        solver_max_linesearch;

    real_t     max_value;
    bool       warmstart;
    bool       solver_outer_earlystop;
    real_t     solver_min_res;
    real_t     solver_min_rel_res;
    real_t     solver_min_grad_inner;
    real_t     solver_min_grad_linesearch;
    bool       solver_update_beta;

    real_t     canal_beta_init;
    real_t     canal_beta_max;
    real_t     canal_beta_lim_init;
    real_t     canal_beta_lim_max;
    real_t     canal_beta_fric_init;
    real_t     canal_beta_fric_max;

    real_t     admm_beta_init;
    real_t     admm_beta_min;
    real_t     admm_beta_max;
    real_t     admm_beta_lim_init;
    real_t     admm_beta_lim_min;
    real_t     admm_beta_lim_max;
    real_t     admm_beta_fric_init;
    real_t     admm_beta_fric_min;
    real_t     admm_beta_fric_max;
    real_t     admm_gamma;
    int        admm_update_iter_max;
    real_t     admm_update_w;
    real_t     admm_update_w_lim;
    real_t     admm_update_w_fric;
  } opt;

  struct visual_t {
    struct {
      Vec3f target;
      float distance;
      float azimuth;
      float elevation;
      float fovy;
      float near;
      float far;
    } cam;

    struct {
      float grid_size;
      float line_width;
      float fade_inner;
      float fade_outer;
      float reflectance;
    } gnd;

    Vec3f bg;
  } vis;

  BumpResource _bump;

  // state
  struct state_t {
    packed<real_t> q0;
  } state;

  // bodies (nbody)
  struct body_t {
    vector<int>         nameid;
    vector<int>         parentid;
    vector<bool>        fixed;

    vector<Vector3r>    pos;
    vector<Quaternionr> quat;

    vector<Vector3r>    ipos;
    vector<Quaternionr> iquat;
    vector<real_t>      mass;
    vector<inertia_t>   inertia;
  } body;

  // joints (nbody)
  struct joint_t {
    vector<joint_e> type;
    vector<int>     nameid;
    vector<int>     qadr;
    vector<int>     vadr;
  } jnt;

  // dofs (nv)
  struct dof_t {
    vector<int>      bodyid;
    vector<int>      parentid;
    vector<int>      parentdof;
    vector<int>      Madr;

    vector<Vector3r> axis;
    packed<real_t>   imp;
  } dof;

  // dampings (ndamp)
  struct damp_t {
    vector<int>    vadr;
    vector<real_t> coeff;
  } damp;

  // frictions (nfric)
  struct fric_t {
    vector<int>    vadr;
    vector<real_t> value;
    vector<real_t> bound;
  } fric;

  // limits (nlim)
  struct lim_t {
    vector<int>      qadr;
    vector<int>      vadr;
    vector<Vector2r> range;
  } lim;

  // geometries (ngeom)
  struct geometry_t {
    vector<geometry_e>  type;
    vector<int>         nameid;
    vector<int>         bodyid;
    vector<int>         assetid;
    vector<int>         matid;

    vector<Vector3r>    pos;
    vector<Quaternionr> quat;
    vector<Vector4r>    size;

    vector<bool>        visual;
    vector<uint8_t>     con_type;
    vector<uint8_t>     con_affinity;

    vector<real_t>      mu;
    vector<real_t>      k;
  } geom;

  // meshes (nmesh)
  struct mesh_t {
    vector<int>       nameid;
    vector<int>       pathid;

    vector<int>       nvert;
    vector<int>       nnormal;
    vector<int>       ntexcoord;
    vector<int>       nface;
    vector<int>       nmat;
    vector<int>       matid0;

    vector<Matrix3Xf> vert;
    vector<Matrix3Xf> normal;
    vector<Matrix2Xf> texcoord;
    vector<Matrix3Xi> face_vert;
    vector<Matrix3Xi> face_normal;
    vector<Matrix3Xi> face_texcoord;
    vector<VectorXi>  face_matidx;
  } mesh;

  // 3d convexes (ncvx)
  struct convex_t {
    vector<int>              nameid;
    vector<int>              nvert;
    vector<Matrix3Xr>        vert;

    vector<uint8_t>          flag;
    vector<real_t>           a_thres;
    vector<vector<Matrix3r>> vert_outer;
  } cvx;

  // 2d convexes (ncvx2d)
  struct convex2d_t {
    vector<int>              nameid;
    vector<int>              nvert;
    vector<Matrix2Xr>        vert;

    vector<uint8_t>          flag;
    vector<real_t>           a_thres;
    vector<vector<Matrix2r>> vert_outer;
  } cvx2d;

  // sdfs (nsdf)
  struct sdf_t {
    vector<int>      type;
    vector<int>      nameid;
    vector<int>      nparam;
    vector<VectorXr> param;
  } sdf;

  // materials (nmat)
  struct material_t {
    vector<int>      nameid;
    vector<int>      texid;
    vector<Vector4f> rgba;
    vector<Vector3f> emission;
    vector<Vector3f> specular;
    vector<float>    shininess;
  } mat;

  // textures (ntex)
  struct texture_t {
    vector<int>            nameid;
    vector<int>            pathid;
    vector<int>            width;
    vector<int>            height;
    vector<vector<byte_t>> rgb;
  } tex;

  // cameras (ncam)
  struct camera_t {
    vector<int>         nameid;
    vector<int>         bodyid;
    vector<Vector3f>    pos;
    vector<Quaternionf> quat;
    vector<float>       fovy;
    vector<float>       near;
    vector<float>       far;

    vector<bool>        overlay;
    vector<int>         width;
    vector<int>         height;
  } cam;

  // lights (nlight)
  struct light_t {
    vector<light_e>  type;
    vector<bool>     enabled;
    vector<bool>     cast_shadow;

    vector<Vector3f> pos;
    vector<Vector3f> dir;

    vector<Vector3f> ambient;
    vector<Vector3f> diffuse;
    vector<Vector3f> specular;

    vector<Vector3f> attenuation;
    vector<float>    cutoff;
    vector<float>    exponent;
  } light;

  // actuators (nu)
  struct actuator_t {
    vector<actuator_e> type;
    vector<int>        jointid;
    vector<int>        local_dof;
    vector<Vector3r>   gain;
    packed<real_t>     u0;
  } act;

  HeapResource _heap;

  // strings
  std::vector<std::string> names;
  std::vector<std::string> paths;
};

using path_t = std::filesystem::path;
CRISP_API void save_model(model_t const& m, path_t const& file_name);
CRISP_API std::unique_ptr<model_t> load_model(path_t const& file_name);
}  // namespace crisp
