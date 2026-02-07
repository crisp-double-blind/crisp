#pragma once  // clang-format off

#include <crisp/model.hpp>

#include <memory>

namespace crisp {
struct broad_pair_t {
  int geomid1, geomid2;
};

struct con_feature_t {
  int             geomid1, geomid2;
  real_t          gap;
  Eigen::Vector3r normal;
  Eigen::Vector3r p1, p2;
};

struct data_t {
  struct size_t {
    int nbroad;
    int ncon;
    int nc;

    std::size_t nbump;
    std::size_t nstack;
  } size;

  struct world_t {
    real_t time;
    int    count;
  } world;

  struct timer_t {
    bool enabled;
    double step;
      double fk;
        double pos;
        double vel;
      double col;
        double broad;
        double narrow;
      double solve;
  } timer;

  struct solver_t {
    solver_e type;
    bool     sparse;
    bool     init;
    int      iter;
    real_t   res;
    real_t   res_p;
    real_t   res_d;
  } solver;

  BumpResource _bump;

  // state
  struct state_t {
    packed<real_t> q;
    packed<real_t> v;
    packed<real_t> vhat;
  } state;

  // bodies (nbody)
  struct body_t {
    vector<Vector3r>    pos;
    vector<Matrix3r>    rot;
    vector<Quaternionr> quat;
    vector<Vector3r>    ipos;
    vector<Matrix3r>    irot;

    vector<Vector6r>    v;
    vector<Vector6r>    vhat;
    vector<Vector6r>    cori;
  } body;

  // dofs (nv)
  struct dof_t {
    vector<Vector6r> screw;

    packed<real_t>   f_bias;
    packed<real_t>   f_con;
  } dof;

  // frictions (nfric)
  struct fric_t {
    packed<real_t> lambda;
  } fric;

  // limits (nlim)
  struct lim_t {
    vector<real_t> upper;
    vector<real_t> lower;
    packed<real_t> lambda;
  } lim;

  // geometries (ngeom)
  struct geom_t {
    vector<Vector3r> pos;
    vector<Matrix3r> rot;

    vector<Vector6r> aabb;
  } geom;

  // dynamics
  struct dynamics_t {
    vector<Vector6r>      sp_f;
    vector<Vector10r>     sp_I;

    vector<Vector6r>      rne_f;
    vector<Vector10r>     crb_I;

    unique<SparseMatrixr> M;
    unique<SparseMatrixr> M_UD;
    packed<real_t>        M_Dinv;
  } dyn;

  // actuators
  struct actuator_t {
    packed<real_t> u;
    packed<real_t> udot;
    packed<real_t> f;
  } act;

  StackResource _stack;

  // contacts
  struct contact_t {
    vector<broad_pair_t>   pair;
    vector<con_feature_t>  feature;
    vector<int>            card;
    vector<real_t>         mu;
    vector<real_t>         R;

    vector<Matrix3r>       rot;
    packed<Vector3r>       e;
    packed<Vector3r>       lambda;

    unique<MatrixXr>       J_d;
    unique<SparseMatrixrR> J_s;
  } con;

  // PGS solver
  struct pgs_t {
    packed<real_t>        b;
    packed<real_t>        Dinv;
    unique<MatrixXr>      MinvJT_d;
    unique<SparseMatrixr> MinvJT_s;
  } pgs;

  // CANAL solver
  struct canal_t {
    packed<Vector3r>      e;
    packed<Vector3r>      z;
    packed<Vector3r>      u;
    packed<Vector3r>      u_old;
    real_t                cost;
    unique<VectorXr>      grad;
    unique<MatrixXr>      hess_d;
    unique<SparseMatrixr> hess_s;
    real_t                beta;

    real_t                beta_lim;
    packed<real_t>        u_lim;
    packed<real_t>        u_lim_old;

    real_t                beta_fric;
    packed<real_t>        u_fric;
    packed<real_t>        u_fric_old;
  } canal;

  // SubADMM solver
  struct sub_admm_t {
    packed<Vector3r>      z;
    packed<Vector3r>      u;
    packed<Vector3r>      u_old;
    real_t                beta;

    real_t                beta_lim;
    packed<real_t>        u_lim;
    packed<real_t>        u_lim_old;
    packed<real_t>        z_lim;

    real_t                beta_fric;
    packed<real_t>        u_fric;
    packed<real_t>        u_fric_old;
    packed<real_t>        z_fric;

    unique<SparseMatrixr> M_orig;
    vector<Vector10r>     sp_I_add;
  } admm;

  HeapResource _heap;
};

std::unique_ptr<data_t> make_data(model_t const& m);
}  // namespace crisp
