/*
 * CRISP: Contact-Rich Simulation Platform
 * Copyright (c) 2026 The CRISP authors.
 * Licensed for academic and non-commercial use only.
 * See the LICENSE file for details.
 */

#pragma once

#include <crisp/app.hpp>
#include <crisp/data.hpp>
#include <crisp/model.hpp>
#include <crisp/render.hpp>
#include <crisp/user.hpp>

#include <functional>

namespace crisp {
CRISP_API void forward(model_t const& m, data_t& d);

CRISP_API void reset(model_t const& m, data_t& d);

CRISP_API void step(model_t const& m, data_t& d);
CRISP_API void step1(model_t const& m, data_t& d);
CRISP_API void step2(model_t const& m, data_t& d);

CRISP_API void apply_option(model_t& m, model_t::option_t const& opt);

struct signed_distance_t {
  real_t phi;
  Eigen::Vector3r grad;
};
using sdf_eval_t = std::function<signed_distance_t(
  Ref<Vector3r const> x, Ref<VectorXr const> p)>;
using sdf_aabb_t =
  std::function<Eigen::Vector6r(Ref<Matrix3r const>, Ref<VectorXr const>)>;
CRISP_API int register_sdf(
  sdf_eval_t&& sdf_eval, sdf_aabb_t&& sdf_aabb, int nparam = 0);
}  // namespace crisp
