/*
 * CRISP: Contact-Rich Simulation Platform
 * Copyright (c) 2026 The CRISP authors.
 * Licensed for academic and non-commercial use only.
 * See the LICENSE file for details.
 */

#pragma once

#include <crisp/arena/types/fwd.hpp>

#include <Eigen/Geometry>

namespace crisp {
template <floating_point Scalar>
struct internal::eigen_traits<Quaternion<Scalar>> {
  using type = Eigen::Quaternion<Scalar>;
};
}  // namespace crisp
