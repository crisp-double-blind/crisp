#pragma once

#include <crisp/arena/types/fwd.hpp>

#include <Eigen/Geometry>

namespace crisp {
template <floating_point Scalar>
struct internal::eigen_traits<Quaternion<Scalar>> {
  using type = Eigen::Quaternion<Scalar>;
};
}  // namespace crisp
