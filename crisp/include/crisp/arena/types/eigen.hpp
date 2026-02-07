#pragma once

#include <Eigen/Core>

namespace crisp {
template <typename T>
using Ref = Eigen::Ref<std::conditional_t<
  std::is_const_v<T>,
  typename internal::eigen_traits<std::remove_const_t<T>>::type const,
  typename internal::eigen_traits<std::remove_const_t<T>>::type>>;

template <order_e Order>
inline constexpr int StorageOrder =
  Order == order_e::row_major ? Eigen::RowMajor : Eigen::ColMajor;
}  // namespace crisp
