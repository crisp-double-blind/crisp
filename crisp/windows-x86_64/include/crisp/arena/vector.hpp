/*
 * CRISP: Contact-Rich Simulation Platform
 * Copyright (c) 2026 The CRISP authors.
 * Licensed for academic and non-commercial use only.
 * See the LICENSE file for details.
 */

#pragma once

#include <crisp/arena/types.ipp>

namespace crisp {
template <typename T>
class packed;

template <typename T>
class unique;
}  // namespace crisp

namespace crisp::arena {
template <typename T>
struct vector_traits {
  using elem = T;
};

template <arithmetic Scalar, int Rows, int Cols, order_e Order>
  requires(!internal::traits<::crisp::Matrix<Scalar, Rows, Cols>>::IsDynamic)
struct vector_traits<::crisp::Matrix<Scalar, Rows, Cols, Order>> {
  using elem = Eigen::Matrix<Scalar, Rows, Cols, StorageOrder<Order>>;
};

template <arithmetic Scalar, int Rows, int Cols, order_e Order>
  requires(internal::traits<::crisp::Matrix<Scalar, Rows, Cols>>::IsDynamic)
struct buffer_traits<Matrix<Scalar, Rows, Cols, Order>> {
  static constexpr auto Policy = buffer_policy_e::reserve_only;
};
template <arithmetic Scalar, int Rows, int Cols, order_e Order>
  requires(internal::traits<::crisp::Matrix<Scalar, Rows, Cols>>::IsDynamic)
struct vector_traits<::crisp::Matrix<Scalar, Rows, Cols, Order>> {
  using elem = Matrix<Scalar, Rows, Cols, Order>;
};

template <floating_point Scalar>
struct vector_traits<::crisp::Quaternion<Scalar>> {
  using elem = Eigen::Quaternion<Scalar>;
};

template <arithmetic Scalar, order_e Order>
struct buffer_traits<SparseMatrix<Scalar, Order>> {
  static constexpr auto Policy = buffer_policy_e::reserve_only;
};
template <arithmetic Scalar, order_e Order>
struct vector_traits<::crisp::SparseMatrix<Scalar, Order>> {
  using elem = SparseMatrix<Scalar, Order>;
};

template <typename T, buffer_policy_e _Policy>
struct buffer_traits<unaligned_buffer<T, _Policy>> {
  static constexpr auto Policy = buffer_policy_e::reserve_only;
};

template <typename T>
struct buffer_traits<::crisp::packed<T>> {
  static constexpr auto Policy = buffer_policy_e::reserve_only;
};

template <typename T>
struct buffer_traits<::crisp::unique<T>> {
  static constexpr auto Policy = buffer_policy_e::reserve_only;
};
}  // namespace crisp::arena

namespace crisp {
template <
  typename T,
  buffer_policy_e Policy =
    arena::buffer_traits<typename arena::vector_traits<T>::elem>::Policy>
using vector =
  arena::unaligned_buffer<typename arena::vector_traits<T>::elem, Policy>;
}  // namespace crisp
