#pragma once

#include <crisp/arena/types/dense.hpp>
#include <crisp/arena/types/eigen.hpp>

#include <Eigen/Dense>

namespace crisp {
template <arithmetic Scalar, int Rows, int Cols, order_e Order>
struct internal::eigen_traits<Matrix<Scalar, Rows, Cols, Order>> {
  using type = Eigen::Matrix<Scalar, Rows, Cols, StorageOrder<Order>>;
};

template <arithmetic Scalar, int Rows, int Cols, order_e Order>
  requires(internal::traits<::crisp::Matrix<Scalar, Rows, Cols>>::IsDynamic)
inline auto arena::Matrix<Scalar, Rows, Cols, Order>::e() const {
  return Eigen::Map<
    Eigen::Matrix<Scalar, Rows, Cols, StorageOrder<Order>> const, Align>(
    data(), rows(), cols());
}

template <arithmetic Scalar, int Rows, int Cols, order_e Order>
  requires(internal::traits<::crisp::Matrix<Scalar, Rows, Cols>>::IsDynamic)
inline auto arena::Matrix<Scalar, Rows, Cols, Order>::e() {
  return Eigen::Map<
    Eigen::Matrix<Scalar, Rows, Cols, StorageOrder<Order>>, Align>(
    data(), rows(), cols());
}

template <arithmetic Scalar, int Rows, int Cols, order_e Order>
  requires(internal::traits<::crisp::Matrix<Scalar, Rows, Cols>>::IsDynamic)
template <class OtherDerived>
arena::Matrix<Scalar, Rows, Cols, Order>::Matrix(
  arena::Resource& pool, Eigen::MatrixBase<OtherDerived> const& xpr)
    : data_(pool, static_cast<int>(xpr.size())),
      rows_(static_cast<int>(xpr.rows())),
      cols_(static_cast<int>(xpr.cols())) {
  e() = xpr;
}

template <arithmetic Scalar, int Rows, int Cols, order_e Order>
  requires(internal::traits<::crisp::Matrix<Scalar, Rows, Cols>>::IsDynamic)
template <class OtherDerived>
inline arena::Matrix<Scalar, Rows, Cols, Order>&
arena::Matrix<Scalar, Rows, Cols, Order>::operator=(
  Eigen::MatrixBase<OtherDerived> const& xpr) {
  e() = xpr;
  return *this;
}
}  // namespace crisp
