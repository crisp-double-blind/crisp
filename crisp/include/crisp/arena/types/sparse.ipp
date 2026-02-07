#pragma once

#include <crisp/arena/types/sparse.hpp>
#include <crisp/arena/types/eigen.hpp>

#include <Eigen/Sparse>

namespace crisp {
template <arithmetic Scalar, order_e Order>
struct internal::eigen_traits<SparseMatrix<Scalar, Order>> {
  using type = Eigen::SparseMatrix<Scalar, StorageOrder<Order>>;
};

template <arithmetic Scalar, order_e Order>
inline auto arena::SparseMatrix<Scalar, Order>::e() const {
  return Eigen::Map<Eigen::SparseMatrix<Scalar, StorageOrder<Order>> const>(
    rows(), cols(), nonZeros(), outerIndexPtr(), innerIndexPtr(), valuePtr());
}

template <arithmetic Scalar, order_e Order>
inline auto arena::SparseMatrix<Scalar, Order>::e() {
  return Eigen::Map<Eigen::SparseMatrix<Scalar, StorageOrder<Order>>>(
    rows(), cols(), nonZeros(), outerIndexPtr(), innerIndexPtr(), valuePtr());
}

template <arithmetic Scalar, order_e Order>
template <class OtherDerived>
arena::SparseMatrix<Scalar, Order>::SparseMatrix(
  arena::Resource& pool, Eigen::SparseMatrixBase<OtherDerived> const& xpr)
    : outer_size_(static_cast<int>(IsRowMajor ? xpr.rows() : xpr.cols())),
      inner_size_(static_cast<int>(IsRowMajor ? xpr.cols() : xpr.rows())) {
  auto& src = xpr.derived();
  if constexpr (
    std::is_same_v<Scalar, typename OtherDerived::Scalar> &&
    (IsRowMajor == OtherDerived::IsRowMajor) &&
    (requires { src.isCompressed(); })) {
    if (src.isCompressed()) {
      int nnz = static_cast<int>(src.nonZeros());
      values_ = unaligned_buffer<Scalar>(pool, nnz);
      inner_indices_ = unaligned_buffer<int>(pool, nnz);
      outer_index_ = unaligned_buffer<int>(pool, outer_size_ + 1);

      std::copy_n(src.valuePtr(), nnz, values_.data());
      std::copy_n(src.innerIndexPtr(), nnz, inner_indices_.data());
      std::copy_n(src.outerIndexPtr(), outer_size_ + 1, outer_index_.data());
      return;
    }
  }

  using Evaluator = Eigen::internal::evaluator<OtherDerived>;
  Evaluator evaluator(src);

  int nnz = 0;
  if constexpr (requires { src.nonZeros(); }) {
    nnz = static_cast<int>(src.nonZeros());
  } else {
    for (Eigen::Index outer = 0; outer < xpr.outerSize(); ++outer)
      for (typename Evaluator::InnerIterator it(evaluator, outer); it; ++it)
        ++nnz;
  }
  values_ = unaligned_buffer<Scalar>(pool, nnz);
  outer_index_ = unaligned_buffer<int>(pool, outer_size_ + 1);
  inner_indices_ = unaligned_buffer<int>(pool, nnz);

  if constexpr (IsRowMajor == OtherDerived::IsRowMajor) {
    int idx = 0;
    for (Eigen::Index outer = 0; outer < xpr.outerSize(); ++outer) {
      outer_index_[static_cast<int>(outer)] = idx;
      for (typename Evaluator::InnerIterator it(evaluator, outer); it; ++it) {
        values_[idx] = static_cast<Scalar>(it.value());
        inner_indices_[idx++] =
          static_cast<int>(IsRowMajor ? it.col() : it.row());
      }
    }
    outer_index_.back() = idx;
  } else {
    SparseMatrix<Scalar, OtherOrder> tr_mat(
      pool, static_cast<int>(xpr.rows()), static_cast<int>(xpr.cols()), nnz);
    int idx = 0;
    for (Eigen::Index outer = 0; outer < xpr.outerSize(); ++outer) {
      tr_mat.outer_index_[static_cast<int>(outer)] = idx;
      for (typename Evaluator::InnerIterator it(evaluator, outer); it; ++it) {
        tr_mat.values_[idx] = static_cast<Scalar>(it.value());
        tr_mat.inner_indices_[idx++] =
          static_cast<int>(tr_mat.IsRowMajor ? it.col() : it.row());
      }
    }
    tr_mat.outer_index_.back() = idx;
    *this = tr_mat;
  }
}

template <arithmetic Scalar, order_e Order>
template <class OtherDerived>
inline arena::SparseMatrix<Scalar, Order>&
arena::SparseMatrix<Scalar, Order>::operator=(
  Eigen::SparseMatrixBase<OtherDerived> const& xpr) {
  static_assert(
    IsRowMajor == OtherDerived::IsRowMajor, "storage order mismatch");
  crisp_assert(
    rows() == xpr.rows() && cols() == xpr.cols(), "matrix shape mismatch");

  auto& src = xpr.derived();
  if constexpr (
    std::is_same_v<Scalar, typename OtherDerived::Scalar> &&
    (requires { src.isCompressed(); })) {
    if (src.isCompressed()) {
      crisp_assert(nonZeros() == src.nonZeros(), "nnz mismatch");
      std::copy_n(src.valuePtr(), nonZeros(), values_.data());
      std::copy_n(src.innerIndexPtr(), nonZeros(), inner_indices_.data());
      std::copy_n(src.outerIndexPtr(), outer_size_ + 1, outer_index_.data());
      return *this;
    }
  }

  using Evaluator = Eigen::internal::evaluator<OtherDerived>;
  Evaluator evaluator(src);

  int nnz = 0;
  if constexpr (requires { src.nonZeros(); }) {
    nnz = static_cast<int>(src.nonZeros());
  } else {
    for (Eigen::Index outer = 0; outer < outer_size_; ++outer)
      for (typename Evaluator::InnerIterator it(evaluator, outer); it; ++it)
        ++nnz;
  }
  crisp_assert(nonZeros() == nnz, "nnz mismatch");

  int idx = 0;
  for (Eigen::Index outer = 0; outer < outer_size_; ++outer) {
    outer_index_[static_cast<int>(outer)] = idx;
    for (typename Evaluator::InnerIterator it(evaluator, outer); it; ++it) {
      values_[idx] = static_cast<Scalar>(it.value());
      inner_indices_[idx++] =
        static_cast<int>(IsRowMajor ? it.col() : it.row());
    }
  }
  outer_index_.back() = idx;
  return *this;
}
}  // namespace crisp
