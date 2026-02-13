/*
 * CRISP: Contact-Rich Simulation Platform
 * Copyright (c) 2026 The CRISP authors.
 * Licensed for academic and non-commercial use only.
 * See the LICENSE file for details.
 */

#pragma once

#include <crisp/arena/types/buffer.hpp>
#include <crisp/arena/types/fwd.hpp>

#include <memory>

namespace Eigen {
template <class Derived>
class SparseMatrixBase;
}  // namespace Eigen

namespace crisp::internal {
template <arithmetic _Scalar, order_e _Order>
struct traits<SparseMatrix<_Scalar, _Order>> {
  using Scalar = _Scalar;
  static constexpr auto Order = _Order;
  static constexpr bool IsRowMajor = (Order == order_e::row_major);
  static constexpr bool IsConst = false;
  static constexpr bool IsResizable = true;
};
}  // namespace crisp::internal

namespace crisp {
template <arithmetic Scalar>
struct basic_triplet_t {
  int row, col;
  Scalar value;
};
}  // namespace crisp

namespace crisp::arena {
template <arithmetic _Scalar, order_e _Order>
class SparseMatrix {
  using traits = internal::traits<::crisp::SparseMatrix<_Scalar, _Order>>;

  template <arithmetic __Scalar, order_e __Order>
  friend class SparseMatrix;

public:
  using Scalar = traits::Scalar;
  static constexpr auto Order = traits::Order;
  static constexpr bool IsRowMajor = traits::IsRowMajor;
  static constexpr bool IsConst = traits::IsConst;
  static constexpr bool IsResizable = traits::IsResizable;

private:
  unaligned_buffer<Scalar> values_;
  unaligned_buffer<int> outer_index_, inner_indices_;
  int outer_size_, inner_size_;
  bool need_manual_detruct_ = false;

public:
  SparseMatrix() = default;

  SparseMatrix(arena::Resource& pool, int rows, int cols, int non_zeros)
      : values_(pool, non_zeros),
        outer_index_(pool, (IsRowMajor ? rows : cols) + 1, 0),
        inner_indices_(pool, non_zeros, -1),
        outer_size_(IsRowMajor ? rows : cols),
        inner_size_(IsRowMajor ? cols : rows) {}

  ~SparseMatrix() {
    if (need_manual_detruct_) {
      if (auto* pool = inner_indices_.pool()) {
        static_cast<StackResource*>(pool)->set_cursor_unsafe(
          reinterpret_cast<byte_t*>(inner_indices_.data()) +
          bytes<unaligned_buffer<int>, 1>(inner_indices_.capacity()));
      }
    }
  }

  SparseMatrix(SparseMatrix const&) = delete;
  SparseMatrix& operator=(SparseMatrix const& other) {
    if (this == &other) return *this;

    crisp_assert(rows() == other.rows() && cols() == other.cols());
    values_.resize(other.nonZeros());
    inner_indices_.resize(other.nonZeros());

    std::copy_n(other.values_.data(), other.values_.size(), values_.data());
    std::copy_n(
      other.inner_indices_.data(), other.inner_indices_.size(),
      inner_indices_.data());
    std::copy_n(
      other.outer_index_.data(), other.outer_index_.size(),
      outer_index_.data());
    return *this;
  }

  SparseMatrix(SparseMatrix&&) noexcept = default;
  SparseMatrix& operator=(SparseMatrix&&) noexcept = default;

  static constexpr auto OtherOrder =
    IsRowMajor ? order_e::col_major : order_e::row_major;
  SparseMatrix& operator=(SparseMatrix<Scalar, OtherOrder> const& other) {
    crisp_assert(rows() == other.rows() && cols() == other.cols());
    values_.resize(other.nonZeros());
    inner_indices_.resize(other.nonZeros());

    // pass 1: count the nnz per inner-vector
    std::fill_n(outer_index_.data(), outer_size_ + 1, 0);
    for (int outer = 0; outer < other.outer_size_; ++outer) {
      int start = other.outer_index_[outer];
      int end = other.outer_index_[outer + 1];
      for (int idx = start; idx < end; ++idx) {
        ++outer_index_[other.inner_indices_[idx] + 1];
      }
    }
    for (int _outer = 0; _outer < outer_size_; ++_outer) {
      outer_index_[_outer + 1] += outer_index_[_outer];
    }

    // pass 2: do the actual copy
    for (int outer = 0; outer < other.outer_size_; ++outer) {
      int start = other.outer_index_[outer];
      int end = other.outer_index_[outer + 1];
      for (int idx = start; idx < end; ++idx) {
        int pos = outer_index_[other.inner_indices_[idx]]++;
        inner_indices_[pos] = outer;
        values_[pos] = other.values_[idx];
      }
    }
    for (int _outer = outer_size_; _outer > 0; --_outer) {
      outer_index_[_outer] = outer_index_[_outer - 1];
    }
    outer_index_[0] = 0;

    return *this;
  }

  template <class OtherDerived>
  SparseMatrix(
    arena::Resource& pool, Eigen::SparseMatrixBase<OtherDerived> const& xpr);
  template <class OtherDerived>
  SparseMatrix& operator=(Eigen::SparseMatrixBase<OtherDerived> const& xpr);

  Scalar operator()(int row, int col) const {
    if (auto* ptr = find(row, col)) return *ptr;
    return 0;
  }
  Scalar& operator()(int row, int col) {
    if (auto* ptr = find(row, col)) return const_cast<Scalar&>(*ptr);
    crisp_assert(false, "non-existent element referenced");
    static Scalar dummy;
    return dummy = 0;
  }

  Scalar operator[](int idx) const {
    crisp_assert(0 <= idx && idx < values_.size(), "idx out of range");
    return values_[idx];
  }
  Scalar& operator[](int idx) {
    crisp_assert(0 <= idx && idx < values_.size(), "idx out of range");
    return values_[idx];
  }

  auto operator()() const { return e(); }
  auto operator()() { return e(); }

  auto e() const;
  auto e();

  Scalar const* valuePtr() const { return values_.data(); }
  Scalar* valuePtr() { return values_.data(); }
  int const* innerIndexPtr() const { return inner_indices_.data(); }
  int* innerIndexPtr() { return inner_indices_.data(); }
  int const* outerIndexPtr() const { return outer_index_.data(); }
  int* outerIndexPtr() { return outer_index_.data(); }

  int innerSize() const { return inner_size_; }
  int outerSize() const { return outer_size_; }
  int nonZeros() const { return static_cast<int>(values_.size()); }

  int rows() const { return IsRowMajor ? outer_size_ : inner_size_; }
  int cols() const { return IsRowMajor ? inner_size_ : outer_size_; }

  Scalar& insert(int row, int col) {
    crisp_assert(0 <= row && row < rows(), "invalid row");
    crisp_assert(0 <= col && col < cols(), "invalid col");
    int outer = IsRowMajor ? row : col;
    int inner = IsRowMajor ? col : row;
    int start = outer_index_[outer];
    int end = outer_index_[outer + 1];
    int insert_pos = start;
    while (insert_pos < end && inner_indices_[insert_pos] < inner)
      ++insert_pos;
    if (insert_pos < end && inner_indices_[insert_pos] == inner)
      return values_[insert_pos];
    int hole = -1;
    for (int i = insert_pos; i < inner_indices_.size(); ++i) {
      if (inner_indices_[i] == -1) {
        hole = i;
        break;
      }
    }
    crisp_assert(hole != -1, "no free slot left");
    for (int i = hole; i > insert_pos; --i) {
      values_[i] = values_[i - 1];
      inner_indices_[i] = inner_indices_[i - 1];
    }
    values_[insert_pos] = 0;
    inner_indices_[insert_pos] = inner;
    for (int i = outer + 1; i <= outer_size_; ++i)
      ++outer_index_[i];
    return values_[insert_pos];
  }

  using Triplet = basic_triplet_t<Scalar>;
  void setFromTriplets(
    arena::Resource& pool, unaligned_buffer<Triplet> const& triplets) {
    // pass 1-3: make temporary transposed matrix from triplets
    auto tr_mat =
      MakeTransposedMatrixFromTriplets(pool, rows(), cols(), triplets);

    // pass 4: transposed copy -> implicit sorting
    *this = tr_mat;
  }

  void setFromTriplets(
    StackResource& pool, int rows, int cols,
    unaligned_buffer<Triplet>&& triplets) {
    *this = MakeFromTriplets(pool, rows, cols, std::move(triplets));
  }

  static SparseMatrix MakeFromTriplets(
    StackResource& pool, int rows, int cols,
    unaligned_buffer<Triplet>&& triplets) {
    // different pool -> no in-place reuse
    if (&pool != triplets.pool()) {
      SparseMatrix mat(pool, rows, cols, triplets.size());
      mat.setFromTriplets(pool, triplets);
      return mat;
    }

    // make empty marix if no triplets
    if (triplets.size() == 0) {
      SparseMatrix mat(pool, rows, cols, 0);
      return mat;
    }

    auto bytes_matrix = bytes<::crisp::SparseMatrix<Scalar, Order>, 1>(
      rows, cols, triplets.size());
    auto bytes_triplets = bytes<unaligned_buffer<Triplet>, 1>(  //
      triplets.capacity());
    // if matrix needs less space and results in empty region (i.e., triplets
    // not last block), handle it manually at destruction
    void* mark = nullptr;
    if (bytes_matrix < bytes_triplets) {
      if (!pool.is_last_end(triplets.data() + triplets.capacity())) {
        mark = pool.get_cursor_unsafe();
      }
    }
    // if matrix needs more space (i.e. not enough nnzs), advance pool cursor
    else if (bytes_matrix > bytes_triplets) {
      crisp_assert(pool.is_last_end(triplets.data() + triplets.capacity()));
      pool.set_cursor_unsafe(
        reinterpret_cast<byte_t*>(triplets.data()) + bytes_matrix);
    }

    // make temporary transposed matrix from triplets
    auto tr_mat = MakeTransposedMatrixFromTriplets(pool, rows, cols, triplets);

    // rewind pool cursor and construct final matrix in-place
    pool.set_cursor_unsafe(
      reinterpret_cast<byte_t*>(triplets.data()) + bytes_triplets);
    std::destroy_at(&triplets);
    triplets.release_unsafe();
    SparseMatrix mat(pool, rows, cols, tr_mat.nonZeros());
    mat = tr_mat;

    // release transient buffers and restore pool cursor if needed
    tr_mat.values_.release_unsafe();
    tr_mat.outer_index_.release_unsafe();
    tr_mat.inner_indices_.release_unsafe();
    if (mark) {
      pool.set_cursor_unsafe(mark);
      mat.need_manual_detruct_ = true;
    }
    return mat;
  }

private:
  Scalar const* find(int row, int col) const {
    int outer = IsRowMajor ? row : col;
    int inner = IsRowMajor ? col : row;
    int start = outer_index_[outer];
    int end = outer_index_[outer + 1];
    for (int idx = start; idx < end; ++idx) {
      if (inner_indices_[idx] == inner) return &values_[idx];
    }
    return nullptr;
  }

  static auto MakeTransposedMatrixFromTriplets(
    arena::Resource& pool, int rows, int cols,
    unaligned_buffer<Triplet> const& triplets) {
    SparseMatrix<Scalar, OtherOrder> tr_mat(pool, rows, cols, triplets.size());

    // pass 1: count the nnz per inner-vector
    std::fill_n(tr_mat.outer_index_.data(), tr_mat.outer_size_ + 1, 0);
    for (auto& [row, col, value] : triplets) {
      crisp_assert(0 <= row && row < rows, "invalid row");
      crisp_assert(0 <= col && col < cols, "invalid col");

      int outer = tr_mat.IsRowMajor ? row : col;
      tr_mat.outer_index_[outer + 1]++;
    }
    for (int outer = 0; outer < tr_mat.outer_size_; ++outer) {
      tr_mat.outer_index_[outer + 1] += tr_mat.outer_index_[outer];
    }

    // pass 2: insert all the elements into tr_mat
    unaligned_buffer<int> inner_non_zeros(pool, tr_mat.outer_size_, 0);
    for (auto& [row, col, value] : triplets) {
      int outer = tr_mat.IsRowMajor ? row : col;
      int inner = tr_mat.IsRowMajor ? col : row;
      crisp_assert(
        inner_non_zeros[outer] <=
        tr_mat.outer_index_[outer + 1] - tr_mat.outer_index_[outer]);

      int idx = tr_mat.outer_index_[outer] + inner_non_zeros[outer]++;
      tr_mat.inner_indices_[idx] = inner;
      tr_mat.values_[idx] = value;
    }

    // pass 3: collapse duplicates
    unaligned_buffer<int> first_pos(pool, tr_mat.inner_size_, -1);
    int count = 0;
    for (int outer = 0; outer < tr_mat.outer_size_; ++outer) {
      int start = count;
      for (int idx = tr_mat.outer_index_[outer];
           idx < tr_mat.outer_index_[outer] + inner_non_zeros[outer]; ++idx) {
        int inner = tr_mat.inner_indices_[idx];
        if (first_pos[inner] < start) {
          tr_mat.values_[count] = tr_mat.values_[idx];
          tr_mat.inner_indices_[count] = tr_mat.inner_indices_[idx];
          first_pos[inner] = count++;
        } else {
          tr_mat.values_[first_pos[inner]] += tr_mat.values_[idx];
        }
      }
      tr_mat.outer_index_[outer] = start;
    }
    tr_mat.outer_index_[tr_mat.outer_size_] = count;
    tr_mat.values_.resize(tr_mat.outer_index_[tr_mat.outer_size_]);
    tr_mat.inner_indices_.resize(tr_mat.outer_index_[tr_mat.outer_size_]);

    return tr_mat;
  }
};

template <typename T>
struct traits;

template <arithmetic Scalar, order_e Order>
struct traits<::crisp::SparseMatrix<Scalar, Order>> {
  template <int Footer>
  static constexpr std::size_t bytes(int rows, int cols, int non_zeros) {
    int outer_size = Order == order_e::row_major ? rows : cols;
    return ::crisp::arena::bytes<unaligned_buffer<Scalar>, Footer>(non_zeros) +
      ::crisp::arena::bytes<unaligned_buffer<int>, Footer>(outer_size + 1) +
      ::crisp::arena::bytes<unaligned_buffer<int>, Footer>(non_zeros);
  }
  static constexpr std::size_t align = std::max(
    {::crisp::arena::align<unaligned_buffer<Scalar>>,
     ::crisp::arena::align<unaligned_buffer<int>>});
};
}  // namespace crisp::arena
