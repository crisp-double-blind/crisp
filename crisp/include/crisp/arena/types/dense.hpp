#pragma once

#include <crisp/arena/types/buffer.hpp>
#include <crisp/arena/types/fwd.hpp>

namespace Eigen {
template <class Derived>
class MatrixBase;
}  // namespace Eigen

namespace crisp::internal {
template <arithmetic _Scalar, int _Rows, int _Cols, order_e _Order>
struct traits<Matrix<_Scalar, _Rows, _Cols, _Order>> {
  using Scalar = _Scalar;
  static constexpr int Rows = _Rows;
  static constexpr int Cols = _Cols;
  static constexpr auto Order = _Order;
  static constexpr int Align = align_bytes;
  static constexpr bool IsDynamic = (Rows == -1 || Cols == -1);
  static constexpr bool IsVector = (Rows == 1 || Cols == 1);
  static constexpr bool IsConst = false;
  static constexpr bool IsResizable = IsDynamic;
};

struct void_t {
  void_t() = default;
  void_t(auto) {}
};
template <typename T, bool B>
using variable_if_t = std::conditional_t<B, T, void_t>;
}  // namespace crisp::internal

namespace crisp {
template <int Size>
class AxisSize {
  static_assert(Size >= -1, "Size must be >= -1");

private:
  internal::variable_if_t<int, Size == -1> size_;

public:
  AxisSize(int size): size_(size) {
    if constexpr (Size == -1) crisp_assert(size >= 0, "size must be >= 0");
    else crisp_assert(Size == size, "size differs from fixed size");
  }

  AxisSize() = default;
  ~AxisSize() = default;

  constexpr int get() const {
    if constexpr (Size == -1) return size_;
    else return Size;
  }
  void set(int size)
    requires(Size == -1)
  {
    crisp_assert(size >= 0, "size must be >= 0");
    size_ = size;
  }
};
}  // namespace crisp

namespace crisp::arena {
template <arithmetic _Scalar, int _Rows, int _Cols, order_e _Order>
  requires(internal::traits<::crisp::Matrix<_Scalar, _Rows, _Cols>>::IsDynamic)
class Matrix {
  using traits =
    internal::traits<::crisp::Matrix<_Scalar, _Rows, _Cols, _Order>>;

public:
  using Scalar = traits::Scalar;
  static constexpr int Rows = traits::Rows;
  static constexpr int Cols = traits::Cols;
  static constexpr auto Order = traits::Order;
  static constexpr int Align = traits::Align;
  static constexpr bool IsDynamic = traits::IsDynamic;
  static constexpr bool IsVector = traits::IsVector;
  static constexpr bool IsConst = traits::IsConst;
  static constexpr bool IsResizable = traits::IsResizable;

private:
  aligned_buffer<Scalar> data_;
  AxisSize<Rows> rows_;
  AxisSize<Cols> cols_;

public:
  Matrix(arena::Resource& pool, int rows, int cols)
      : data_(pool, rows * cols), rows_(rows), cols_(cols) {}

  Matrix(arena::Resource& pool, int size)
    requires(IsVector)
      : data_(pool, size),
        rows_(Rows == 1 ? 1 : size),
        cols_(Cols == 1 ? 1 : size) {}

  Matrix() = default;
  ~Matrix() = default;

  Matrix(Matrix const&) = delete;
  Matrix& operator=(Matrix const& other) {
    if (this == &other) return *this;

    if constexpr (Rows == -1) crisp_assert(rows() == other.rows());
    if constexpr (Cols == -1) crisp_assert(cols() == other.cols());

    if constexpr (IsDynamic) {
      std::copy_n(other.data_.data(), other.data_.size(), data_.data());
    } else {
      std::copy_n(other.data_.data(), Rows * Cols, data_.data());
    }
    return *this;
  }

  Matrix(Matrix&&) noexcept = default;
  Matrix& operator=(Matrix&&) noexcept = default;

  template <class OtherDerived>
  Matrix(arena::Resource& pool, Eigen::MatrixBase<OtherDerived> const& xpr);
  template <class OtherDerived>
  Matrix& operator=(Eigen::MatrixBase<OtherDerived> const& xpr);

  Scalar operator()(int row, int col) const {
    crisp_assert(0 <= row && row < rows(), "row out of range");
    crisp_assert(0 <= col && col < cols(), "col out of range");
    return data()[index(row, col)];
  }
  Scalar& operator()(int row, int col) {
    crisp_assert(0 <= row && row < rows(), "invalid row");
    crisp_assert(0 <= col && col < cols(), "invalid col");
    return data()[index(row, col)];
  }

  Scalar operator[](int idx) const {
    crisp_assert(0 <= idx && idx < rows() * cols(), "idx out of range");
    return data()[idx];
  }
  Scalar& operator[](int idx) {
    crisp_assert(0 <= idx && idx < rows() * cols(), "invalid idx");
    return data()[idx];
  }

  auto operator()() const { return e(); }
  auto operator()() { return e(); }

  auto e() const;
  auto e();

  Scalar const* data() const { return data_.data(); }
  Scalar* data() { return data_.data(); }

  constexpr int rows() const { return rows_.get(); }
  constexpr int cols() const { return cols_.get(); }

private:
  std::size_t index(int row, int col) const {
    if constexpr (Order == order_e::row_major) return row * cols() + col;
    else return row + col * rows();
  }
};

template <typename T>
struct traits;

template <arithmetic Scalar, int Rows, int Cols, order_e Order>
  requires(internal::traits<::crisp::Matrix<Scalar, Rows, Cols>>::IsDynamic)
struct traits<::crisp::Matrix<Scalar, Rows, Cols, Order>> {
  template <int Footer>
  static constexpr std::size_t bytes(int rows, int cols) {
    return ::crisp::arena::bytes<aligned_buffer<Scalar>, Footer>(rows * cols);
  }
  template <int Footer>
  static constexpr std::size_t bytes(int size)
    requires(internal::traits<::crisp::Matrix<Scalar, Rows, Cols>>::IsVector)
  {
    return ::crisp::arena::bytes<aligned_buffer<Scalar>, Footer>(size);
  }
  static constexpr std::size_t align =
    ::crisp::arena::align<aligned_buffer<Scalar>>;
};
}  // namespace crisp::arena
