/*
 * CRISP: Contact-Rich Simulation Platform
 * Copyright (c) 2026 The CRISP authors.
 * Licensed for academic and non-commercial use only.
 * See the LICENSE file for details.
 */

#pragma once

#include <crisp/arena/types.ipp>

#include <array>
#include <memory>

namespace crisp {
template <typename T>
class packed;

template <arithmetic Scalar>
class packed<Scalar> {
private:
  arena::aligned_buffer<Scalar> buffer_;

public:
  packed(arena::Resource& pool, int size): buffer_(pool, size) {}

  packed() = default;
  ~packed() = default;

  packed(packed const&) = delete;
  packed& operator=(packed const&) = delete;

  packed(packed&&) noexcept = default;
  packed& operator=(packed&&) noexcept = default;

  Scalar operator[](int idx) const {
    crisp_assert(0 <= idx && idx < buffer_.size(), "idx out of range");
    return buffer_[idx];
  }
  Scalar& operator[](int idx) {
    crisp_assert(0 <= idx && idx < buffer_.size(), "idx out of range");
    return buffer_[idx];
  }

  auto operator()() const { return e(); }
  auto operator()() { return e(); }

  Scalar const* data() const { return buffer_.data(); }
  Scalar* data() { return buffer_.data(); }

  int size() const { return buffer_.size(); }

  auto e() const {
    return Eigen::Map<Eigen::Matrix<Scalar, -1, 1> const, align_bytes>(
      buffer_.data(), buffer_.size());
  }
  auto e() {
    return Eigen::Map<Eigen::Matrix<Scalar, -1, 1>, align_bytes>(
      buffer_.data(), buffer_.size());
  }
};

template <arithmetic Scalar>
struct arena::traits<packed<Scalar>> {
  template <int Footer>
  static constexpr std::size_t bytes(int size) {
    return ::crisp::arena::bytes<aligned_buffer<Scalar>, Footer>(size);
  }
  static constexpr std::size_t align =
    ::crisp::arena::align<aligned_buffer<Scalar>>;
};

template <arithmetic Scalar, int Rows, int Cols, order_e Order>
class packed<Matrix<Scalar, Rows, Cols, Order>> {
  using traits = internal::traits<Matrix<Scalar, Rows, Cols>>;

private:
  std::conditional_t<
    traits::IsDynamic, arena::aligned_buffer<Scalar>,
    arena::aligned_buffer<
      std::array<Scalar, static_cast<std::size_t>(Rows* Cols)>>>
    buffer_;
  AxisSize<Rows> rows_;
  AxisSize<Cols> cols_;

public:
  packed(arena::Resource& pool, int rows, int cols, int size)
    requires(traits::IsDynamic)
      : buffer_(pool, rows * cols * size), rows_(rows), cols_(cols) {}

  packed(arena::Resource& pool, int size)
    requires(!traits::IsDynamic)
      : buffer_(pool, size) {}

  packed() = default;
  ~packed() = default;

  packed(packed const&) = delete;
  packed& operator=(packed const&) = delete;

  packed(packed&&) noexcept = default;
  packed& operator=(packed&&) noexcept = default;

  auto operator[](int idx) const {
    crisp_assert(0 <= idx && idx < buffer_.size(), "idx out of range");
    if constexpr (!traits::IsDynamic)
      return Eigen::Map<
        Eigen::Matrix<Scalar, Rows, Cols, StorageOrder<Order>> const>(
        buffer_[idx].data());
    else
      return Eigen::Map<
        Eigen::Matrix<Scalar, Rows, Cols, StorageOrder<Order>> const>(
        buffer_.data() + rows_.get() * cols_.get() * idx,  //
        rows_.get(), cols_.get());
  }
  auto operator[](int idx) {
    crisp_assert(0 <= idx && idx < buffer_.size(), "idx out of range");
    if constexpr (!traits::IsDynamic)
      return Eigen::Map<Eigen::Matrix<Scalar, Rows, Cols, StorageOrder<Order>>>(
        buffer_[idx].data());
    else
      return Eigen::Map<Eigen::Matrix<Scalar, Rows, Cols, StorageOrder<Order>>>(
        buffer_.data() + rows_.get() * cols_.get() * idx,  //
        rows_.get(), cols_.get());
  }

  auto operator()() const {
    if constexpr (traits::IsVector) return e_vec();
    else return e();
  }
  auto operator()() {
    if constexpr (traits::IsVector) return e_vec();
    else return e();
  }

  auto* data() const { return buffer_.data(); }
  auto* data() { return buffer_.data(); }

  int rows() const { return rows_.get(); }
  int cols() const { return cols_.get(); }
  int size() const { return buffer_.size(); }

  auto e() const {
    if constexpr (traits::IsDynamic) {
      if constexpr (Order == order_e::row_major)
        return Eigen::Map<
          Eigen::Matrix<Scalar, -1, Cols, Eigen::RowMajor> const, align_bytes>(
          buffer_.data(),
          buffer_.size() == 0 ? 0
                              : static_cast<int>(buffer_.size() / cols_.get()),
          cols_.get());
      else
        return Eigen::Map<
          Eigen::Matrix<Scalar, Rows, -1, Eigen::ColMajor> const, align_bytes>(
          buffer_.data(),  //
          rows_.get(),
          buffer_.size() == 0 ? 0
                              : static_cast<int>(buffer_.size() / rows_.get()));
    } else if constexpr (traits::IsVector) {
      if constexpr (Rows == 1)
        return Eigen::Map<
          Eigen::Matrix<Scalar, -1, Cols, Eigen::RowMajor> const, align_bytes>(
          buffer_[0].data(), static_cast<int>(buffer_.size()), Cols);
      else
        return Eigen::Map<
          Eigen::Matrix<Scalar, Rows, -1, Eigen::ColMajor> const, align_bytes>(
          buffer_[0].data(), Rows, static_cast<int>(buffer_.size()));
    } else {
      if constexpr (Order == order_e::row_major)
        return Eigen::Map<
          Eigen::Matrix<Scalar, -1, Cols, Eigen::RowMajor> const, align_bytes>(
          buffer_[0].data(), static_cast<int>(Rows * buffer_.size()), Cols);
      else
        return Eigen::Map<
          Eigen::Matrix<Scalar, Rows, -1, Eigen::ColMajor> const, align_bytes>(
          buffer_[0].data(), Rows, static_cast<int>(Cols * buffer_.size()));
    }
  }
  auto e() {
    if constexpr (traits::IsDynamic) {
      if constexpr (Order == order_e::row_major)
        return Eigen::Map<
          Eigen::Matrix<Scalar, -1, Cols, Eigen::RowMajor>, align_bytes>(
          buffer_.data(),
          buffer_.size() == 0 ? 0
                              : static_cast<int>(buffer_.size() / cols_.get()),
          cols_.get());
      else
        return Eigen::Map<
          Eigen::Matrix<Scalar, Rows, -1, Eigen::ColMajor>, align_bytes>(
          buffer_.data(),  //
          rows_.get(),
          buffer_.size() == 0 ? 0
                              : static_cast<int>(buffer_.size() / rows_.get()));
    } else if constexpr (traits::IsVector) {
      if constexpr (Rows == 1)
        return Eigen::Map<
          Eigen::Matrix<Scalar, -1, Cols, Eigen::RowMajor>, align_bytes>(
          buffer_[0].data(), static_cast<int>(buffer_.size()), Cols);
      else
        return Eigen::Map<
          Eigen::Matrix<Scalar, Rows, -1, Eigen::ColMajor>, align_bytes>(
          buffer_[0].data(), Rows, static_cast<int>(buffer_.size()));
    } else {
      if constexpr (Order == order_e::row_major)
        return Eigen::Map<
          Eigen::Matrix<Scalar, -1, Cols, Eigen::RowMajor>, align_bytes>(
          buffer_[0].data(), static_cast<int>(Rows * buffer_.size()), Cols);
      else
        return Eigen::Map<
          Eigen::Matrix<Scalar, Rows, -1, Eigen::ColMajor>, align_bytes>(
          buffer_[0].data(), Rows, static_cast<int>(Cols * buffer_.size()));
    }
  }

  auto e_vec() const
    requires(traits::IsVector)
  {
    if constexpr (Rows == 1)
      return Eigen::Map<
        Eigen::Matrix<Scalar, 1, -1, Eigen::RowMajor> const, align_bytes>(
        buffer_[0].data(), static_cast<int>(Cols * buffer_.size()));
    else
      return Eigen::Map<
        Eigen::Matrix<Scalar, -1, 1, Eigen::ColMajor> const, align_bytes>(
        buffer_[0].data(), static_cast<int>(Rows * buffer_.size()));
  }
  auto e_vec()
    requires(traits::IsVector)
  {
    if constexpr (Rows == 1)
      return Eigen::Map<
        Eigen::Matrix<Scalar, 1, -1, Eigen::RowMajor>, align_bytes>(
        buffer_[0].data(), static_cast<int>(Cols * buffer_.size()));
    else
      return Eigen::Map<
        Eigen::Matrix<Scalar, -1, 1, Eigen::ColMajor>, align_bytes>(
        buffer_[0].data(), static_cast<int>(Rows * buffer_.size()));
  }
};

template <arithmetic Scalar, int Rows, int Cols, order_e Order>
struct arena::traits<packed<::crisp::Matrix<Scalar, Rows, Cols, Order>>> {
  template <int Footer>
  static constexpr std::size_t bytes(int rows, int cols, int size)
    requires(internal::traits<::crisp::Matrix<Scalar, Rows, Cols>>::IsDynamic)
  {
    return ::crisp::arena::bytes<  //
      aligned_buffer<Scalar>, Footer>(rows * cols * size);
  }
  template <int Footer>
  static constexpr std::size_t bytes(int size)
    requires(!internal::traits<::crisp::Matrix<Scalar, Rows, Cols>>::IsDynamic)
  {
    return ::crisp::arena::bytes<
      aligned_buffer<std::array<Scalar, static_cast<std::size_t>(Rows * Cols)>>,
      Footer>(size);
  }
  static constexpr std::size_t align = [] {
    using _traits = internal::traits<::crisp::Matrix<Scalar, Rows, Cols>>;
    if constexpr (_traits::IsDynamic)
      return ::crisp::arena::align<aligned_buffer<Scalar>>;
    else
      return ::crisp::arena::align<aligned_buffer<
        std::array<Scalar, static_cast<std::size_t>(Rows * Cols)>>>;
  }();
};
}  // namespace crisp
