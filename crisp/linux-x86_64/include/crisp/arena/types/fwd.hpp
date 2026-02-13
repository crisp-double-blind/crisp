/*
 * CRISP: Contact-Rich Simulation Platform
 * Copyright (c) 2026 The CRISP authors.
 * Licensed for academic and non-commercial use only.
 * See the LICENSE file for details.
 */

#pragma once  // clang-format off

#include <type_traits>

namespace crisp::internal {
template <typename T>
struct traits;
template <typename T>
struct eigen_traits;
}  // namespace crisp::internal

namespace crisp {
template <typename T>
concept arithmetic = std::is_arithmetic_v<T>;
template <typename T>
concept floating_point = std::is_floating_point_v<T>;

enum class order_e { row_major, col_major, _default = col_major };

template <arithmetic Scalar, int Rows, int Cols,
  order_e Order = (Rows == 1 && Cols != 1) ? order_e::row_major
                : (Cols == 1 && Rows != 1) ? order_e::col_major
                                           : order_e::_default>
class Matrix;

template <arithmetic Scalar, int Size>
using Vector = Matrix<Scalar, Size, 1>;

template <floating_point Scalar>
class Quaternion;

template <arithmetic Scalar, order_e Order = order_e::_default>
class SparseMatrix;
}  // namespace crisp
