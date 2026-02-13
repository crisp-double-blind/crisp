/*
 * CRISP: Contact-Rich Simulation Platform
 * Copyright (c) 2026 The CRISP authors.
 * Licensed for academic and non-commercial use only.
 * See the LICENSE file for details.
 */

#pragma once

#include <crisp/arena/types/dense.hpp>
#include <crisp/arena/types/sparse.hpp>

namespace crisp {
#if defined(CRISP_SINGLE_PRECISION)
using real_t = float;
#else
using real_t = double;
#endif

using triplet_t = basic_triplet_t<real_t>;
}  // namespace crisp

#define CRISP_MAKE_BASIC_MATRIX_TYPES(Type, TypeSuffix, Size, SizeSuffix) \
  namespace crisp {                                                       \
  using Matrix##SizeSuffix##TypeSuffix = Matrix<Type, Size, Size>;        \
  using Vector##SizeSuffix##TypeSuffix = Matrix<Type, Size, 1>;           \
  using Matrix##SizeSuffix##TypeSuffix##R =                               \
    Matrix<Type, Size, Size, order_e::row_major>;                         \
  using Matrix##SizeSuffix##TypeSuffix##C =                               \
    Matrix<Type, Size, Size, order_e::col_major>;                         \
  }
#define CRISP_MAKE_MIXED_MATRIX_TYPES(Type, TypeSuffix, Size) \
  namespace crisp {                                           \
  using Matrix##Size##X##TypeSuffix = Matrix<Type, Size, -1>; \
  using Matrix##X##Size##TypeSuffix = Matrix<Type, -1, Size>; \
  using Matrix##Size##X##TypeSuffix##R =                      \
    Matrix<Type, Size, -1, order_e::row_major>;               \
  using Matrix##X##Size##TypeSuffix##R =                      \
    Matrix<Type, -1, Size, order_e::row_major>;               \
  using Matrix##Size##X##TypeSuffix##C =                      \
    Matrix<Type, Size, -1, order_e::col_major>;               \
  using Matrix##X##Size##TypeSuffix##C =                      \
    Matrix<Type, -1, Size, order_e::col_major>;               \
  }
#define CRISP_MAKE_MATRIX_TYPES(Type, TypeSuffix, Size)       \
  CRISP_MAKE_BASIC_MATRIX_TYPES(Type, TypeSuffix, Size, Size) \
  CRISP_MAKE_MIXED_MATRIX_TYPES(Type, TypeSuffix, Size)
#define CRISP_MAKE_ALL_MATRIX_TYPES(Type, TypeSuffix) \
  CRISP_MAKE_MATRIX_TYPES(Type, TypeSuffix, 2)        \
  CRISP_MAKE_MATRIX_TYPES(Type, TypeSuffix, 3)        \
  CRISP_MAKE_MATRIX_TYPES(Type, TypeSuffix, 4)        \
  CRISP_MAKE_MATRIX_TYPES(Type, TypeSuffix, 5)        \
  CRISP_MAKE_MATRIX_TYPES(Type, TypeSuffix, 6)        \
  CRISP_MAKE_MATRIX_TYPES(Type, TypeSuffix, 7)        \
  CRISP_MAKE_MATRIX_TYPES(Type, TypeSuffix, 8)        \
  CRISP_MAKE_MATRIX_TYPES(Type, TypeSuffix, 9)        \
  CRISP_MAKE_MATRIX_TYPES(Type, TypeSuffix, 10)       \
  CRISP_MAKE_BASIC_MATRIX_TYPES(Type, TypeSuffix, -1, X)

CRISP_MAKE_ALL_MATRIX_TYPES(int, i)
CRISP_MAKE_ALL_MATRIX_TYPES(float, f)
CRISP_MAKE_ALL_MATRIX_TYPES(double, d)
CRISP_MAKE_ALL_MATRIX_TYPES(real_t, r)

#undef CRISP_MAKE_ALL_MATRIX_TYPES
#undef CRISP_MAKE_MATRIX_TYPES
#undef CRISP_MAKE_MIXED_MATRIX_TYPES
#undef CRISP_MAKE_BASIC_MATRIX_TYPES

#define CRISP_MAKE_ALL_QUAT_TYPE(Type, TypeSuffix) \
  namespace crisp {                                \
  using Quaternion##TypeSuffix = Quaternion<Type>; \
  }

CRISP_MAKE_ALL_QUAT_TYPE(float, f)
CRISP_MAKE_ALL_QUAT_TYPE(double, d)
CRISP_MAKE_ALL_QUAT_TYPE(real_t, r)

#undef CRISP_MAKE_ALL_QUAT_TYPE

#define CRISP_MAKE_ALL_SPARSE_MATRIX_TYPE(Type, TypeSuffix)                   \
  namespace crisp {                                                           \
  using SparseMatrix##TypeSuffix = SparseMatrix<Type>;                        \
  using SparseMatrix##TypeSuffix##R = SparseMatrix<Type, order_e::row_major>; \
  using SparseMatrix##TypeSuffix##C = SparseMatrix<Type, order_e::col_major>; \
  }

CRISP_MAKE_ALL_SPARSE_MATRIX_TYPE(float, f)
CRISP_MAKE_ALL_SPARSE_MATRIX_TYPE(double, d)
CRISP_MAKE_ALL_SPARSE_MATRIX_TYPE(real_t, r)

#undef CRISP_MAKE_ALL_SPARSE_MATRIX_TYPE
