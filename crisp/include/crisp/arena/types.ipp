#pragma once

#include <crisp/arena/types.hpp>
#include <crisp/arena/types/dense.ipp>
#include <crisp/arena/types/quaternion.ipp>
#include <crisp/arena/types/sparse.ipp>

#include <concepts>
#include <sstream>

#include <spdlog/fmt/fmt.h>

#define EIGEN_MAKE_BASIC_MATRIX_TYPES(Type, TypeSuffix, Size, SizeSuffix) \
  namespace Eigen {                                                       \
  using Matrix##SizeSuffix##TypeSuffix = Matrix<Type, Size, Size>;        \
  using Vector##SizeSuffix##TypeSuffix = Matrix<Type, Size, 1>;           \
  }
#define EIGEN_MAKE_MIXED_MATRIX_TYPES(Type, TypeSuffix, Size)      \
  namespace Eigen {                                                \
  using Matrix##Size##X##TypeSuffix = Matrix<Type, Size, Dynamic>; \
  using Matrix##X##Size##TypeSuffix = Matrix<Type, Dynamic, Size>; \
  }
#define EIGEN_MAKE_MATRIX_TYPES(Type, TypeSuffix, Size)       \
  EIGEN_MAKE_BASIC_MATRIX_TYPES(Type, TypeSuffix, Size, Size) \
  EIGEN_MAKE_MIXED_MATRIX_TYPES(Type, TypeSuffix, Size)
#define EIGEN_MAKE_ALL_MATRIX_TYPES(Type, TypeSuffix) \
  EIGEN_MAKE_MATRIX_TYPES(Type, TypeSuffix, 2)        \
  EIGEN_MAKE_MATRIX_TYPES(Type, TypeSuffix, 3)        \
  EIGEN_MAKE_MATRIX_TYPES(Type, TypeSuffix, 4)        \
  EIGEN_MAKE_MATRIX_TYPES(Type, TypeSuffix, 5)        \
  EIGEN_MAKE_MATRIX_TYPES(Type, TypeSuffix, 6)        \
  EIGEN_MAKE_MATRIX_TYPES(Type, TypeSuffix, 7)        \
  EIGEN_MAKE_MATRIX_TYPES(Type, TypeSuffix, 8)        \
  EIGEN_MAKE_MATRIX_TYPES(Type, TypeSuffix, 9)        \
  EIGEN_MAKE_MATRIX_TYPES(Type, TypeSuffix, 10)       \
  EIGEN_MAKE_BASIC_MATRIX_TYPES(Type, TypeSuffix, Dynamic, X)

EIGEN_MAKE_ALL_MATRIX_TYPES(::crisp::real_t, r)

#undef EIGEN_MAKE_ALL_MATRIX_TYPES
#undef EIGEN_MAKE_MATRIX_TYPES
#undef EIGEN_MAKE_MIXED_MATRIX_TYPES
#undef EIGEN_MAKE_BASIC_MATRIX_TYPES

#define EIGEN_MAKE_ALL_QUAT_TYPE(Type, TypeSuffix) \
  namespace Eigen {                                \
  using Quaternion##TypeSuffix = Quaternion<Type>; \
  }

EIGEN_MAKE_ALL_QUAT_TYPE(::crisp::real_t, r)

#undef EIGEN_MAKE_ALL_QUAT_TYPE

#define EIGEN_MAKE_ALL_SPARSE_MATRIX_TYPE(Type, TypeSuffix) \
  namespace Eigen {                                         \
  using SparseMatrix##TypeSuffix = SparseMatrix<Type>;      \
  }

EIGEN_MAKE_ALL_SPARSE_MATRIX_TYPE(float, f)
EIGEN_MAKE_ALL_SPARSE_MATRIX_TYPE(double, d)
EIGEN_MAKE_ALL_SPARSE_MATRIX_TYPE(::crisp::real_t, r)

#undef EIGEN_MAKE_ALL_SPARSE_MATRIX_TYPE

namespace fmt {
template <typename Derived>
  requires(
    std::derived_from<Derived, Eigen::EigenBase<Derived>> ||
    std::derived_from<Derived, Eigen::RotationBase<Derived, 3>>)
struct formatter<Derived> {
  constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }
  auto format(Derived const& value, format_context& ctx) const {
    std::ostringstream oss;
    oss << value;
    return fmt::format_to(ctx.out(), "{}", oss.str());
  }
};
}  // namespace fmt
