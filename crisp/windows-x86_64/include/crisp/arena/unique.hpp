/*
 * CRISP: Contact-Rich Simulation Platform
 * Copyright (c) 2026 The CRISP authors.
 * Licensed for academic and non-commercial use only.
 * See the LICENSE file for details.
 */

#pragma once

#include <crisp/arena/types.ipp>

#include <memory>

namespace crisp::arena {
template <typename T>
struct unique_traits {
  using elem = T;
  static constexpr bool IsDynamic = false;
};

template <arithmetic Scalar, int Rows, int Cols, order_e Order>
  requires(!internal::traits<::crisp::Matrix<Scalar, Rows, Cols>>::IsDynamic)
struct unique_traits<::crisp::Matrix<Scalar, Rows, Cols, Order>> {
  using elem = Eigen::Matrix<Scalar, Rows, Cols, StorageOrder<Order>>;
  static constexpr bool IsDynamic = false;
};

template <arithmetic Scalar, int Rows, int Cols, order_e Order>
  requires(internal::traits<::crisp::Matrix<Scalar, Rows, Cols>>::IsDynamic)
struct unique_traits<::crisp::Matrix<Scalar, Rows, Cols, Order>> {
  using elem = Matrix<Scalar, Rows, Cols, Order>;
  static constexpr bool IsDynamic = true;
};

template <floating_point Scalar>
struct unique_traits<::crisp::Quaternion<Scalar>> {
  using elem = Eigen::Quaternion<Scalar>;
  static constexpr bool IsDynamic = false;
};

template <arithmetic Scalar, order_e Order>
struct unique_traits<::crisp::SparseMatrix<Scalar, Order>> {
  using elem = SparseMatrix<Scalar, Order>;
  static constexpr bool IsDynamic = true;
};

template <typename T>
struct resource_delete {
  arena::Resource* pool = nullptr;

  void operator()(T* ptr) const {
    if (!ptr) return;
    std::destroy_at(ptr);
    crisp_assert(pool);
    pool->deallocate(ptr, sizeof(T));
  }
};
}  // namespace crisp::arena

namespace crisp {
template <typename T>
class unique {
  using traits = arena::unique_traits<T>;
  using elem_t = typename traits::elem;
  using ptr_t = std::unique_ptr<elem_t, arena::resource_delete<elem_t>>;

private:
  std::conditional_t<traits::IsDynamic, elem_t, ptr_t> elem_;

public:
  template <typename... Args>
  unique(arena::Resource& pool, Args&&... args) {
    if constexpr (traits::IsDynamic) {
      std::construct_at(&elem_, pool, std::forward<Args>(args)...);
    } else {
      void* raw = pool.allocate(sizeof(elem_t), alignof(elem_t));
      elem_t* elem = std::construct_at(
        static_cast<elem_t*>(raw), std::forward<Args>(args)...);
      elem_ = ptr_t(elem, {&pool});
    }
  }

  unique() = default;
  ~unique() = default;

  unique(unique const&) = delete;
  unique& operator=(unique const&) = delete;

  unique(unique&&) noexcept = default;
  unique& operator=(unique&&) noexcept = default;

  elem_t const* get() const {
    if constexpr (traits::IsDynamic) return &elem_;
    else return elem_.get();
  }
  elem_t* get() {
    if constexpr (traits::IsDynamic) return &elem_;
    else return elem_.get();
  }

  elem_t const& operator*() const { return *get(); }
  elem_t& operator*() { return *get(); }

  elem_t const* operator->() const { return get(); }
  elem_t* operator->() { return get(); }

  auto operator()() const { return get()->e(); }
  auto operator()() { return get()->e(); }
};

template <typename T>
struct arena::traits<unique<T>> {
  template <int Footer>
  static constexpr std::size_t bytes() {
    using _traits = unique_traits<T>;
    if constexpr (_traits::IsDynamic)
      return ::crisp::arena::bytes<typename _traits::elem, Footer>();
    else  //
      return sizeof(typename _traits::elem) + Footer;
  }
  static constexpr std::size_t align = [] {
    using _traits = unique_traits<T>;
    if constexpr (_traits::IsDynamic)
      return ::crisp::arena::align<typename _traits::elem>;
    else  //
      return alignof(typename _traits::elem);
  }();
};

template <arithmetic Scalar, int Rows, int Cols, order_e Order>
  requires(internal::traits<::crisp::Matrix<Scalar, Rows, Cols>>::IsDynamic)
struct arena::traits<unique<::crisp::Matrix<Scalar, Rows, Cols, Order>>>:
    arena::traits<::crisp::Matrix<Scalar, Rows, Cols, Order>> {};

template <arithmetic Scalar, order_e Order>
struct arena::traits<unique<::crisp::SparseMatrix<Scalar, Order>>>:
    arena::traits<::crisp::SparseMatrix<Scalar, Order>> {};
}  // namespace crisp
