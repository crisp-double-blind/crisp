/*
 * CRISP: Contact-Rich Simulation Platform
 * Copyright (c) 2026 The CRISP authors.
 * Licensed for academic and non-commercial use only.
 * See the LICENSE file for details.
 */

#pragma once

#include <crisp/arena/types/resource.hpp>

#include <memory>

namespace crisp {
enum class buffer_policy_e { reserve_only, construct_n };
}

namespace crisp::arena {
template <typename T, int Align, buffer_policy_e Policy>
class buffer {
  static_assert((Align & (Align - 1)) == 0, "alignment must be a power of 2");

private:
  static constexpr int align = alignof(T) > Align ? alignof(T) : Align;
  Resource* pool_ = nullptr;

protected:
  T* data_ = nullptr;
  int size_ = 0;
  int capacity_ = 0;

public:
  buffer() = default;

  buffer(Resource& pool, int count = 0): pool_(&pool), capacity_(count) {
    if (capacity_ > 0) {
      void* raw = pool_->allocate(sizeof(T) * capacity_, align);
      data_ = static_cast<T*>(raw);
      if constexpr (Policy == buffer_policy_e::construct_n) {
        size_ = capacity_;
        std::uninitialized_default_construct_n(data_, size_);
      }
    }
  }
  buffer(Resource& pool, int size, T const& value)
      : pool_(&pool), size_(size), capacity_(size_) {
    if (capacity_ > 0) {
      void* raw = pool_->allocate(sizeof(T) * capacity_, align);
      data_ = static_cast<T*>(raw);
      std::uninitialized_fill_n(data_, capacity_, value);
    }
  }

  ~buffer() {
    if (pool_ && data_) {
      std::destroy_n(data_, size_);
      pool_->deallocate(data_, sizeof(T) * capacity_);
    }
  }

  buffer(buffer const&) = delete;
  buffer& operator=(buffer const&) = delete;

  buffer(buffer&& other) noexcept
      : pool_(other.pool_),
        data_(other.data_),
        size_(other.size_),
        capacity_(other.capacity_) {
    other.pool_ = nullptr;
    other.data_ = nullptr;
    other.size_ = other.capacity_ = 0;
  }
  buffer& operator=(buffer&& other) noexcept {
    if (this == &other) return *this;

    pool_ = other.pool_;
    data_ = other.data_;
    size_ = other.size_;
    capacity_ = other.capacity_;
    other.pool_ = nullptr;
    other.data_ = nullptr;
    other.size_ = other.capacity_ = 0;
    return *this;
  }

  T const& operator[](int idx) const {
    crisp_assert(0 <= idx && idx < size_, "idx out of range");
    return data_[idx];
  }
  T& operator[](int idx) {
    crisp_assert(0 <= idx && idx < size_, "idx out of range");
    return data_[idx];
  }

  T const& front() const {
    crisp_assert(size_ > 0, "buffer is empty");
    return data_[0];
  }
  T& front() {
    crisp_assert(size_ > 0, "buffer is empty");
    return data_[0];
  }

  T const& back() const {
    crisp_assert(size_ > 0, "buffer is empty");
    return data_[size_ - 1];
  }
  T& back() {
    crisp_assert(size_ > 0, "buffer is empty");
    return data_[size_ - 1];
  }

  T const* data() const { return data_; }
  T* data() { return data_; }

  T const* begin() const { return data_; }
  T* begin() { return data_; }

  T const* end() const { return data_ + size_; }
  T* end() { return data_ + size_; }

  int empty() const { return size_ == 0; }
  int size() const { return size_; }
  int capacity() const { return capacity_; }

  Resource const* pool() const { return pool_; }
  Resource* pool() { return pool_; }

  void reserve(int new_cap) {
    crisp_assert(new_cap >= capacity_, "cannot shrink");
    if (new_cap == capacity_) return;
    crisp_assert(pool_, "no resource assigned");
    if (!data_) {
      void* raw = pool_->allocate(sizeof(T) * new_cap, align);
      data_ = static_cast<T*>(raw);
    } else {
      pool_->adjust_end(&data_[capacity_], &data_[new_cap]);
    }
    capacity_ = new_cap;
  }

  void shrink_to_fit() {
    if (size_ == capacity_) return;
    crisp_assert(pool_, "no resource assigned");
    pool_->adjust_end(&data_[capacity_], &data_[size_]);
    capacity_ = size_;
    if (size_ == 0) {
      data_ = nullptr;
    }
  }

  void resize(int new_size) {
    crisp_assert(0 <= new_size, "size must be >= 0");
    if (new_size == size_) {
      return;
    } else if (new_size > size_) {
      crisp_assert(new_size <= capacity_, "buffer overflow");
      std::uninitialized_default_construct_n(&data_[size_], new_size - size_);
    } else {
      std::destroy_n(&data_[new_size], size_ - new_size);
    }
    size_ = new_size;
  }

  void clear() {
    if (data_) {
      std::destroy_n(data_, size_);
      size_ = 0;
    }
  }

  template <typename... Args>
  T& emplace_back(Args&&... args) {
    crisp_assert(size_ < capacity_, "buffer overflow");
    return *std::construct_at(&data_[size_++], std::forward<Args>(args)...);
  }
  void push_back(T const& value) { emplace_back(value); }
  void push_back(T&& value) { emplace_back(std::move(value)); }

  template <typename... Args>
  T& emplace_back_grow(Args&&... args) {
    if (size_ >= capacity_) {
      crisp_assert(pool_, "no resource assigned");
      if (!data_) {
        void* raw = pool_->allocate(sizeof(T), align);
        data_ = static_cast<T*>(raw);
      } else {
        pool_->adjust_end(&data_[capacity_], &data_[capacity_ + 1]);
      }
      ++capacity_;
    }
    return *std::construct_at(&data_[size_++], std::forward<Args>(args)...);
  }
  void push_back_grow(T const& value) { emplace_back_grow(value); }
  void push_back_grow(T&& value) { emplace_back_grow(std::move(value)); }

  void pop_back() {
    crisp_assert(size_ > 0, "buffer underflow");
    std::destroy_at(&data_[--size_]);
  }

  void release_unsafe() noexcept {
    pool_ = nullptr;
    data_ = nullptr;
    size_ = capacity_ = 0;
  }
};

template <typename T, int Align, buffer_policy_e Policy>
struct traits<buffer<T, Align, Policy>> {
  template <int Footer>
  static constexpr std::size_t bytes(int size) {
    return sizeof(T) * size + Footer;
  }
  static constexpr std::size_t align =  //
    alignof(T) > Align ? alignof(T) : Align;
};

template <typename T>
struct buffer_traits {
  static constexpr auto Policy = buffer_policy_e::construct_n;
};

template <typename T, buffer_policy_e Policy = buffer_traits<T>::Policy>
using aligned_buffer = buffer<T, align_bytes, Policy>;

template <typename T, buffer_policy_e Policy = buffer_traits<T>::Policy>
using unaligned_buffer = buffer<T, 0, Policy>;
}  // namespace crisp::arena
