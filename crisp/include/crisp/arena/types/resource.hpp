#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>

#include <crisp/constants.hpp>
#include <crisp/macro.hpp>

namespace crisp::arena {
class Resource {
public:
  [[nodiscard]] virtual void* allocate(
    std::size_t bytes, std::size_t align) noexcept = 0;
  virtual void deallocate(void* ptr, std::size_t bytes) noexcept = 0;

  virtual bool is_last_end(void const* end) const noexcept = 0;
  virtual void adjust_end(void const* old_end, void* new_end) noexcept = 0;
};

template <typename T>
struct traits;

template <typename T, int Footer = 0, typename... Args>
constexpr std::size_t bytes(Args&&... args) {
  return traits<T>::template bytes<Footer>(std::forward<Args>(args)...);
}
template <typename T>
constexpr std::size_t align = traits<T>::align;
}  // namespace crisp::arena

namespace crisp {
using byte_t = std::uint8_t;

class BumpResource: public arena::Resource {
protected:
  std::size_t capacity_ = 0;
  byte_t* buffer_ = nullptr;
  byte_t* cursor_ = nullptr;

public:
  BumpResource() noexcept = default;

  BumpResource(std::size_t capacity): capacity_(capacity) {
    buffer_ = static_cast<byte_t*>(
      ::operator new(capacity, std::align_val_t(align_bytes)));
    cursor_ = buffer_;
  }

  ~BumpResource() noexcept {
    if (buffer_) ::operator delete(buffer_, std::align_val_t(align_bytes));
  }

  BumpResource(BumpResource const&) = delete;
  BumpResource& operator=(BumpResource const&) = delete;

  BumpResource(BumpResource&& other) noexcept
      : capacity_(other.capacity_),
        buffer_(other.buffer_),
        cursor_(other.cursor_) {
    other.capacity_ = 0;
    other.buffer_ = nullptr;
    other.cursor_ = nullptr;
  }
  BumpResource& operator=(BumpResource&& other) noexcept {
    if (this == &other) return *this;

    if (buffer_) ::operator delete(buffer_, std::align_val_t(align_bytes));
    capacity_ = other.capacity_;
    buffer_ = other.buffer_;
    cursor_ = other.cursor_;

    other.capacity_ = 0;
    other.buffer_ = nullptr;
    other.cursor_ = nullptr;
    return *this;
  }

  byte_t const* data() const noexcept { return buffer_; }
  byte_t* data() noexcept { return buffer_; }

  std::size_t capacity() const noexcept { return capacity_; }
  std::size_t used() const noexcept { return cursor_ - buffer_; }
  std::size_t available() const noexcept { return capacity_ - used(); }

  [[nodiscard]] void* allocate(
    std::size_t bytes, std::size_t align) noexcept override {
    void* ptr = cursor_;
    std::size_t space = available();
    [[maybe_unused]] void* aligned = std::align(align, bytes, ptr, space);
    crisp_assert(aligned != nullptr && space >= bytes, "allocation failed");

    cursor_ = static_cast<byte_t*>(ptr) + bytes;
    return ptr;
  }

  // nothing to do
  void deallocate(void*, std::size_t) noexcept override {}

  bool is_last_end(void const* end) const noexcept override {
    return static_cast<byte_t const*>(end) == cursor_;
  }

  void adjust_end(void const* old_end, void* new_end) noexcept override {
    crisp_assert(is_last_end(old_end), "not last end");
    cursor_ = static_cast<byte_t*>(new_end);
  }
};

class StackResource: public BumpResource {
private:
  std::size_t peak_ = 0;

public:
  StackResource(std::size_t size): BumpResource(size) {}

  StackResource() noexcept = default;
  ~StackResource() noexcept = default;

  StackResource(StackResource const&) = delete;
  StackResource& operator=(StackResource const&) = delete;

  StackResource(StackResource&&) noexcept = default;
  StackResource& operator=(StackResource&&) noexcept = default;

  std::size_t peak() const noexcept { return peak_; }

  void rewind() noexcept {
    cursor_ = buffer_;
    peak_ = 0;
  }

  [[nodiscard]] void* allocate(
    std::size_t bytes, std::size_t align) noexcept final {
    void* ptr = cursor_;
    std::size_t space = available();
    [[maybe_unused]] void* aligned = std::align(align, bytes + 1, ptr, space);
    crisp_assert(aligned != nullptr && space >= bytes, "allocation failed");

    auto offset = static_cast<byte_t*>(ptr) - cursor_;
    crisp_assert(offset <= 255, "offset out of range");
    cursor_ = static_cast<byte_t*>(ptr) + bytes + 1;
    cursor_[-1] = static_cast<byte_t>(offset);

    if (used() > peak_) peak_ = used();
    return ptr;
  }

  void deallocate(void* ptr, std::size_t bytes) noexcept final {
    auto* end = static_cast<byte_t*>(ptr) + bytes + 1;
    if (end != cursor_) [[unlikely]]
      return;

    byte_t offset = cursor_[-1];
    cursor_ = static_cast<byte_t*>(ptr) - offset;
  }

  bool is_last_end(void const* end) const noexcept final {
    return static_cast<byte_t const*>(end) + 1 == cursor_;
  }

  void adjust_end(void const* old_end, void* new_end) noexcept final {
    crisp_assert(is_last_end(old_end), "not last end");
    byte_t offset = cursor_[-1];
    cursor_ = static_cast<byte_t*>(new_end) + 1;
    cursor_[-1] = offset;
    if (used() > peak_) peak_ = used();
  }

  void* get_cursor_unsafe() const noexcept { return cursor_; }

  void set_cursor_unsafe(void* ptr) noexcept {
    auto* _ptr = static_cast<byte_t*>(ptr);
    crisp_assert(
      buffer_ <= _ptr && _ptr <= buffer_ + capacity_, "ptr out of range");
    cursor_ = _ptr;
  }
};
}  // namespace crisp
