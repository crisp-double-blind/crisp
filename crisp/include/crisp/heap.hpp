#pragma once

#include <crisp/macro.hpp>

#include <array>

namespace crisp {
class HeapResource {
private:
  static constexpr int nslot = 8;
  struct Slot {
    void* ptr = nullptr;
    void (*deleter)(void*) = nullptr;
  };
  std::array<Slot, nslot> slots_;

public:
  HeapResource() noexcept = default;
  ~HeapResource() noexcept { clear(); }

  template <typename T, typename... Args>
  T* emplace(int idx, Args&&... args) noexcept {
    crisp_assert(0 <= idx && idx < nslot, "slot index out of range");
    if (idx < 0 || idx >= nslot) return nullptr;

    auto& slot = slots_[idx];
    if (slot.ptr && slot.deleter) {
      slot.deleter(slot.ptr);
    }
    slot.ptr = new T(std::forward<Args>(args)...);
    slot.deleter = [](void* p) { delete static_cast<T*>(p); };
    return static_cast<T*>(slot.ptr);
  }

  template <typename T>
  T* get(int idx) const noexcept {
    crisp_assert(0 <= idx && idx < nslot, "slot index out of range");
    return static_cast<T*>(slots_[idx].ptr);
  }

  void clear() noexcept {
    for (auto& slot : slots_) {
      if (slot.ptr && slot.deleter) {
        slot.deleter(slot.ptr);
        slot.ptr = nullptr;
        slot.deleter = nullptr;
      }
    }
  }
};
}  // namespace crisp
