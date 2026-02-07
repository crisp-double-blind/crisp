#pragma once

#include <memory>

#include <spdlog/spdlog.h>

namespace crisp {
extern std::shared_ptr<spdlog::logger> __logger__;
}

#if defined(CRISP_ENABLE_ASSERTS)
#if defined(NDEBUG)
#define _crisp_assert_impl_1(cond)                                      \
  ((void)((cond) ||                                                     \
          (::crisp::__logger__->critical(                               \
             "assertion failed: {}, file {}, line {}", #cond, __FILE__, \
             __LINE__),                                                 \
           std::abort(), 0)))
#define _crisp_assert_impl_2(cond, msg)                                    \
  ((void)((cond) ||                                                        \
          (::crisp::__logger__->critical(                                  \
             "assertion failed: {}, \"{}\", file {}, line {}", #cond, msg, \
             __FILE__, __LINE__),                                          \
           std::abort(), 0)))
#else
#include <cassert>
#define _crisp_assert_impl_1(cond) assert(cond)
#define _crisp_assert_impl_2(cond, msg) assert((cond) && msg)
#endif
#define _crisp_assert_dispatch(_1, _2, NAME, ...) NAME

#define crisp_assert(...) \
  _crisp_assert_dispatch( \
    __VA_ARGS__, _crisp_assert_impl_2, _crisp_assert_impl_1)(__VA_ARGS__)
#else
#define crisp_assert(...) ((void)0)
#endif

#if defined(_MSC_VER)
#define crisp_nodefault  \
  do {                   \
    crisp_assert(false); \
    __assume(0);         \
  } while (0)
#else
#define crisp_nodefault      \
  do {                       \
    crisp_assert(false);     \
    __builtin_unreachable(); \
  } while (0)
#endif

#if defined(_MSC_VER)
#define crisp_optimize_off __pragma(optimize("", off))
#define crisp_optimize_on __pragma(optimize("", on))

#elif defined(__clang__)
#define crisp_optimize_off _Pragma("clang optimize off")
#define crisp_optimize_on _Pragma("clang optimize on")

#elif defined(__GNUC__)
#define crisp_optimize_off \
  _Pragma("GCC push_options") _Pragma("GCC optimize (\"O0\")")
#define crisp_optimize_on _Pragma("GCC pop_options")

#else
#define crisp_optimize_off
#define crisp_optimize_on
#endif
