/*
 * CRISP: Contact-Rich Simulation Platform
 * Copyright (c) 2026 The CRISP authors.
 * Licensed for academic and non-commercial use only.
 * See the LICENSE file for details.
 */

#pragma once

#if !defined(CRISP_DISABLE_SIMD)
#if defined(CRISP_USE_AVX512)
#if !defined(__AVX512F__)
#error "[CRISP] SIMD mismatch: AVX-512 required."
#endif

#elif defined(CRISP_USE_AVX2)
#if !defined(__AVX2__)
#error "[CRISP] SIMD mismatch: AVX2 required."
#endif

#elif defined(CRISP_USE_SSE2)
#if !defined(__SSE2__) && !defined(_M_X64)
#error "[CRISP] SIMD mismatch: SSE2 required."
#endif

#elif defined(CRISP_USE_NEON)
#if !defined(__ARM_NEON) && !defined(__ARM_NEON__) && !defined(_M_ARM64)
#error "[CRISP] SIMD mismatch: ARM NEON required."
#endif
#endif
#endif

namespace crisp {
inline constexpr int align_bytes =
#if defined(CRISP_DISABLE_SIMD)
  0;
#elif defined(CRISP_USE_AVX512)
  64;
#elif defined(CRISP_USE_AVX2)
  32;
#elif defined(CRISP_USE_SSE2) || defined(CRISP_USE_NEON)
  16;
#else
  8;
#endif
}  // namespace crisp
