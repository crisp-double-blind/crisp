/*
 * CRISP: Contact-Rich Simulation Platform
 * Copyright (c) 2026 The CRISP authors.
 * Licensed for academic and non-commercial use only.
 * See the LICENSE file for details.
 */

#pragma once

#if defined(_WIN32) || defined(__CYGWIN__)
#ifdef crisp_EXPORTS
#define CRISP_API __declspec(dllexport)
#else
#define CRISP_API __declspec(dllimport)
#endif
#else
#if __GNUC__ >= 4
#define CRISP_API __attribute__((visibility("default")))
#else
#define CRISP_API
#endif
#endif
