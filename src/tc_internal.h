/**
 * @file tc_internal.h
 * @brief Helpers shared by the tinyctrl sources. Not part of the public API.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef TINYCTRL_TC_INTERNAL_H
#define TINYCTRL_TC_INTERNAL_H

#include <math.h>

#include "tinyctrl/tc_types.h"

/* The update functions skip NaN and infinite samples. With -ffast-math or -ffinite-math-only the compiler may assume
 * such values never occur and silently delete those checks. */
#if defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__
#error "Compile tinyctrl without -ffast-math / -ffinite-math-only (or add -fno-finite-math-only for its sources)"
#endif

/* Initialisation code computes coefficients in double precision: it runs once, and the extra precision keeps
 * float builds as close as possible to the exact coefficients. The update functions use tc_real only. */
#define TC_PI_D 3.14159265358979323846

static inline int tc_is_finite(tc_real x)
{
    return isfinite(x) ? 1 : 0;
}

static inline int tc_is_nonzero(tc_real x)
{
    return (x < TC_R(0.0)) || (x > TC_R(0.0));
}

static inline tc_real tc_abs(tc_real x)
{
    return x < TC_R(0.0) ? -x : x;
}

#ifdef TINYCTRL_USE_DOUBLE
#define TC_EPS DBL_EPSILON
#else
#define TC_EPS FLT_EPSILON
#endif

#endif /* TINYCTRL_TC_INTERNAL_H */
