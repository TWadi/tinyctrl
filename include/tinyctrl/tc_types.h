/**
 * @file tc_types.h
 * @brief Common types and helpers shared by all tinyctrl modules.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef TINYCTRL_TC_TYPES_H
#define TINYCTRL_TC_TYPES_H

#include <float.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Real number type used by every tinyctrl module.
 *
 * tinyctrl uses single precision by default, which most microcontrollers with an FPU handle in hardware.
 * Define TINYCTRL_USE_DOUBLE (for the library and every file that includes it) to use double precision.
 */
#ifdef TINYCTRL_USE_DOUBLE
typedef double tc_real;
#define TC_REAL_MAX DBL_MAX
#else
typedef float tc_real;
#define TC_REAL_MAX FLT_MAX
#endif

/** @brief Converts a literal to tc_real without an implicit double promotion. */
#define TC_R(x) ((tc_real)(x))

/** @brief Status returned by the initialisation functions. */
typedef enum
{
    TC_OK = 0,      /**< Success. */
    TC_ERR_ARG = -1 /**< An argument was invalid, for example a null pointer or a non-positive sample time. */
} tc_status;

/**
 * @brief Clamps a value to a range.
 * @param x  Value to clamp.
 * @param lo Lower bound.
 * @param hi Upper bound (must be >= lo).
 * @return x limited to [lo, hi].
 */
static inline tc_real tc_clamp(tc_real x, tc_real lo, tc_real hi)
{
    if (x < lo)
    {
        return lo;
    }
    if (x > hi)
    {
        return hi;
    }
    return x;
}

#ifdef __cplusplus
}
#endif

#endif /* TINYCTRL_TC_TYPES_H */
