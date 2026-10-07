/**
 * @file tc_biquad.h
 * @brief Second-order IIR filter section (biquad) in transposed direct form II.
 *
 * The section implements
 *
 *            b0 + b1 z^-1 + b2 z^-2
 *     H(z) = ----------------------
 *            1  + a1 z^-1 + a2 z^-2
 *
 * First-order sections use b2 = a2 = 0. Coefficients usually come from tc_tustin() or a filter design tool.
 *
 * Like every tinyctrl block, the filter starts in the steady state of its first input: the first call to
 * tc_biquad_update() calls tc_biquad_reset() with its input, so a filter that sees a constant signal from the start
 * outputs H(1) times that signal without a start-up transient. Call tc_biquad_reset(f, 0) after tc_biquad_init() to
 * start from rest instead; the output then matches MATLAB's `filter([b0 b1 b2], [1 a1 a2], x)`.
 *
 * A NaN or infinite input is ignored: tc_biquad_update() returns its previous output (0 before the first finite
 * input) and leaves the state unchanged.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef TINYCTRL_TC_BIQUAD_H
#define TINYCTRL_TC_BIQUAD_H

#include "tc_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Biquad coefficients, normalised so that a0 = 1. */
typedef struct
{
    tc_real b0; /**< Numerator coefficient of z^0. */
    tc_real b1; /**< Numerator coefficient of z^-1. */
    tc_real b2; /**< Numerator coefficient of z^-2. */
    tc_real a1; /**< Denominator coefficient of z^-1. */
    tc_real a2; /**< Denominator coefficient of z^-2. */
} tc_biquad_coeffs;

/** @brief Biquad filter state. Treat the members as private; use the tc_biquad_* functions. */
typedef struct
{
    tc_biquad_coeffs c;
    tc_real s1;
    tc_real s2;
    tc_real y;  /* last output, returned when the input is not finite */
    int primed; /* non-zero once the state has been set by a reset or an update */
} tc_biquad;

/**
 * @brief Initialises a biquad with the given coefficients. The next update primes the state, see above.
 * @return TC_OK, or TC_ERR_ARG if a pointer is null or a coefficient is not finite.
 */
tc_status tc_biquad_init(tc_biquad *f, const tc_biquad_coeffs *c);

/**
 * @brief Sets the state to the steady state for a constant input.
 *
 * After the reset, a constant input @p x produces the constant output H(1) * x, so the filter starts without a
 * transient. If the filter has a pole at z = 1 (an integrator), H(1) is undefined and the state is set to zero.
 * Nothing happens if @p f is null or @p x is not finite.
 */
void tc_biquad_reset(tc_biquad *f, tc_real x);

/**
 * @brief Filters one sample.
 * @param f Filter (initialised with tc_biquad_init()).
 * @param x Input sample.
 * @return Output sample.
 */
tc_real tc_biquad_update(tc_biquad *f, tc_real x);

#ifdef __cplusplus
}
#endif

#endif /* TINYCTRL_TC_BIQUAD_H */
