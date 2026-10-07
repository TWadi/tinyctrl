/**
 * @file tc_discretize.h
 * @brief Tustin (bilinear) discretisation of continuous transfer functions up to second order.
 *
 * Controllers and filters are usually designed in the s-domain. tc_tustin() converts
 *
 *            n[0] s^2 + n[1] s + n[2]
 *     H(s) = ------------------------
 *            d[0] s^2 + d[1] s + d[2]
 *
 * into biquad coefficients by substituting
 *
 *     s = K (z - 1) / (z + 1),   K = 2 / Ts                      (plain Tustin)
 *                                K = w / tan(w Ts / 2)           (prewarped at w rad/s)
 *
 * The result matches MATLAB's `c2d(tf(n, d), Ts, 'tustin')`, or with prewarping
 * `c2d(tf(n, d), Ts, c2dOptions('Method', 'tustin', 'PrewarpFrequency', w))`.
 *
 * Prewarping makes the discrete frequency response match the continuous one exactly at w, which matters for
 * notch filters and for crossover frequencies close to the Nyquist frequency.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef TINYCTRL_TC_DISCRETIZE_H
#define TINYCTRL_TC_DISCRETIZE_H

#include "tc_biquad.h"
#include "tc_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Discretises a continuous transfer function of order <= 2 with the Tustin method.
 *
 * Coefficients are given in descending powers of s, like MATLAB's tf(). For a first-order system set n[0] and
 * d[0] to zero; the result then has b2 = a2 = 0. For a static gain set n[0], n[1], d[0] and d[1] to zero.
 *
 * @param num        Numerator coefficients {s^2, s^1, s^0}.
 * @param den        Denominator coefficients {s^2, s^1, s^0}.
 * @param ts         Sample time [s], > 0.
 * @param prewarp    Prewarp frequency [rad/s] in [0, pi / ts); 0 means no prewarping.
 * @param out        Resulting biquad coefficients.
 * @return TC_OK, or TC_ERR_ARG if a pointer is null, an input is not finite, ts or prewarp is out of range, the
 *         transfer function is improper (numerator order above denominator order), the denominator is zero, or
 *         the system has a pole at (or within about 1e-6 relative of) s = K, which Tustin maps to z = infinity.
 */
tc_status tc_tustin(const tc_real num[3], const tc_real den[3], tc_real ts, tc_real prewarp, tc_biquad_coeffs *out);

#ifdef __cplusplus
}
#endif

#endif /* TINYCTRL_TC_DISCRETIZE_H */
