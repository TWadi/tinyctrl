/**
 * @file tc_lowpass.h
 * @brief First-order low-pass filter (exponential smoothing) for noisy measurements.
 *
 * The filter computes
 *
 *     y[k] = y[k-1] + alpha * (x[k] - y[k-1]),   alpha = 1 - exp(-Ts / tau)
 *
 * which places the discrete pole exactly at exp(-Ts / tau), the image of the continuous pole -1 / tau. This is a
 * pole-matched discretisation, not Tustin: it has no zero at z = -1, so it attenuates less near the Nyquist
 * frequency than tc_tustin() would. Its DC gain is 1 and a step response reaches 63.2 % after one time constant.
 *
 * Precision: in float, an update stops moving the output once alpha * |x - y| falls below half a unit in the last
 * place of y, so the output can settle up to about |y| * 6e-8 / alpha away from a constant input (0.06 % of |y|
 * for alpha = 1e-4, which is a cutoff of about fs / 60000). Use a larger alpha or the double build if that matters.
 *
 * A NaN or infinite input is ignored: tc_lowpass_update() returns its previous output (0 before the first finite
 * input) and leaves the state unchanged.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef TINYCTRL_TC_LOWPASS_H
#define TINYCTRL_TC_LOWPASS_H

#include "tc_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Low-pass filter state. Treat the members as private; use the tc_lowpass_* functions. */
typedef struct
{
    tc_real alpha;
    tc_real y;
    int primed; /* non-zero once y holds a real value */
} tc_lowpass;

/**
 * @brief Initialises the filter from its time constant.
 *
 * The first call to tc_lowpass_update() returns its input unchanged, so the filter starts without a transient
 * from zero. Call tc_lowpass_reset() to start from another value.
 *
 * @param f   Filter.
 * @param tau Time constant [s], > 0.
 * @param ts  Sample time [s], > 0.
 * @return TC_OK, or TC_ERR_ARG if f is null or tau or ts is not positive.
 */
tc_status tc_lowpass_init(tc_lowpass *f, tc_real tau, tc_real ts);

/**
 * @brief Initialises the filter from its -3 dB cutoff frequency, tau = 1 / (2 pi fc).
 * @param f         Filter.
 * @param cutoff_hz Cutoff frequency [Hz], > 0.
 * @param ts        Sample time [s], > 0.
 * @return TC_OK, or TC_ERR_ARG if f is null or cutoff_hz or ts is not positive.
 */
tc_status tc_lowpass_init_cutoff(tc_lowpass *f, tc_real cutoff_hz, tc_real ts);

/**
 * @brief Initialises the filter directly from its smoothing factor alpha.
 *
 * Unlike tc_lowpass_init(), this does not call exp(), which keeps the floating-point library out of very small
 * builds. Compute alpha = 1 - exp(-Ts / tau) offline.
 *
 * @param f     Filter.
 * @param alpha Smoothing factor in (0, 1]; 1 passes the input through unchanged.
 * @return TC_OK, or TC_ERR_ARG if f is null or alpha is out of range.
 */
tc_status tc_lowpass_init_alpha(tc_lowpass *f, tc_real alpha);

/** @brief Sets the filter output to @p value. Nothing happens if @p f is null or @p value is not finite. */
void tc_lowpass_reset(tc_lowpass *f, tc_real value);

/**
 * @brief Filters one sample.
 * @return The filtered value.
 */
tc_real tc_lowpass_update(tc_lowpass *f, tc_real x);

#ifdef __cplusplus
}
#endif

#endif /* TINYCTRL_TC_LOWPASS_H */
