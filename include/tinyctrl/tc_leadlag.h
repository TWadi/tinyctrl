/**
 * @file tc_leadlag.h
 * @brief Lead or lag compensator.
 *
 * The compensator implements
 *
 *     C(s) = gain * (s / wz + 1) / (s / wp + 1)
 *
 * discretised with the Tustin method. With wz < wp it is a lead compensator (phase lead between wz and wp, used
 * to add phase margin); with wz > wp it is a lag compensator (more low-frequency gain, used to reduce the
 * steady-state error). The maximum phase change occurs at sqrt(wz * wp).
 *
 * The discrete compensator matches MATLAB's `c2d(gain * tf([1/wz 1], [1/wp 1]), Ts, 'tustin')`. Like the biquad
 * it is built on, it starts in the steady state of its first input (call tc_leadlag_reset(c, 0) to start from
 * rest instead) and ignores NaN or infinite inputs.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef TINYCTRL_TC_LEADLAG_H
#define TINYCTRL_TC_LEADLAG_H

#include "tc_biquad.h"
#include "tc_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Lead/lag compensator state. Treat the members as private; use the tc_leadlag_* functions. */
typedef struct
{
    tc_biquad section;
} tc_leadlag;

/**
 * @brief Initialises a lead/lag compensator. The first update sets the state to the steady state of its input.
 * @param c    Compensator.
 * @param gain DC gain.
 * @param wz   Zero frequency [rad/s], > 0.
 * @param wp   Pole frequency [rad/s], > 0.
 * @param ts   Sample time [s], > 0.
 * @return TC_OK, or TC_ERR_ARG if c is null, an argument is not finite, or a frequency or ts is not positive.
 */
tc_status tc_leadlag_init(tc_leadlag *c, tc_real gain, tc_real wz, tc_real wp, tc_real ts);

/** @brief Sets the state to the steady state for a constant input @p x (output gain * x). See tc_biquad_reset(). */
void tc_leadlag_reset(tc_leadlag *c, tc_real x);

/**
 * @brief Runs one compensator step.
 * @return Compensator output.
 */
tc_real tc_leadlag_update(tc_leadlag *c, tc_real x);

#ifdef __cplusplus
}
#endif

#endif /* TINYCTRL_TC_LEADLAG_H */
