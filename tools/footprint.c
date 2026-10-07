/**
 * @file footprint.c
 * @brief Minimal firmware used by CI to link tinyctrl for microcontrollers and report its size.
 *
 * Built as is, it uses every module, including the initialisation functions that compute coefficients at run time
 * (tc_tustin() and tc_lowpass_init() need tan() and exp() in double precision). With -DFOOTPRINT_MINIMAL it only
 * uses coefficients computed offline, which keeps double-precision math out of the image.
 *
 * SPDX-License-Identifier: MIT
 */
#include "tinyctrl/tinyctrl.h"

/* Stand-ins for a sensor input and an actuator output. */
volatile tc_real setpoint_in;
volatile tc_real measurement_in;
volatile tc_real actuator_out;

int main(void)
{
    const tc_pid_config pid_cfg = {
        .kp = TC_R(2.0),
        .ki = TC_R(5.0),
        .kd = TC_R(0.01),
        .tf = TC_R(0.001),
        .ts = TC_R(0.001),
        .out_min = TC_R(-12.0),
        .out_max = TC_R(12.0),
    };
    tc_pid pid;
    tc_lowpass sensor_filter;
    tc_biquad notch;
#ifdef FOOTPRINT_MINIMAL
    /* The same 100 Hz notch as below, discretised offline. */
    const tc_biquad_coeffs notch_coeffs = {
        .b0 = TC_R(0.9445),
        .b1 = TC_R(-1.5282),
        .b2 = TC_R(0.9445),
        .a1 = TC_R(-1.5282),
        .a2 = TC_R(0.8890),
    };

    if ((tc_pid_init(&pid, &pid_cfg) != TC_OK) || (tc_lowpass_init_alpha(&sensor_filter, TC_R(0.27)) != TC_OK) ||
        (tc_biquad_init(&notch, &notch_coeffs) != TC_OK))
    {
        return 1;
    }
    for (;;)
    {
        const tc_real y = tc_biquad_update(&notch, tc_lowpass_update(&sensor_filter, measurement_in));

        actuator_out = tc_pid_update(&pid, setpoint_in, y);
    }
#else
    const tc_real w0 = TC_R(628.3);
    const tc_real notch_num[3] = {TC_R(1.0), TC_R(0.0), w0 * w0};
    const tc_real notch_den[3] = {TC_R(1.0), TC_R(0.2) * w0, w0 * w0};
    tc_biquad_coeffs notch_coeffs;
    tc_leadlag lead;

    if ((tc_pid_init(&pid, &pid_cfg) != TC_OK) ||
        (tc_lowpass_init_cutoff(&sensor_filter, TC_R(50.0), TC_R(0.001)) != TC_OK) ||
        (tc_tustin(notch_num, notch_den, TC_R(0.001), w0, &notch_coeffs) != TC_OK) ||
        (tc_biquad_init(&notch, &notch_coeffs) != TC_OK) ||
        (tc_leadlag_init(&lead, TC_R(1.0), TC_R(10.0), TC_R(100.0), TC_R(0.001)) != TC_OK))
    {
        return 1;
    }
    for (;;)
    {
        const tc_real y = tc_biquad_update(&notch, tc_lowpass_update(&sensor_filter, measurement_in));

        actuator_out = tc_leadlag_update(&lead, tc_pid_update(&pid, setpoint_in, y));
    }
#endif
}
