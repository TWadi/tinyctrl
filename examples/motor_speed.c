/**
 * @file motor_speed.c
 * @brief Closed-loop speed control of a simulated DC motor with tinyctrl.
 *
 * The motor is a first-order system from voltage to speed, w(s) / u(s) = K / (tau s + 1), with K = 2 rad/s per
 * volt and tau = 0.5 s. The PI controller uses Kp = tau / (K * 0.1) = 2.5, which gives a closed-loop time
 * constant of about 0.1 s, and Ki = 10, so the integrator removes load disturbances within about half a second.
 *
 * The supply limits the voltage to +-12 V, so the step to 15 rad/s saturates the output and exercises the
 * anti-windup. The speed sensor is noisy and filtered with a 50 Hz low-pass filter. At t = 1.5 s a load torque
 * slows the motor down.
 *
 * The program prints CSV (time, setpoint, speed, voltage) every 10 ms. Plot it with tools/plot_example.py.
 *
 * SPDX-License-Identifier: MIT
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "tinyctrl/tinyctrl.h"

#define TS         0.001 /* sample time [s] */
#define DURATION   3.0   /* simulated time [s] */
#define MOTOR_K    2.0   /* steady-state speed per volt [rad/s / V] */
#define MOTOR_TAU  0.5   /* mechanical time constant [s] */
#define SUPPLY     12.0  /* voltage limit [V] */
#define LOAD_DROP  4.0   /* speed drop caused by the load torque [rad/s] */
#define NOISE_AMPL 0.5   /* peak sensor noise [rad/s] */

/* Small deterministic pseudo-random generator so that every run prints the same numbers. */
static double noise(void)
{
    static unsigned long state = 12345UL;

    state = (state * 1103515245UL + 12345UL) & 0x7fffffffUL;
    return NOISE_AMPL * (2.0 * (double)state / 2147483647.0 - 1.0);
}

int main(void)
{
    const tc_pid_config cfg = {
        .kp = TC_R(2.5),
        .ki = TC_R(10.0),
        .kd = TC_R(0.0),
        .tf = TC_R(0.0),
        .ts = TC_R(TS),
        .out_min = TC_R(-SUPPLY),
        .out_max = TC_R(SUPPLY),
    };
    const double a = exp(-TS / MOTOR_TAU); /* exact discretisation of the motor */
    const long steps = (long)(DURATION / TS);
    tc_pid pid;
    tc_lowpass sensor_filter;
    double speed = 0.0;
    long k;

    if (tc_pid_init(&pid, &cfg) != TC_OK || tc_lowpass_init_cutoff(&sensor_filter, TC_R(50.0), TC_R(TS)) != TC_OK)
    {
        fprintf(stderr, "invalid controller parameters\n");
        return EXIT_FAILURE;
    }

    printf("time_s,setpoint_rad_s,speed_rad_s,voltage_v\n");
    for (k = 0; k < steps; ++k)
    {
        const double t = (double)k * TS;
        const tc_real setpoint = t >= 0.1 ? TC_R(15.0) : TC_R(0.0);
        const double load = t >= 1.5 ? LOAD_DROP : 0.0;
        const tc_real measured = tc_lowpass_update(&sensor_filter, (tc_real)(speed + noise()));
        const tc_real voltage = tc_pid_update(&pid, setpoint, measured);

        if (k % 10 == 0)
        {
            printf("%.3f,%.3f,%.4f,%.4f\n", t, (double)setpoint, speed, (double)voltage);
        }

        /* Motor: speed approaches K * u - load with time constant tau. */
        speed = a * speed + (1.0 - a) * (MOTOR_K * (double)voltage - load);
    }

    return EXIT_SUCCESS;
}
