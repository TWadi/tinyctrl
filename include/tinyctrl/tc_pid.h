/**
 * @file tc_pid.h
 * @brief Discrete PID controller for sampled control loops.
 *
 * The controller implements the parallel form
 *
 *     u = Kp * e + Ki / s * e - Kd * s / (Tf * s + 1) * y,     e = r - y
 *
 * discretised with the Tustin (trapezoidal) method:
 *
 * - The integral term uses trapezoidal integration of the error.
 * - The derivative term acts on the measurement y, not on the error, so a setpoint step does not cause a
 *   "derivative kick". Its first-order filter (time constant Tf) limits the gain at high frequencies.
 * - When the output saturates, the integrator stops integrating in the direction that would drive the output
 *   further into the limit (conditional integration), which prevents integrator windup.
 *
 * Starting from tc_pid_reset(pid, 0, 0, 0) and without saturation, the output matches MATLAB's
 * `c2d(pid(Kp, Ki, Kd, Tf), Ts, 'tustin')` acting on the error, with the derivative applied to -y.
 *
 * A NaN or infinite setpoint or measurement is ignored: tc_pid_update() returns its previous output and leaves
 * the state unchanged, so one bad sensor reading cannot poison the integrator.
 *
 * Typical use:
 * @code
 * tc_pid pid;
 * const tc_pid_config cfg = {
 *     .kp = TC_R(2.0), .ki = TC_R(5.0), .kd = TC_R(0.1), .tf = TC_R(0.01),
 *     .ts = TC_R(0.001), .out_min = TC_R(-12.0), .out_max = TC_R(12.0),
 * };
 * if (tc_pid_init(&pid, &cfg) != TC_OK) { handle_error(); }
 * for (;;) {
 *     const tc_real u = tc_pid_update(&pid, setpoint, read_sensor());
 *     write_actuator(u);
 *     wait_for_next_sample();
 * }
 * @endcode
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef TINYCTRL_TC_PID_H
#define TINYCTRL_TC_PID_H

#include "tc_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Parameters of a PID controller. */
typedef struct
{
    tc_real kp;      /**< Proportional gain. */
    tc_real ki;      /**< Integral gain [1/s]. */
    tc_real kd;      /**< Derivative gain [s]. */
    tc_real tf;      /**< Derivative filter time constant [s], >= 0. Must be >= ts / 2 when kd != 0. */
    tc_real ts;      /**< Sample time [s]. Must be > 0. */
    tc_real out_min; /**< Lower output limit. Use -TC_REAL_MAX for no limit. */
    tc_real out_max; /**< Upper output limit (>= out_min). Use TC_REAL_MAX for no limit. */
} tc_pid_config;

/**
 * @brief PID controller state. Treat the members as private; use the tc_pid_* functions.
 */
typedef struct
{
    tc_real kp;
    tc_real ki_ts_half; /* Ki * Ts / 2 */
    tc_real d_pole;     /* (2 Tf - Ts) / (2 Tf + Ts) */
    tc_real d_gain;     /* 2 Kd / (2 Tf + Ts) */
    tc_real out_min;
    tc_real out_max;
    tc_real integral;
    tc_real derivative;
    tc_real prev_error;
    tc_real prev_measurement;
    tc_real last_output; /* returned when an input is not finite */
    int primed;          /* non-zero once prev_error and prev_measurement hold real samples */
} tc_pid;

/**
 * @brief Initialises a PID controller.
 *
 * The state starts empty: the first call to tc_pid_update() takes its previous error and measurement from its
 * own inputs, so the controller starts without a derivative kick. Its integral term then adds Ki * Ts * e for
 * that first sample. Call tc_pid_reset() instead to start from a known output (bumpless transfer).
 *
 * @param pid Controller to initialise.
 * @param cfg Parameters, see tc_pid_config.
 * @return TC_OK, or TC_ERR_ARG if a pointer is null, a gain or time is not finite, ts <= 0, tf < 0,
 *         tf < ts / 2 with kd != 0, or out_min > out_max.
 */
tc_status tc_pid_init(tc_pid *pid, const tc_pid_config *cfg);

/**
 * @brief Changes the output limits without touching the controller state.
 * @return TC_OK, or TC_ERR_ARG if pid is null or out_min > out_max.
 */
tc_status tc_pid_set_limits(tc_pid *pid, tc_real out_min, tc_real out_max);

/**
 * @brief Resets the controller so that it continues smoothly from a known output.
 *
 * Use this when switching from manual to automatic control (bumpless transfer) or after a pause: call it with the
 * output that is currently applied. The controller then behaves as if it had produced @p output at this sample,
 * and the next tc_pid_update() continues from there. With the setpoint and measurement unchanged, that update
 * returns output + Ki * Ts * e (just @p output when e = 0 or Ki = 0); the derivative state is cleared.
 *
 * The integrator holds output - Kp * e afterwards. With Ki = 0 it never changes, so it acts as a constant bias
 * (the classic "manual reset" of a P or PD controller); tc_pid_init() starts with a bias of zero.
 *
 * Nothing happens if @p pid is null or an argument is not finite.
 *
 * @param pid         Controller.
 * @param setpoint    Current setpoint.
 * @param measurement Current measurement.
 * @param output      Output the controller should continue from (clamped to the output limits).
 */
void tc_pid_reset(tc_pid *pid, tc_real setpoint, tc_real measurement, tc_real output);

/**
 * @brief Runs one controller step. Call it once per sample time.
 *
 * If the setpoint or measurement is NaN or infinite, the previous output is returned and the state is unchanged.
 *
 * @param pid         Controller (initialised with tc_pid_init()).
 * @param setpoint    Desired value r.
 * @param measurement Measured value y.
 * @return Controller output u, limited to [out_min, out_max].
 */
tc_real tc_pid_update(tc_pid *pid, tc_real setpoint, tc_real measurement);

#ifdef __cplusplus
}
#endif

#endif /* TINYCTRL_TC_PID_H */
