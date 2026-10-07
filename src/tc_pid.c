/**
 * @file tc_pid.c
 * @brief Discrete PID controller.
 *
 * SPDX-License-Identifier: MIT
 */
#include "tinyctrl/tc_pid.h"

#include <stddef.h>

#include "tc_internal.h"

tc_status tc_pid_init(tc_pid *pid, const tc_pid_config *cfg)
{
    tc_real denom;

    if ((pid == NULL) || (cfg == NULL))
    {
        return TC_ERR_ARG;
    }
    if (!tc_is_finite(cfg->kp) || !tc_is_finite(cfg->ki) || !tc_is_finite(cfg->kd) || !tc_is_finite(cfg->tf) ||
        !tc_is_finite(cfg->ts))
    {
        return TC_ERR_ARG;
    }
    if (!(cfg->ts > TC_R(0.0)) || (cfg->tf < TC_R(0.0)) || !(cfg->out_min <= cfg->out_max))
    {
        return TC_ERR_ARG;
    }
    /* The derivative filter pole is (2 Tf - Ts) / (2 Tf + Ts). Below Tf = Ts / 2 it is negative and the derivative
     * term alternates in sign from one sample to the next (at Tf = 0 it never decays), so such filters are rejected.
     * Tf = Ts / 2 gives a pole at 0: a plain backward difference Kd * (y[k] - y[k-1]) / Ts. */
    if (tc_is_nonzero(cfg->kd) && !(TC_R(2.0) * cfg->tf >= cfg->ts))
    {
        return TC_ERR_ARG;
    }

    denom = TC_R(2.0) * cfg->tf + cfg->ts;

    pid->kp = cfg->kp;
    pid->ki_ts_half = cfg->ki * cfg->ts * TC_R(0.5);
    pid->d_pole = tc_is_nonzero(cfg->kd) ? (TC_R(2.0) * cfg->tf - cfg->ts) / denom : TC_R(0.0);
    pid->d_gain = TC_R(2.0) * cfg->kd / denom;
    pid->out_min = cfg->out_min;
    pid->out_max = cfg->out_max;
    pid->integral = TC_R(0.0);
    pid->derivative = TC_R(0.0);
    pid->prev_error = TC_R(0.0);
    pid->prev_measurement = TC_R(0.0);
    pid->last_output = tc_clamp(TC_R(0.0), cfg->out_min, cfg->out_max);
    pid->primed = 0;

    return TC_OK;
}

tc_status tc_pid_set_limits(tc_pid *pid, tc_real out_min, tc_real out_max)
{
    if ((pid == NULL) || !(out_min <= out_max))
    {
        return TC_ERR_ARG;
    }

    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->last_output = tc_clamp(pid->last_output, out_min, out_max);

    return TC_OK;
}

void tc_pid_reset(tc_pid *pid, tc_real setpoint, tc_real measurement, tc_real output)
{
    const tc_real error = setpoint - measurement;

    if ((pid == NULL) || !tc_is_finite(error) || !tc_is_finite(output))
    {
        return;
    }

    pid->last_output = tc_clamp(output, pid->out_min, pid->out_max);
    pid->prev_error = error;
    pid->prev_measurement = measurement;
    pid->derivative = TC_R(0.0);
    pid->integral = pid->last_output - pid->kp * error;
    pid->primed = 1;
}

tc_real tc_pid_update(tc_pid *pid, tc_real setpoint, tc_real measurement)
{
    const tc_real error = setpoint - measurement;
    tc_real integral;
    tc_real output;

    /* A NaN or infinite sample would otherwise poison the integrator and derivative state for good. */
    if (!tc_is_finite(error))
    {
        return pid->last_output;
    }
    if (!pid->primed)
    {
        pid->prev_error = error;
        pid->prev_measurement = measurement;
        pid->primed = 1;
    }

    pid->derivative = pid->d_pole * pid->derivative - pid->d_gain * (measurement - pid->prev_measurement);
    integral = pid->integral + pid->ki_ts_half * (error + pid->prev_error);
    output = pid->kp * error + integral + pid->derivative;

    /* Conditional integration: while saturated, only let the integrator move back towards the linear range. */
    if (output > pid->out_max)
    {
        output = pid->out_max;
        if (integral > pid->integral)
        {
            integral = pid->integral;
        }
    }
    else if (output < pid->out_min)
    {
        output = pid->out_min;
        if (integral < pid->integral)
        {
            integral = pid->integral;
        }
    }

    pid->integral = integral;
    pid->prev_error = error;
    pid->prev_measurement = measurement;
    pid->last_output = output;

    return output;
}
