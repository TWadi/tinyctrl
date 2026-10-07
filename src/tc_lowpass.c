/**
 * @file tc_lowpass.c
 * @brief First-order low-pass filter (exponential smoothing).
 *
 * SPDX-License-Identifier: MIT
 */
#include "tinyctrl/tc_lowpass.h"

#include <stddef.h>

#include "tc_internal.h"

/* 1 - exp(-x) for x > 0. For small x the subtraction would cancel most significant digits (badly so where double
 * is only 32 bits wide, as on AVR), so a short Taylor series is used there instead. expm1() is avoided because
 * some embedded C libraries do not provide it. */
static double tc_one_minus_exp_neg(double x)
{
    if (x < 1e-2)
    {
        return x * (1.0 - x / 2.0 * (1.0 - x / 3.0 * (1.0 - x / 4.0)));
    }
    return 1.0 - exp(-x);
}

tc_status tc_lowpass_init(tc_lowpass *f, tc_real tau, tc_real ts)
{
    if ((f == NULL) || !tc_is_finite(tau) || !tc_is_finite(ts) || !(tau > TC_R(0.0)) || !(ts > TC_R(0.0)))
    {
        return TC_ERR_ARG;
    }

    f->alpha = (tc_real)tc_one_minus_exp_neg((double)ts / (double)tau);
    f->y = TC_R(0.0);
    f->primed = 0;

    return TC_OK;
}

tc_status tc_lowpass_init_cutoff(tc_lowpass *f, tc_real cutoff_hz, tc_real ts)
{
    if (!tc_is_finite(cutoff_hz) || !(cutoff_hz > TC_R(0.0)))
    {
        return TC_ERR_ARG;
    }

    return tc_lowpass_init(f, (tc_real)(1.0 / (2.0 * TC_PI_D * (double)cutoff_hz)), ts);
}

tc_status tc_lowpass_init_alpha(tc_lowpass *f, tc_real alpha)
{
    if ((f == NULL) || !tc_is_finite(alpha) || !(alpha > TC_R(0.0)) || !(alpha <= TC_R(1.0)))
    {
        return TC_ERR_ARG;
    }

    f->alpha = alpha;
    f->y = TC_R(0.0);
    f->primed = 0;

    return TC_OK;
}

void tc_lowpass_reset(tc_lowpass *f, tc_real value)
{
    if ((f == NULL) || !tc_is_finite(value))
    {
        return;
    }

    f->y = value;
    f->primed = 1;
}

tc_real tc_lowpass_update(tc_lowpass *f, tc_real x)
{
    if (!tc_is_finite(x))
    {
        return f->y;
    }
    if (f->primed)
    {
        f->y += f->alpha * (x - f->y);
    }
    else
    {
        f->y = x;
        f->primed = 1;
    }

    return f->y;
}
