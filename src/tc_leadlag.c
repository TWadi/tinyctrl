/**
 * @file tc_leadlag.c
 * @brief Lead or lag compensator.
 *
 * SPDX-License-Identifier: MIT
 */
#include "tinyctrl/tc_leadlag.h"

#include <stddef.h>

#include "tc_internal.h"
#include "tinyctrl/tc_discretize.h"

tc_status tc_leadlag_init(tc_leadlag *c, tc_real gain, tc_real wz, tc_real wp, tc_real ts)
{
    tc_biquad_coeffs coeffs;
    tc_status status;

    if ((c == NULL) || !tc_is_finite(gain) || !tc_is_finite(wz) || !tc_is_finite(wp) || !(wz > TC_R(0.0)) ||
        !(wp > TC_R(0.0)))
    {
        return TC_ERR_ARG;
    }

    {
        /* C(s) = gain * (s / wz + 1) / (s / wp + 1) */
        const tc_real num[3] = {TC_R(0.0), gain / wz, gain};
        const tc_real den[3] = {TC_R(0.0), TC_R(1.0) / wp, TC_R(1.0)};

        status = tc_tustin(num, den, ts, TC_R(0.0), &coeffs);
    }
    if (status != TC_OK)
    {
        return status;
    }

    return tc_biquad_init(&c->section, &coeffs);
}

void tc_leadlag_reset(tc_leadlag *c, tc_real x)
{
    if (c == NULL)
    {
        return;
    }
    tc_biquad_reset(&c->section, x);
}

tc_real tc_leadlag_update(tc_leadlag *c, tc_real x)
{
    return tc_biquad_update(&c->section, x);
}
