/**
 * @file tc_biquad.c
 * @brief Second-order IIR filter section in transposed direct form II.
 *
 * SPDX-License-Identifier: MIT
 */
#include "tinyctrl/tc_biquad.h"

#include <stddef.h>

#include "tc_internal.h"

tc_status tc_biquad_init(tc_biquad *f, const tc_biquad_coeffs *c)
{
    if ((f == NULL) || (c == NULL))
    {
        return TC_ERR_ARG;
    }
    if (!tc_is_finite(c->b0) || !tc_is_finite(c->b1) || !tc_is_finite(c->b2) || !tc_is_finite(c->a1) ||
        !tc_is_finite(c->a2))
    {
        return TC_ERR_ARG;
    }

    f->c = *c;
    f->s1 = TC_R(0.0);
    f->s2 = TC_R(0.0);
    f->y = TC_R(0.0);
    f->primed = 0;

    return TC_OK;
}

void tc_biquad_reset(tc_biquad *f, tc_real x)
{
    tc_real den;
    tc_real tol;

    if ((f == NULL) || !tc_is_finite(x))
    {
        return;
    }

    /* 1 + a1 + a2 is the denominator at z = 1. When it is within rounding error of zero the filter has a pole at
     * z = 1 (an integrator), H(1) is meaningless, and the state is cleared instead. */
    den = TC_R(1.0) + f->c.a1 + f->c.a2;
    tol = TC_R(8.0) * TC_EPS * (TC_R(1.0) + tc_abs(f->c.a1) + tc_abs(f->c.a2));
    if (tc_abs(den) > tol)
    {
        f->y = (f->c.b0 + f->c.b1 + f->c.b2) / den * x;
        f->s2 = f->c.b2 * x - f->c.a2 * f->y;
        f->s1 = f->c.b1 * x - f->c.a1 * f->y + f->s2;
    }
    else
    {
        f->y = TC_R(0.0);
        f->s1 = TC_R(0.0);
        f->s2 = TC_R(0.0);
    }
    f->primed = 1;
}

tc_real tc_biquad_update(tc_biquad *f, tc_real x)
{
    tc_real y;

    if (!tc_is_finite(x))
    {
        return f->y;
    }
    if (!f->primed)
    {
        tc_biquad_reset(f, x);
    }

    y = f->c.b0 * x + f->s1;
    f->s1 = f->c.b1 * x - f->c.a1 * y + f->s2;
    f->s2 = f->c.b2 * x - f->c.a2 * y;
    f->y = y;

    return y;
}
