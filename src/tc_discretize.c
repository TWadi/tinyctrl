/**
 * @file tc_discretize.c
 * @brief Tustin (bilinear) discretisation of continuous transfer functions up to second order.
 *
 * Substituting s = K (z - 1) / (z + 1) and multiplying numerator and denominator by (z + 1)^2 gives, for
 * N(s) = n0 s^2 + n1 s + n2:
 *
 *     z^2: n0 K^2 + n1 K + n2      z^1: 2 (n2 - n0 K^2)      z^0: n0 K^2 - n1 K + n2
 *
 * and the same for the denominator. First-order systems are multiplied by (z + 1) only, which avoids a pole and
 * zero cancelling at z = -1.
 *
 * SPDX-License-Identifier: MIT
 */
#include "tinyctrl/tc_discretize.h"

#include <stddef.h>

#include "tc_internal.h"

static int tc_all_finite(const tc_real v[3])
{
    return tc_is_finite(v[0]) && tc_is_finite(v[1]) && tc_is_finite(v[2]);
}

tc_status tc_tustin(const tc_real num[3], const tc_real den[3], tc_real ts, tc_real prewarp, tc_biquad_coeffs *out)
{
    double k;
    double b0;
    double b1;
    double b2;
    double a0;
    double a1;
    double a2;

    if ((num == NULL) || (den == NULL) || (out == NULL))
    {
        return TC_ERR_ARG;
    }
    if (!tc_all_finite(num) || !tc_all_finite(den) || !tc_is_finite(ts) || !tc_is_finite(prewarp))
    {
        return TC_ERR_ARG;
    }
    if (!(ts > TC_R(0.0)) || (prewarp < TC_R(0.0)) || !((double)prewarp * (double)ts < TC_PI_D))
    {
        return TC_ERR_ARG;
    }

    if (prewarp > TC_R(0.0))
    {
        k = (double)prewarp / tan((double)prewarp * (double)ts / 2.0);
    }
    else
    {
        k = 2.0 / (double)ts;
    }

    if (tc_is_nonzero(den[0]))
    {
        const double k2 = k * k;

        b0 = (double)num[0] * k2 + (double)num[1] * k + (double)num[2];
        b1 = 2.0 * ((double)num[2] - (double)num[0] * k2);
        b2 = (double)num[0] * k2 - (double)num[1] * k + (double)num[2];
        a0 = (double)den[0] * k2 + (double)den[1] * k + (double)den[2];
        a1 = 2.0 * ((double)den[2] - (double)den[0] * k2);
        a2 = (double)den[0] * k2 - (double)den[1] * k + (double)den[2];
    }
    else if (tc_is_nonzero(num[0]))
    {
        return TC_ERR_ARG; /* improper: numerator of higher order than denominator */
    }
    else if (tc_is_nonzero(den[1]))
    {
        b0 = (double)num[1] * k + (double)num[2];
        b1 = (double)num[2] - (double)num[1] * k;
        b2 = 0.0;
        a0 = (double)den[1] * k + (double)den[2];
        a1 = (double)den[2] - (double)den[1] * k;
        a2 = 0.0;
    }
    else if (tc_is_nonzero(num[1]))
    {
        return TC_ERR_ARG; /* improper */
    }
    else if (tc_is_nonzero(den[2]))
    {
        b0 = (double)num[2];
        b1 = 0.0;
        b2 = 0.0;
        a0 = (double)den[2];
        a1 = 0.0;
        a2 = 0.0;
    }
    else
    {
        return TC_ERR_ARG; /* zero denominator */
    }

    /* a0 is the denominator evaluated at s = K. A (nearly) zero a0 means a continuous pole at (or rounding-close
     * to) s = K, which maps to z = infinity and leaves coefficients made of rounding error. */
    {
        const double scale = fabs((double)den[0]) * k * k + fabs((double)den[1]) * k + fabs((double)den[2]);

        if (!(fabs(a0) > 1e-6 * scale))
        {
            return TC_ERR_ARG;
        }
    }

    out->b0 = (tc_real)(b0 / a0);
    out->b1 = (tc_real)(b1 / a0);
    out->b2 = (tc_real)(b2 / a0);
    out->a1 = (tc_real)(a1 / a0);
    out->a2 = (tc_real)(a2 / a0);

    return TC_OK;
}
