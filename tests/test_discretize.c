/**
 * @file test_discretize.c
 * @brief Unit tests for tc_tustin.
 *
 * SPDX-License-Identifier: MIT
 */
#include <math.h>
#include <stddef.h>

#include "reference/reference_data.h"
#include "test_helpers.h"
#include "tinyctrl/tc_discretize.h"
#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

/* Coefficients are computed in double precision and rounded once, so they are more accurate than the
 * accumulated time-domain results. */
#ifdef TINYCTRL_USE_DOUBLE
#define COEFF_TOL 1e-12
#else
#define COEFF_TOL 1e-6
#endif

static void to_real3(const double in[3], tc_real out[3])
{
    out[0] = (tc_real)in[0];
    out[1] = (tc_real)in[1];
    out[2] = (tc_real)in[2];
}

static void test_matches_matlab_c2d(void)
{
    int i;

    for (i = 0; i < REF_TUSTIN_COUNT; ++i)
    {
        const ref_tustin_case *ref = &ref_tustin_cases[i];
        tc_real num[3];
        tc_real den[3];
        tc_biquad_coeffs c;

        to_real3(ref->num, num);
        to_real3(ref->den, den);
        TEST_ASSERT_EQUAL_INT_MESSAGE(TC_OK, tc_tustin(num, den, (tc_real)ref->ts, (tc_real)ref->prewarp, &c),
                                      "tc_tustin failed");
        ASSERT_REAL_WITHIN(COEFF_TOL, ref->b0, c.b0);
        ASSERT_REAL_WITHIN(COEFF_TOL, ref->b1, c.b1);
        ASSERT_REAL_WITHIN(COEFF_TOL, ref->b2, c.b2);
        ASSERT_REAL_WITHIN(COEFF_TOL, ref->a1, c.a1);
        ASSERT_REAL_WITHIN(COEFF_TOL, ref->a2, c.a2);
    }
}

static void test_preserves_dc_gain(void)
{
    /* H(s) = 4 / (s^2 + 3 s + 2) has DC gain 2; Tustin maps s = 0 to z = 1. */
    const tc_real num[3] = {TC_R(0.0), TC_R(0.0), TC_R(4.0)};
    const tc_real den[3] = {TC_R(1.0), TC_R(3.0), TC_R(2.0)};
    tc_biquad_coeffs c;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_tustin(num, den, TC_R(0.01), TC_R(0.0), &c));
    ASSERT_REAL_CLOSE(2.0, (c.b0 + c.b1 + c.b2) / (TC_R(1.0) + c.a1 + c.a2));
}

static void test_first_order_result_has_no_second_order_terms(void)
{
    const tc_real num[3] = {TC_R(0.0), TC_R(1.0), TC_R(2.0)};
    const tc_real den[3] = {TC_R(0.0), TC_R(0.5), TC_R(1.0)};
    tc_biquad_coeffs c;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_tustin(num, den, TC_R(0.01), TC_R(0.0), &c));
    ASSERT_REAL_CLOSE(0.0, c.b2);
    ASSERT_REAL_CLOSE(0.0, c.a2);
}

typedef struct
{
    double re;
    double im;
} complex_t;

static complex_t c_div(complex_t a, complex_t b)
{
    const double d = b.re * b.re + b.im * b.im;
    complex_t q;

    q.re = (a.re * b.re + a.im * b.im) / d;
    q.im = (a.im * b.re - a.re * b.im) / d;

    return q;
}

/* p[0] s^2 + p[1] s + p[2] at s = j w */
static complex_t poly_at_jw(const tc_real p[3], double w)
{
    complex_t v;

    v.re = (double)p[2] - (double)p[0] * w * w;
    v.im = (double)p[1] * w;

    return v;
}

/* c0 + c1 z^-1 + c2 z^-2 at z = exp(j theta) */
static complex_t poly_at_unit_circle(double c0, double c1, double c2, double theta)
{
    complex_t v;

    v.re = c0 + c1 * cos(theta) + c2 * cos(2.0 * theta);
    v.im = -(c1 * sin(theta) + c2 * sin(2.0 * theta));

    return v;
}

/* Tustin maps z = exp(j theta) to s = j K tan(theta / 2), so the discrete response there must equal the continuous
 * response at w = K tan(theta / 2). This checks every coefficient, including combinations that a time-domain
 * comparison with a single input could miss. */
static void check_frequency_response(const double num_d[3], const double den_d[3], double prewarp)
{
    const double ts = 1e-3;
    tc_real num[3];
    tc_real den[3];
    tc_biquad_coeffs c;
    double k;
    double theta;

    to_real3(num_d, num);
    to_real3(den_d, den);
    TEST_ASSERT_EQUAL_INT(TC_OK, tc_tustin(num, den, (tc_real)ts, (tc_real)prewarp, &c));

    k = prewarp > 0.0 ? (double)(tc_real)prewarp / tan((double)(tc_real)prewarp * (double)(tc_real)ts / 2.0)
                      : 2.0 / (double)(tc_real)ts;
    for (theta = 0.05; theta < 3.1; theta += 0.15)
    {
        const double w = k * tan(theta / 2.0);
        const complex_t h_s = c_div(poly_at_jw(num, w), poly_at_jw(den, w));
        const complex_t h_z = c_div(poly_at_unit_circle((double)c.b0, (double)c.b1, (double)c.b2, theta),
                                    poly_at_unit_circle(1.0, (double)c.a1, (double)c.a2, theta));
        const double magnitude = sqrt(h_s.re * h_s.re + h_s.im * h_s.im);
        const double tol = 1e-4 * (magnitude > 1.0 ? magnitude : 1.0);

        ASSERT_REAL_WITHIN(tol, h_s.re, h_z.re);
        ASSERT_REAL_WITHIN(tol, h_s.im, h_z.im);
    }
}

static void test_frequency_response_matches_at_warped_frequencies(void)
{
    const double band_pass_num[3] = {0.0, 100.0, 0.0};
    const double band_pass_den[3] = {1.0, 100.0, 4e5};
    const double general_num[3] = {1e-3, 0.5, 20.0};
    const double general_den[3] = {2e-3, 3.0, 400.0};
    const double first_order_num[3] = {0.0, 0.01, 1.0};
    const double first_order_den[3] = {0.0, 0.002, 1.0};

    check_frequency_response(band_pass_num, band_pass_den, 0.0);
    check_frequency_response(band_pass_num, band_pass_den, 600.0);
    check_frequency_response(general_num, general_den, 0.0);
    check_frequency_response(first_order_num, first_order_den, 0.0);
    check_frequency_response(first_order_num, first_order_den, 1000.0);
}

static void test_rejects_invalid_arguments(void)
{
    const tc_real num[3] = {TC_R(0.0), TC_R(0.0), TC_R(1.0)};
    const tc_real den[3] = {TC_R(1.0), TC_R(2.0), TC_R(1.0)};
    const tc_real improper_num[3] = {TC_R(1.0), TC_R(0.0), TC_R(1.0)};
    const tc_real first_order_den[3] = {TC_R(0.0), TC_R(1.0), TC_R(1.0)};
    const tc_real zero_den[3] = {TC_R(0.0), TC_R(0.0), TC_R(0.0)};
    const tc_real ts = TC_R(0.01);
    /* A pole at s = +2/Ts maps to z = infinity. */
    const tc_real pole_at_k_den[3] = {TC_R(0.0), TC_R(1.0), TC_R(-200.0)};
    tc_biquad_coeffs c;

    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_tustin(NULL, den, ts, TC_R(0.0), &c));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_tustin(num, NULL, ts, TC_R(0.0), &c));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_tustin(num, den, ts, TC_R(0.0), NULL));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_tustin(num, den, TC_R(0.0), TC_R(0.0), &c));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_tustin(num, den, ts, TC_R(-1.0), &c));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_tustin(num, den, ts, TC_R(315.0), &c)); /* >= pi / Ts */
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_tustin(improper_num, first_order_den, ts, TC_R(0.0), &c));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_tustin(num, zero_den, ts, TC_R(0.0), &c));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_tustin(num, pole_at_k_den, ts, TC_R(0.0), &c));
}

static void test_rejects_improper_systems_and_non_finite_inputs(void)
{
    const tc_real num[3] = {TC_R(0.0), TC_R(0.0), TC_R(1.0)};
    const tc_real den[3] = {TC_R(1.0), TC_R(2.0), TC_R(1.0)};
    const tc_real first_order_num[3] = {TC_R(0.0), TC_R(1.0), TC_R(0.0)};
    const tc_real static_den[3] = {TC_R(0.0), TC_R(0.0), TC_R(1.0)};
    const tc_real ts = TC_R(0.01);
    tc_real bad[3] = {TC_R(0.0), TC_R(0.0), TC_R(1.0)};
    tc_biquad_coeffs c;

    /* A pure differentiator s / 1 is improper. */
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_tustin(first_order_num, static_den, ts, TC_R(0.0), &c));

    bad[2] = (tc_real)NAN;
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_tustin(bad, den, ts, TC_R(0.0), &c));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_tustin(num, bad, ts, TC_R(0.0), &c));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_tustin(num, den, (tc_real)INFINITY, TC_R(0.0), &c));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_tustin(num, den, ts, (tc_real)NAN, &c));
}

static void test_static_gain(void)
{
    const tc_real num[3] = {TC_R(0.0), TC_R(0.0), TC_R(3.0)};
    const tc_real den[3] = {TC_R(0.0), TC_R(0.0), TC_R(2.0)};
    tc_biquad_coeffs c;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_tustin(num, den, TC_R(0.01), TC_R(0.0), &c));
    ASSERT_REAL_CLOSE(1.5, c.b0);
    ASSERT_REAL_CLOSE(0.0, c.b1);
    ASSERT_REAL_CLOSE(0.0, c.b2);
    ASSERT_REAL_CLOSE(0.0, c.a1);
    ASSERT_REAL_CLOSE(0.0, c.a2);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_matches_matlab_c2d);
    RUN_TEST(test_preserves_dc_gain);
    RUN_TEST(test_first_order_result_has_no_second_order_terms);
    RUN_TEST(test_frequency_response_matches_at_warped_frequencies);
    RUN_TEST(test_rejects_invalid_arguments);
    RUN_TEST(test_rejects_improper_systems_and_non_finite_inputs);
    RUN_TEST(test_static_gain);
    return UNITY_END();
}
