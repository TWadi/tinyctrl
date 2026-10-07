/**
 * @file test_biquad.c
 * @brief Unit tests for tc_biquad.
 *
 * SPDX-License-Identifier: MIT
 */
#include <float.h>
#include <math.h>
#include <stddef.h>

#include "reference/reference_data.h"
#include "test_helpers.h"
#include "tinyctrl/tc_biquad.h"
#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

static tc_biquad_coeffs coeffs_from_reference(const ref_tustin_case *ref)
{
    tc_biquad_coeffs c;

    c.b0 = (tc_real)ref->b0;
    c.b1 = (tc_real)ref->b1;
    c.b2 = (tc_real)ref->b2;
    c.a1 = (tc_real)ref->a1;
    c.a2 = (tc_real)ref->a2;

    return c;
}

static void test_matches_matlab_filter(void)
{
    /* Case 3 of the reference data is a 100 Hz notch filter sampled at 2 kHz. */
    const tc_biquad_coeffs c = coeffs_from_reference(&ref_tustin_cases[2]);
    tc_biquad f;
    tc_real output[REF_N];
    int k;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_biquad_init(&f, &c));
    tc_biquad_reset(&f, TC_R(0.0)); /* MATLAB's filter() starts from rest */
    for (k = 0; k < REF_N; ++k)
    {
        output[k] = tc_biquad_update(&f, (tc_real)ref_input[k]);
    }

    ASSERT_SEQUENCE_CLOSE(ref_biquad_notch_output, output, REF_N);
}

static void test_passthrough(void)
{
    const tc_biquad_coeffs c = {TC_R(1.0), TC_R(0.0), TC_R(0.0), TC_R(0.0), TC_R(0.0)};
    tc_biquad f;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_biquad_init(&f, &c));
    ASSERT_REAL_CLOSE(3.5, tc_biquad_update(&f, TC_R(3.5)));
    ASSERT_REAL_CLOSE(-1.25, tc_biquad_update(&f, TC_R(-1.25)));
}

static void test_reset_starts_in_steady_state(void)
{
    /* Case 1 of the reference data is a unity-gain second-order low-pass. */
    const tc_biquad_coeffs c = coeffs_from_reference(&ref_tustin_cases[0]);
    tc_biquad f;
    int k;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_biquad_init(&f, &c));
    tc_biquad_reset(&f, TC_R(2.0));
    for (k = 0; k < 20; ++k)
    {
        ASSERT_REAL_CLOSE(2.0, tc_biquad_update(&f, TC_R(2.0)));
    }
}

static void test_reset_of_integrator_clears_state(void)
{
    /* Case 5 is a PI controller: its pole at z = 1 has no steady state, so the reset clears the state. */
    const tc_biquad_coeffs c = coeffs_from_reference(&ref_tustin_cases[4]);
    tc_biquad f;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_biquad_init(&f, &c));
    (void)tc_biquad_update(&f, TC_R(1.0));
    tc_biquad_reset(&f, TC_R(1.0));
    ASSERT_REAL_CLOSE(0.0, tc_biquad_update(&f, TC_R(0.0)));
}

static void test_first_update_starts_in_steady_state(void)
{
    const tc_biquad_coeffs c = coeffs_from_reference(&ref_tustin_cases[0]);
    tc_biquad f;
    int k;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_biquad_init(&f, &c));
    for (k = 0; k < 20; ++k)
    {
        ASSERT_REAL_CLOSE(2.0, tc_biquad_update(&f, TC_R(2.0)));
    }
}

static void test_reset_to_zero_starts_from_rest(void)
{
    const tc_biquad_coeffs c = coeffs_from_reference(&ref_tustin_cases[0]);
    tc_biquad f;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_biquad_init(&f, &c));
    tc_biquad_reset(&f, TC_R(0.0));
    ASSERT_REAL_CLOSE(2.0 * (double)c.b0, tc_biquad_update(&f, TC_R(2.0)));
}

static void test_reset_treats_a_pole_within_rounding_of_one_as_an_integrator(void)
{
    /* (z - 1)(z - 0.5) with a2 off by a few units in the last place: 1 + a1 + a2 is rounding noise, and a steady
     * state computed from it would be about 1 / epsilon times too large. */
    const tc_real eps = (tc_real)(sizeof(tc_real) == sizeof(float) ? (double)FLT_EPSILON : DBL_EPSILON);
    const tc_biquad_coeffs c = {TC_R(1.0), TC_R(0.0), TC_R(0.0), TC_R(-1.5), TC_R(0.5) + TC_R(2.0) * eps};
    tc_biquad f;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_biquad_init(&f, &c));
    tc_biquad_reset(&f, TC_R(1.0));
    ASSERT_REAL_CLOSE(0.0, tc_biquad_update(&f, TC_R(0.0)));
}

static void test_non_finite_input_holds_the_output(void)
{
    const tc_biquad_coeffs c = coeffs_from_reference(&ref_tustin_cases[2]);
    tc_biquad f;
    tc_biquad twin;
    tc_real held;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_biquad_init(&f, &c));
    TEST_ASSERT_EQUAL_INT(TC_OK, tc_biquad_init(&twin, &c));
    tc_biquad_reset(&f, TC_R(0.0));
    tc_biquad_reset(&twin, TC_R(0.0));

    held = tc_biquad_update(&f, TC_R(1.0));
    (void)tc_biquad_update(&twin, TC_R(1.0));
    ASSERT_REAL_CLOSE(held, tc_biquad_update(&f, (tc_real)NAN));
    ASSERT_REAL_CLOSE(held, tc_biquad_update(&f, (tc_real)INFINITY));
    ASSERT_REAL_CLOSE(tc_biquad_update(&twin, TC_R(-1.0)), tc_biquad_update(&f, TC_R(-1.0)));
}

static void test_non_finite_first_input_does_not_prime(void)
{
    const tc_biquad_coeffs c = coeffs_from_reference(&ref_tustin_cases[0]);
    tc_biquad f;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_biquad_init(&f, &c));
    ASSERT_REAL_CLOSE(0.0, tc_biquad_update(&f, (tc_real)NAN));
    ASSERT_REAL_CLOSE(3.0, tc_biquad_update(&f, TC_R(3.0)));
}

static void test_reset_ignores_invalid_arguments(void)
{
    const tc_biquad_coeffs c = coeffs_from_reference(&ref_tustin_cases[0]);
    tc_biquad f;

    tc_biquad_reset(NULL, TC_R(1.0)); /* must not crash */
    TEST_ASSERT_EQUAL_INT(TC_OK, tc_biquad_init(&f, &c));
    tc_biquad_reset(&f, TC_R(2.0));
    tc_biquad_reset(&f, (tc_real)NAN);
    ASSERT_REAL_CLOSE(2.0, tc_biquad_update(&f, TC_R(2.0)));
}

static void test_init_rejects_invalid_arguments(void)
{
    const tc_biquad_coeffs c = {TC_R(1.0), TC_R(0.0), TC_R(0.0), TC_R(0.0), TC_R(0.0)};
    tc_biquad f;
    int i;

    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_biquad_init(NULL, &c));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_biquad_init(&f, NULL));

    for (i = 0; i < 5; ++i)
    {
        tc_biquad_coeffs bad = c;
        tc_real *const field[5] = {&bad.b0, &bad.b1, &bad.b2, &bad.a1, &bad.a2};

        *field[i] = (tc_real)NAN;
        TEST_ASSERT_EQUAL_INT_MESSAGE(TC_ERR_ARG, tc_biquad_init(&f, &bad), "NaN coefficient accepted");
        *field[i] = (tc_real)INFINITY;
        TEST_ASSERT_EQUAL_INT_MESSAGE(TC_ERR_ARG, tc_biquad_init(&f, &bad), "infinite coefficient accepted");
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_matches_matlab_filter);
    RUN_TEST(test_passthrough);
    RUN_TEST(test_reset_starts_in_steady_state);
    RUN_TEST(test_reset_of_integrator_clears_state);
    RUN_TEST(test_first_update_starts_in_steady_state);
    RUN_TEST(test_reset_to_zero_starts_from_rest);
    RUN_TEST(test_reset_treats_a_pole_within_rounding_of_one_as_an_integrator);
    RUN_TEST(test_non_finite_input_holds_the_output);
    RUN_TEST(test_non_finite_first_input_does_not_prime);
    RUN_TEST(test_reset_ignores_invalid_arguments);
    RUN_TEST(test_init_rejects_invalid_arguments);
    return UNITY_END();
}
