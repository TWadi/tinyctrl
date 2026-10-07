/**
 * @file test_helpers.h
 * @brief Tolerance-based comparisons shared by the unit tests.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef TINYCTRL_TEST_HELPERS_H
#define TINYCTRL_TEST_HELPERS_H

#include <math.h>
#include <stdio.h>

#include "tinyctrl/tc_types.h"
#include "unity.h"

/* Relative tolerance (with an absolute floor of the same size for values near zero). The reference data is
 * double precision; single-precision builds lose about seven significant digits. */
#ifdef TINYCTRL_USE_DOUBLE
#define TC_TEST_TOL 1e-9
#else
#define TC_TEST_TOL 2e-4
#endif

static inline int tc_test_close(double expected, double actual, double tol)
{
    const double scale = fabs(expected) > 1.0 ? fabs(expected) : 1.0;
    return fabs(actual - expected) <= tol * scale;
}

static inline void tc_test_assert_close(double expected, double actual, double tol, unsigned line, const char *what)
{
    char message[160];

    if (!tc_test_close(expected, actual, tol))
    {
        (void)snprintf(message, sizeof message, "%s: expected %.10g, got %.10g", what, expected, actual);
        UNITY_TEST_FAIL(line, message);
    }
}

static inline void tc_test_assert_sequence(const double *expected, const tc_real *actual, int count, double tol,
                                           unsigned line)
{
    char message[160];
    int i;

    for (i = 0; i < count; ++i)
    {
        if (!tc_test_close(expected[i], (double)actual[i], tol))
        {
            (void)snprintf(message, sizeof message, "sample %d: expected %.10g, got %.10g", i, expected[i],
                           (double)actual[i]);
            UNITY_TEST_FAIL(line, message);
        }
    }
}

/** Asserts that two reals agree within TC_TEST_TOL (relative, with an absolute floor). */
#define ASSERT_REAL_CLOSE(expected, actual)                                                                            \
    tc_test_assert_close((double)(expected), (double)(actual), TC_TEST_TOL, (unsigned)__LINE__, #actual)

/** Asserts that two reals agree within an explicit tolerance. */
#define ASSERT_REAL_WITHIN(tol, expected, actual)                                                                      \
    tc_test_assert_close((double)(expected), (double)(actual), (tol), (unsigned)__LINE__, #actual)

/** Asserts that a tc_real sequence matches double reference data within TC_TEST_TOL. */
#define ASSERT_SEQUENCE_CLOSE(expected, actual, count)                                                                 \
    tc_test_assert_sequence((expected), (actual), (count), TC_TEST_TOL, (unsigned)__LINE__)

#endif /* TINYCTRL_TEST_HELPERS_H */
