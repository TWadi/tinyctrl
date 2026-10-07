# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[Semantic Versioning](https://semver.org/).

## [0.1.0] - 2026-10-07

### Added

- `tc_pid`: PID controller with trapezoidal integration, filtered derivative on the measurement, output limits
  with conditional-integration anti-windup, and bumpless reset.
- `tc_tustin`: Tustin discretisation of continuous transfer functions up to second order, with prewarping.
- `tc_biquad`: second-order IIR section in transposed direct form II with steady-state reset.
- `tc_leadlag`: lead/lag compensator.
- `tc_lowpass`: first-order low-pass filter from a time constant, a cutoff frequency or a precomputed smoothing
  factor (`tc_lowpass_init_alpha`, which needs no `exp()` on the target).
- Every block starts in the steady state of its first sample unless it is reset, and ignores NaN and infinite
  inputs (the update returns the previous output and leaves the state unchanged).
- Unit tests against SciPy reference data, a MATLAB generator for the same data, a DC motor example, and CI for
  GCC, Clang, MSVC, sanitizers, and linked firmware images for Cortex-M0, Cortex-M4F and AVR.
- CMake package configuration for `find_package(tinyctrl)`.

[0.1.0]: https://github.com/TWadi/tinyctrl/releases/tag/v0.1.0
