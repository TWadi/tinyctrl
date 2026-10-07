# Theory behind tinyctrl

This page explains the math each module implements and the design choices behind it. The notation follows
the usual control-engineering convention: `s` is the Laplace variable, `z` the z-transform variable, `Ts` the
sample time.

## 1. Tustin (bilinear) discretisation

A digital controller only sees the signal at the sample instants, so a continuous transfer function `H(s)` has to
be turned into a difference equation. The Tustin method replaces every integrator `1/s` by the trapezoidal rule,
which is the substitution

```
s = (2 / Ts) * (z - 1) / (z + 1)
```

Properties that make it the default choice for controllers:

- **Stability is preserved.** The left half of the s-plane maps exactly onto the inside of the unit circle, so a
  stable continuous controller always gives a stable discrete one.
- **DC gain is preserved.** `s = 0` maps to `z = 1`.
- **Frequencies are warped.** A continuous frequency `wc` appears at the discrete frequency
  `wd = (2 / Ts) * atan(wc * Ts / 2)`. The error is small far below the Nyquist frequency but grows towards it.

**Prewarping** fixes the warping at one chosen frequency `w0` by using

```
s = K * (z - 1) / (z + 1),   K = w0 / tan(w0 * Ts / 2)
```

instead of `K = 2 / Ts`. The discrete response then matches the continuous one exactly at `w0`. This matters for
notch filters (the notch must sit exactly on the disturbance) and for crossover frequencies near Nyquist.

`tc_tustin()` substitutes `s` into a second-order `H(s) = (n0 s^2 + n1 s + n2) / (d0 s^2 + d1 s + d2)` and
multiplies numerator and denominator by `(z + 1)^2`:

| power | numerator | denominator |
|---|---|---|
| `z^2` | `n0 K^2 + n1 K + n2` | `d0 K^2 + d1 K + d2` |
| `z^1` | `2 (n2 - n0 K^2)` | `2 (d2 - d0 K^2)` |
| `z^0` | `n0 K^2 - n1 K + n2` | `d0 K^2 - d1 K + d2` |

Dividing everything by the `z^2` denominator coefficient (`a0`) gives the biquad coefficients. First-order
systems are multiplied by `(z + 1)` only; using `(z + 1)^2` would add a pole and a zero at `z = -1` that cancel
only in exact arithmetic. `a0` is the denominator evaluated at `s = K`; if it is zero, a continuous pole sits at
`s = K` and maps to `z = infinity`. `tc_tustin()` rejects the system when `|a0|` is below `1e-6` times the sum of
the magnitudes of its terms, because the coefficients would then consist mostly of rounding error. Only unstable
systems (a real pole at `s = +K`) can trigger this.

The coefficients are computed in double precision, even in float builds: initialisation runs once, and rounding
only the final coefficients keeps them as accurate as the target type allows.

## 2. PID controller

`tc_pid` implements the parallel PID

```
u = Kp e + (Ki / s) e - (Kd s / (Tf s + 1)) y,      e = r - y
```

### Integral term

Trapezoidal integration (Tustin) of the error:

```
I[k] = I[k-1] + (Ki Ts / 2) (e[k] + e[k-1])
```

### Derivative term

Two practical changes compared with the textbook `Kd s e`:

1. **Derivative on the measurement.** Differentiating the error turns every setpoint step into a huge spike in the
   output (a "derivative kick"). The setpoint is usually piecewise constant, so differentiating `-y` instead gives
   the same disturbance response without the kick.
2. **A first-order filter** `1 / (Tf s + 1)`. A pure derivative amplifies measurement noise without limit at high
   frequencies; the filter caps the high-frequency gain at `Kd / Tf`. A common choice is `Tf = Kd / (N Kp)` with
   `N` between 5 and 20.

Tustin discretisation of `-Kd s / (Tf s + 1)` gives

```
D[k] = ((2 Tf - Ts) / (2 Tf + Ts)) D[k-1]  -  (2 Kd / (2 Tf + Ts)) (y[k] - y[k-1])
```

The pole of this recursion, `(2 Tf - Ts) / (2 Tf + Ts)`, is negative for `Tf < Ts / 2`: the derivative term then
flips sign from one sample to the next, and at `Tf = 0` (pole at `z = -1`) it never decays. `tc_pid_init()`
therefore requires `Tf >= Ts / 2` whenever `Kd != 0`. At `Tf = Ts / 2` the pole is at 0 and the term reduces to the
backward difference `-Kd (y[k] - y[k-1]) / Ts`.

Starting from rest (`tc_pid_reset(pid, 0, 0, 0)`) and without saturation, the controller is exactly
`c2d(pid(Kp, Ki, Kd, Tf), Ts, 'tustin')` applied to the error, with the derivative fed by `-y`; the unit tests check
this sample by sample.

### Anti-windup

When the actuator saturates (a motor driver at full voltage, a valve fully open), the plant no longer responds to
larger controller outputs, but a naive integrator keeps accumulating the error. Once the error changes sign, the
oversized integral must first unwind, which causes large overshoot and slow recovery: *integrator windup*.

`tc_pid` uses **conditional integration**: when the output exceeds a limit, the integrator may only move back
towards the linear range, never further into saturation. The example in the README shows the effect: the output
leaves the 12 V limit as soon as the motor approaches the setpoint, without overshoot.

### Bumpless transfer

When switching from manual to automatic control, the controller output should continue from the current actuator
value instead of jumping. `tc_pid_reset(pid, r, y, u)` puts the controller in the state it would have if it had
just produced `u`: the history holds the current error and measurement, the derivative state is cleared and the
integrator holds `u - Kp e`. The next update then adds the usual increment, so with `r` and `y` unchanged it
returns `u + Ki Ts e`. Calling the reset with the applied output on every sample while in manual mode keeps the
controller ready for a bumpless switch.

With `Ki = 0` the integrator never changes after a reset, so it acts as a constant bias `u - Kp e` (the "manual
reset" of a P or PD controller). `tc_pid_init()` starts with a bias of zero.

## 3. First-order low-pass filter

`tc_lowpass` implements exponential smoothing:

```
y[k] = y[k-1] + alpha (x[k] - y[k-1]),     alpha = 1 - exp(-Ts / tau)
```

The discrete pole `1 - alpha = exp(-Ts / tau)` is the exact image of the continuous pole `-1 / tau` (pole
matching), and the DC gain is 1. A cutoff frequency `fc` corresponds to `tau = 1 / (2 pi fc)`. Unlike the other
modules this filter is *not* a Tustin discretisation: it has no zero at `z = -1`, so near the Nyquist frequency it
attenuates less than the Tustin version of `1 / (tau s + 1)` would. In exchange, the update is a single
multiply-add.

For `Ts << tau`, `1 - exp(-x)` cancels most significant digits, especially where `double` is only 32 bits wide
(AVR). tinyctrl uses a short Taylor series for `x < 0.01` instead (`expm1()` is not available in every embedded C
library). `tc_lowpass_init_alpha()` skips the computation entirely for firmware that cannot afford `exp()`.

In float, an update stops changing the output once `alpha |x - y|` is smaller than half a unit in the last place of
`y`. The output can therefore settle up to about `|y| * 6e-8 / alpha` away from a constant input: 0.06 % of `|y|`
for `alpha = 1e-4`, which corresponds to a cutoff of about `fs / 60000`.

## 4. Biquad (second-order IIR section)

`tc_biquad` implements

```
H(z) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2)
```

in transposed direct form II:

```
y   = b0 x + s1
s1' = b1 x - a1 y + s2
s2' = b2 x - a2 y
```

This form needs only two state variables and behaves well in floating point. Higher-order filters should be built
as cascades of biquads rather than one high-order polynomial, whose coefficients are extremely sensitive to
rounding.

A limitation of single precision: low cutoff frequencies at high sample rates put the poles very close to `z = 1`,
where rounding the coefficients to float moves them noticeably. For a second-order low-pass (`zeta = 0.7`) the DC
gain error caused by float coefficients is:

| `fc / fs` | 1e-2 | 3e-3 | 1e-3 | 3e-4 | 1e-4 |
|---|---:|---:|---:|---:|---:|
| DC gain error | 0.0008 % | 0.02 % | 0.09 % | 0.8 % | 10 % |

As a rule of thumb, keep `fc / fs` above `1e-3` in float builds. Below that, use `TINYCTRL_USE_DOUBLE` or run the
filter at a lower sample rate. Prewarping does not help here: it moves where the response is exact, not how
precisely the coefficients are stored.

`tc_biquad_reset(f, x)` computes the steady state from `H(1) = (b0 + b1 + b2) / (1 + a1 + a2)`. When
`1 + a1 + a2` is within a few rounding errors of zero, the filter has (numerically) a pole at `z = 1`, an
integrator without a steady state, and the reset clears the state instead.

## 5. Lead and lag compensators

```
C(s) = gain * (s / wz + 1) / (s / wp + 1)
```

- **Lead** (`wz < wp`): adds phase between `wz` and `wp`, used to increase the phase margin. The maximum phase lead
  `phi_max = asin((a - 1) / (a + 1))` with `a = wp / wz` occurs at `sqrt(wz * wp)`.
- **Lag** (`wz > wp`): increases the low-frequency gain by `wz / wp` relative to the high-frequency gain, used to
  reduce steady-state error without changing the crossover region much.

`tc_leadlag` discretises it with `tc_tustin()` and runs it as a first-order biquad.

## 6. Start-up and invalid samples

Every module follows the same two rules:

- **No start-up transient.** A module that has not been reset takes its initial state from the first sample it
  sees: the PID uses it as its previous error and measurement (no derivative kick), the filters start in the steady
  state for that input. Call the `*_reset` function to start from a different state, for example
  `tc_biquad_reset(f, 0)` to reproduce MATLAB's `filter()`, which starts from rest.
- **Non-finite samples are ignored.** A NaN or infinity from a broken sensor or a division by zero elsewhere would
  otherwise enter the integrator or filter state and stay there for good. The `*_update` functions return their
  previous output instead and leave the state untouched; `*_reset` functions ignore non-finite arguments.

| Module | First `*_update` without a reset | Before any finite sample |
|---|---|---|
| `tc_pid` | history = first sample; integral term `Ki Ts e` | output 0, limited to `[out_min, out_max]` |
| `tc_biquad`, `tc_leadlag` | steady state for the first input | output 0 |
| `tc_lowpass` | output = first input | output 0 |

These checks rely on IEEE 754 semantics. Do not compile tinyctrl with `-ffast-math` (or `-ffinite-math-only`),
which lets the compiler assume that NaN and infinity never occur and remove the checks.
