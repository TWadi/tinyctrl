# Contributing to tinyctrl

Thanks for your interest! Bug reports, questions and pull requests are all welcome.

## Before you start

For new features, please open an issue first so we can agree on the API before you invest time.

## Development

```sh
cmake -S . -B build -DTINYCTRL_WARNINGS_AS_ERRORS=ON
cmake --build build
cd build && ctest --output-on-failure
```

Please also run the tests in double precision (`-DTINYCTRL_USE_DOUBLE=ON`) and, on Linux or macOS, with
sanitizers (`-DTINYCTRL_SANITIZE=ON`).

## Guidelines

- **C99 only**, no dynamic memory, no global or static mutable state in the library.
- **Validate parameters in `*_init` functions** and return `TC_ERR_ARG`; keep `*_update` functions fast. Their
  only check is the one that skips NaN and infinite inputs (see section 6 of [docs/theory.md](docs/theory.md)),
  and new modules should follow the same start-up and invalid-sample rules.
- **Every change comes with tests.** Bug fixes need a test that fails without the fix. New numerical features
  should be checked against reference data: extend `tools/gen_reference.py` (and `tools/gen_reference.m`), then
  regenerate `tests/reference/reference_data.h` with `python tools/gen_reference.py`.
- **No warnings.** The build uses `-Wall -Wextra -Wpedantic -Wconversion -Wdouble-promotion` and more with
  `-Werror`; CI also compiles for Cortex-M0, Cortex-M4F and AVR.
- **Formatting:** run `clang-format -i` (version 20) on the files you change.
- **Commit messages:** a short imperative summary line (for example "Add setpoint weighting to tc_pid"), then a
  blank line and the reason for the change.
