#!/usr/bin/env python3
"""Plots the CSV output of examples/motor_speed.c.

Usage:
    ./motor_speed > motor_speed.csv
    python tools/plot_example.py motor_speed.csv docs/img/motor_speed.png

SPDX-License-Identifier: MIT
"""
import csv
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    with open(sys.argv[1], newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    t = [float(r["time_s"]) for r in rows]
    setpoint = [float(r["setpoint_rad_s"]) for r in rows]
    speed = [float(r["speed_rad_s"]) for r in rows]
    voltage = [float(r["voltage_v"]) for r in rows]

    fig, (ax_speed, ax_voltage) = plt.subplots(2, 1, sharex=True, figsize=(8, 5.5), constrained_layout=True)

    ax_speed.plot(t, setpoint, "--", color="0.45", label="setpoint")
    ax_speed.plot(t, speed, color="#1f6fb4", linewidth=1.8, label="motor speed")
    ax_speed.axvline(1.5, color="#c0392b", linewidth=0.8, linestyle=":")
    ax_speed.annotate("load torque applied", xy=(1.5, 14.5), xytext=(1.75, 9.5),
                      arrowprops={"arrowstyle": "->", "color": "#c0392b"}, color="#c0392b")
    ax_speed.set_ylabel("speed [rad/s]")
    ax_speed.set_title("tinyctrl: PI speed control of a DC motor (noisy sensor, 12 V supply)")
    ax_speed.legend(loc="lower right")
    ax_speed.grid(alpha=0.3)

    ax_voltage.plot(t, voltage, color="#e67e22", linewidth=1.0, label="controller output")
    ax_voltage.axhline(12.0, color="0.3", linewidth=0.8, linestyle="--", label="supply limit")
    ax_voltage.annotate("saturated, anti-windup active", xy=(0.2, 12.0), xytext=(0.55, 13.6), color="0.25",
                        arrowprops={"arrowstyle": "->", "color": "0.3"})
    ax_voltage.set_ylim(-1.0, 15.0)
    ax_voltage.set_xlabel("time [s]")
    ax_voltage.set_ylabel("voltage [V]")
    ax_voltage.legend(loc="lower right")
    ax_voltage.grid(alpha=0.3)

    fig.savefig(sys.argv[2], dpi=130)
    print(f"Wrote {sys.argv[2]}")


if __name__ == "__main__":
    main()
