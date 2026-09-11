#!/usr/bin/env python3
"""Calculate software zeros for the coaxial five-bar reference algorithm."""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path


LEGS = ("RF", "LF", "RH", "LH")
ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "Control" / "robot_config.h"


def config_value(name: str) -> float:
    pattern = re.compile(
        rf"^#define\s+SIMPLE_{name}\s+([-+]?[0-9.]+)[fFuU]*\s*$"
    )
    for line in CONFIG_PATH.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line.strip())
        if match:
            return float(match.group(1))
    raise ValueError(f"missing SIMPLE_{name} in {CONFIG_PATH}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Place the foot at a measured point relative to the common motor axis, "
            "then enter the two CAN-order motor feedback values in radians."
        )
    )
    parser.add_argument("--leg", choices=LEGS, required=True)
    parser.add_argument("--x-mm", type=float,
                        default=config_value("ZERO_CALIBRATION_X_MM"),
                        help="foot X from the common axis; forward is positive")
    parser.add_argument("--z-mm", type=float,
                        default=config_value("ZERO_CALIBRATION_Z_MM"),
                        help="vertical distance down from the common axis")
    parser.add_argument("--motor1", type=float, required=True,
                        help="first CAN-order motor feedback, rad")
    parser.add_argument("--motor2", type=float, required=True,
                        help="second CAN-order motor feedback, rad")
    return parser.parse_args()


def coaxial_theta(leg: str, x_mm: float, z_mm: float) -> tuple[float, float]:
    active = config_value("ACTIVE_LENGTH_MM")
    passive = config_value("PASSIVE_LENGTH_MM")
    radius = math.hypot(x_mm, z_mm)
    minimum = abs(passive - active)
    maximum = passive + active
    if z_mm <= 0.0 or not minimum < radius < maximum:
        raise ValueError(
            f"pose radius {radius:.3f} mm is outside ({minimum:.3f}, {maximum:.3f})"
        )
    x_sign = config_value("COAXIAL_X_SIGN")
    if x_sign not in (-1.0, 1.0):
        raise ValueError("SIMPLE_COAXIAL_X_SIGN must be +1 or -1")
    n = x_sign * math.asin(x_mm / radius)
    cosine = (radius**2 + active**2 - passive**2) / (2.0 * active * radius)
    if not -1.0 <= cosine <= 1.0:
        raise ValueError("pose has no real coaxial five-bar solution")
    m = math.acos(cosine)
    theta_sign = config_value("COAXIAL_THETA_SIGN")
    if theta_sign not in (-1.0, 1.0):
        raise ValueError("SIMPLE_COAXIAL_THETA_SIGN must be +1 or -1")
    low = theta_sign * (m - n - math.pi / 2.0)
    high = theta_sign * (m + n - math.pi / 2.0)
    return (high, low) if leg in ("RF", "RH") else (low, high)


def main() -> int:
    args = parse_args()
    theta1, theta2 = coaxial_theta(args.leg, args.x_mm, args.z_mm)
    theta2_first = config_value(f"{args.leg}_RIGHT_MOTOR_FIRST") != 0.0
    if theta2_first:
        motor_theta1 = args.motor2
        motor_theta2 = args.motor1
        order = "motor1=theta2, motor2=theta1"
    else:
        motor_theta1 = args.motor1
        motor_theta2 = args.motor2
        order = "motor1=theta1, motor2=theta2"

    sign1 = config_value(f"SIGN_{args.leg}_LEFT")
    sign2 = config_value(f"SIGN_{args.leg}_RIGHT")
    zero1 = theta1 - motor_theta1 / sign1
    zero2 = theta2 - motor_theta2 / sign2

    print(f"{args.leg}: {order}")
    print(f"pose: x={args.x_mm:.3f} mm, z_down={args.z_mm:.3f} mm")
    print(f"theta1={theta1:.6f} rad ({math.degrees(theta1):.3f} deg)")
    print(f"theta2={theta2:.6f} rad ({math.degrees(theta2):.3f} deg)")
    print(f"SIMPLE_ZERO_{args.leg}_LEFT_RAD  = {zero1:.6f}f")
    print(f"SIMPLE_ZERO_{args.leg}_RIGHT_RAD = {zero2:.6f}f")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
