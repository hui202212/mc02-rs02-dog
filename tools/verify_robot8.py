#!/usr/bin/env python3
"""Offline checks using the firmware's robot_config.h values."""

from __future__ import annotations

import math
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "Control" / "robot_config.h"
DEFINE = re.compile(
    r"^#define\s+SIMPLE_([A-Z0-9_]+)\s+"
    r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)[fFuU]*"
    r"\s*(?://.*)?$"
)


def load_config() -> dict[str, float]:
    values: dict[str, float] = {}
    for line in CONFIG_PATH.read_text(encoding="utf-8").splitlines():
        match = DEFINE.match(line.strip())
        if match:
            values[match.group(1)] = float(match.group(2))
    return values


C = load_config()
LEGS = range(int(C["LEG_COUNT"]))
TROT_OFFSET = (0.0, 0.5, 0.5, 0.0)
CRAWL_OFFSET = (0.0, 0.5, 0.25, 0.75)
X_TRIM = (C["RF_X_TRIM_MM"], C["LF_X_TRIM_MM"],
          C["RH_X_TRIM_MM"], C["LH_X_TRIM_MM"])
Z_TRIM = (C["RF_Z_TRIM_MM"], C["LF_Z_TRIM_MM"],
          C["RH_Z_TRIM_MM"], C["LH_Z_TRIM_MM"])
LIFT_EXTRA = (C["RF_LIFT_EXTRA_MM"], C["LF_LIFT_EXTRA_MM"],
              C["RH_LIFT_EXTRA_MM"], C["LH_LIFT_EXTRA_MM"])
MOTOR_POSITION_LIMIT_RAD = 12.57
MOTOR_SPEED_LIMIT_RAD_S = 44.0


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def cycloid(u: float) -> float:
    u = min(1.0, max(0.0, u))
    return u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)


def smoothstep5(u: float) -> float:
    """Match the firmware pose interpolator used by every jump stage."""
    u = min(1.0, max(0.0, u))
    return u**3 * (10.0 + u * (-15.0 + 6.0 * u))


def foot_curve(phase: float, swing_ratio: float, step: float, height: float,
               base_z_down: float) -> tuple[float, float]:
    phase %= 1.0
    start_x = C["STAND_X_MM"] - 0.5 * step
    end_x = C["STAND_X_MM"] + 0.5 * step
    if phase < swing_ratio:
        u = phase / swing_ratio
        if C["ARTICLE_TRAJECTORY_ENABLE"] == 1.0:
            lift = 0.5 * (1.0 - math.cos(2.0 * math.pi * u))
        else:
            if u < 0.25:
                lift = 0.5 * (1.0 - math.cos(math.pi * u / 0.25))
            elif u <= 0.75:
                lift = 1.0
            else:
                lift = 0.5 * (1.0 + math.cos(math.pi * (u - 0.75) / 0.25))
        return (
            start_x + (end_x - start_x) * cycloid(u),
            base_z_down - height * lift,
        )
    u = (phase - swing_ratio) / (1.0 - swing_ratio)
    progress = u if C["ARTICLE_TRAJECTORY_ENABLE"] == 1.0 else cycloid(u)
    return (
        end_x + (start_x - end_x) * progress,
        base_z_down,
    )


def approach(current: float, target: float, max_delta: float) -> float:
    return current + min(max(target - current, -max_delta), max_delta)


def ik_angles(leg: int, foot_x: float, z_down: float) -> tuple[float, float]:
    """Return legacy theta1/theta2 for the coaxial five-bar."""
    active = C["ACTIVE_LENGTH_MM"]
    passive = C["PASSIVE_LENGTH_MM"]
    require(z_down >= C["MIN_Z_DOWN_MM"], "foot is above the allowed motor line")
    radius = math.hypot(foot_x, z_down)
    minimum = abs(passive - active) + C["WORKSPACE_MARGIN_MM"]
    maximum = passive + active - C["WORKSPACE_MARGIN_MM"]
    require(minimum <= radius <= maximum, "foot target is outside workspace")
    cosine = (radius**2 + active**2 - passive**2) / (2.0 * active * radius)
    require(-1.0001 <= cosine <= 1.0001, "invalid inverse-kinematics cosine")
    x_sign = C["COAXIAL_X_SIGN"]
    require(x_sign in (-1.0, 1.0), "COAXIAL_X_SIGN must be +1 or -1")
    n = x_sign * math.asin(max(-1.0, min(1.0, foot_x / radius)))
    m = math.acos(max(-1.0, min(1.0, cosine)))
    theta_sign = C["COAXIAL_THETA_SIGN"]
    require(theta_sign in (-1.0, 1.0), "COAXIAL_THETA_SIGN must be +1 or -1")
    low = theta_sign * (m - n - math.pi / 2.0)
    high = theta_sign * (m + n - math.pi / 2.0)
    return (high, low) if leg in (0, 2) else (low, high)


def check_boundaries(ratio: float, step: float, height: float,
                     base_z_down: float) -> None:
    start = (C["STAND_X_MM"] - 0.5 * step, base_z_down)
    end = (C["STAND_X_MM"] + 0.5 * step, base_z_down)
    curve = lambda phase: foot_curve(phase, ratio, step, height, base_z_down)
    require(curve(0.0) == start, "curve start is not exact")
    require(math.dist(curve(ratio), end) < 1e-9,
            "touchdown is discontinuous")
    require(math.dist(curve(ratio - 1e-6), curve(ratio + 1e-6)) < 1e-3,
            "curve jumps at touchdown")
    require(math.dist(curve(1.0 - 1e-6), curve(1e-6)) < 1e-3,
            "curve jumps at phase wrap")
    midpoint = curve(0.5 * ratio)
    require(abs(midpoint[1] - (base_z_down - height)) < 1e-9,
            "swing midpoint does not reach configured lift height")


def check_four_beat_schedule(ratio: float, name: str) -> None:
    """A four-beat gait must never command two swing legs at once."""
    require(ratio <= 0.25,
            f"{name}: four-beat swing ratio must be at most 0.25")
    swung = set()
    for sample in range(1000):
        phase = sample / 1000.0
        swinging = [leg for leg in LEGS
                    if (phase + CRAWL_OFFSET[leg]) % 1.0 < ratio]
        require(len(swinging) <= 1,
                f"{name}: four-beat schedule lifts more than one leg")
        swung.update(swinging)
    require(swung == set(LEGS),
            f"{name}: four-beat schedule does not swing every leg")


def check_trot_schedule() -> None:
    """Normal walking may lift only one diagonal pair at a time."""
    ratio = C["WALK_SWING_RATIO"]
    require(ratio < 0.5, "trot swing ratio must leave double-support time")
    allowed_pairs = ({0, 3}, {1, 2})
    swung = set()
    for sample in range(1000):
        phase = sample / 1000.0
        swinging = {leg for leg in LEGS
                    if (phase + TROT_OFFSET[leg]) % 1.0 < ratio}
        require(not swinging or swinging in allowed_pairs,
                "trot schedule does not lift a diagonal pair")
        swung.update(swinging)
    require(swung == set(LEGS), "trot schedule does not swing every leg")


def check_fast_crawl_schedule() -> None:
    """Fast low-body gait must keep the same diagonal pairing as trot."""
    ratio = C["FAST_CRAWL_SWING_RATIO"]
    require(ratio < 0.5,
            "fast crawl swing ratio must leave double-support time")
    allowed_pairs = ({0, 3}, {1, 2})
    swung = set()
    for sample in range(1000):
        phase = sample / 1000.0
        swinging = {leg for leg in LEGS
                    if (phase + TROT_OFFSET[leg]) % 1.0 < ratio}
        require(not swinging or swinging in allowed_pairs,
                "fast crawl does not lift a diagonal pair")
        swung.update(swinging)
    require(swung == set(LEGS), "fast crawl does not swing every leg")


def check_transit_schedule() -> None:
    """The dedicated L1 gait must retain diagonal pairing and support time."""
    ratio = C["TRANSIT_SWING_RATIO"]
    require(ratio < 0.5,
            "L1 transit swing ratio must leave double-support time")
    allowed_pairs = ({0, 3}, {1, 2})
    swung = set()
    for sample in range(1000):
        phase = sample / 1000.0
        swinging = {leg for leg in LEGS
                    if (phase + TROT_OFFSET[leg]) % 1.0 < ratio}
        require(not swinging or swinging in allowed_pairs,
                "L1 transit does not lift a diagonal pair")
        swung.update(swinging)
    require(swung == set(LEGS), "L1 transit does not swing every leg")


def check_motor_config() -> None:
    """Catch duplicate CAN mappings and invalid direction/order values."""
    used: set[tuple[int, int]] = set()
    for leg in ("RF", "LF", "RH", "LH"):
        bus = int(C[f"{leg}_BUS"])
        require(bus in (0, 1), f"{leg}: BUS must be 0 or 1")
        for motor in (1, 2):
            index = int(C[f"{leg}_MOTOR{motor}_INDEX"])
            require(0 <= index <= 3, f"{leg}: motor index must be 0..3")
            require((bus, index) not in used,
                    f"{leg}: duplicate CAN mapping bus={bus}, index={index}")
            used.add((bus, index))
        require(C[f"{leg}_RIGHT_MOTOR_FIRST"] in (0.0, 1.0),
                f"{leg}: RIGHT_MOTOR_FIRST must be 0 or 1")
        for side in ("LEFT", "RIGHT"):
            sign = C[f"SIGN_{leg}_{side}"]
            require(sign in (-1.0, 1.0),
                    f"{leg} {side}: sign must be +1 or -1")
            zero = C[f"ZERO_{leg}_{side}_RAD"]
            require(math.isfinite(zero), f"{leg} {side}: zero is not finite")
            # Coaxial IK keeps theta in [-pi, pi]. Check the complete possible
            # theta range, not only the currently sampled gait trajectories.
            require(abs(zero) + math.pi <= MOTOR_POSITION_LIMIT_RAD,
                    f"{leg} {side}: zero can exceed the motor position range")
            motor_extremes = (sign * (-math.pi - zero),
                              sign * (math.pi - zero))
            require(min(motor_extremes) >= C["POSITION_MIN_RAD"] and
                    max(motor_extremes) <= C["POSITION_MAX_RAD"],
                    f"{leg} {side}: software position limit rejects valid IK")
        expected_sign = -1.0 if leg in ("RF", "RH") else 1.0
        require(C[f"SIGN_{leg}_LEFT"] == expected_sign and
                C[f"SIGN_{leg}_RIGHT"] == expected_sign,
                f"{leg}: coaxial legacy mapping requires equal motor signs")
    require(len(used) == 8, "eight motors must have unique CAN mappings")
    require(C["ARTICLE_TRAJECTORY_ENABLE"] in (0.0, 1.0),
            "ARTICLE_TRAJECTORY_ENABLE must be 0 or 1")
    require(C["FAST_CRAWL_ENABLE"] in (0.0, 1.0),
            "FAST_CRAWL_ENABLE must be 0 or 1")
    require(C["OBSTACLE_MODE_ENABLE"] in (0.0, 1.0),
            "OBSTACLE_MODE_ENABLE must be 0 or 1")
    require(0.0 <= C["CROSS_SLOPE_FIXED_COMP_MM"]
            <= C["CROSS_SLOPE_COMP_MAX_MM"],
            "cross-slope fixed compensation exceeds its limit")
    require(C["IMU_BALANCE_ENABLE"] in (0.0, 1.0),
            "IMU_BALANCE_ENABLE must be 0 or 1")
    require(C["IMU_ROLL_SIGN"] in (-1.0, 1.0),
            "IMU_ROLL_SIGN must be +1 or -1")
    require(C["JUMP_ENABLE"] in (0.0, 1.0),
            "JUMP_ENABLE must be 0 or 1")
    require(C["WALK_USE_LOW_POSE"] in (0.0, 1.0),
            "WALK_USE_LOW_POSE must be 0 or 1")
    require(C["TURN_USE_LOW_POSE"] in (0.0, 1.0),
            "TURN_USE_LOW_POSE must be 0 or 1")
    require(0.0 < C["TRANSIT_COMBINED_TURN_SCALE"] <= 1.0,
            "TRANSIT_COMBINED_TURN_SCALE must be in (0, 1]")
    require(C["COMMAND_SLEW_S"] > 0.0,
            "COMMAND_SLEW_S must be positive")
    require(-0.2 <= C["TRANSIT_FORWARD_YAW_TRIM"] <= 0.2,
            "TRANSIT_FORWARD_YAW_TRIM is too large")
    require(-0.2 <= C["TRANSIT_REVERSE_YAW_TRIM"] <= 0.2,
            "TRANSIT_REVERSE_YAW_TRIM is too large")
    require(all(abs(trim) <= C["X_TRIM_MAX_ABS_MM"] for trim in X_TRIM),
            "a per-leg X trim exceeds X_TRIM_MAX_ABS_MM")
    require(all(abs(trim) <= C["Z_TRIM_MAX_ABS_MM"] for trim in Z_TRIM),
            "a per-leg Z trim exceeds Z_TRIM_MAX_ABS_MM")
    require(0.0 <= C["STAND_SIDE_Z_COMP_MM"] <=
            C["STAND_SIDE_Z_COMP_MAX_MM"],
            "stand side-height compensation exceeds its limit")
    require(all(0.0 <= extra <= C["LIFT_EXTRA_MAX_MM"]
                for extra in LIFT_EXTRA),
            "a per-leg lift extra is negative or exceeds its limit")
    for suffix in ("COMPRESS", "THRUST", "FLIGHT", "LANDING", "ABSORB"):
        require(0.0 <= C[f"JUMP_{suffix}_KP"] <= C["KP_MAX"],
                f"JUMP_{suffix}_KP exceeds the firmware limit")
        require(0.0 <= C[f"JUMP_{suffix}_KD"] <= C["KD_MAX"],
                f"JUMP_{suffix}_KD exceeds the firmware limit")
    require(0.0 <= C["T_DOWN_KP"] <= C["KP_MAX"],
            "T_DOWN_KP exceeds the firmware limit")
    require(0.0 <= C["T_DOWN_KD"] <= C["KD_MAX"],
            "T_DOWN_KD exceeds the firmware limit")
    require(0.0 <= C["RIGHT_STAND_KP"] <= C["KP_MAX"],
            "RIGHT_STAND_KP exceeds the firmware limit")
    require(0.0 <= C["RIGHT_STAND_KD"] <= C["KD_MAX"],
            "RIGHT_STAND_KD exceeds the firmware limit")


def check_jump_sequence(profile: str = "vertical") -> float:
    """Sample one configured obstacle jump in angle space at 1 ms."""
    stand_x = C["STAND_X_MM"]
    compress_x = thrust_x = tuck_x = landing_x = absorb_x = stand_x
    compress_z = C["JUMP_COMPRESS_Z_MM"]
    extend_z = C["JUMP_EXTEND_Z_MM"]
    tuck_z = C["JUMP_TUCK_Z_MM"]
    landing_z = C["JUMP_LANDING_Z_MM"]
    absorb_z = C["JUMP_ABSORB_Z_MM"]
    thrust_s = C["JUMP_THRUST_S"]
    prefixes = {
        "forward": "FORWARD_JUMP",
        "bridge-b": "BRIDGE_B_JUMP",
        "t-up": "T_UP",
        "wall": "WALL_JUMP",
    }
    if profile in prefixes:
        prefix = prefixes[profile]
        compress_x = C[f"{prefix}_COMPRESS_X_MM"]
        thrust_x = C[f"{prefix}_THRUST_X_MM"]
        tuck_x = C[f"{prefix}_TUCK_X_MM"]
        landing_x = C[f"{prefix}_LANDING_X_MM"]
        absorb_x = C[f"{prefix}_ABSORB_X_MM"]
        if profile != "forward":
            compress_z = C[f"{prefix}_COMPRESS_Z_MM"]
            extend_z = C[f"{prefix}_EXTEND_Z_MM"]
            tuck_z = C[f"{prefix}_TUCK_Z_MM"]
            landing_z = C[f"{prefix}_LANDING_Z_MM"]
            absorb_z = C[f"{prefix}_ABSORB_Z_MM"]
            thrust_s = C[f"{prefix}_THRUST_S"]
    else:
        require(profile == "vertical", f"unknown jump profile {profile}")
    stages = (
        ("compress", stand_x, C["STAND_Z_DOWN_MM"],
         compress_x, compress_z,
         C["JUMP_COMPRESS_S"]),
        ("ready", compress_x, compress_z, compress_x, compress_z,
         C["JUMP_READY_HOLD_S"]),
        ("thrust", compress_x, compress_z, thrust_x, extend_z, thrust_s),
        ("tuck", thrust_x, extend_z, tuck_x, tuck_z,
         C["JUMP_TUCK_S"]),
        ("landing", tuck_x, tuck_z, landing_x, landing_z,
         C["JUMP_LANDING_S"]),
        ("absorb", landing_x, landing_z, absorb_x, absorb_z,
         C["JUMP_ABSORB_S"]),
        ("absorb-hold", absorb_x, absorb_z, absorb_x, absorb_z,
         C["JUMP_ABSORB_HOLD_S"]),
        ("recover", absorb_x, absorb_z,
         stand_x, C["STAND_Z_DOWN_MM"],
         C["JUMP_RECOVER_S"]),
    )
    dt = C["CONTROL_DT_S"]
    max_joint_delta = 0.0
    for name, start_x, start_z, end_x, end_z, duration in stages:
        require(duration > 0.0, f"jump {name} duration must be positive")
        samples = max(1, math.ceil(duration / dt))
        for leg in LEGS:
            start = ik_angles(leg, start_x + X_TRIM[leg],
                              start_z + Z_TRIM[leg])
            end = ik_angles(leg, end_x + X_TRIM[leg],
                            end_z + Z_TRIM[leg])
            previous = start
            for sample in range(1, samples + 1):
                progress = smoothstep5(min(sample * dt / duration, 1.0))
                current = tuple(a + (b - a) * progress
                                for a, b in zip(start, end))
                max_joint_delta = max(
                    max_joint_delta,
                    abs(current[0] - previous[0]),
                    abs(current[1] - previous[1]),
                )
                previous = current
    require(C["JUMP_READY_HOLD_S"] <= C["JUMP_READY_TIMEOUT_S"],
            "jump ready timeout is shorter than the minimum hold")
    require(max_joint_delta < C["MAX_IK_STEP_RAD"],
            "jump exceeds the locked-branch angle step")
    require(max_joint_delta / dt <= MOTOR_SPEED_LIMIT_RAD_S,
            "jump target speed exceeds the motor protocol range")
    return max_joint_delta


def check_step_down_sequence() -> float:
    """Sample the staged one-leg-at-a-time T-step descent at 1 ms."""
    stand = [(C["STAND_X_MM"], C["STAND_Z_DOWN_MM"])] * 4

    def shifted() -> list[tuple[float, float]]:
        return [
            (C["T_DOWN_SHIFT_FRONT_X_MM"], C["T_DOWN_SUPPORT_Z_MM"]),
            (C["T_DOWN_SHIFT_FRONT_X_MM"], C["T_DOWN_SUPPORT_Z_MM"]),
            (C["T_DOWN_SHIFT_REAR_X_MM"], C["STAND_Z_DOWN_MM"]),
            (C["T_DOWN_SHIFT_REAR_X_MM"], C["STAND_Z_DOWN_MM"]),
        ]

    targets: list[tuple[str, list[tuple[float, float]], float]] = []
    pose = list(stand)
    pose[0] = (C["T_DOWN_STEP_X_MM"], C["T_DOWN_LIFT_Z_MM"])
    targets.append(("rf-lift", pose, C["T_DOWN_LIFT_S"]))
    pose = list(stand)
    pose[0] = (C["T_DOWN_STEP_X_MM"], C["T_DOWN_REACH_Z_MM"])
    targets.append(("rf-reach", pose, C["T_DOWN_REACH_S"]))
    pose = list(stand)
    pose[0] = (C["T_DOWN_STEP_X_MM"], C["T_DOWN_SUPPORT_Z_MM"])
    pose[1] = (C["T_DOWN_STEP_X_MM"], C["T_DOWN_LIFT_Z_MM"])
    targets.append(("lf-lift", pose, C["T_DOWN_LIFT_S"]))
    pose = list(stand)
    pose[0] = (C["T_DOWN_STEP_X_MM"], C["T_DOWN_SUPPORT_Z_MM"])
    pose[1] = (C["T_DOWN_STEP_X_MM"], C["T_DOWN_REACH_Z_MM"])
    targets.append(("lf-reach", pose, C["T_DOWN_REACH_S"]))
    targets.append(("body-shift", shifted(), C["T_DOWN_SHIFT_S"]))
    pose = shifted()
    pose[2] = (C["T_DOWN_SHIFT_REAR_X_MM"],
               C["T_DOWN_REAR_LIFT_Z_MM"])
    targets.append(("rh-lift", pose, C["T_DOWN_REAR_LIFT_S"]))
    pose = shifted()
    pose[2] = (C["T_DOWN_REAR_STEP_X_MM"],
               C["T_DOWN_REAR_LIFT_Z_MM"])
    targets.append(("rh-swing", pose, C["T_DOWN_REAR_SWING_S"]))
    pose = shifted()
    pose[2] = (C["T_DOWN_REAR_STEP_X_MM"], C["T_DOWN_REACH_Z_MM"])
    targets.append(("rh-reach", pose, C["T_DOWN_REACH_S"]))
    pose = shifted()
    pose[2] = (C["T_DOWN_REAR_STEP_X_MM"], C["T_DOWN_SUPPORT_Z_MM"])
    pose[3] = (C["T_DOWN_SHIFT_REAR_X_MM"],
               C["T_DOWN_REAR_LIFT_Z_MM"])
    targets.append(("lh-lift", pose, C["T_DOWN_REAR_LIFT_S"]))
    pose = shifted()
    pose[2] = (C["T_DOWN_REAR_STEP_X_MM"], C["T_DOWN_SUPPORT_Z_MM"])
    pose[3] = (C["T_DOWN_REAR_STEP_X_MM"],
               C["T_DOWN_REAR_LIFT_Z_MM"])
    targets.append(("lh-swing", pose, C["T_DOWN_REAR_SWING_S"]))
    pose = shifted()
    pose[2] = (C["T_DOWN_REAR_STEP_X_MM"], C["T_DOWN_SUPPORT_Z_MM"])
    pose[3] = (C["T_DOWN_REAR_STEP_X_MM"], C["T_DOWN_REACH_Z_MM"])
    targets.append(("lh-reach", pose, C["T_DOWN_REACH_S"]))
    targets.append(("recover", list(stand), C["T_DOWN_RECOVER_S"]))

    dt = C["CONTROL_DT_S"]
    previous_pose = list(stand)
    max_joint_delta = 0.0
    for name, target_pose, duration in targets:
        require(duration > 0.0, f"step-down {name} duration must be positive")
        samples = max(1, math.ceil(duration / dt))
        for leg in LEGS:
            start = ik_angles(leg, previous_pose[leg][0] + X_TRIM[leg],
                              previous_pose[leg][1] + Z_TRIM[leg])
            end = ik_angles(leg, target_pose[leg][0] + X_TRIM[leg],
                            target_pose[leg][1] + Z_TRIM[leg])
            previous = start
            for sample in range(1, samples + 1):
                progress = smoothstep5(min(sample * dt / duration, 1.0))
                current = tuple(a + (b - a) * progress
                                for a, b in zip(start, end))
                max_joint_delta = max(
                    max_joint_delta,
                    abs(current[0] - previous[0]),
                    abs(current[1] - previous[1]),
                )
                previous = current
        previous_pose = target_pose
    require(max_joint_delta < C["MAX_IK_STEP_RAD"],
            "step-down exceeds the locked-branch angle step")
    require(max_joint_delta / dt <= MOTOR_SPEED_LIMIT_RAD_S,
            "step-down target speed exceeds the motor protocol range")
    return max_joint_delta


def check_step_up_sequence() -> float:
    """Sample the no-flight one-level T-step ascent at 1 ms."""
    stand_z = C["STAND_Z_DOWN_MM"]
    step_x = C["T_UP_STEP_X_MM"]
    prelift_z = C["T_UP_PRELIFT_Z_MM"]
    swing_z = C["T_UP_SWING_Z_MM"]
    support_z = C["T_UP_SUPPORT_Z_MM"]
    stand = [(C["STAND_X_MM"], stand_z)] * 4

    def shifted() -> list[tuple[float, float]]:
        return [
            (C["T_UP_SHIFT_FRONT_X_MM"], support_z),
            (C["T_UP_SHIFT_FRONT_X_MM"], support_z),
            (C["T_UP_SHIFT_REAR_X_MM"], stand_z),
            (C["T_UP_SHIFT_REAR_X_MM"], stand_z),
        ]

    targets: list[tuple[str, list[tuple[float, float]], float]] = []
    pose = list(stand)
    pose[0] = (0.0, prelift_z)
    targets.append(("rf-lift", pose, C["T_UP_LIFT_S"]))
    pose = list(stand)
    pose[0] = (step_x, swing_z)
    targets.append(("rf-swing", pose, C["T_UP_SWING_S"]))
    pose = list(stand)
    pose[0] = (step_x, support_z)
    targets.append(("rf-lower", pose, C["T_UP_LOWER_S"]))
    pose = list(stand)
    pose[0] = (step_x, support_z)
    pose[1] = (0.0, prelift_z)
    targets.append(("lf-lift", pose, C["T_UP_LIFT_S"]))
    pose = list(stand)
    pose[0] = (step_x, support_z)
    pose[1] = (step_x, swing_z)
    targets.append(("lf-swing", pose, C["T_UP_SWING_S"]))
    pose = list(stand)
    pose[0] = (step_x, support_z)
    pose[1] = (step_x, support_z)
    targets.append(("lf-lower", pose, C["T_UP_LOWER_S"]))
    targets.append(("body-shift", shifted(), C["T_UP_SHIFT_S"]))
    pose = shifted()
    pose[2] = (C["T_UP_SHIFT_REAR_X_MM"], prelift_z)
    targets.append(("rh-lift", pose, C["T_UP_LIFT_S"]))
    pose = shifted()
    pose[2] = (C["T_UP_SHIFT_FRONT_X_MM"], swing_z)
    targets.append(("rh-swing", pose, C["T_UP_SWING_S"]))
    pose = shifted()
    pose[2] = (C["T_UP_SHIFT_FRONT_X_MM"], support_z)
    targets.append(("rh-lower", pose, C["T_UP_LOWER_S"]))
    pose = shifted()
    pose[2] = (C["T_UP_SHIFT_FRONT_X_MM"], support_z)
    pose[3] = (C["T_UP_SHIFT_REAR_X_MM"], prelift_z)
    targets.append(("lh-lift", pose, C["T_UP_LIFT_S"]))
    pose = shifted()
    pose[2] = (C["T_UP_SHIFT_FRONT_X_MM"], support_z)
    pose[3] = (C["T_UP_SHIFT_FRONT_X_MM"], swing_z)
    targets.append(("lh-swing", pose, C["T_UP_SWING_S"]))
    pose = shifted()
    pose[2] = (C["T_UP_SHIFT_FRONT_X_MM"], support_z)
    pose[3] = (C["T_UP_SHIFT_FRONT_X_MM"], support_z)
    targets.append(("lh-lower", pose, C["T_UP_LOWER_S"]))
    targets.append(("recover", list(stand), C["T_UP_RECOVER_S"]))

    dt = C["CONTROL_DT_S"]
    previous_pose = list(stand)
    max_joint_delta = 0.0
    for name, target_pose, duration in targets:
        require(duration > 0.0, f"step-up {name} duration must be positive")
        samples = max(1, math.ceil(duration / dt))
        for leg in LEGS:
            start = ik_angles(leg, previous_pose[leg][0] + X_TRIM[leg],
                              previous_pose[leg][1] + Z_TRIM[leg])
            end = ik_angles(leg, target_pose[leg][0] + X_TRIM[leg],
                            target_pose[leg][1] + Z_TRIM[leg])
            previous = start
            for sample in range(1, samples + 1):
                progress = smoothstep5(min(sample * dt / duration, 1.0))
                current = tuple(a + (b - a) * progress
                                for a, b in zip(start, end))
                max_joint_delta = max(
                    max_joint_delta,
                    abs(current[0] - previous[0]),
                    abs(current[1] - previous[1]),
                )
                previous = current
        previous_pose = target_pose
    require(max_joint_delta < C["MAX_IK_STEP_RAD"],
            "step-up exceeds the locked-branch angle step")
    require(max_joint_delta / dt <= MOTOR_SPEED_LIMIT_RAD_S,
            "step-up target speed exceeds the motor protocol range")
    return max_joint_delta


def check_dance_sequence() -> float:
    """Sample the demo dance: one 20 mm leg lift with three-leg support."""
    stand = [(C["STAND_X_MM"], C["STAND_Z_DOWN_MM"])] * 4
    order = (0, 1, 2, 3, 3, 2, 1, 0)
    targets: list[tuple[str, list[tuple[float, float]], float]] = []
    for stage, leg in enumerate(order):
        pose = list(stand)
        pose[leg] = (C["STAND_X_MM"], C["DANCE_LIFT_Z_MM"])
        targets.append((f"stage-{stage}", pose, C["DANCE_LIFT_S"]))
    # enter_state(STAND) performs the final smooth recovery after stage 7.
    targets.append(("recover", list(stand), C["STAND_TRANSITION_S"]))

    dt = C["CONTROL_DT_S"]
    previous_pose = list(stand)
    max_joint_delta = 0.0
    for name, target_pose, duration in targets:
        require(duration > 0.0, f"dance {name} duration must be positive")
        samples = max(1, math.ceil(duration / dt))
        for leg in LEGS:
            start = ik_angles(leg, previous_pose[leg][0] + X_TRIM[leg],
                              previous_pose[leg][1] + Z_TRIM[leg])
            end = ik_angles(leg, target_pose[leg][0] + X_TRIM[leg],
                            target_pose[leg][1] + Z_TRIM[leg])
            previous = start
            for sample in range(1, samples + 1):
                progress = smoothstep5(min(sample * dt / duration, 1.0))
                current = tuple(a + (b - a) * progress
                                for a, b in zip(start, end))
                max_joint_delta = max(
                    max_joint_delta,
                    abs(current[0] - previous[0]),
                    abs(current[1] - previous[1]),
                )
                previous = current
        previous_pose = target_pose
    require(max_joint_delta < C["MAX_IK_STEP_RAD"],
            "dance exceeds the locked-branch angle step")
    require(max_joint_delta / dt <= MOTOR_SPEED_LIMIT_RAD_S,
            "dance target speed exceeds the motor protocol range")
    return max_joint_delta


def check_bridge_b_gap_sequence() -> float:
    """Sample the no-flight 150 mm gap-crossing sequence at 1 ms."""
    base_z = C["BRIDGE_B_Z_DOWN_MM"]
    lift_z = C["BRIDGE_B_GAP_LIFT_Z_MM"]
    step = C["BRIDGE_B_GAP_STEP_X_MM"]
    half = 0.5 * step
    base = [(C["STAND_X_MM"], base_z)] * 4

    def shifted() -> list[tuple[float, float]]:
        return [(half, base_z), (half, base_z),
                (-half, base_z), (-half, base_z)]

    targets: list[tuple[str, list[tuple[float, float]], float]] = []
    pose = list(base)
    pose[0] = (0.0, lift_z)
    targets.append(("rf-lift", pose, C["BRIDGE_B_GAP_LIFT_S"]))
    pose = list(base)
    pose[0] = (step, lift_z)
    targets.append(("rf-swing", pose, C["BRIDGE_B_GAP_SWING_S"]))
    pose = list(base)
    pose[0] = (step, base_z)
    targets.append(("rf-lower", pose, C["BRIDGE_B_GAP_LOWER_S"]))
    pose = list(base)
    pose[0] = (step, base_z)
    pose[1] = (0.0, lift_z)
    targets.append(("lf-lift", pose, C["BRIDGE_B_GAP_LIFT_S"]))
    pose = list(base)
    pose[0] = (step, base_z)
    pose[1] = (step, lift_z)
    targets.append(("lf-swing", pose, C["BRIDGE_B_GAP_SWING_S"]))
    pose = list(base)
    pose[0] = (step, base_z)
    pose[1] = (step, base_z)
    targets.append(("lf-lower", pose, C["BRIDGE_B_GAP_LOWER_S"]))
    targets.append(("body-shift-1", shifted(), C["BRIDGE_B_GAP_SHIFT_S"]))
    pose = shifted()
    pose[2] = (-half, lift_z)
    targets.append(("rh-lift", pose, C["BRIDGE_B_GAP_LIFT_S"]))
    pose = shifted()
    pose[2] = (half, lift_z)
    targets.append(("rh-swing", pose, C["BRIDGE_B_GAP_SWING_S"]))
    pose = shifted()
    pose[2] = (half, base_z)
    targets.append(("rh-lower", pose, C["BRIDGE_B_GAP_LOWER_S"]))
    pose = shifted()
    pose[2] = (half, base_z)
    pose[3] = (-half, lift_z)
    targets.append(("lh-lift", pose, C["BRIDGE_B_GAP_LIFT_S"]))
    pose = shifted()
    pose[2] = (half, base_z)
    pose[3] = (half, lift_z)
    targets.append(("lh-swing", pose, C["BRIDGE_B_GAP_SWING_S"]))
    pose = shifted()
    pose[2] = (half, base_z)
    pose[3] = (half, base_z)
    targets.append(("lh-lower", pose, C["BRIDGE_B_GAP_LOWER_S"]))
    targets.append(("body-shift-2", list(base),
                    C["BRIDGE_B_GAP_RECOVER_S"]))

    require(step >= 170.0,
            "bridge-b gap step does not clear the 150 mm rule gap")
    dt = C["CONTROL_DT_S"]
    previous_pose = list(base)
    max_joint_delta = 0.0
    for name, target_pose, duration in targets:
        require(duration > 0.0,
                f"bridge-b gap {name} duration must be positive")
        samples = max(1, math.ceil(duration / dt))
        for leg in LEGS:
            start = ik_angles(leg, previous_pose[leg][0] + X_TRIM[leg],
                              previous_pose[leg][1] + Z_TRIM[leg])
            end = ik_angles(leg, target_pose[leg][0] + X_TRIM[leg],
                            target_pose[leg][1] + Z_TRIM[leg])
            previous = start
            for sample in range(1, samples + 1):
                progress = smoothstep5(min(sample * dt / duration, 1.0))
                current = tuple(a + (b - a) * progress
                                for a, b in zip(start, end))
                max_joint_delta = max(
                    max_joint_delta,
                    abs(current[0] - previous[0]),
                    abs(current[1] - previous[1]),
                )
                previous = current
        previous_pose = target_pose
    require(max_joint_delta < C["MAX_IK_STEP_RAD"],
            "bridge-b gap step exceeds the locked-branch angle step")
    require(max_joint_delta / dt <= MOTOR_SPEED_LIMIT_RAD_S,
            "bridge-b gap step exceeds the motor protocol speed")
    return max_joint_delta


def check_speed_mapping() -> None:
    """Stride divided by period must be proportional to stick command."""
    modes = (
        ("WALK", C["WALK_STEP_MAX_MM"]),
        ("CRAWL", C["CRAWL_STEP_MAX_MM"]),
        ("FAST_CRAWL", C["CRAWL_STEP_MAX_MM"]),
        ("TRANSIT", C["TRANSIT_STEP_MAX_MM"]),
        ("FAST_CRAWL", C["PIT_STEP_MAX_MM"]),
        ("FAST_CRAWL", C["LIMIT_BAR_STEP_MAX_MM"]),
        ("FAST_CRAWL", C["CROSS_SLOPE_STEP_MAX_MM"]),
        ("BRIDGE_A", C["BRIDGE_A_STEP_MAX_MM"]),
        ("BRIDGE_B", C["BRIDGE_B_STEP_MAX_MM"]),
    )
    for prefix, step_max in modes:
        period_min = C[f"{prefix}_PERIOD_MIN_S"]
        period_max = C[f"{prefix}_PERIOD_MAX_S"]
        require(0.0 < period_min <= period_max,
                f"{prefix}: invalid period range")
        expected_gain = step_max / period_min
        for command in (0.25, 0.5, 1.0):
            period = period_max - (period_max - period_min) * command
            step = command * step_max * period / period_min
            require(abs(step / period - command * expected_gain) < 1e-9,
                    f"{prefix}: speed-to-stride mapping is not linear")


def check_gait(command: float, turn: bool, crawl: bool = False,
               fast_crawl: bool = False,
               combined: bool = False,
               obstacle: str | None = None,
               combined_turn: float | None = None) -> tuple[float, float]:
    phase = 0.0
    ramp = 0.0
    filtered_forward = 0.0
    filtered_turn = 0.0
    previous_foot: list[tuple[float, float] | None] = [None] * 4
    previous_joint: list[tuple[float, float] | None] = [None] * 4
    max_foot_delta = 0.0
    max_joint_delta = 0.0
    dt = C["CONTROL_DT_S"]

    for _ in range(4000):
        max_command_delta = dt / C["COMMAND_SLEW_S"]
        forward_target = command if (combined or not turn) else 0.0
        turn_target = ((command if combined_turn is None else combined_turn)
                       if (combined or turn) else 0.0)
        if obstacle == "transit":
            manual_turn = turn_target
            if forward_target != 0.0 and turn_target != 0.0:
                manual_turn *= C["TRANSIT_COMBINED_TURN_SCALE"]
            yaw_trim = abs(forward_target) * (
                C["TRANSIT_FORWARD_YAW_TRIM"] if forward_target >= 0.0
                else C["TRANSIT_REVERSE_YAW_TRIM"]
            )
            turn_target = manual_turn + yaw_trim
            turn_target = min(1.0, max(-1.0, turn_target))
        elif obstacle == "bridge-a":
            turn_target += forward_target * C["BRIDGE_A_YAW_TRIM"]
            turn_target = min(1.0, max(-1.0, turn_target))
        elif obstacle == "bridge-b":
            turn_target += forward_target * C["BRIDGE_B_YAW_TRIM"]
            turn_target = min(1.0, max(-1.0, turn_target))
        filtered_forward = approach(filtered_forward,
                                    forward_target,
                                    max_command_delta)
        filtered_turn = approach(filtered_turn,
                                 turn_target,
                                 max_command_delta)
        magnitude = max(abs(filtered_forward), abs(filtered_turn))
        if obstacle == "bridge-a":
            prefix = "BRIDGE_A"
        elif obstacle == "bridge-b":
            prefix = "BRIDGE_B"
        elif obstacle == "transit":
            prefix = "TRANSIT"
        elif fast_crawl:
            prefix = "FAST_CRAWL"
        elif crawl:
            prefix = "CRAWL"
        else:
            prefix = "WALK"
        period = C[f"{prefix}_PERIOD_MAX_S"] - (
            C[f"{prefix}_PERIOD_MAX_S"] - C[f"{prefix}_PERIOD_MIN_S"]
        ) * magnitude
        ramp = min(1.0, ramp + dt / C["GAIT_RAMP_S"])
        phase = (phase + dt / period) % 1.0

        for leg in LEGS:
            side = 1.0 if leg in (0, 2) else -1.0
            if crawl:
                step_max = C["CRAWL_STEP_MAX_MM"]
                turn_max = C["CRAWL_TURN_STEP_MAX_MM"]
                height = C["CRAWL_STEP_HEIGHT_MM"]
                base_z = C["CRAWL_Z_DOWN_MM"]
                terrain_prefix = {
                    "transit": "TRANSIT",
                    "pit": "PIT",
                    "limit-bar": "LIMIT_BAR",
                    "bridge-a": "BRIDGE_A",
                    "bridge-b": "BRIDGE_B",
                }.get(obstacle)
                if terrain_prefix:
                    step_max = C[f"{terrain_prefix}_STEP_MAX_MM"]
                    turn_max = C[f"{terrain_prefix}_TURN_STEP_MAX_MM"]
                    height = C[f"{terrain_prefix}_STEP_HEIGHT_MM"]
                    base_z = C[f"{terrain_prefix}_Z_DOWN_MM"]
                elif obstacle == "cross-slope":
                    step_max = C["CROSS_SLOPE_STEP_MAX_MM"]
                    turn_max = C["CROSS_SLOPE_TURN_STEP_MAX_MM"]
                    height = C["CROSS_SLOPE_STEP_HEIGHT_MM"]
                    base_z = C["CROSS_SLOPE_Z_DOWN_MM"]
                step = filtered_forward * step_max
                step += filtered_turn * side * turn_max
                step *= period / C[f"{prefix}_PERIOD_MIN_S"] * ramp
                if obstacle == "bridge-a":
                    ratio = C["BRIDGE_A_SWING_RATIO"]
                elif obstacle == "bridge-b":
                    ratio = C["BRIDGE_B_SWING_RATIO"]
                elif obstacle == "transit":
                    ratio = C["TRANSIT_SWING_RATIO"]
                elif fast_crawl:
                    ratio = C["FAST_CRAWL_SWING_RATIO"]
                else:
                    ratio = C["CRAWL_SWING_RATIO"]
                offset = TROT_OFFSET[leg] if fast_crawl else CRAWL_OFFSET[leg]
                if obstacle == "cross-slope":
                    compensation = C["CROSS_SLOPE_FIXED_COMP_MM"]
                    base_z += -compensation if leg in (0, 2) else compensation
            else:
                step = (filtered_turn * C["TURN_STEP_MAX_MM"] * side
                        if turn else filtered_forward * C["WALK_STEP_MAX_MM"])
                step *= period / C["WALK_PERIOD_MIN_S"] * ramp
                height = C["STEP_HEIGHT_MM"]
                ratio = C["WALK_SWING_RATIO"]
                offset = TROT_OFFSET[leg]
                base_z = C["STAND_Z_DOWN_MM"]
                height += LIFT_EXTRA[leg]
            foot = foot_curve((phase + offset) % 1.0, ratio, step,
                              height, base_z)
            joint = ik_angles(leg, foot[0] + X_TRIM[leg],
                              foot[1] + Z_TRIM[leg])

            if previous_foot[leg] is not None:
                max_foot_delta = max(max_foot_delta,
                                     math.dist(previous_foot[leg], foot))
            previous_foot[leg] = foot
            previous = previous_joint[leg]
            if previous is not None:
                max_joint_delta = max(max_joint_delta,
                                      abs(joint[0] - previous[0]),
                                      abs(joint[1] - previous[1]))
            previous_joint[leg] = joint

    require(max_joint_delta < C["MAX_IK_STEP_RAD"],
            "gait exceeds the locked-branch angle step")
    require(max_joint_delta / dt <= MOTOR_SPEED_LIMIT_RAD_S,
            "gait target speed exceeds the motor protocol range")
    return max_foot_delta, max_joint_delta


def main() -> int:
    try:
        check_motor_config()
        for leg in LEGS:
            stand_side_comp = (C["STAND_SIDE_Z_COMP_MM"]
                               if leg in (0, 2)
                               else -C["STAND_SIDE_Z_COMP_MM"])
            ik_angles(leg, C["STAND_X_MM"] + X_TRIM[leg],
                      C["STAND_Z_DOWN_MM"] + Z_TRIM[leg] +
                      stand_side_comp)
            ik_angles(leg, C["DOWN_X_MM"] + X_TRIM[leg],
                      C["DOWN_Z_DOWN_MM"] + Z_TRIM[leg])
        centered = ik_angles(0, 0.0, C["STAND_Z_DOWN_MM"])
        require(abs(centered[0] - centered[1]) < 1e-9,
                "centered coaxial pose must have equal theta values")
        check_boundaries(C["WALK_SWING_RATIO"], C["WALK_STEP_MAX_MM"],
                         C["STEP_HEIGHT_MM"], C["STAND_Z_DOWN_MM"])
        check_boundaries(C["CRAWL_SWING_RATIO"], C["CRAWL_STEP_MAX_MM"],
                         C["CRAWL_STEP_HEIGHT_MM"], C["CRAWL_Z_DOWN_MM"])
        check_boundaries(C["TRANSIT_SWING_RATIO"],
                         C["TRANSIT_STEP_MAX_MM"],
                         C["TRANSIT_STEP_HEIGHT_MM"], C["TRANSIT_Z_DOWN_MM"])
        check_boundaries(C["FAST_CRAWL_SWING_RATIO"],
                         C["PIT_STEP_MAX_MM"],
                         C["PIT_STEP_HEIGHT_MM"], C["PIT_Z_DOWN_MM"])
        check_boundaries(C["FAST_CRAWL_SWING_RATIO"],
                         C["LIMIT_BAR_STEP_MAX_MM"],
                         C["LIMIT_BAR_STEP_HEIGHT_MM"], C["LIMIT_BAR_Z_DOWN_MM"])
        check_boundaries(C["FAST_CRAWL_SWING_RATIO"],
                         C["CROSS_SLOPE_STEP_MAX_MM"],
                         C["CROSS_SLOPE_STEP_HEIGHT_MM"],
                         C["CROSS_SLOPE_Z_DOWN_MM"] -
                         C["CROSS_SLOPE_FIXED_COMP_MM"])
        check_boundaries(C["FAST_CRAWL_SWING_RATIO"],
                         C["CROSS_SLOPE_STEP_MAX_MM"],
                         C["CROSS_SLOPE_STEP_HEIGHT_MM"],
                         C["CROSS_SLOPE_Z_DOWN_MM"] +
                         C["CROSS_SLOPE_FIXED_COMP_MM"])
        check_boundaries(C["BRIDGE_A_SWING_RATIO"],
                         C["BRIDGE_A_STEP_MAX_MM"],
                         C["BRIDGE_A_STEP_HEIGHT_MM"],
                         C["BRIDGE_A_Z_DOWN_MM"])
        check_boundaries(C["BRIDGE_B_SWING_RATIO"],
                         C["BRIDGE_B_STEP_MAX_MM"],
                         C["BRIDGE_B_STEP_HEIGHT_MM"],
                         C["BRIDGE_B_Z_DOWN_MM"])
        check_trot_schedule()
        check_four_beat_schedule(C["CRAWL_SWING_RATIO"], "crawl")
        check_four_beat_schedule(C["BRIDGE_A_SWING_RATIO"], "bridge-a")
        check_four_beat_schedule(C["BRIDGE_B_SWING_RATIO"], "bridge-b")
        check_fast_crawl_schedule()
        check_transit_schedule()
        check_speed_mapping()
        jump_results = [("dance", check_dance_sequence()),
                        ("t-up-step", check_step_up_sequence()),
                        ("bridge-b-forward", check_jump_sequence("bridge-b"))]
        jump_results.append(("t-down-step", check_step_down_sequence()))
        results = [("turn" if turn else "walk", command,
                    *check_gait(command, turn))
                   for turn in (False, True) for command in (-1.0, 1.0)]
        results += [("crawl", command, *check_gait(command, False, True))
                    for command in (-1.0, 1.0)]
        results += [("fast-crawl", command,
                     *check_gait(command, False, True, True))
                    for command in (-1.0, 1.0)]
        results += [("fast-crawl-turn", command,
                     *check_gait(command, True, True, True))
                    for command in (-1.0, 1.0)]
        results += [("fast-crawl-combined", command,
                     *check_gait(command, False, True, True, True))
                    for command in (-1.0, 1.0)]
        results += [("cross-slope-forward", command,
                     *check_gait(command, False, True, True,
                                  obstacle="cross-slope"))
                    for command in (-1.0, 1.0)]
        results += [("cross-slope-turn", command,
                     *check_gait(command, True, True, True,
                                 obstacle="cross-slope"))
                    for command in (-1.0, 1.0)]
        results += [("cross-slope-combined", command,
                     *check_gait(command, False, True, True, True,
                                 obstacle="cross-slope"))
                    for command in (-1.0, 1.0)]
        results += [("bridge-a", command,
                     *check_gait(command, False, True, False,
                                  obstacle="bridge-a"))
                    for command in (-1.0, 1.0)]
        results += [("bridge-b", command,
                     *check_gait(command, False, True, False,
                                  obstacle="bridge-b"))
                    for command in (-1.0, 1.0)]
        results += [("transit", command,
                     *check_gait(command, False, True, True,
                                 obstacle="transit"))
                    for command in (-1.0, 1.0)]
        results += [(f"transit-combined-turn={turn_command:+.0f}",
                     forward_command,
                     *check_gait(forward_command, False, True, True, True,
                                 obstacle="transit",
                                 combined_turn=turn_command))
                    for forward_command in (-1.0, 1.0)
                    for turn_command in (-1.0, 1.0)]

        print(f"PASS: loaded firmware config from {CONFIG_PATH.relative_to(ROOT)}")
        print("PASS: eight unique CAN mappings, orders and motor signs")
        print("PASS: all software zeros fit the full motor position range")
        print("PASS: coaxial stand/down targets and sampled gait workspace")
        print("PASS: diagonal trot, four-beat crawl and fast-crawl schedules")
        print("PASS: per-leg X trims and linear speed-to-stride mapping")
        print("PASS: lift, touchdown and phase-wrap continuity")
        for name, joint_delta in jump_results:
            action = ("demo dance" if name == "dance" else
                      ("slow descent" if name == "t-down-step" else
                      ("gap step" if name == "bridge-b-gap" else
                       ("step ascent" if name == "t-up-step" else "jump"))))
            print(f"PASS: {name} {action}, max 1ms joint delta="
                  f"{joint_delta:.6f} rad")
        for name, command, foot_delta, joint_delta in results:
            print(f"PASS: {name} cmd={command:+.0f}, "
                  f"max 1ms foot delta={foot_delta:.4f} mm, "
                  f"joint delta={joint_delta:.6f} rad")
        if C["ROBOT_CALIBRATED"] == 0.0:
            print("SAFE: SIMPLE_ROBOT_CALIBRATED=0, motion remains locked")
        return 0
    except (AssertionError, KeyError, ValueError) as error:
        print(f"FAIL: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
