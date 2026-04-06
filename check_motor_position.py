#!/usr/bin/env python3
"""HIL check: the motor reaches requested position via voltbro.foc.command.1.0."""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

from cyphal_hil_common import (
    CyphalPorts,
    YakutClient,
    average,
    collect_state_angles,
    parse_first_finite_float,
)

ERRORS_REGISTER = "state.errors"
MIN_ANGLE_REGISTER = "limit.min_angle"
MAX_ANGLE_REGISTER = "limit.max_angle"
SPEED_LIMIT_REGISTER = "limit.speed"
TORQUE_LIMIT_REGISTER = "limit.torque"
CURRENT_LIMIT_REGISTER = "limit.current"


def read_optional_register_float(
    client: YakutClient,
    node_id: int,
    register_name: str,
    timeout_s: float,
) -> float | None:
    text = client.read_register(node_id, register_name, timeout_s=timeout_s)
    try:
        return parse_first_finite_float(text)
    except ValueError:
        return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--node-id", type=int, required=True, help="Cyphal node ID of VBDrive")
    parser.add_argument("--target-angle", type=float, required=True, help="Desired shaft angle in radians")
    parser.add_argument("--tolerance", type=float, default=0.08, help="Allowed absolute angle error, rad")
    parser.add_argument("--timeout", type=float, default=12.0, help="Max wait time for reaching target, s")
    parser.add_argument("--settle-samples", type=int, default=5, help="How many state samples to average")
    parser.add_argument("--sample-sleep", type=float, default=0.05, help="Sleep between samples, s")
    parser.add_argument("--yakut-bin", default=os.environ.get("YAKUT_BIN", "yakut"), help="yakut binary path")
    parser.add_argument("--yakut-timeout", type=float, default=12.0, help="Timeout for each yakut call, s")
    parser.add_argument(
        "--command-base",
        type=int,
        default=CyphalPorts.foc_command_base,
        help="Base subject-ID for FOC command (final ID = base + node_id)",
    )
    parser.add_argument("--angle-kp", type=float, default=7.0, help="Angle feedback gain for foc.command")
    parser.add_argument("--velocity-kp", type=float, default=0.5, help="Velocity feedback gain for foc.command")
    parser.add_argument("--i-kp", type=float, default=4.0, help="Current regulator Kp for foc.command")
    parser.add_argument("--i-ki", type=float, default=1600.0, help="Current regulator Ki for foc.command")
    parser.add_argument("--state-port", type=int, default=CyphalPorts.state_port, help="State telemetry subject-ID")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    ports = CyphalPorts(foc_command_base=args.command_base, state_port=args.state_port)
    client = YakutClient(yakut_bin=args.yakut_bin)
    control_subject = ports.foc_command_port(args.node_id)
    errors_text = client.read_register(args.node_id, ERRORS_REGISTER, timeout_s=args.yakut_timeout)
    baseline_errors = int(parse_first_finite_float(errors_text))
    min_angle = read_optional_register_float(client, args.node_id, MIN_ANGLE_REGISTER, args.yakut_timeout)
    max_angle = read_optional_register_float(client, args.node_id, MAX_ANGLE_REGISTER, args.yakut_timeout)
    speed_limit = read_optional_register_float(client, args.node_id, SPEED_LIMIT_REGISTER, args.yakut_timeout)
    torque_limit = read_optional_register_float(client, args.node_id, TORQUE_LIMIT_REGISTER, args.yakut_timeout)
    current_limit = read_optional_register_float(client, args.node_id, CURRENT_LIMIT_REGISTER, args.yakut_timeout)
    initial_angles = collect_state_angles(
        client=client,
        state_subject=args.state_port,
        count=args.settle_samples,
        poll_sleep_s=args.sample_sleep,
    )
    initial_mean_angle = average(initial_angles)
    expected_reported_angle = args.target_angle

    print(f"[INFO] Commanding foc.command angle target: {args.target_angle:.6f} rad on subject {control_subject}")
    print(f"[INFO] Initial {ERRORS_REGISTER}={baseline_errors}")
    print(f"[INFO] Initial mean telemetry angle={initial_mean_angle:.6f} rad")
    print(
        "[INFO] Limits: "
        f"min_angle={min_angle if min_angle is not None else 'null'}, "
        f"max_angle={max_angle if max_angle is not None else 'null'}, "
        f"speed={speed_limit if speed_limit is not None else 'null'}, "
        f"torque={torque_limit if torque_limit is not None else 'null'}, "
        f"current={current_limit if current_limit is not None else 'null'}"
    )
    print(f"[INFO] Expected telemetry angle target={expected_reported_angle:.6f} rad")
    if (
        min_angle is not None and max_angle is not None and
        (args.target_angle < min_angle or args.target_angle > max_angle)
    ):
        print(
            "[FAIL] Requested angle is outside firmware limits: "
            f"{args.target_angle:.6f} not in [{min_angle:.6f}, {max_angle:.6f}]"
        )
        return 4
    client.publish_foc_command(
        subject_id=control_subject,
        torque_nm=0.0,
        angle_rad=args.target_angle,
        velocity_rad_s=0.0,
        angle_kp=args.angle_kp,
        velocity_kp=args.velocity_kp,
        i_kp=args.i_kp,
        i_ki=args.i_ki,
        burst_count=10,
        burst_period_s=0.05,
        timeout_s=args.yakut_timeout,
    )

    deadline = time.time() + args.timeout
    best_error = math.inf
    best_angle = initial_mean_angle
    while time.time() < deadline:
        client.publish_foc_command(
            subject_id=control_subject,
            torque_nm=0.0,
            angle_rad=args.target_angle,
            velocity_rad_s=0.0,
            angle_kp=args.angle_kp,
            velocity_kp=args.velocity_kp,
            i_kp=args.i_kp,
            i_ki=args.i_ki,
            burst_count=3,
            burst_period_s=0.05,
            timeout_s=args.yakut_timeout,
        )
        angles = collect_state_angles(
            client=client,
            state_subject=args.state_port,
            count=args.settle_samples,
            poll_sleep_s=args.sample_sleep,
        )
        mean_angle = average(angles)
        error = abs(mean_angle - expected_reported_angle)
        if error < best_error:
            best_error = error
            best_angle = mean_angle
        current_errors = int(
            parse_first_finite_float(
                client.read_register(args.node_id, ERRORS_REGISTER, timeout_s=args.yakut_timeout)
            )
        )
        print(f"[INFO] mean_angle={mean_angle:.6f} rad, error={error:.6f} rad")
        if current_errors > baseline_errors:
            print(
                "[FAIL] Command rejected by firmware: "
                f"{ERRORS_REGISTER} increased from {baseline_errors} to {current_errors}."
            )
            print(
                "[INFO] Since the error counter increased by the number of published transfers, "
                "the subject-ID is likely correct and the handler is being invoked. "
                "Most likely causes are command validation/limits or an incompatible payload layout."
            )
            return 2
        if error <= args.tolerance:
            print("[PASS] Position reached within tolerance.")
            return 0

    if math.isfinite(best_angle) and abs(best_angle - initial_mean_angle) <= args.tolerance * 0.25:
        print(
            "[FAIL] Position did not change. Command was not visibly applied. "
            "Possible causes: wrong subject-ID, incompatible voltbro.foc.command.1.0 schema, "
            "motor disabled, or no fresh state telemetry."
        )
        return 3

    print(
        f"[FAIL] Position not reached. Best error={best_error:.6f} rad, "
        f"tolerance={args.tolerance:.6f} rad"
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
