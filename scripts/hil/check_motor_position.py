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

OFFSET_REGISTER = "limit.angle_offset"


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
    offset_text = client.read_register(args.node_id, OFFSET_REGISTER, timeout_s=args.yakut_timeout)
    offset = parse_first_finite_float(offset_text)
    expected_reported_angle = args.target_angle + offset

    print(f"[INFO] Commanding foc.command angle target: {args.target_angle:.6f} rad on subject {control_subject}")
    print(f"[INFO] Current {OFFSET_REGISTER}={offset:.6f} rad")
    print(f"[INFO] Expected telemetry angle target={expected_reported_angle:.6f} rad")
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
        best_error = min(best_error, error)
        print(f"[INFO] mean_angle={mean_angle:.6f} rad, error={error:.6f} rad")
        if error <= args.tolerance:
            print("[PASS] Position reached within tolerance.")
            return 0

    print(f"[FAIL] Position not reached. Best error={best_error:.6f} rad, tolerance={args.tolerance:.6f} rad")
    return 1


if __name__ == "__main__":
    sys.exit(main())
