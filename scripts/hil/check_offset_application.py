#!/usr/bin/env python3
"""HIL check: angle offset is applied in telemetry and can be persisted."""

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
SAVE_REGISTER = "config.save_now"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--node-id", type=int, required=True, help="Cyphal node ID of VBDrive")
    parser.add_argument("--delta-offset", type=float, required=True, help="Offset delta to apply, rad")
    parser.add_argument(
        "--telemetry-shift-tolerance",
        type=float,
        default=0.03,
        help="Allowed abs error between measured and expected angle shift, rad",
    )
    parser.add_argument(
        "--register-tolerance",
        type=float,
        default=1.0e-4,
        help="Allowed abs error for register value checks, rad",
    )
    parser.add_argument("--samples", type=int, default=6, help="How many telemetry samples to average")
    parser.add_argument("--sample-sleep", type=float, default=0.05, help="Sleep between samples, s")
    parser.add_argument(
        "--command-base",
        type=int,
        default=CyphalPorts.foc_command_base,
        help="Base subject-ID for FOC command (final ID = base + node_id)",
    )
    parser.add_argument(
        "--angle-kp",
        type=float,
        default=0.0,
        help="Angle feedback gain for foc.command during offset test (default 0 for neutral check)",
    )
    parser.add_argument(
        "--velocity-kp",
        type=float,
        default=0.0,
        help="Velocity feedback gain for foc.command during offset test (default 0 for neutral check)",
    )
    parser.add_argument("--i-kp", type=float, default=4.0, help="Current regulator Kp for foc.command")
    parser.add_argument("--i-ki", type=float, default=1600.0, help="Current regulator Ki for foc.command")
    parser.add_argument("--skip-persist", action="store_true", help="Do not test config.save_now + reboot")
    parser.add_argument("--yakut-bin", default=os.environ.get("YAKUT_BIN", "yakut"), help="yakut binary path")
    parser.add_argument("--yakut-timeout", type=float, default=12.0, help="Timeout for each yakut call, s")
    parser.add_argument("--state-port", type=int, default=CyphalPorts.state_port, help="State telemetry subject-ID")
    return parser.parse_args()


def assert_close(actual: float, expected: float, tolerance: float, name: str) -> None:
    error = abs(actual - expected)
    if error > tolerance:
        raise RuntimeError(
            f"{name} mismatch: actual={actual:.6f}, expected={expected:.6f}, "
            f"error={error:.6f}, tolerance={tolerance:.6f}"
        )


def main() -> int:
    args = parse_args()
    ports = CyphalPorts(foc_command_base=args.command_base, state_port=args.state_port)
    client = YakutClient(yakut_bin=args.yakut_bin)
    command_subject = ports.foc_command_port(args.node_id)

    # 1) Read and remember current offset.
    initial_reg_text = client.read_register(args.node_id, OFFSET_REGISTER, timeout_s=args.yakut_timeout)
    initial_offset = parse_first_finite_float(initial_reg_text)
    target_offset = initial_offset + args.delta_offset
    print(f"[INFO] Initial {OFFSET_REGISTER}={initial_offset:.6f} rad")
    print(f"[INFO] Target  {OFFSET_REGISTER}={target_offset:.6f} rad")

    # 2) Estimate current shaft angle and publish neutral foc.command.
    pre_hold_angles = collect_state_angles(
        client=client,
        state_subject=args.state_port,
        count=args.samples,
        poll_sleep_s=args.sample_sleep,
        timeout_s=args.yakut_timeout,
    )
    pre_hold_mean = average(pre_hold_angles)
    neutral_target_shaft = pre_hold_mean - initial_offset
    print(f"[INFO] Estimated shaft angle for neutral command={neutral_target_shaft:.6f} rad")

    def publish_neutral_command() -> None:
        client.publish_foc_command(
            subject_id=command_subject,
            torque_nm=0.0,
            angle_rad=neutral_target_shaft,
            velocity_rad_s=0.0,
            angle_kp=args.angle_kp,
            velocity_kp=args.velocity_kp,
            i_kp=args.i_kp,
            i_ki=args.i_ki,
            burst_count=max(4, args.samples),
            burst_period_s=0.05,
            timeout_s=args.yakut_timeout,
        )

    publish_neutral_command()
    time.sleep(0.2)

    # 3) Collect baseline angle under neutral foc.command.
    baseline_angles = collect_state_angles(
        client=client,
        state_subject=args.state_port,
        count=args.samples,
        poll_sleep_s=args.sample_sleep,
        timeout_s=args.yakut_timeout,
    )
    baseline_mean = average(baseline_angles)
    print(f"[INFO] Baseline mean angle={baseline_mean:.6f} rad")

    # 4) Apply new offset through register.
    client.write_register(args.node_id, OFFSET_REGISTER, f"{target_offset:.9f}", timeout_s=args.yakut_timeout)
    after_write_text = client.read_register(args.node_id, OFFSET_REGISTER, timeout_s=args.yakut_timeout)
    after_write_offset = parse_first_finite_float(after_write_text)
    assert_close(after_write_offset, target_offset, args.register_tolerance, "Offset register after write")
    print(f"[INFO] Register after write={after_write_offset:.6f} rad")

    # 5) Keep same neutral foc.command and check telemetry shift.
    publish_neutral_command()
    time.sleep(0.2)
    shifted_angles = collect_state_angles(
        client=client,
        state_subject=args.state_port,
        count=args.samples,
        poll_sleep_s=args.sample_sleep,
        timeout_s=args.yakut_timeout,
    )
    shifted_mean = average(shifted_angles)
    observed_shift = shifted_mean - baseline_mean
    print(f"[INFO] Shifted mean angle={shifted_mean:.6f} rad")
    print(f"[INFO] Observed angle shift={observed_shift:.6f} rad, expected={args.delta_offset:.6f} rad")
    assert_close(observed_shift, args.delta_offset, args.telemetry_shift_tolerance, "Telemetry angle shift")

    if args.skip_persist:
        print("[PASS] Offset application check passed (persist step skipped).")
        return 0

    # 6) Persist and validate after power cycle/reboot.
    client.write_register_bool(args.node_id, SAVE_REGISTER, True, timeout_s=args.yakut_timeout)
    print(f"[INFO] Wrote {SAVE_REGISTER}=true. Reboot/power-cycle the controller now.")
    input("[ACTION] After reboot is complete, press Enter to continue...")

    restored_text = client.read_register(args.node_id, OFFSET_REGISTER, timeout_s=args.yakut_timeout)
    restored_offset = parse_first_finite_float(restored_text)
    assert_close(restored_offset, target_offset, args.register_tolerance, "Offset register after reboot")
    print(f"[PASS] Offset persisted. Restored {OFFSET_REGISTER}={restored_offset:.6f} rad")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # noqa: BLE001
        print(f"[FAIL] {exc}")
        sys.exit(1)
