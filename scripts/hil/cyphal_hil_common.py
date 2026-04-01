#!/usr/bin/env python3
"""Shared helpers for Cyphal HIL checks executed via yakut CLI.

These helpers intentionally keep runtime dependencies minimal (stdlib only).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
import re
import shlex
import subprocess
import time
from typing import Iterable, Sequence


FLOAT_RE = r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?|[-+]?(?:inf|nan)"


class YakutError(RuntimeError):
    pass


@dataclass(frozen=True)
class CyphalPorts:
    foc_command_base: int = 2107
    specific_control_base: int = 3407
    state_port: int = 3811

    def foc_command_port(self, node_id: int) -> int:
        return self.foc_command_base + node_id

    def specific_control_port(self, node_id: int) -> int:
        return self.specific_control_base + node_id


class YakutClient:
    def __init__(self, yakut_bin: str = "yakut", extra_env: dict[str, str] | None = None) -> None:
        self._yakut_bin = yakut_bin
        self._env = os.environ.copy()
        if extra_env:
            self._env.update(extra_env)

    def _run_raw(self, argv: Sequence[str], timeout_s: float) -> tuple[int, str, str]:
        try:
            proc = subprocess.run(
                argv,
                capture_output=True,
                text=True,
                timeout=timeout_s,
                env=self._env,
            )
            return proc.returncode, proc.stdout, proc.stderr
        except subprocess.TimeoutExpired as ex:
            out = ex.stdout if isinstance(ex.stdout, str) else ""
            err = ex.stderr if isinstance(ex.stderr, str) else ""
            err = (err + "\n" if err else "") + f"Command timed out after {timeout_s:.1f}s"
            return 124, out, err

    def run_variants(self, variants: Iterable[Sequence[str]], timeout_s: float, action: str) -> str:
        errors: list[str] = []
        for args in variants:
            argv = [self._yakut_bin, *args]
            code, out, err = self._run_raw(argv, timeout_s=timeout_s)
            if code == 0:
                return out
            errors.append(
                f"$ {' '.join(shlex.quote(a) for a in argv)}\nexit={code}\nstdout:\n{out}\nstderr:\n{err}"
            )
        joined = "\n\n---\n\n".join(errors)
        if "Module not found" in joined and "voltbro" in joined:
            joined += (
                "\n\nHint: voltbro data type module is not resolvable.\n"
                "Try:\n"
                "  export PYTHONPATH=\"$PWD/Drivers/libcxxcanard/libs:${PYTHONPATH:-}\"\n"
                "  export CYPHAL_PATH=\"$PWD/Drivers/libcxxcanard/libs:${CYPHAL_PATH:-}\"\n"
            )
        raise YakutError(f"Failed to {action}. Tried {len(errors)} command variants:\n{joined}")

    def read_register(self, node_id: int, register_name: str, timeout_s: float = 6.0) -> str:
        return self.run_variants(
            variants=[
                ("reg", str(node_id), register_name),
            ],
            timeout_s=timeout_s,
            action=f"read register {register_name}",
        )

    def write_register(self, node_id: int, register_name: str, value_expr: str, timeout_s: float = 6.0) -> str:
        return self.run_variants(
            variants=[
                ("reg", str(node_id), register_name, value_expr),
            ],
            timeout_s=timeout_s,
            action=f"write register {register_name}",
        )

    def write_register_bool(self, node_id: int, register_name: str, value: bool, timeout_s: float = 6.0) -> str:
        if value:
            variants = [
                ("reg", str(node_id), register_name, "true"),
                ("reg", str(node_id), register_name, "1"),
                ("reg", str(node_id), register_name, "[true]"),
                ("reg", str(node_id), register_name, "[1]"),
            ]
        else:
            variants = [
                ("reg", str(node_id), register_name, "false"),
                ("reg", str(node_id), register_name, "0"),
                ("reg", str(node_id), register_name, "[false]"),
                ("reg", str(node_id), register_name, "[0]"),
            ]
        return self.run_variants(
            variants=variants,
            timeout_s=timeout_s,
            action=f"write boolean register {register_name}",
        )

    def publish_specific_control(
        self,
        subject_id: int,
        set_point_type: int,
        set_point_value: float,
        burst_count: int = 8,
        burst_period_s: float = 0.05,
        timeout_s: float = 12.0,
    ) -> str:
        dtype = "voltbro.foc.specific_control.1.0"
        payload = f"{{set_point_type: {set_point_type}, set_point_value: {set_point_value}}}"
        return self.run_variants(
            variants=[
                ("pub", "--count", str(burst_count), "--period", str(burst_period_s), f"{subject_id}:{dtype}", payload),
                ("pub", "-N", str(burst_count), "-T", str(burst_period_s), f"{subject_id}:{dtype}", payload),
            ],
            timeout_s=timeout_s,
            action="publish specific control setpoint",
        )

    def publish_foc_command(
        self,
        subject_id: int,
        torque_nm: float,
        angle_rad: float,
        velocity_rad_s: float,
        angle_kp: float,
        velocity_kp: float,
        i_kp: float,
        i_ki: float,
        dtype_candidates: Sequence[str] | None = None,
        burst_count: int = 8,
        burst_period_s: float = 0.05,
        timeout_s: float = 12.0,
    ) -> str:
        dtypes = list(
            dtype_candidates
            if dtype_candidates is not None
            else (
                "voltbro.foc.command.1.0",
                "voltbro.foc.command_1_0",
                "voltbro.foc.command_1_0.command_1_0",
            )
        )
        payloads = [
            (
                "{torque: {newton_meter: " + f"{torque_nm}" + "}, "
                "angle: {radian: " + f"{angle_rad}" + "}, "
                "velocity: {radian_per_second: " + f"{velocity_rad_s}" + "}, "
                "angle_kp: {value: " + f"{angle_kp}" + "}, "
                "velocity_kp: {value: " + f"{velocity_kp}" + "}, "
                "I_kp: {value: " + f"{i_kp}" + "}, "
                "I_ki: {value: " + f"{i_ki}" + "}}"
            ),
            (
                "{_torque: {newton_meter: " + f"{torque_nm}" + "}, "
                "angle: {radian: " + f"{angle_rad}" + "}, "
                "velocity: {radian_per_second: " + f"{velocity_rad_s}" + "}, "
                "angle_kp: {value: " + f"{angle_kp}" + "}, "
                "velocity_kp: {value: " + f"{velocity_kp}" + "}, "
                "I_kp: {value: " + f"{i_kp}" + "}, "
                "I_ki: {value: " + f"{i_ki}" + "}}"
            ),
            (
                "{torque: {newton_meter: " + f"{torque_nm}" + "}, "
                "angle: {radian: " + f"{angle_rad}" + "}, "
                "velocity: {radian_per_second: " + f"{velocity_rad_s}" + "}, "
                "position_feedback_gain: {value: " + f"{angle_kp}" + "}, "
                "velocity_feedback_gain: {value: " + f"{velocity_kp}" + "}, "
                "I_kp: {value: " + f"{i_kp}" + "}, "
                "I_ki: {value: " + f"{i_ki}" + "}}"
            ),
            (
                "{_torque: {newton_meter: " + f"{torque_nm}" + "}, "
                "angle: {radian: " + f"{angle_rad}" + "}, "
                "velocity: {radian_per_second: " + f"{velocity_rad_s}" + "}, "
                "position_feedback_gain: {value: " + f"{angle_kp}" + "}, "
                "velocity_feedback_gain: {value: " + f"{velocity_kp}" + "}, "
                "I_kp: {value: " + f"{i_kp}" + "}, "
                "I_ki: {value: " + f"{i_ki}" + "}}"
            ),
        ]
        variants: list[Sequence[str]] = []
        for dtype in dtypes:
            for payload in payloads:
                variants.append(
                    ("pub", "--count", str(burst_count), "--period", str(burst_period_s), f"{subject_id}:{dtype}", payload)
                )
                variants.append(("pub", "-N", str(burst_count), "-T", str(burst_period_s), f"{subject_id}:{dtype}", payload))
        return self.run_variants(
            variants=variants,
            timeout_s=timeout_s,
            action="publish foc command",
        )

    def read_state_sample(
        self,
        subject_id: int,
        timeout_s: float = 6.0,
        dtype_candidates: Sequence[str] | None = None,
    ) -> str:
        dtypes = list(
            dtype_candidates
            if dtype_candidates is not None
            else (
                "voltbro.foc.state_simple.1.0",
                "voltbro.foc.state_simple_1_0",
                "voltbro.foc.state_simple_1_0.state_simple_1_0",
            )
        )
        variants: list[Sequence[str]] = []
        for dtype in dtypes:
            variants.append(("sub", "--count", "1", f"{subject_id}:{dtype}"))
            variants.append(("sub", "-N", "1", f"{subject_id}:{dtype}"))
        return self.run_variants(
            variants=variants,
            timeout_s=timeout_s,
            action="read one state sample",
        )


def parse_first_finite_float(text: str) -> float:
    for match in re.finditer(FLOAT_RE, text, flags=re.IGNORECASE):
        try:
            value = float(match.group(0))
        except ValueError:
            continue
        if math.isfinite(value):
            return value
    raise ValueError(f"No finite float found in text:\n{text}")


def parse_angle_radian(text: str) -> float:
    yaml_style = re.search(r"angle\s*:\s*.*?radian\s*:\s*(" + FLOAT_RE + r")", text, flags=re.IGNORECASE | re.DOTALL)
    if yaml_style:
        value = float(yaml_style.group(1))
        if math.isfinite(value):
            return value

    json_style = re.search(r'"angle"\s*:\s*\{[^{}]*"radian"\s*:\s*(' + FLOAT_RE + r")", text, flags=re.IGNORECASE)
    if json_style:
        value = float(json_style.group(1))
        if math.isfinite(value):
            return value

    raise ValueError(f"Cannot parse state.angle.radian from output:\n{text}")


def average(samples: Sequence[float]) -> float:
    if not samples:
        raise ValueError("Cannot average empty sequence")
    return sum(samples) / len(samples)


def collect_state_angles(
    client: YakutClient,
    state_subject: int,
    count: int,
    poll_sleep_s: float,
    timeout_s: float = 6.0,
    state_dtype_candidates: Sequence[str] | None = None,
) -> list[float]:
    out: list[float] = []
    for _ in range(count):
        text = client.read_state_sample(
            state_subject,
            timeout_s=timeout_s,
            dtype_candidates=state_dtype_candidates,
        )
        out.append(parse_angle_radian(text))
        time.sleep(poll_sleep_s)
    return out
