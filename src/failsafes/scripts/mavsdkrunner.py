#!/usr/bin/env python3
"""Manual PX4 shell runner for multi-drone failsafe testing."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List

try:
    from pymavlink import mavutil
except ImportError as exc:  # pragma: no cover - runtime environment dependency
    print(f"Failed to import pymavlink: {exc}")
    print("Install it with: pip3 install --user pymavlink")
    sys.exit(1)


SERIAL_DEVICE_SHELL = 10
SERIAL_PAYLOAD_BYTES = 70
# launch_classic_3_iris_lz.sh starts sitl_multiple_run with "iris:1:..."
# which maps the three PX4 MAVLink UDP ports to 14541, 14542, 14543.
DEFAULT_PORTS = (14541, 14542, 14543)


@dataclass(frozen=True)
class Target:
    name: str
    port: int


class Px4ShellClient:
    def __init__(self, target: Target, baudrate: int = 57600):
        self.target = target
        self.baudrate = baudrate
        self._mav = None

    @property
    def label(self) -> str:
        return f"{self.target.name}:{self.target.port}"

    def connect(self, timeout_s: float) -> None:
        connection_uri = f"udpin:0.0.0.0:{self.target.port}"
        self._mav = mavutil.mavlink_connection(connection_uri, autoreconnect=True, baud=self.baudrate)
        self._mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        self._mav.wait_heartbeat(timeout=timeout_s)
        self.send_raw("\n")
        self.read_output(max_wait_s=1.0, idle_timeout_s=0.2)

    def close(self) -> None:
        if self._mav is not None:
            self._mav.mav.serial_control_send(SERIAL_DEVICE_SHELL, 0, 0, 0, 0, [0] * SERIAL_PAYLOAD_BYTES)

    def send_raw(self, text: str) -> None:
        if self._mav is None:
            raise RuntimeError(f"Target {self.label} is not connected")

        data = text.encode("utf-8", errors="ignore")
        index = 0
        while index < len(data):
            chunk = data[index : index + SERIAL_PAYLOAD_BYTES]
            payload = list(chunk) + [0] * (SERIAL_PAYLOAD_BYTES - len(chunk))
            self._mav.mav.serial_control_send(
                SERIAL_DEVICE_SHELL,
                mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
                | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                0,
                0,
                len(chunk),
                payload,
            )
            index += len(chunk)

    def run_command(self, command: str, response_timeout_s: float) -> str:
        self.send_raw(command.rstrip() + "\n")
        return self.read_output(max_wait_s=response_timeout_s, idle_timeout_s=0.25)

    def read_output(self, max_wait_s: float, idle_timeout_s: float) -> str:
        if self._mav is None:
            return ""

        start = time.monotonic()
        last_data = None
        chunks: List[str] = []

        while time.monotonic() - start < max_wait_s:
            msg = self._mav.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.05)
            if msg and getattr(msg, "count", 0) > 0:
                data = msg.data[: msg.count]
                chunks.append(bytes(data).decode("utf-8", errors="replace"))
                last_data = time.monotonic()
                continue

            if last_data is not None and (time.monotonic() - last_data) >= idle_timeout_s:
                break

        return "".join(chunks).strip()


def parse_ports(raw: str) -> List[int]:
    ports: List[int] = []
    for item in raw.split(","):
        token = item.strip()
        if not token:
            continue
        port = int(token)
        if port <= 0:
            raise ValueError(f"Invalid port: {token}")
        ports.append(port)
    if not ports:
        raise ValueError("No ports provided")
    return ports


def build_targets(ports: Iterable[int]) -> List[Target]:
    return [Target(name=f"drone{idx}", port=port) for idx, port in enumerate(ports)]


def print_help() -> None:
    print("\nRunner commands:")
    print("  :help                 Show this help")
    print("  :targets              Show known/connected PX4 targets")
    print("  :use all              Send shell commands to all connected targets")
    print("  :use <name|index|port> Send shell commands to one target")
    print("  :exit                 Exit runner")
    print("")
    print("Any non-':' input is sent to PX4 shell directly.")
    print("Examples:")
    print("  param set SYS_FAILURE_EN 1")
    print("  failure gps off")
    print("  failure gps ok")


def resolve_selection(token: str, clients: Dict[str, Px4ShellClient]) -> List[Px4ShellClient]:
    key = token.strip().lower()
    if key == "all":
        return list(clients.values())

    if key in clients:
        return [clients[key]]

    if key.isdigit():
        by_index = f"drone{int(key)}"
        if by_index in clients:
            return [clients[by_index]]
        by_port = next((client for client in clients.values() if client.target.port == int(key)), None)
        if by_port is not None:
            return [by_port]

    raise ValueError(f"Unknown target selector: {token}")


def print_targets(clients: Dict[str, Px4ShellClient]) -> None:
    print("\nConnected targets:")
    for name, client in clients.items():
        print(f"  - {name} (port {client.target.port})")


def run_repl(clients: Dict[str, Px4ShellClient], response_timeout_s: float) -> int:
    selected: List[Px4ShellClient] = list(clients.values())
    print("\nMAVSDK Manual PX4 Command Runner")
    print("Type ':help' for runner commands.")
    print("Default target: all connected PX4 instances.\n")

    while True:
        try:
            raw = input("px4-manual> ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\nExiting runner...")
            return 0

        if not raw:
            continue

        if raw.startswith(":"):
            command = raw[1:].strip()
            if command in {"exit", "quit"}:
                print("Exiting runner...")
                return 0
            if command == "help":
                print_help()
                continue
            if command == "targets":
                print_targets(clients)
                continue
            if command.startswith("use "):
                selector = command[4:].strip()
                try:
                    selected = resolve_selection(selector, clients)
                except ValueError as exc:
                    print(f"[runner] {exc}")
                    continue
                label = "all" if len(selected) > 1 else selected[0].label
                print(f"[runner] Selected target: {label}")
                continue
            print(f"[runner] Unknown runner command: {raw}")
            print("[runner] Try ':help'")
            continue

        for client in selected:
            print(f"\n[{client.label}] $ {raw}")
            try:
                output = client.run_command(raw, response_timeout_s=response_timeout_s)
            except Exception as exc:  # pragma: no cover - runtime network error
                print(f"[{client.label}] ERROR: {exc}")
                continue

            if output:
                print(output)
            else:
                print(f"[{client.label}] (no output)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Manual PX4 shell command runner for failsafe testing on multi-drone SITL."
    )
    parser.add_argument(
        "--ports",
        default=",".join(str(p) for p in DEFAULT_PORTS),
        help=f"Comma-separated PX4 MAVLink UDP ports (default: {','.join(str(p) for p in DEFAULT_PORTS)})",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=8.0,
        help="Heartbeat wait timeout per target in seconds (default: 8.0)",
    )
    parser.add_argument(
        "--response-timeout",
        type=float,
        default=2.5,
        help="Command output collection window in seconds (default: 2.5)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        ports = parse_ports(args.ports)
    except ValueError as exc:
        print(f"Invalid --ports value: {exc}")
        return 2

    clients: Dict[str, Px4ShellClient] = {}
    for target in build_targets(ports):
        client = Px4ShellClient(target)
        print(f"[connect] {client.label} ...", end="", flush=True)
        try:
            client.connect(timeout_s=args.connect_timeout)
        except Exception as exc:  # pragma: no cover - runtime network error
            print(f" failed ({exc})")
            continue
        print(" connected")
        clients[target.name] = client

    if not clients:
        print("No PX4 targets connected. Make sure launch_classic_3_iris_lz.sh is running.")
        return 1

    try:
        return run_repl(clients, response_timeout_s=args.response_timeout)
    finally:
        for client in clients.values():
            client.close()


if __name__ == "__main__":
    raise SystemExit(main())
