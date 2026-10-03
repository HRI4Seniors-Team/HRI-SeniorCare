"""Capture ESP32-S3 HWCSV telemetry and raw MaixCam packets."""

from __future__ import annotations

import argparse
import time
from pathlib import Path


def demo_log(path: Path, duration: float = 5.0) -> None:
    """Create a clearly synthetic log for testing the analysis pipeline."""
    start = time.time()
    seq = 0
    with path.open("w", encoding="utf-8") as handle:
        handle.write("# SYNTHETIC_DEMO_ONLY; do not report as hardware measurement\n")
        while time.time() - start < duration:
            now = int((time.time() - start) * 1_000_000)
            handle.write(f"HWCSV,VISION,{now},{seq},1,1,0.82,0.12,2.0,13.0,1.0,3.0,19.0,demo\n")
            handle.write(f"HWCSV,SER,{now},{seq},480,4200,1,210000\n")
            handle.write(f"HWCSV,FUSION,{now},{seq},80,0,209500,1\n")
            handle.flush()
            seq += 1
            time.sleep(0.1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", help="ESP32-S3 serial port, e.g. COM8")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--out", type=Path, default=Path("hardware_run.log"))
    parser.add_argument("--demo", action="store_true", help="write a synthetic parser test log")
    args = parser.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    if args.demo:
        demo_log(args.out, args.duration)
        print(f"wrote synthetic demo log to {args.out}")
        return
    if not args.port:
        parser.error("--port is required unless --demo is used")
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("pyserial is missing; install with: python -m pip install -r requirements.txt") from exc
    with serial.Serial(args.port, args.baud, timeout=0.2) as port, args.out.open("w", encoding="utf-8") as handle:
        handle.write("# PHYSICAL_SERIAL_CAPTURE; verify device, firmware and model hashes in the session notes\n")
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            line = port.readline()
            if line:
                text = line.decode("utf-8", "replace")
                handle.write(text)
                handle.flush()
                print(text, end="")
    print(f"saved raw serial capture to {args.out}")


if __name__ == "__main__":
    main()
