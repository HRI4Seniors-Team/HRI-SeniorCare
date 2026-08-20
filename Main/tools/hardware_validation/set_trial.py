"""Set the trial identifier on a MaixCam UART control link."""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("trial_id")
    parser.add_argument("--port", required=True, help="MaixCam UART control port")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()
    if len(args.trial_id) > 31 or any(char.isspace() for char in args.trial_id):
        parser.error("trial_id must be <=31 characters and contain no whitespace")
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("pyserial is missing; install with: python -m pip install -r requirements.txt") from exc
    with serial.Serial(args.port, args.baud, timeout=1.0) as port:
        port.write(f"SET_TRIAL {args.trial_id}\n".encode("ascii"))
        response = port.readline().decode("ascii", "replace").strip()
    print(response)
    if response != f"ACK_TRIAL {args.trial_id}":
        raise SystemExit("MaixCam did not acknowledge the trial identifier")


if __name__ == "__main__":
    main()
