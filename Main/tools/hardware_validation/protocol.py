"""Host-side protocol helpers for the MaixCAM-Pro <-> ESP32-S3 bench.

The wire CRC is computed over the exact UTF-8/ASCII bytes before the final
`,'crc'` field.  Keeping this module dependency-free makes it usable on a
lab laptop before pyserial is installed.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Optional


def crc8(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x07) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


def encode_vision_packet(packet: Dict[str, Any]) -> bytes:
    payload = json.dumps(packet, ensure_ascii=True, separators=(",", ":"))
    checksum = crc8(payload[:-1].encode("ascii"))
    return (payload[:-1] + ',"crc":"{:02X}"}}\n'.format(checksum)).encode("ascii")


def _crc_valid(raw_line: bytes, expected: int) -> bool:
    marker = b',"crc":"'
    index = raw_line.find(marker)
    if index < 0:
        return False
    return crc8(raw_line[:index]) == expected


def parse_vision_packet(line: bytes | str) -> Dict[str, Any]:
    raw = line.encode("ascii") if isinstance(line, str) else line
    raw = raw.strip()
    packet = json.loads(raw.decode("ascii"))
    crc_text = packet.get("crc")
    if not isinstance(crc_text, str):
        raise ValueError("vision packet has no CRC field")
    try:
        expected = int(crc_text, 16)
    except ValueError as exc:
        raise ValueError("invalid CRC field") from exc
    packet["crc_valid"] = _crc_valid(raw, expected)
    return packet


@dataclass
class HwCsvRecord:
    kind: str
    host_us: int
    fields: list[str]

    @property
    def seq(self) -> Optional[int]:
        if len(self.fields) < 1:
            return None
        try:
            return int(self.fields[0])
        except ValueError:
            return None


def parse_hwcsv_line(line: str) -> Optional[HwCsvRecord]:
    text = line.strip()
    start = text.find("HWCSV,")
    if start < 0:
        return None
    parts = text[start:].split(",")
    if len(parts) < 3 or parts[0] != "HWCSV":
        return None
    try:
        host_us = int(parts[2])
    except ValueError:
        return None
    return HwCsvRecord(kind=parts[1], host_us=host_us, fields=parts[3:])


def iter_hwcsv(lines: Iterable[str]) -> Iterable[HwCsvRecord]:
    for line in lines:
        record = parse_hwcsv_line(line)
        if record is not None:
            yield record
