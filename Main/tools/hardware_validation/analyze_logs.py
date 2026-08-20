"""Summarize HWCSV telemetry without treating synthetic logs as measurements."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path

from protocol import iter_hwcsv


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * q
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def summarize(path: Path) -> tuple[list[dict[str, object]], dict[str, object]]:
    source_text = path.read_text(encoding="utf-8", errors="replace")
    records: list[dict[str, object]] = []
    sequence_by_kind: dict[str, list[int]] = {}
    latency_fields = {
        "VISION": ("capture_ms", 5, "detect_ms", 6, "align_ms", 7, "fer_ms", 8, "total_ms", 9),
        "SER": ("elapsed_us", 2),
        "FUSION": ("elapsed_us", 1, "conflict_raw", 5, "conflict_norm", 6, "dominant", 7, "dominant_score", 8),
    }
    for item in iter_hwcsv(source_text.splitlines()):
        row: dict[str, object] = {"kind": item.kind, "host_us": item.host_us}
        for index, value in enumerate(item.fields):
            row[f"field_{index}"] = value
        if item.kind in latency_fields:
            names_and_indexes = latency_fields[item.kind]
            for name, index in zip(names_and_indexes[::2], names_and_indexes[1::2]):
                if index < len(item.fields):
                    try:
                        row[name] = float(item.fields[index])
                    except ValueError:
                        pass
        if item.seq is not None:
            row["seq"] = item.seq
            sequence_by_kind.setdefault(item.kind, []).append(item.seq)
        records.append(row)

    summary: dict[str, object] = {
        "source": str(path),
        "measurement_status": (
            "synthetic_demo" if "SYNTHETIC_DEMO_ONLY" in source_text
            else "physical_serial_capture" if "PHYSICAL_SERIAL_CAPTURE" in source_text
            else "unverified_log"
        ),
        "record_count": len(records),
        "by_kind": dict(Counter(str(row["kind"]) for row in records)),
    }
    for kind in ("VISION", "SER", "FUSION"):
        rows = [row for row in records if row["kind"] == kind]
        summary[f"{kind.lower()}_count"] = len(rows)
        if kind == "VISION":
            for field in ("capture_ms", "detect_ms", "align_ms", "fer_ms", "total_ms"):
                values = [float(row[field]) for row in rows if field in row]
                summary[f"{field}_mean"] = sum(values) / len(values) if values else None
                summary[f"{field}_p50"] = percentile(values, 0.50)
                summary[f"{field}_p95"] = percentile(values, 0.95)
            summary["face_rate"] = sum(row.get("field_1") == "1" for row in rows) / len(rows) if rows else None
            summary["crc_invalid_count"] = sum(row.get("field_2") == "0" for row in rows)
            heaps: list[float] = []
        elif kind == "SER":
            values = [float(row["elapsed_us"]) for row in rows if "elapsed_us" in row]
            summary["ser_elapsed_us_p95"] = percentile(values, 0.95)
            summary["ser_ready_rate"] = sum(row.get("field_3") == "1" for row in rows) / len(rows) if rows else None
            heaps = [float(row["field_4"]) for row in rows if str(row.get("field_4", "")).isdigit()]
        else:
            values = [float(row["elapsed_us"]) for row in rows if "elapsed_us" in row]
            summary["fusion_elapsed_us_p95"] = percentile(values, 0.95)
            summary["fusion_high_conflict_rate"] = sum(row.get("field_2") == "1" for row in rows) / len(rows) if rows else None
            k_values = [float(row["conflict_raw"]) for row in rows if "conflict_raw" in row]
            c_values = [float(row["conflict_norm"]) for row in rows if "conflict_norm" in row]
            score_values = [float(row["dominant_score"]) for row in rows if "dominant_score" in row]
            summary["fusion_conflict_raw_mean"] = sum(k_values) / len(k_values) if k_values else None
            summary["fusion_conflict_norm_mean"] = sum(c_values) / len(c_values) if c_values else None
            summary["fusion_conflict_norm_p95"] = percentile(c_values, 0.95)
            summary["fusion_dominant_score_mean"] = sum(score_values) / len(score_values) if score_values else None
            heaps = [float(row["field_3"]) for row in rows if str(row.get("field_3", "")).isdigit()]
        if heaps:
            summary[f"{kind.lower()}_heap_min_bytes"] = min(heaps)
            summary[f"{kind.lower()}_heap_end_minus_start_bytes"] = heaps[-1] - heaps[0]
    gaps: dict[str, int] = {}
    for kind, sequence in sequence_by_kind.items():
        gaps[kind] = sum(max(0, current - previous - 1) for previous, current in zip(sequence, sequence[1:]))
    summary["sequence_gaps"] = gaps
    return records, summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("analysis"))
    args = parser.parse_args()
    records, summary = summarize(args.log)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    with (args.out_dir / "records.csv").open("w", newline="", encoding="utf-8") as handle:
        fieldnames = sorted({key for row in records for key in row})
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)
    (args.out_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
