"""Generate a balanced, actor-independent visual/audio validation manifest."""

from __future__ import annotations

import argparse
import csv
import random
from pathlib import Path


LABELS = ("H", "S", "N", "A")
CONDITIONS = {
    "congruent": (("H", "H"), ("S", "S"), ("N", "N"), ("A", "A")),
    "masked_positive_negative": (("H", "S"), ("N", "S"), ("H", "A"), ("N", "A")),
    "general_incongruence": (("S", "H"), ("A", "H"), ("S", "N"), ("A", "N")),
}


def build_rows(actors: int, repeats: int, seed: int) -> list[dict[str, str]]:
    rng = random.Random(seed)
    rows: list[dict[str, str]] = []
    for actor in range(1, actors + 1):
        for repetition in range(1, repeats + 1):
            for condition, pairs in CONDITIONS.items():
                for visual, audio in pairs:
                    rows.append({
                        "trial_id": f"a{actor:02d}_r{repetition:02d}_{condition[:3]}_{visual}{audio}",
                        "actor_id": f"actor_{actor:02d}",
                        "repetition": str(repetition),
                        "condition": condition,
                        "visual_label": visual,
                        "audio_label": audio,
                        "target_conflict": str(visual != audio).lower(),
                        "scene_note": "neutral background, fixed camera distance",
                    })
    rng.shuffle(rows)
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--actors", type=int, default=8)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--seed", type=int, default=20260811)
    parser.add_argument("--out", type=Path, default=Path("trial_manifest.csv"))
    args = parser.parse_args()
    if args.actors < 1 or args.repeats < 1:
        parser.error("--actors and --repeats must be positive")
    rows = build_rows(args.actors, args.repeats, args.seed)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {len(rows)} trials to {args.out}")


if __name__ == "__main__":
    main()
