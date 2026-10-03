"""Generate RAVDESS actor-recorded audiovisual re-pairing trials.

The output is a controlled evaluation manifest for reviewer response work:
visual and audio streams are paired within the same actor, statement,
repetition, and intensity whenever possible.  It uses only RAVDESS speech
recordings in the four Resona classes: happy, sad, neutral, and angry.
"""

from __future__ import annotations

import argparse
import csv
import random
from dataclasses import dataclass
from pathlib import Path


RAVDESS_EMOTIONS = {
    "01": "N",
    "03": "H",
    "04": "S",
    "05": "A",
}

CONDITIONS = {
    "congruent": (("H", "H"), ("S", "S"), ("N", "N"), ("A", "A")),
    "masked_negative": (("H", "S"), ("N", "S"), ("H", "A"), ("N", "A")),
    "general_incongruence": (("S", "H"), ("A", "H"), ("S", "N"), ("A", "N")),
}


@dataclass(frozen=True)
class RavdessItem:
    path: Path
    modality: str
    actor: int
    emotion: str
    intensity: str
    statement: str
    repetition: str

    @property
    def match_key(self) -> tuple[int, str, str, str]:
        return (self.actor, self.intensity, self.statement, self.repetition)


def parse_item(path: Path, modality: str) -> RavdessItem | None:
    parts = path.stem.split("-")
    if len(parts) != 7:
        return None
    expected_modality = "03" if modality == "audio" else "01"
    if parts[0] != expected_modality or parts[1] != "01":
        return None
    emotion = RAVDESS_EMOTIONS.get(parts[2])
    if emotion is None:
        return None
    return RavdessItem(
        path=path,
        modality=modality,
        actor=int(parts[6]),
        emotion=emotion,
        intensity=parts[3],
        statement=parts[4],
        repetition=parts[5],
    )


def scan_items(root: Path) -> tuple[list[RavdessItem], list[RavdessItem]]:
    audio: list[RavdessItem] = []
    video: list[RavdessItem] = []
    for path in root.rglob("*.wav"):
        item = parse_item(path, "audio")
        if item is not None and "Audio_Speech" in str(path):
            audio.append(item)
    for path in root.rglob("*.mp4"):
        item = parse_item(path, "video")
        if item is not None and ("Video_Speech" in str(path) or "RAVDESS_Speech_Normalized" in str(path)):
            video.append(item)
    return audio, video


def index_by_actor_emotion(items: list[RavdessItem]) -> dict[tuple[int, str], list[RavdessItem]]:
    indexed: dict[tuple[int, str], list[RavdessItem]] = {}
    for item in items:
        indexed.setdefault((item.actor, item.emotion), []).append(item)
    for values in indexed.values():
        values.sort(key=lambda item: (item.intensity, item.statement, item.repetition, str(item.path)))
    return indexed


def choose_pair(
    video_items: list[RavdessItem],
    audio_items: list[RavdessItem],
) -> tuple[RavdessItem, RavdessItem] | None:
    audio_by_key = {item.match_key: item for item in audio_items}
    for video_item in video_items:
        audio_item = audio_by_key.get(video_item.match_key)
        if audio_item is not None:
            return video_item, audio_item
    return None


def build_rows(root: Path, seed: int) -> list[dict[str, str]]:
    audio_items, video_items = scan_items(root)
    audio_index = index_by_actor_emotion(audio_items)
    video_index = index_by_actor_emotion(video_items)
    actors = sorted({item.actor for item in audio_items} & {item.actor for item in video_items})

    rows: list[dict[str, str]] = []
    for actor in actors:
        for condition, pairs in CONDITIONS.items():
            for visual_label, audio_label in pairs:
                chosen = choose_pair(
                    video_index.get((actor, visual_label), []),
                    audio_index.get((actor, audio_label), []),
                )
                if chosen is None:
                    continue
                video_item, audio_item = chosen
                target_masked = condition == "masked_negative"
                rows.append({
                    "trial_id": f"a{actor:02d}_{condition}_{visual_label}{audio_label}",
                    "actor_id": f"actor_{actor:02d}",
                    "condition": condition,
                    "visual_label": visual_label,
                    "audio_label": audio_label,
                    "target_conflict": str(visual_label != audio_label).lower(),
                    "target_masked_negative": str(target_masked).lower(),
                    "intensity": video_item.intensity,
                    "statement": video_item.statement,
                    "repetition": video_item.repetition,
                    "video_path": str(video_item.path),
                    "audio_path": str(audio_item.path),
                })

    rng = random.Random(seed)
    rng.shuffle(rows)
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ravdess-root", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=20260813)
    parser.add_argument("--out", type=Path, default=Path("ravdess_repair_manifest.csv"))
    args = parser.parse_args()

    rows = build_rows(args.ravdess_root, args.seed)
    if not rows:
        raise SystemExit("No matched RAVDESS speech audio/video trials found.")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    actor_count = len({row["actor_id"] for row in rows})
    print(f"wrote {len(rows)} trials from {actor_count} actors to {args.out}")


if __name__ == "__main__":
    main()
