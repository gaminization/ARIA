#!/usr/bin/env python3
"""P0-DET training: YOLOv8n at 320 px on the auto-labelled Gazebo frames.

The pre-registration reduced Phase 0f to "1,000 auto-labelled Gazebo frames,
YOLOv8n at 320 px ... shortfall reported".  It also declared the original run
as CPU-only because no CUDA device was available.  CUDA is now available, so
this script trains on the GPU; DEVIATIONS.md records that change.  The model,
resolution, data, split, and metric are unchanged.

Writes:
    data/journal/raw/detector/runs/train/weights/best.pt
    data/journal/models/p0_det_yolov8n.pt
    data/journal/raw/P0-DET_train.json
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from aria_study.provenance import assert_clean, stamp  # noqa: E402

OUT = ROOT / "data" / "journal" / "raw" / "detector"
RAW = ROOT / "data" / "journal" / "raw"
MODELS = ROOT / "data" / "journal" / "models"

EXPECTED = {"train": 1000, "val": 120, "test": 120}
BASE_WEIGHTS_CANDIDATES = [
    MODELS / "yolov8n.pt",
    Path("/home/gaminizer/Projects/ARIA/arm_vision/models/yolov8n.pt"),
    Path("/home/gaminizer/Projects/ARIA/models/yolov8n.pt"),
]


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def dataset_counts() -> dict:
    counts = {"train": 0, "val": 0, "test": 0}
    with (RAW / "P0-DET_frames.csv").open(newline="") as f:
        for row in csv.DictReader(f):
            counts[row["split"]] += 1
    return counts


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=80)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--imgsz", type=int, default=320)
    ap.add_argument("--device", default="0")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--patience", type=int, default=20)
    ap.add_argument("--smoke", action="store_true", help="1 epoch, 20%% of data")
    args = ap.parse_args()

    assert_clean("P0-DET training")
    prov = stamp()

    counts = dataset_counts()
    if not args.smoke:
        for split, expected in EXPECTED.items():
            if counts.get(split) != expected:
                raise SystemExit(
                    f"{split}: expected {expected} frames, found {counts.get(split)}")
    if not (OUT / "data.yaml").exists():
        raise SystemExit(f"missing {OUT / 'data.yaml'}")

    MODELS.mkdir(parents=True, exist_ok=True)
    base = next((p for p in BASE_WEIGHTS_CANDIDATES if p.exists()), None)
    if base is None:
        raise SystemExit(
            "no local yolov8n.pt found; place one at data/journal/models/yolov8n.pt")
    if base != MODELS / "yolov8n.pt":
        shutil.copy2(base, MODELS / "yolov8n.pt")
    base = MODELS / "yolov8n.pt"
    print(f"base weights {base} sha256={sha256_file(base)[:16]}")

    import torch
    from ultralytics import YOLO

    if args.device not in ("cpu",) and not torch.cuda.is_available():
        raise SystemExit("CUDA requested but torch.cuda.is_available() is False")
    device_name = (torch.cuda.get_device_name(0) if torch.cuda.is_available()
                   else "cpu")

    started = time.time()
    model = YOLO(str(base))
    train_kwargs = dict(
        data=str(OUT / "data.yaml"),
        epochs=1 if args.smoke else args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        seed=args.seed,
        deterministic=True,
        workers=args.workers,
        patience=args.patience,
        project=str(OUT / "runs"),
        name="train",
        exist_ok=True,
        verbose=True,
        plots=False,
        val=True,
        amp=True,
        optimizer="auto",
    )
    if args.smoke:
        train_kwargs["fraction"] = 0.2
        train_kwargs["name"] = "smoke"
    results = model.train(**train_kwargs)
    finished = time.time()

    run_dir = Path(results.save_dir)
    best = run_dir / "weights" / "best.pt"
    last = run_dir / "weights" / "last.pt"
    weights = best if best.exists() else last
    if not weights.exists():
        raise SystemExit(f"no weights written under {run_dir}")
    target = MODELS / ("p0_det_yolov8n_smoke.pt" if args.smoke
                       else "p0_det_yolov8n.pt")
    shutil.copy2(weights, target)

    results_csv = run_dir / "results.csv"
    final = {}
    if results_csv.exists():
        with results_csv.open(newline="") as f:
            table = list(csv.DictReader(f))
        if table:
            final = table[-1]

    summary = {
        "exp_id": "P0-DET",
        "phase": "training",
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(started)),
        "finished_at": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(finished)),
        "duration_s": finished - started,
        "commit": prov["commit"],
        "dirty": False,
        "dataset": {
            "frames": counts,
            "scenes": {"train": [1, 20], "val": [21, 24], "test": [25, 30]},
            "images_dir": str((OUT / "images").relative_to(ROOT)),
        },
        "config": {
            "model": "yolov8n",
            "imgsz": args.imgsz,
            "epochs": 1 if args.smoke else args.epochs,
            "batch": args.batch,
            "seed": args.seed,
            "deterministic": True,
            "patience": args.patience,
            "device": args.device,
            "device_name": device_name,
            "amp": True,
            "optimizer": "auto",
            "fraction": 0.2 if args.smoke else 1.0,
        },
        "base_weights": {
            "path": str(base.relative_to(ROOT)),
            "sha256": sha256_file(base),
        },
        "run_dir": str(run_dir.relative_to(ROOT)),
        "weights": {
            "path": str(target.relative_to(ROOT)),
            "sha256": sha256_file(target),
            "size_bytes": target.stat().st_size,
        },
        "final_row": final,
        "smoke": bool(args.smoke),
    }
    name = "P0-DET_train_smoke.json" if args.smoke else "P0-DET_train.json"
    (RAW / name).write_text(json.dumps(summary, indent=1))
    print(f"trained {summary['config']['epochs']} epochs in {summary['duration_s']:.0f}s")
    print(f"weights {target} sha256={summary['weights']['sha256'][:16]}")
    if final:
        print("final row:", {k: v for k, v in final.items() if k.strip()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
