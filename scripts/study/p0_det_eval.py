#!/usr/bin/env python3
"""P0-DET evaluation: mAP50 / mAP50-95 on the held-out validation and test scenes.

Uses the pre-registered split (scenes 21-24 validation, 25-30 test) and the
320 px YOLOv8n checkpoint written by ``p0_det_train.py``.  Reports the
Ultralytics detection metrics plus a per-image greedy TP/FP/FN table at a
fixed operating point (confidence 0.25, IoU 0.5), so the detector can be
audited frame by frame.

Writes:
    data/journal/raw/P0-DET_predictions.csv
    data/journal/raw/P0-DET_summary.json
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from aria_study.provenance import assert_clean, stamp  # noqa: E402

OUT = ROOT / "data" / "journal" / "raw" / "detector"
RAW = ROOT / "data" / "journal" / "raw"
MODELS = ROOT / "data" / "journal" / "models"


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_gt(label_path: Path, width: int, height: int) -> list[list[float]]:
    boxes = []
    if not label_path.exists():
        return boxes
    for line in label_path.read_text().splitlines():
        if not line.strip():
            continue
        cls, xc, yc, bw, bh = line.split()
        xc, yc, bw, bh = (float(v) for v in (xc, yc, bw, bh))
        boxes.append([0.0,
                      (xc - bw / 2) * width, (yc - bh / 2) * height,
                      (xc + bw / 2) * width, (yc + bh / 2) * height])
    return boxes


def iou(a, b) -> float:
    x0, y0 = max(a[1], b[1]), max(a[2], b[2])
    x1, y1 = min(a[3], b[3]), min(a[4], b[4])
    inter = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    area_a = max(0.0, a[3] - a[1]) * max(0.0, a[4] - a[2])
    area_b = max(0.0, b[3] - b[1]) * max(0.0, b[4] - b[2])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def match_image(preds, gts, iou_thr=0.5):
    """Greedy confidence-ordered matching; returns tp, fp, fn, matched ious."""
    order = np.argsort([-p[0] for p in preds]) if preds else []
    used = [False] * len(gts)
    tp = fp = 0
    ious = []
    for idx in order:
        p = preds[idx]
        best_j, best_iou = -1, 0.0
        for j, g in enumerate(gts):
            if used[j] or int(p[1]) != int(g[0]):
                continue
            v = iou([p[1], *p[2:6]], g)
            if v > best_iou:
                best_iou, best_j = v, j
        if best_j >= 0 and best_iou >= iou_thr:
            used[best_j] = True
            tp += 1
            ious.append(best_iou)
        else:
            fp += 1
    fn = sum(1 for u in used if not u)
    return tp, fp, fn, ious


def metric_dict(m) -> dict:
    box = m.box
    out = {
        "map50": float(box.map50),
        "map50_95": float(box.map),
        "map75": float(box.map75),
        "precision": float(box.mp),
        "recall": float(box.mr),
        "n_images": int(getattr(m, "seen", 0) or 0),
    }
    try:
        out["per_class"] = {
            str(k): {
                "map50": float(v[3]) if isinstance(v, (list, tuple)) else float(v),
            }
            for k, v in box.ap_class_index.items()
        }
    except Exception:  # noqa: BLE001 - optional detail
        pass
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=str(MODELS / "p0_det_yolov8n.pt"))
    ap.add_argument("--imgsz", type=int, default=320)
    ap.add_argument("--device", default="0")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=0.5)
    args = ap.parse_args()

    assert_clean("P0-DET evaluation")
    prov = stamp()

    weights = Path(args.weights)
    if not weights.exists():
        raise SystemExit(f"missing weights {weights}")
    train_json = RAW / "P0-DET_train.json"
    if not train_json.exists():
        raise SystemExit(f"missing {train_json}")
    train = json.loads(train_json.read_text())
    if train.get("weights", {}).get("sha256") != sha256_file(weights):
        raise SystemExit("checkpoint hash does not match the training record")

    import torch
    from ultralytics import YOLO

    if args.device != "cpu" and not torch.cuda.is_available():
        raise SystemExit("CUDA requested but unavailable")
    device_name = (torch.cuda.get_device_name(0) if torch.cuda.is_available()
                   else "cpu")

    started = time.time()
    model = YOLO(str(weights))
    data_yaml = str(OUT / "data.yaml")

    results = {}
    for split in ("val", "test"):
        m = model.val(data=data_yaml, split=split, imgsz=args.imgsz,
                      device=args.device, conf=0.001, iou=0.7,
                      verbose=False, plots=False)
        results[split] = metric_dict(m)
        results[split]["speed_ms"] = {
            k: float(v) for k, v in getattr(m, "speed", {}).items()
        }

    # Per-image operating-point audit on the test split.
    test_dir = OUT / "images" / "test"
    label_dir = OUT / "labels" / "test"
    rows = []
    pred_stream = model.predict(source=str(test_dir), imgsz=args.imgsz,
                                conf=args.conf, iou=args.iou, device=args.device,
                                stream=True, verbose=False)
    for r in pred_stream:
        img_path = Path(r.path)
        h, w = r.orig_shape
        gt = load_gt(label_dir / (img_path.stem + ".txt"), w, h)
        preds = []
        if r.boxes is not None and len(r.boxes) > 0:
            xyxy = r.boxes.xyxy.cpu().numpy()
            conf = r.boxes.conf.cpu().numpy()
            cls = r.boxes.cls.cpu().numpy()
            for box, c, k in zip(xyxy, conf, cls):
                preds.append([float(c), float(k), *[float(v) for v in box]])
        tp, fp, fn, ious = match_image(preds, gt, args.iou)
        rows.append({
            "frame_id": img_path.stem,
            "n_gt": len(gt),
            "n_pred": len(preds),
            "tp": tp,
            "fp": fp,
            "fn": fn,
            "mean_iou_tp": float(np.mean(ious)) if ious else "",
            "conf_thr": args.conf,
            "iou_thr": args.iou,
        })

    fields = ["frame_id", "n_gt", "n_pred", "tp", "fp", "fn", "mean_iou_tp",
              "conf_thr", "iou_thr"]
    with (RAW / "P0-DET_predictions.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    tp = sum(r["tp"] for r in rows)
    fp = sum(r["fp"] for r in rows)
    fn = sum(r["fn"] for r in rows)
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / (tp + fn) if tp + fn else 0.0
    f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0

    frames = {}
    with (RAW / "P0-DET_frames.csv").open(newline="") as f:
        for row in csv.DictReader(f):
            frames[row["split"]] = frames.get(row["split"], 0) + 1

    summary = {
        "exp_id": "P0-DET",
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(started)),
        "finished_at": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(time.time())),
        "commit": prov["commit"],
        "dirty": False,
        "dataset": {
            "frames": frames,
            "scenes": {"train": [1, 20], "val": [21, 24], "test": [25, 30]},
            "class": "workpiece",
        },
        "config": {
            "model": "yolov8n",
            "imgsz": args.imgsz,
            "epochs": train["config"].get("epochs", ""),
            "device": args.device,
            "device_name": device_name,
            "conf_map": 0.001,
            "iou_map": 0.7,
            "conf_operating": args.conf,
            "iou_operating": args.iou,
        },
        "weights": {
            "path": str(weights.relative_to(ROOT)),
            "sha256": sha256_file(weights),
            "size_bytes": weights.stat().st_size,
        },
        "train_record": str(train_json.relative_to(ROOT)),
        "val": results["val"],
        "test": results["test"],
        "operating_point": {
            "split": "test",
            "tp": tp,
            "fp": fp,
            "fn": fn,
            "precision": precision,
            "recall": recall,
            "f1": f1,
            "n_frames": len(rows),
        },
        "shortfall": {
            "requested_frames": 5000,
            "produced_frames": sum(frames.values()),
            "protocol_train_frames": frames.get("train", 0),
            "note": ("Pre-registration reduced Phase 0f to 1,000 auto-labelled "
                     "Gazebo frames; the 5,000-frame request is not met."),
        },
    }
    (RAW / "P0-DET_summary.json").write_text(json.dumps(summary, indent=1))
    print(f"val  mAP50={results['val']['map50']:.4f} mAP50-95={results['val']['map50_95']:.4f}")
    print(f"test mAP50={results['test']['map50']:.4f} mAP50-95={results['test']['map50_95']:.4f}")
    print(f"test op point P={precision:.4f} R={recall:.4f} F1={f1:.4f} "
          f"({len(rows)} frames)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
