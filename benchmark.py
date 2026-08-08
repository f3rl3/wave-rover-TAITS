"""
Kamera-Benchmark: Seitenverhältnis vs. Verarbeitungsgeschwindigkeit
====================================================================
Testet Auflösungen mit gleicher Pixelfläche aber unterschiedlichem
Seitenverhältnis (z.B. 640×240 vs. 480×320 vs. 320×480).

Fragestellung:
  Beeinflusst das Seitenverhältnis die FPS-Leistung bei gleicher Pixelzahl?
  Hilft ein breiteres Bild (mehr links/rechts) oder höheres Bild (mehr vorne/hinten)?

Ergebnisse → benchmark_results.json
Diagramm   → python plot_results.py

Verwendung:
    python benchmark.py                  # alle vordefinierten Gruppen
    python benchmark.py --frames 150     # 150 Frames pro Auflösung
    python benchmark.py --camera 1       # Kamera-Index 1
"""

import argparse
import json
import time
import cv2
from pathlib import Path

from path_detector import PathDetector

# ── Auflösungsgruppen: gleiche Pixelfläche, verschiedene Seitenverhältnisse ───
#
# Drei Pixelklassen, jeweils 4 Seitenverhältnisse:
#   Breit   = mehr links/rechts Sichtfeld (besser fürs Lenken?)
#   Mittel  = ausgewogen
#   Hoch    = mehr vorne/hinten Sichtfeld (besser für Kurven?)
#   Sehr hoch = extremes Hochformat
#
RESOLUTION_GROUPS = [
    {
        "name":  "~76 K Pixel",
        "resolutions": [
            (480, 160),   # 3:1  – sehr breit
            (320, 240),   # 4:3  – klassisch breit
            (240, 320),   # 3:4  – leicht hoch
            (160, 480),   # 1:3  – sehr hoch
        ],
    },
    {
        "name": "~150 K Pixel",
        "resolutions": [
            (640, 240),   # 8:3  – sehr breit
            (480, 320),   # 3:2  – breit
            (320, 480),   # 2:3  – hoch
            (240, 640),   # 3:8  – sehr hoch
        ],
    },
    {
        "name": "~300 K Pixel",
        "resolutions": [
            (800, 384),   # ~2:1 – breit
            (640, 480),   # 4:3  – ausgewogen
            (480, 640),   # 3:4  – hoch
            (384, 800),   # ~1:2 – sehr hoch
        ],
    },
]

TARGET_FPS    = 30
WARMUP_FRAMES = 20
OUTPUT_FILE   = Path(__file__).parent / "benchmark_results.json"


def parse_args():
    p = argparse.ArgumentParser(description="Kamera-Benchmark: Seitenverhältnis vs. FPS")
    p.add_argument("--frames", type=int, default=100,
                   help="Frames pro Auflösung (Standard: 100)")
    p.add_argument("--camera", type=int, default=0,
                   help="Kamera-Index (Standard: 0)")
    return p.parse_args()


def aspect_label(w: int, h: int) -> str:
    """Gibt 'Breit' / 'Ausgewogen' / 'Hoch' zurück je nach Seitenverhältnis."""
    ratio = w / h
    if ratio >= 2.5:   return "sehr breit"
    if ratio >= 1.4:   return "breit"
    if ratio >= 0.85:  return "ausgewogen"
    if ratio >= 0.45:  return "hoch"
    return "sehr hoch"


def benchmark_one(cap: cv2.VideoCapture, detector: PathDetector,
                  width: int, height: int, n_frames: int,
                  group_name: str) -> dict | None:
    """Misst eine einzelne Auflösung."""

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    ratio    = aspect_label(actual_w, actual_h)

    print(f"  {width}x{height:>4}  ->  tatsaechlich {actual_w}x{actual_h}  [{ratio}]")

    capture_ms_list = []
    process_ms_list = []

    for i in range(WARMUP_FRAMES + n_frames):
        t0 = time.perf_counter()
        ret, frame = cap.read()
        t1 = time.perf_counter()

        if not ret or frame is None:
            print(f"    [!] Kein Frame - ueberspringe diese Aufloesung")
            return None

        detector.process(frame)
        t2 = time.perf_counter()

        if i >= WARMUP_FRAMES:
            capture_ms_list.append((t1 - t0) * 1000)
            process_ms_list.append((t2 - t1) * 1000)

    n = len(capture_ms_list)
    if n == 0:
        return None

    avg_cap  = sum(capture_ms_list) / n
    avg_proc = sum(process_ms_list) / n
    avg_tot  = avg_cap + avg_proc
    fps      = 1000.0 / avg_tot if avg_tot > 0 else 0

    print(f"    capture {avg_cap:5.1f} ms  +  process {avg_proc:5.1f} ms  "
          f"=  {avg_tot:5.1f} ms/Frame  ->  {fps:.1f} FPS")

    return {
        "group":        group_name,
        "requested_w":  width,
        "requested_h":  height,
        "actual_w":     actual_w,
        "actual_h":     actual_h,
        "pixels":       actual_w * actual_h,
        "label":        f"{actual_w}×{actual_h}",
        "aspect":       ratio,
        "aspect_ratio": round(actual_w / actual_h, 3),
        "n_frames":     n,
        "capture_ms":   round(avg_cap,  2),
        "process_ms":   round(avg_proc, 2),
        "total_ms":     round(avg_tot,  2),
        "fps":          round(fps, 2),
        "capture_min":  round(min(capture_ms_list), 2),
        "capture_max":  round(max(capture_ms_list), 2),
        "process_min":  round(min(process_ms_list), 2),
        "process_max":  round(max(process_ms_list), 2),
    }


def main():
    args = parse_args()

    print("=" * 60)
    print("  Benchmark: Seitenverhältnis vs. FPS (gleiche Pixelzahl)")
    print("=" * 60)
    print(f"  Kamera : {args.camera}  |  Frames/Test : {args.frames}")
    print(f"  Ziel-FPS : {TARGET_FPS}  |  Warmup : {WARMUP_FRAMES} Frames")

    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("[FEHLER] Kamera konnte nicht geoeffnet werden!")
        return

    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    detector = PathDetector()
    all_results = []

    for group in RESOLUTION_GROUPS:
        print(f"\n-- Gruppe: {group['name']} ------------------------------")
        for w, h in group["resolutions"]:
            r = benchmark_one(cap, detector, w, h, args.frames, group["name"])
            if r:
                all_results.append(r)

    cap.release()

    output = {
        "target_fps":      TARGET_FPS,
        "frames_per_test": args.frames,
        "warmup_frames":   WARMUP_FRAMES,
        "groups":          [g["name"] for g in RESOLUTION_GROUPS],
        "results":         all_results,
    }
    OUTPUT_FILE.write_text(json.dumps(output, indent=2), encoding="utf-8")

    print(f"\n[OK] Ergebnisse gespeichert: {OUTPUT_FILE}")
    print("   Diagramm erstellen:  python plot_results.py")

    # Zusammenfassung pro Gruppe
    print("\n-- Zusammenfassung ------------------------------------------")
    current_group = None
    for r in all_results:
        if r["group"] != current_group:
            current_group = r["group"]
            print(f"\n  {current_group}")
            print(f"    {'Auflösung':<12} {'Verhältnis':<12} {'FPS':>6}  "
                  f"{'Capture':>9}  {'Prozess':>9}")
            print(f"    {'-'*12} {'-'*12} {'-'*6}  {'-'*9}  {'-'*9}")
        print(f"    {r['label']:<12} {r['aspect']:<12} {r['fps']:>6.1f}  "
              f"{r['capture_ms']:>7.1f} ms  {r['process_ms']:>7.1f} ms")


if __name__ == "__main__":
    main()
