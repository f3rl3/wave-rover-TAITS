"""
Kamera-Benchmark: Auflösung vs. Geschwindigkeit
================================================
Testet verschiedene Auflösungen und misst:
  - Kamera-FPS      (wie schnell Frames geliefert werden)
  - Verarbeitungs-ms (wie lange detector.process() dauert)
  - Gesamt-FPS      (was der Main-Loop tatsächlich schafft)

Ergebnisse werden in benchmark_results.json gespeichert.
plot_results.py liest diese Datei und erstellt die Diagramme.

Verwendung:
    python benchmark.py                    # alle vordefinierten Auflösungen
    python benchmark.py --frames 200       # 200 Frames pro Test
    python benchmark.py --res 640x480 1280x720   # nur bestimmte Auflösungen
"""

import argparse
import json
import time
import cv2
import numpy as np
from pathlib import Path

# Pfad-Detektor aus dem Hauptprojekt importieren
from path_detector import PathDetector

# ── Standardauflösungen ───────────────────────────────────────────────────────
DEFAULT_RESOLUTIONS = [
    (320,  240),
    (640,  480),
    (800,  600),
    (1280, 720),
    (1920, 1080),
]

OUTPUT_FILE = Path(__file__).parent / "benchmark_results.json"
WARMUP_FRAMES = 15   # wird gemessen aber nicht gewertet


def parse_args():
    p = argparse.ArgumentParser(description="Kamera-Benchmark Auflösung vs. FPS")
    p.add_argument(
        "--res", nargs="+", metavar="WxH",
        help="Auflösungen zum Testen, z.B. 640x480 1280x720"
    )
    p.add_argument(
        "--frames", type=int, default=100,
        help="Anzahl Frames pro Auflösung (Standard: 100)"
    )
    p.add_argument(
        "--camera", type=int, default=0,
        help="Kamera-Index (Standard: 0)"
    )
    return p.parse_args()


def parse_resolution(s: str):
    try:
        w, h = s.lower().split("x")
        return int(w), int(h)
    except ValueError:
        raise argparse.ArgumentTypeError(f"Ungültiges Format '{s}', erwartet: WxH")


def benchmark_resolution(cap: cv2.VideoCapture, detector: PathDetector,
                          width: int, height: int, n_frames: int) -> dict:
    """Misst Kamera- und Verarbeitungszeit für eine Auflösung."""

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # Tatsächlich gesetzte Auflösung abfragen (Kamera rundet ggf. auf nächste)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"\n  Auflösung: {width}×{height}  →  tatsächlich: {actual_w}×{actual_h}")

    capture_times   = []
    process_times   = []
    total_times     = []

    frames_done = 0
    phase = "Aufwärmen"

    for i in range(WARMUP_FRAMES + n_frames):
        if i == WARMUP_FRAMES:
            phase = "Messen"
            print(f"  {phase}... ", end="", flush=True)

        t0 = time.perf_counter()
        ret, frame = cap.read()
        t1 = time.perf_counter()

        if not ret or frame is None:
            print(f"  ⚠ Kein Frame bei {width}×{height} – überspringe")
            continue

        _, _ = detector.process(frame)
        t2 = time.perf_counter()

        if i >= WARMUP_FRAMES:
            capture_times.append((t1 - t0) * 1000)
            process_times.append((t2 - t1) * 1000)
            total_times.append((t2 - t0) * 1000)
            frames_done += 1

    if frames_done == 0:
        return None

    avg_capture = sum(capture_times) / frames_done
    avg_process = sum(process_times) / frames_done
    avg_total   = sum(total_times)   / frames_done
    fps         = 1000.0 / avg_total if avg_total > 0 else 0

    print(f"fertig ({frames_done} Frames)")
    print(f"    Capture:      {avg_capture:6.1f} ms/Frame")
    print(f"    Verarbeitung: {avg_process:6.1f} ms/Frame")
    print(f"    Gesamt:       {avg_total:6.1f} ms/Frame  →  {fps:.1f} FPS")

    return {
        "requested_w":  width,
        "requested_h":  height,
        "actual_w":     actual_w,
        "actual_h":     actual_h,
        "label":        f"{actual_w}×{actual_h}",
        "n_frames":     frames_done,
        "capture_ms":   round(avg_capture, 2),
        "process_ms":   round(avg_process, 2),
        "total_ms":     round(avg_total, 2),
        "fps":          round(fps, 2),
        "capture_min":  round(min(capture_times), 2),
        "capture_max":  round(max(capture_times), 2),
        "process_min":  round(min(process_times), 2),
        "process_max":  round(max(process_times), 2),
    }


def main():
    args = parse_args()

    # Auflösungen bestimmen
    if args.res:
        resolutions = [parse_resolution(r) for r in args.res]
    else:
        resolutions = DEFAULT_RESOLUTIONS

    print("=" * 55)
    print("  Kamera-Benchmark: Auflösung vs. Geschwindigkeit")
    print("=" * 55)
    print(f"  Kamera-Index : {args.camera}")
    print(f"  Frames/Test  : {args.frames}")
    print(f"  Auflösungen  : {resolutions}")

    # Kamera öffnen
    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("❌ Kamera konnte nicht geöffnet werden!")
        return

    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    detector = PathDetector()
    results = []

    for w, h in resolutions:
        r = benchmark_resolution(cap, detector, w, h, args.frames)
        if r:
            results.append(r)

    cap.release()

    # Ergebnisse speichern
    output = {
        "camera_index": args.camera,
        "frames_per_test": args.frames,
        "warmup_frames": WARMUP_FRAMES,
        "results": results,
    }
    OUTPUT_FILE.write_text(json.dumps(output, indent=2), encoding="utf-8")

    print(f"\n✅ Ergebnisse gespeichert: {OUTPUT_FILE}")
    print("   Diagramme erstellen mit:  python plot_results.py")

    # Kurze Zusammenfassung
    print("\n── Zusammenfassung ──────────────────────────────")
    print(f"  {'Auflösung':<14} {'FPS':>6}  {'Verarb. ms':>12}")
    print(f"  {'-'*14} {'-'*6}  {'-'*12}")
    for r in results:
        print(f"  {r['label']:<14} {r['fps']:>6.1f}  {r['process_ms']:>10.1f} ms")


if __name__ == "__main__":
    main()
