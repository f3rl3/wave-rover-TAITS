"""
Wave Rover -- CPS Benchmark
============================
Measures Checks Per Second (CPS): how many times per second the full
camera-grab + path-detection loop can execute at a given resolution.

For every resolution tested, FRAME_WIDTH / FRAME_HEIGHT in config.py
are patched so the rover config stays in sync with what is being benchmarked.
After a full run the fastest resolution is written to config.py permanently.

Usage:
    python benchmark.py                          # all predefined groups
    python benchmark.py --frames 150             # 150 frames per resolution
    python benchmark.py --camera 1               # camera index 1
    python benchmark.py --width 640 --height 480 # one resolution + write to config
"""

from __future__ import annotations   # allow dict | None on Python 3.9

import argparse
import importlib
import json
import re
import time
from pathlib import Path

import cv2

# -- matplotlib: headless-safe (no display needed on Pi) ----------------------
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import matplotlib.ticker as ticker
    _HAS_MPL = True
except ImportError:
    _HAS_MPL = False
    print("[WARN] matplotlib not installed -- charts will be skipped")

# =============================================================================
#  Constants
# =============================================================================

CONFIG_FILE   = Path(__file__).parent / "config.py"
OUTPUT_FILE   = Path(__file__).parent / "benchmark_results.json"
CHART_DIR     = Path(__file__).parent
WARMUP_FRAMES = 20

# Resolution groups: equal pixel count, different aspect ratios.
# Question: does the aspect ratio affect CPS at equal pixel count?
RESOLUTION_GROUPS = [
    {
        "name": "~76K Pixel",
        "resolutions": [
            (480, 160),   # 3:1  -- very wide
            (320, 240),   # 4:3  -- classic
            (240, 320),   # 3:4  -- portrait
            (160, 480),   # 1:3  -- very tall
        ],
    },
    {
        "name": "~150K Pixel",
        "resolutions": [
            (640, 240),   # 8:3  -- very wide
            (480, 320),   # 3:2  -- wide
            (320, 480),   # 2:3  -- tall
            (240, 640),   # 3:8  -- very tall
        ],
    },
    {
        "name": "~300K Pixel",
        "resolutions": [
            (800, 384),   # ~2:1 -- wide
            (640, 480),   # 4:3  -- balanced
            (480, 640),   # 3:4  -- tall
            (384, 800),   # ~1:2 -- very tall
        ],
    },
]

# =============================================================================
#  Config patching
# =============================================================================

def _config_read_resolution() -> tuple[int, int]:
    """Read FRAME_WIDTH / FRAME_HEIGHT currently stored in config.py."""
    text = CONFIG_FILE.read_text(encoding="utf-8")
    w = int(re.search(r"^FRAME_WIDTH\s*=\s*(\d+)",  text, re.MULTILINE).group(1))
    h = int(re.search(r"^FRAME_HEIGHT\s*=\s*(\d+)", text, re.MULTILINE).group(1))
    return w, h


def _config_write_resolution(width: int, height: int) -> None:
    """Patch FRAME_WIDTH / FRAME_HEIGHT in config.py in-place."""
    text = CONFIG_FILE.read_text(encoding="utf-8")
    text = re.sub(r"^(FRAME_WIDTH\s*=\s*)\d+",  rf"\g<1>{width}",  text, flags=re.MULTILINE)
    text = re.sub(r"^(FRAME_HEIGHT\s*=\s*)\d+", rf"\g<1>{height}", text, flags=re.MULTILINE)
    CONFIG_FILE.write_text(text, encoding="utf-8")

# =============================================================================
#  Detector factory
# =============================================================================

def _make_detector():
    """
    Reload config and path_detector so PathDetector.__init__ reads the
    FRAME_WIDTH that was just written to config.py.
    """
    import config as _cfg
    importlib.reload(_cfg)
    import path_detector as _pd
    importlib.reload(_pd)
    return _pd.PathDetector()

# =============================================================================
#  Measurement
# =============================================================================

def _measure_one(cap: cv2.VideoCapture, width: int, height: int,
                 n_frames: int) -> dict | None:
    """
    Measure CPS for one resolution.
    Patches config.py, rebuilds PathDetector, sets camera, warms up, measures.
    """
    _config_write_resolution(width, height)
    detector = _make_detector()

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"  {width}x{height:<4}  actual {actual_w}x{actual_h}", end="", flush=True)

    durations: list[float] = []
    for i in range(WARMUP_FRAMES + n_frames):
        t0 = time.perf_counter()
        ret, frame = cap.read()
        if not ret or frame is None:
            print("  [no frame - skip]")
            return None
        detector.process(frame)
        elapsed = time.perf_counter() - t0
        if i >= WARMUP_FRAMES:
            durations.append(elapsed)

    if not durations:
        print("  [no data]")
        return None

    avg_s  = sum(durations) / len(durations)
    cps    = 1.0 / avg_s if avg_s > 0 else 0.0
    avg_ms = avg_s * 1000.0

    print(f"    avg {avg_ms:5.1f} ms/iter  ->  {cps:5.1f} CPS")

    return {
        "label":    f"{actual_w}x{actual_h}",
        "actual_w": actual_w,
        "actual_h": actual_h,
        "pixels":   actual_w * actual_h,
        "avg_ms":   round(avg_ms, 2),
        "cps":      round(cps,    1),
        "n_frames": len(durations),
    }

# =============================================================================
#  Chart
# =============================================================================

# Dark theme tokens (match robot debug dashboard)
_BG    = "#0f1520"
_SURF  = "#111825"
_GRID  = "#1e2a3c"
_TEXT  = "#c0cad6"   # primary text  -- never used on bars
_MUTED = "#6a7d91"   # axis labels

# Categorical palette -- first 3 slots of the validated default (dataviz skill).
# Dark-mode variants stepped up in lightness for contrast on _SURF.
# Blue / green / amber -- well-separated under protanopia + deuteranopia.
_C_WIDE = "#5599dd"   # wide  (ratio >= 1.3)   -- blue
_C_BAL  = "#2ecc71"   # balanced (0.75..1.3)   -- green (matches stripe)
_C_TALL = "#e08c3a"   # tall  (< 0.75)         -- amber


def _aspect_color(actual_w: int, actual_h: int) -> str:
    r = actual_w / actual_h
    if r >= 1.3:  return _C_WIDE
    if r >= 0.75: return _C_BAL
    return _C_TALL


def _plot_group(group_name: str, results: list[dict]) -> None:
    """Save a bar chart PNG for one resolution group."""
    if not _HAS_MPL or not results:
        return

    labels   = [r["label"] for r in results]
    cps_vals = [r["cps"]   for r in results]
    colors   = [_aspect_color(r["actual_w"], r["actual_h"]) for r in results]
    max_cps  = max(cps_vals) if cps_vals else 1.0

    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    fig.patch.set_facecolor(_BG)
    ax.set_facecolor(_SURF)

    # ── Bars: square at baseline, rounded cap ─────────────────────────────
    bar_w     = 0.50
    n         = len(results)
    positions = list(range(n))
    radius    = 0.04   # rounded cap, in data units

    for x, val, col in zip(positions, cps_vals, colors):
        draw_h = max(val, radius * 2)
        patch = mpatches.FancyBboxPatch(
            (x - bar_w / 2, 0),
            bar_w, draw_h,
            boxstyle=f"round,pad=0,rounding_size={radius}",
            linewidth=0,
            facecolor=col,
            clip_on=True,
        )
        ax.add_patch(patch)

    ax.set_xlim(-0.6, n - 0.4)
    ax.set_ylim(0, max_cps * 1.28)

    # ── Value labels (cap) ─────────────────────────────────────────────────
    for x, val in zip(positions, cps_vals):
        ax.text(
            x, val + max_cps * 0.028,
            f"{val:.0f}",
            ha="center", va="bottom",
            fontsize=9.5, fontweight="600",
            color=_TEXT,
            fontfamily="monospace",
        )

    # ── Axes & grid ────────────────────────────────────────────────────────
    ax.set_xticks(positions)
    ax.set_xticklabels(labels, fontsize=8.5, color=_MUTED)
    ax.tick_params(axis="y", colors=_MUTED, labelsize=8.0)
    ax.tick_params(axis="x", length=0)

    for spine in ax.spines.values():
        spine.set_visible(False)

    ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=True, nbins=5))
    ax.grid(axis="y", color=_GRID, linewidth=0.8, zorder=0)
    ax.set_axisbelow(True)

    # ── Titles & axis labels ───────────────────────────────────────────────
    ax.set_title(
        f"Checks per Second  --  {group_name}",
        fontsize=10.5, fontweight="600",
        color=_TEXT, pad=12,
    )
    ax.set_xlabel("Resolution  (width x height)", fontsize=8.5, color=_MUTED, labelpad=8)
    ax.set_ylabel("CPS  (checks / second)",       fontsize=8.5, color=_MUTED, labelpad=8)

    # ── Legend (3 series categories present) ──────────────────────────────
    legend_handles = [
        mpatches.Patch(color=_C_WIDE, label="wide  (>= 1.3 : 1)"),
        mpatches.Patch(color=_C_BAL,  label="balanced"),
        mpatches.Patch(color=_C_TALL, label="tall   (<  0.75 : 1)"),
    ]
    ax.legend(
        handles=legend_handles,
        fontsize=7.5, labelcolor=_TEXT,
        facecolor=_BG, edgecolor=_GRID, framealpha=0.95,
        loc="upper right",
    )

    plt.tight_layout(pad=1.2)

    safe = re.sub(r"[^\w]", "_", group_name)
    out  = CHART_DIR / f"benchmark_{safe}.png"
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=_BG)
    plt.close(fig)
    print(f"  Chart -> {out.name}")

# =============================================================================
#  CLI + main
# =============================================================================

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Wave Rover CPS Benchmark",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "examples:\n"
            "  python benchmark.py                        # full benchmark, all groups\n"
            "  python benchmark.py --frames 150\n"
            "  python benchmark.py --width 640 --height 480   # single + write config"
        ),
    )
    p.add_argument("--frames", type=int, default=100,
                   help="Measured frames per resolution (default: 100)")
    p.add_argument("--camera", type=int, default=0,
                   help="Camera index (default: 0)")
    p.add_argument("--width",  type=int, default=None,
                   help="Single-resolution width  (requires --height)")
    p.add_argument("--height", type=int, default=None,
                   help="Single-resolution height (requires --width)")
    return p.parse_args()


def main() -> None:
    args = _parse_args()

    if (args.width is None) != (args.height is None):
        print("[ERROR] --width and --height must be used together.")
        return

    single_mode = args.width is not None
    groups = (
        [{"name": "custom", "resolutions": [(args.width, args.height)]}]
        if single_mode
        else RESOLUTION_GROUPS
    )

    # -- Open camera ----------------------------------------------------------
    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("[ERROR] Camera could not be opened!")
        return
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    print("=" * 62)
    print("  Wave Rover -- CPS Benchmark")
    print("=" * 62)
    print(f"  Camera : {args.camera}  |  Frames/test : {args.frames}  |  Warmup : {WARMUP_FRAMES}")
    print()

    # -- Run measurements -----------------------------------------------------
    all_results:      list[dict]      = []
    results_by_group: dict[str, list] = {}
    best_cps = 0.0
    best_res = _config_read_resolution()

    for group in groups:
        gname = group["name"]
        print(f"-- {gname} " + "-" * max(0, 58 - len(gname)))
        group_results: list[dict] = []

        for w, h in group["resolutions"]:
            r = _measure_one(cap, w, h, args.frames)
            if r is None:
                continue
            r["group"] = gname
            all_results.append(r)
            group_results.append(r)
            if r["cps"] > best_cps:
                best_cps = r["cps"]
                best_res = (r["actual_w"], r["actual_h"])

        results_by_group[gname] = group_results
        print()

    cap.release()

    # -- Write resolution to config.py ----------------------------------------
    if single_mode:
        final_w, final_h = args.width, args.height
        label = "requested"
    else:
        final_w, final_h = best_res
        label = "fastest"

    _config_write_resolution(final_w, final_h)
    print(f"[config.py] FRAME_WIDTH={final_w}  FRAME_HEIGHT={final_h}  ({label}  {best_cps:.1f} CPS)")

    # -- Save JSON ------------------------------------------------------------
    OUTPUT_FILE.write_text(
        json.dumps(
            {"frames_per_test": args.frames, "warmup": WARMUP_FRAMES, "results": all_results},
            indent=2,
        ),
        encoding="utf-8",
    )
    print(f"[OK] Data  -> {OUTPUT_FILE.name}")

    # -- Charts (full mode only) ----------------------------------------------
    if not single_mode:
        print("\n-- Charts " + "-" * 52)
        for gname, results in results_by_group.items():
            _plot_group(gname, results)

    # -- Summary table --------------------------------------------------------
    if not all_results:
        return

    print("\n-- Summary " + "-" * 51)
    print(f"  {'Resolution':<14} {'Group':<18} {'CPS':>7}  {'ms/iter':>8}")
    print(f"  {'-'*14} {'-'*18} {'-'*7}  {'-'*8}")
    for r in sorted(all_results, key=lambda x: x["cps"], reverse=True):
        mark = "  <-- best" if (r["actual_w"], r["actual_h"]) == best_res else ""
        print(f"  {r['label']:<14} {r['group']:<18} {r['cps']:>7.1f}  {r['avg_ms']:>6.1f} ms{mark}")


if __name__ == "__main__":
    main()
