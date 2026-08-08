"""
Diagramm-Generator: Seitenverhältnis vs. FPS
=============================================
Liest benchmark_results.json und zeigt:

  Oben:   Gruppierte Balken – pro Pixelklasse alle Seitenverhältnisse
          → Vergleich: breit vs. hoch bei gleicher Pixelzahl
          → Vergleich: wie skaliert FPS mit Pixelzahl?

  Unten:  Zeitaufschlüsselung – Capture-Zeit vs. Verarbeitungszeit
          → Wo ist der Flaschenhals?

Verwendung:
    python plot_results.py
    python plot_results.py --input meine_daten.json
    python plot_results.py --output ergebnis.png --no-show
"""

import argparse
import json
from pathlib import Path
from itertools import groupby

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import numpy as np
except ImportError:
    print("❌ matplotlib ist nicht installiert.")
    print("   Installieren mit:  pip install matplotlib")
    raise SystemExit(1)

DEFAULT_INPUT  = Path(__file__).parent / "benchmark_results.json"
DEFAULT_OUTPUT = Path(__file__).parent / "benchmark_results.png"

# Farben je Seitenverhältnis – von breit (blau) bis hoch (orange)
ASPECT_COLORS = {
    "sehr breit":  "#4fc3f7",  # hellblau
    "breit":       "#42a5f5",  # blau
    "ausgewogen":  "#66bb6a",  # grün
    "hoch":        "#ef5350",  # rot-orange
    "sehr hoch":   "#ff7043",  # orange
}
FALLBACK_COLORS = ["#ab47bc", "#26a69a", "#ffa726", "#78909c"]


def parse_args():
    p = argparse.ArgumentParser(description="Diagramm: Seitenverhältnis vs. FPS")
    p.add_argument("--input",  default=str(DEFAULT_INPUT))
    p.add_argument("--output", default=str(DEFAULT_OUTPUT))
    p.add_argument("--no-show", action="store_true")
    return p.parse_args()


def load_data(path: str) -> tuple[list, list]:
    p = Path(path)
    if not p.exists():
        print(f"❌ Datei nicht gefunden: {p}")
        print("   Zuerst benchmark.py ausführen!")
        raise SystemExit(1)
    raw     = json.loads(p.read_text(encoding="utf-8"))
    results = raw.get("results", [])
    groups  = raw.get("groups", [])
    if not results:
        print("❌ Keine Ergebnisse in der Datei.")
        raise SystemExit(1)
    return results, groups


def group_by_group(results: list, group_names: list) -> dict:
    """Gibt {group_name: [result, ...]} zurück, sortiert nach group_names."""
    grouped = {}
    for name in group_names:
        grouped[name] = [r for r in results if r["group"] == name]
    # Gruppen ohne Treffer ignorieren
    return {k: v for k, v in grouped.items() if v}


def make_chart(results: list, group_names: list, output_path: str, show: bool):
    grouped = group_by_group(results, group_names)
    n_groups = len(grouped)

    # Alle vorkommenden Seitenverhältnisse (für Legende + Farbzuweisung)
    all_aspects = []
    seen = set()
    for r in results:
        a = r["aspect"]
        if a not in seen:
            all_aspects.append(a)
            seen.add(a)

    # Farbmap aufbauen
    color_map = {}
    for aspect in all_aspects:
        color_map[aspect] = ASPECT_COLORS.get(aspect, FALLBACK_COLORS[len(color_map) % len(FALLBACK_COLORS)])

    # ── Layout ────────────────────────────────────────────────────────────────
    fig, (ax1, ax2) = plt.subplots(
        nrows=2, figsize=(max(8, n_groups * 2.8 + 2), 10),
        facecolor="#1e1e1e"
    )
    fig.suptitle(
        "Seitenverhältnis vs. FPS  (gleiche Pixelzahl pro Gruppe)",
        color="white", fontsize=14, fontweight="bold", y=0.99
    )

    # Positionen der Gruppen-Cluster
    group_labels = list(grouped.keys())
    x_centers = np.arange(n_groups)

    for ax in (ax1, ax2):
        ax.set_facecolor("#2a2a2a")
        ax.tick_params(colors="white")
        ax.spines[:].set_color("#555")
        ax.grid(axis="y", color="#444", linestyle=":", linewidth=0.7)

    # ── Subplot 1: FPS ────────────────────────────────────────────────────────
    max_fps = 0
    bar_w   = 0.8 / max(max(len(v) for v in grouped.values()), 1)

    for gi, (gname, entries) in enumerate(grouped.items()):
        n = len(entries)
        offsets = np.linspace(-(n - 1) / 2, (n - 1) / 2, n) * bar_w

        for ei, (entry, offset) in enumerate(zip(entries, offsets)):
            color = color_map.get(entry["aspect"], "#aaa")
            bar = ax1.bar(
                gi + offset, entry["fps"], width=bar_w * 0.88,
                color=color, edgecolor="#111", linewidth=0.6
            )
            ax1.text(
                gi + offset, entry["fps"] + 0.3,
                f"{entry['fps']:.1f}",
                ha="center", va="bottom", color="white", fontsize=8, fontweight="bold"
            )
            # Auflösungs-Label unter dem Balken
            ax1.text(
                gi + offset, -1.5,
                entry["label"],
                ha="center", va="top", color="#aaa", fontsize=7,
                rotation=45
            )
            max_fps = max(max_fps, entry["fps"])

    ax1.axhline(30, color="#ffb74d", linestyle="--", linewidth=1.2, label="Ziel: 30 FPS")
    ax1.set_xticks(x_centers)
    ax1.set_xticklabels(group_labels, color="white", fontsize=10)
    ax1.set_ylabel("FPS", color="white", fontsize=11)
    ax1.set_title("Verarbeitungs-FPS je Pixelklasse und Seitenverhältnis",
                  color="#bbbbbb", fontsize=11)
    ax1.set_ylim(-5, max_fps * 1.2)
    ax1.set_xlim(-0.6, n_groups - 0.4)

    # Legende: Seitenverhältnisse
    patches = [mpatches.Patch(color=color_map[a], label=a) for a in all_aspects]
    ax1.legend(handles=patches, title="Seitenverhältnis",
               facecolor="#333", edgecolor="#555", labelcolor="white",
               title_color="white", fontsize=8, loc="upper right")
    legend1 = ax1.get_legend()
    if legend1:
        ax1.get_legend().get_title().set_color("white")

    # ── Subplot 2: Zeitaufschlüsselung ────────────────────────────────────────
    for gi, (gname, entries) in enumerate(grouped.items()):
        n = len(entries)
        offsets = np.linspace(-(n - 1) / 2, (n - 1) / 2, n) * bar_w

        for entry, offset in zip(entries, offsets):
            color = color_map.get(entry["aspect"], "#aaa")
            # Capture-Teil (dunklere Variante der Farbe)
            ax2.bar(gi + offset, entry["capture_ms"], width=bar_w * 0.88,
                    color=color, edgecolor="#111", linewidth=0.6, alpha=0.6,
                    label=None)
            # Prozess-Teil (volle Farbe oben drauf)
            ax2.bar(gi + offset, entry["process_ms"], width=bar_w * 0.88,
                    bottom=entry["capture_ms"],
                    color=color, edgecolor="#111", linewidth=0.6,
                    label=None)
            # Auflösungs-Label
            ax2.text(
                gi + offset, -1.5,
                entry["label"],
                ha="center", va="top", color="#aaa", fontsize=7, rotation=45
            )

    # Legende für Capture vs. Verarbeitung
    p_cap  = mpatches.Patch(color="#aaa", alpha=0.5, label="Capture (Kamera-Puffer)")
    p_proc = mpatches.Patch(color="#aaa",              label="Verarbeitung (HSV+Morphologie)")
    ax2.legend(handles=[p_cap, p_proc],
               facecolor="#333", edgecolor="#555", labelcolor="white",
               fontsize=8, loc="upper left")

    ax2.set_xticks(x_centers)
    ax2.set_xticklabels(group_labels, color="white", fontsize=10)
    ax2.set_ylabel("Zeit pro Frame (ms)", color="white", fontsize=11)
    ax2.set_title("Zeitaufschlüsselung: Kamera-Puffer vs. Verarbeitung",
                  color="#bbbbbb", fontsize=11)
    ax2.set_xlim(-0.6, n_groups - 0.4)
    ax2.set_ylim(-5, None)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.subplots_adjust(hspace=0.45)

    out = Path(output_path)
    plt.savefig(out, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"✅ Diagramm gespeichert: {out.resolve()}")

    if show:
        plt.show()
    plt.close()


def main():
    args    = parse_args()
    results, groups = load_data(args.input)

    print(f"  {len(results)} Messungen geladen aus: {args.input}")
    print()
    current = None
    for r in results:
        if r["group"] != current:
            current = r["group"]
            print(f"  ── {current}")
        print(f"     {r['label']:<12}  {r['aspect']:<12}  "
              f"{r['fps']:>5.1f} FPS  "
              f"(cap {r['capture_ms']:>5.1f} ms  +  proc {r['process_ms']:>5.1f} ms)")

    make_chart(results, groups, args.output, show=not args.no_show)


if __name__ == "__main__":
    main()
