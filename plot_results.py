"""
Diagramm-Generator: Auflösung vs. Geschwindigkeit
==================================================
Liest benchmark_results.json (erstellt von benchmark.py) und
erzeugt ein Diagramm mit zwei Subplots:

  1. FPS je Auflösung (Balkendiagramm)
  2. Zeitaufschlüsselung: Capture-Zeit vs. Verarbeitungszeit (gestapelte Balken)

Das Bild wird als benchmark_results.png gespeichert und optional angezeigt.

Verwendung:
    python plot_results.py                      # liest benchmark_results.json
    python plot_results.py --input my_data.json
    python plot_results.py --no-show            # nur speichern, nicht anzeigen
    python plot_results.py --output mein_plot.png
"""

import argparse
import json
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    import numpy as np
except ImportError:
    print("❌ matplotlib ist nicht installiert.")
    print("   Installieren mit:  pip install matplotlib")
    raise SystemExit(1)

DEFAULT_INPUT  = Path(__file__).parent / "benchmark_results.json"
DEFAULT_OUTPUT = Path(__file__).parent / "benchmark_results.png"


def parse_args():
    p = argparse.ArgumentParser(description="Diagramm: Auflösung vs. FPS")
    p.add_argument("--input",  default=str(DEFAULT_INPUT),  help="Pfad zur JSON-Ergebnisdatei")
    p.add_argument("--output", default=str(DEFAULT_OUTPUT), help="Ausgabepfad für das Bild (.png)")
    p.add_argument("--no-show", action="store_true", help="Bild nicht anzeigen, nur speichern")
    return p.parse_args()


def load_results(path: str) -> list:
    p = Path(path)
    if not p.exists():
        print(f"❌ Datei nicht gefunden: {p}")
        print("   Zuerst benchmark.py ausführen!")
        raise SystemExit(1)
    data = json.loads(p.read_text(encoding="utf-8"))
    results = data.get("results", [])
    if not results:
        print("❌ Keine Messergebnisse in der Datei gefunden.")
        raise SystemExit(1)
    return results


def make_chart(results: list, output_path: str, show: bool):
    labels       = [r["label"]      for r in results]
    fps_vals     = [r["fps"]        for r in results]
    capture_ms   = [r["capture_ms"] for r in results]
    process_ms   = [r["process_ms"] for r in results]
    pixels       = [r["actual_w"] * r["actual_h"] for r in results]

    x = np.arange(len(labels))
    bar_w = 0.55

    fig, axes = plt.subplots(
        nrows=2, ncols=1,
        figsize=(max(7, len(labels) * 1.6), 9),
        facecolor="#1e1e1e",
    )
    fig.suptitle(
        "Kamera-Benchmark: Auflösung vs. Geschwindigkeit",
        color="white", fontsize=14, fontweight="bold", y=0.98
    )

    # ── Subplot 1: FPS ─────────────────────────────────────────────────────────
    ax1 = axes[0]
    ax1.set_facecolor("#2a2a2a")

    bars = ax1.bar(x, fps_vals, width=bar_w, color="#4fc3f7", edgecolor="#1e88e5", linewidth=0.8)

    # Wert über jedem Balken
    for bar, val in zip(bars, fps_vals):
        ax1.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.4,
            f"{val:.1f}",
            ha="center", va="bottom", color="white", fontsize=10, fontweight="bold"
        )

    # Ziel-FPS Linie (30 FPS)
    ax1.axhline(30, color="#ffb74d", linestyle="--", linewidth=1.2, label="Ziel: 30 FPS")
    ax1.legend(facecolor="#333", edgecolor="#555", labelcolor="white", fontsize=9)

    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, color="white", fontsize=10)
    ax1.set_ylabel("FPS", color="white", fontsize=11)
    ax1.set_title("Verarbeitungs-FPS je Auflösung", color="#bbbbbb", fontsize=11)
    ax1.tick_params(colors="white")
    ax1.spines[:].set_color("#555")
    ax1.set_ylim(0, max(fps_vals) * 1.25)
    ax1.yaxis.set_major_locator(ticker.MultipleLocator(5))
    ax1.grid(axis="y", color="#444", linestyle=":", linewidth=0.7)

    # Pixelanzahl als sekundäre Info unter den Labels
    pixel_labels = [f"{w*h//1000}K px" for r in results for w, h in [(r["actual_w"], r["actual_h"])]]
    for xi, pl in zip(x, pixel_labels):
        ax1.text(xi, -max(fps_vals) * 0.06, pl, ha="center", va="top",
                 color="#888", fontsize=8, transform=ax1.transData)

    # ── Subplot 2: Zeitaufschlüsselung ────────────────────────────────────────
    ax2 = axes[1]
    ax2.set_facecolor("#2a2a2a")

    b_cap = ax2.bar(x, capture_ms,  width=bar_w, label="Kamera (capture)",
                    color="#81c784", edgecolor="#43a047", linewidth=0.8)
    b_pro = ax2.bar(x, process_ms, width=bar_w, bottom=capture_ms,
                    label="Verarbeitung (HSV + Morphologie)",
                    color="#e57373", edgecolor="#c62828", linewidth=0.8)

    # Gesamtzeit und FPS über dem Stapel
    for xi, cap, proc, fps in zip(x, capture_ms, process_ms, fps_vals):
        total = cap + proc
        ax2.text(xi, total + 0.5, f"{fps:.1f} FPS",
                 ha="center", va="bottom", color="white", fontsize=9)

    ax2.set_xticks(x)
    ax2.set_xticklabels(labels, color="white", fontsize=10)
    ax2.set_ylabel("Zeit pro Frame (ms)", color="white", fontsize=11)
    ax2.set_title("Zeitaufschlüsselung: Capture vs. Verarbeitung", color="#bbbbbb", fontsize=11)
    ax2.tick_params(colors="white")
    ax2.spines[:].set_color("#555")
    ax2.grid(axis="y", color="#444", linestyle=":", linewidth=0.7)
    ax2.legend(facecolor="#333", edgecolor="#555", labelcolor="white", fontsize=9,
               loc="upper left")

    plt.tight_layout(rect=[0, 0, 1, 0.96])

    # Speichern
    out = Path(output_path)
    plt.savefig(out, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"✅ Diagramm gespeichert: {out.resolve()}")

    if show:
        plt.show()
    plt.close()


def main():
    args = parse_args()
    results = load_results(args.input)

    print(f"  {len(results)} Auflösungen geladen aus: {args.input}")
    for r in results:
        print(f"    {r['label']:<14}  {r['fps']:>6.1f} FPS  "
              f"(capture {r['capture_ms']:.1f} ms  +  process {r['process_ms']:.1f} ms)")

    make_chart(results, args.output, show=not args.no_show)


if __name__ == "__main__":
    main()
