"""
plot_scalability.py
===================
Trace les courbes de scalabilité à partir de results/benchmark_scalability.csv

Usage :
    python3 tests/plot_scalability.py
    python3 tests/plot_scalability.py --csv results/benchmark_scalability.csv
                                      --out results/scalability.png
"""

import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")          # pas d'écran requis
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

# ─── lecture CSV ─────────────────────────────────────────────────────────────

def load_csv(path):
    import csv
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    return rows

# ─── helpers ─────────────────────────────────────────────────────────────────

def measured_mask(rows):
    """Retourne un masque booléen pour les lignes mesurées réellement."""
    return [bool(r["nobatch_measured"]) for r in rows]

# ─── figure principale ────────────────────────────────────────────────────────

def plot(rows, out_path):
    Ns        = np.array([r["N"]               for r in rows])
    ss_batch  = np.array([r["ss_batch"]         for r in rows])
    ss_nobatch= np.array([r["ss_nobatch"]        for r in rows])
    t_batch   = np.array([r["time_batch_ms"]     for r in rows])
    t_nobatch = np.array([r["time_nobatch_ms"]   for r in rows])
    gain_t    = np.array([r["gain_time"]         for r in rows])
    gain_ss   = np.array([r["gain_ss"]           for r in rows])
    measured  = measured_mask(rows)

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle(
        "Scalabilité du batching FHE — Détection de collisions 3D de drones",
        fontsize=13, fontweight="bold", y=1.02
    )

    colors = {
        "batch":   "#2196F3",   # bleu
        "nobatch": "#F44336",   # rouge
        "gain":    "#4CAF50",   # vert
    }

    # ── Graphe 1 : Scheme-switches ────────────────────────────────────────────
    ax = axes[0]
    ax.plot(Ns, ss_batch,   "o-", color=colors["batch"],   lw=2,
            markersize=6,   label="Avec batching  (O(1))")
    ax.plot(Ns, ss_nobatch, "s--", color=colors["nobatch"], lw=2,
            markersize=6,   label="Sans batching  (O(N))")
    # zone extrapolée en pointillés légers
    ext_mask = [not m for m in measured]
    if any(ext_mask):
        Ns_ext = Ns[ext_mask];  ss_e = ss_nobatch[ext_mask]
        ax.plot(Ns_ext, ss_e, "s:", color=colors["nobatch"], lw=1.5,
                alpha=0.55, label="Extrapolé")
    ax.set_xlabel("Nombre de segments voisins (N)", fontsize=11)
    ax.set_ylabel("Nombre de scheme-switches", fontsize=11)
    ax.set_title("Scheme-switches vs N", fontsize=11)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_xticks(Ns)
    # annotation valeur max
    ax.annotate(
        f"×{ss_nobatch[-1]/max(ss_batch[-1],1):.0f} SS de plus",
        xy=(Ns[-1], ss_nobatch[-1]),
        xytext=(Ns[-2], ss_nobatch[-1] * 0.7),
        arrowprops=dict(arrowstyle="->", color="gray"),
        fontsize=8, color="gray"
    )

    # ── Graphe 2 : Temps (ms) ─────────────────────────────────────────────────
    ax = axes[1]
    ax.plot(Ns, t_batch,   "o-",  color=colors["batch"],   lw=2,
            markersize=6,  label="Avec batching")
    ax.plot(Ns, t_nobatch, "s--", color=colors["nobatch"],  lw=2,
            markersize=6,  label="Sans batching")
    if any(ext_mask):
        ax.plot(Ns_ext, t_nobatch[ext_mask], "s:", color=colors["nobatch"],
                lw=1.5, alpha=0.55, label="Extrapolé")
    ax.set_xlabel("Nombre de segments voisins (N)", fontsize=11)
    ax.set_ylabel("Temps (ms)", fontsize=11)
    ax.set_title("Temps de calcul vs N", fontsize=11)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_xticks(Ns)

    # ── Graphe 3 : Gain (temps et SS) ────────────────────────────────────────
    ax = axes[2]
    ax.plot(Ns, gain_t,  "D-",  color=colors["gain"],    lw=2,
            markersize=6, label="Gain temps")
    ax.plot(Ns, gain_ss, "^--", color="#FF9800",          lw=2,
            markersize=6, label="Gain scheme-switches")
    ax.axhline(1.0, color="gray", lw=1, ls=":", label="Gain = 1 (pas d'avantage)")
    ax.set_xlabel("Nombre de segments voisins (N)", fontsize=11)
    ax.set_ylabel("Facteur de gain  (×)", fontsize=11)
    ax.set_title("Facteur de gain du batching vs N", fontsize=11)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_xticks(Ns)
    # annotation gain max
    if len(gain_ss) > 0:
        gmax = gain_ss[-1]
        ax.annotate(
            f"×{gmax:.1f} en SS",
            xy=(Ns[-1], gmax),
            xytext=(Ns[-2], gmax * 0.85),
            arrowprops=dict(arrowstyle="->", color="orange"),
            fontsize=9, color="darkorange", fontweight="bold"
        )

    # ── Tableau récapitulatif sous les graphes ────────────────────────────────
    col_labels = ["N", "SS batch", "SS no-batch", "Gain SS", "t batch (ms)", "t no-batch (ms)", "Gain t"]
    table_data = []
    for r in rows:
        meas = "" if r["nobatch_measured"] else "*"
        table_data.append([
            int(r["N"]),
            int(r["ss_batch"]),
            f"{int(r['ss_nobatch'])}{meas}",
            f"×{r['gain_ss']:.1f}",
            f"{r['time_batch_ms']:.1f}",
            f"{r['time_nobatch_ms']:.1f}{meas}",
            f"×{r['gain_time']:.2f}",
        ])

    fig2, ax2 = plt.subplots(figsize=(12, 2.5))
    ax2.axis("off")
    tbl = ax2.table(
        cellText=table_data,
        colLabels=col_labels,
        cellLoc="center",
        loc="center",
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(10)
    tbl.scale(1.2, 1.6)
    # colorier entête
    for j in range(len(col_labels)):
        tbl[0, j].set_facecolor("#1565C0")
        tbl[0, j].set_text_props(color="white", fontweight="bold")
    # colorier colonnes "batch" en bleu clair, "nobatch" en rouge clair
    for i in range(1, len(rows) + 1):
        tbl[i, 1].set_facecolor("#BBDEFB")
        tbl[i, 2].set_facecolor("#FFCDD2")
        tbl[i, 4].set_facecolor("#BBDEFB")
        tbl[i, 5].set_facecolor("#FFCDD2")
        tbl[i, 3].set_facecolor("#C8E6C9")
        tbl[i, 6].set_facecolor("#C8E6C9")
    ax2.set_title("* valeurs extrapolées à partir de N=1", fontsize=8,
                  loc="right", color="gray")

    # ── Sauvegarde ────────────────────────────────────────────────────────────
    fig.tight_layout()
    out_curves = out_path
    out_table  = out_path.replace(".png", "_table.png")

    fig.savefig(out_curves, dpi=150, bbox_inches="tight")
    fig2.savefig(out_table, dpi=150, bbox_inches="tight")

    print(f"Courbes  sauvegardées : {out_curves}")
    print(f"Tableau  sauvegardé   : {out_table}")

# ─── entrée ───────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", default="results/benchmark_scalability.csv")
    parser.add_argument("--out", default="results/scalability.png")
    args = parser.parse_args()

    if not os.path.exists(args.csv):
        print(f"[ERREUR] Fichier CSV introuvable : {args.csv}")
        print("Lancer d'abord : ./build/test_benchmark_scalability")
        sys.exit(1)

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    rows = load_csv(args.csv)
    print(f"[*] {len(rows)} lignes lues depuis {args.csv}")
    plot(rows, args.out)

if __name__ == "__main__":
    main()
