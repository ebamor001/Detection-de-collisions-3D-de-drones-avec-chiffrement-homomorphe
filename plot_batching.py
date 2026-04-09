#!/usr/bin/env python3
"""
plot_batching.py — Courbes comparant le batching FHE vs sans batching.
Lance depuis la racine du projet :
    python3 plot_batching.py
Génère : batching_analysis.png
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import os

# ── Paramètres du modèle (tirés de l'implémentation réelle) ──────────────────
SS_MS        = 820.0    # ms par scheme-switch (mesuré sur CPU)
KEYGEN_MS    = 45_000.0 # ms pour KeyGen + setup scheme switching
SS_PER_PAIR  = 9        # scheme-switches par paire SANS batching
SS_BATCH     = 6        # scheme-switches TOTAL AVEC batching (constant)

N_values = np.arange(1, 51)  # de 1 à 50 paires

# ── Calculs ───────────────────────────────────────────────────────────────────
ss_nobatch  = N_values * SS_PER_PAIR          # linéaire
ss_batch    = np.full_like(N_values, SS_BATCH) # constant = 6

time_nobatch_s = (KEYGEN_MS + ss_nobatch  * SS_MS) / 1000
time_batch_s   = (KEYGEN_MS + ss_batch    * SS_MS) / 1000
gain_ss        = ss_nobatch  / ss_batch
gain_time      = time_nobatch_s / time_batch_s

# ── Figure ────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(16, 12))
fig.patch.set_facecolor('#0f0f1a')
gs  = gridspec.GridSpec(2, 2, figure=fig, hspace=0.45, wspace=0.35)

COLORS = {
    'batch':   '#00d4ff',
    'nobatch': '#ff4757',
    'gain':    '#2ed573',
    'time_b':  '#00d4ff',
    'time_nb': '#ff4757',
    'grid':    '#2a2a3e',
    'text':    '#e0e0e0',
    'bg':      '#1a1a2e',
    'title':   '#ffffff',
}

def style_ax(ax, title, xlabel, ylabel):
    ax.set_facecolor(COLORS['bg'])
    ax.set_title(title, color=COLORS['title'], fontsize=13, fontweight='bold', pad=10)
    ax.set_xlabel(xlabel, color=COLORS['text'], fontsize=10)
    ax.set_ylabel(ylabel, color=COLORS['text'], fontsize=10)
    ax.tick_params(colors=COLORS['text'])
    ax.spines[:].set_color(COLORS['grid'])
    ax.grid(True, color=COLORS['grid'], linestyle='--', alpha=0.5)
    for spine in ax.spines.values():
        spine.set_linewidth(0.5)

# ── Graphique 1 : Scheme-switches vs N ───────────────────────────────────────
ax1 = fig.add_subplot(gs[0, 0])
style_ax(ax1, "Scheme-switches vs Nombre de paires", "Paires de segments (N)", "Scheme-switches")
ax1.plot(N_values, ss_nobatch, color=COLORS['nobatch'], linewidth=2.5,
         label=f"Sans batching  (N × {SS_PER_PAIR})", marker='o', markersize=3)
ax1.plot(N_values, ss_batch,   color=COLORS['batch'],   linewidth=2.5,
         label=f"Avec batching  (constant = {SS_BATCH})", linestyle='--', marker='s', markersize=3)
ax1.fill_between(N_values, ss_batch, ss_nobatch, alpha=0.15, color=COLORS['gain'],
                 label="Économie de SS")
ax1.legend(facecolor='#1a1a2e', edgecolor=COLORS['grid'], labelcolor=COLORS['text'], fontsize=9)
ax1.annotate(f"N=16 : {16*SS_PER_PAIR} SS\nvs {SS_BATCH} SS",
             xy=(16, 16*SS_PER_PAIR), xytext=(25, 100),
             color=COLORS['text'], fontsize=8,
             arrowprops=dict(arrowstyle='->', color=COLORS['nobatch']))

# ── Graphique 2 : Temps estimé vs N ──────────────────────────────────────────
ax2 = fig.add_subplot(gs[0, 1])
style_ax(ax2, "Temps estimé vs Nombre de paires", "Paires de segments (N)", "Temps (secondes)")
ax2.plot(N_values, time_nobatch_s, color=COLORS['time_nb'], linewidth=2.5,
         label="Sans batching")
ax2.plot(N_values, time_batch_s,   color=COLORS['time_b'],   linewidth=2.5,
         linestyle='--', label="Avec batching")
ax2.fill_between(N_values, time_batch_s, time_nobatch_s, alpha=0.15, color=COLORS['gain'],
                 label="Temps économisé")
ax2.axhline(y=KEYGEN_MS/1000, color='#ffa502', linewidth=1, linestyle=':',
            label=f"KeyGen seul ({KEYGEN_MS/1000:.0f}s)")
ax2.legend(facecolor='#1a1a2e', edgecolor=COLORS['grid'], labelcolor=COLORS['text'], fontsize=9)

# ── Graphique 3 : Gain en SS ──────────────────────────────────────────────────
ax3 = fig.add_subplot(gs[1, 0])
style_ax(ax3, "Gain en Scheme-switches (×)", "Paires de segments (N)", "Gain (×)")
ax3.plot(N_values, gain_ss, color=COLORS['gain'], linewidth=2.5)
ax3.fill_between(N_values, 1, gain_ss, alpha=0.2, color=COLORS['gain'])
for n in [4, 10, 16, 25, 50]:
    if n <= N_values[-1]:
        g = int(n * SS_PER_PAIR / SS_BATCH)
        ax3.annotate(f"×{g}", xy=(n, g), xytext=(n+1, g+1),
                     color=COLORS['text'], fontsize=8,
                     arrowprops=dict(arrowstyle='->', color=COLORS['gain'], lw=0.8))
ax3.axhline(y=1, color=COLORS['grid'], linewidth=1, linestyle=':')

# ── Graphique 4 : Bar chart pour notre test (N=16) ───────────────────────────
ax4 = fig.add_subplot(gs[1, 1])
style_ax(ax4, "Notre test : 4 seg × 4 seg = 16 paires", "", "Scheme-switches")
categories = ['Sans batching\n(144 SS)', 'Avec batching\n(6 SS)']
values      = [16 * SS_PER_PAIR, SS_BATCH]
colors      = [COLORS['nobatch'], COLORS['batch']]
bars = ax4.bar(categories, values, color=colors, width=0.45, edgecolor='white', linewidth=0.5)
for bar, val in zip(bars, values):
    ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
             str(int(val)), ha='center', va='bottom',
             color=COLORS['title'], fontweight='bold', fontsize=14)
ax4.set_ylim(0, max(values) * 1.2)
gain_label = f"Gain : ×{int(values[0]/values[1])}"
ax4.text(0.5, 0.92, gain_label, transform=ax4.transAxes,
         ha='center', color=COLORS['gain'], fontsize=14, fontweight='bold')

# ── Titre global ──────────────────────────────────────────────────────────────
fig.suptitle(
    "Analyse du Batching FHE — Détection de Collisions 3D de Drones\n"
    "CKKS + FHEW Scheme Switching (OpenFHE)",
    color=COLORS['title'], fontsize=14, fontweight='bold', y=0.98
)

# ── Sauvegarde ────────────────────────────────────────────────────────────────
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "batching_analysis.png")
plt.savefig(out, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
print(f"Courbes sauvegardees : {out}")
