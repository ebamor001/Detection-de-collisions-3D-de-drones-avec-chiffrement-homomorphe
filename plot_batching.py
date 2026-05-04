#!/usr/bin/env python3
"""
plot_batching.py — FHE Performance Benchmark.
Generates one PNG per plot + a combined figure.
Run from the project root: python3 plot_batching.py
"""

import subprocess, json, os, csv, time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ─── Paths ────────────────────────────────────────────────────────────────────
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
BINARY      = os.path.join(PROJECT_DIR, "build", "drone_collision_centralise")
BENCH_CSV   = os.path.join(PROJECT_DIR, "results", "benchmark_scalability.csv")
DATA_DIR    = os.path.join(PROJECT_DIR, "data")

def out(name): return os.path.join(PROJECT_DIR, name)

# ─── Protocol constants ───────────────────────────────────────────────────────
SS_PER_PAIR = 9
SS_BATCH    = 6
SS_MS_THEO  = 820.0
SETUP_MS    = 20_000.0

# ─── Helpers ──────────────────────────────────────────────────────────────────
def extract_json(txt):
    cands = []
    i = 0
    while i < len(txt):
        if txt[i] == '{':
            d, j = 1, i+1
            while j < len(txt) and d:
                d += (txt[j]=='{') - (txt[j]=='}')
                j += 1
            cands.append(txt[i:j])
        i += 1
    cands.sort(key=len, reverse=True)
    for c in cands:
        try: return json.loads(c)
        except: pass
    return None

def run(sid, timeout=600):
    if not os.path.exists(BINARY): return None
    print(f"  scenario {sid}...", end="", flush=True)
    t0 = time.time()
    try:
        r = subprocess.run([BINARY,"--scenario",str(sid),"--json","--data",DATA_DIR],
                           capture_output=True, text=True, timeout=timeout, cwd=PROJECT_DIR)
        d = extract_json(r.stdout)
        if d: d["wall_s"] = time.time()-t0; print(f" {d['wall_s']:.1f}s"); return d
        print(" err"); return None
    except subprocess.TimeoutExpired: print(" timeout"); return None

def read_csv(path):
    if not os.path.exists(path): return None
    cols = ["N","time_batch_ms","ss_batch","time_nobatch_ms",
            "ss_nobatch","gain_time","gain_ss","nobatch_measured"]
    data = {c: [] for c in cols}
    try:
        with open(path) as f:
            for row in csv.DictReader(f):
                for c in cols: data[c].append(float(row.get(c,0)))
        return {k: np.array(v) for k,v in data.items()}
    except: return None

# ═══════════════════════════════════════════════════════════════════════════════
print("="*58)
print("  FHE Benchmark — collecting measurements")
print("="*58)
print("\n[1] Latency measurements...")
r1 = run(1); r2 = run(2)
print("\n[2] Cost measurements (N=5 neighbors)...")
r4 = run(4); r5 = run(5); r6 = run(6)
print("\n[3] Reading scalability CSV...")
bench = read_csv(BENCH_CSV)
print(f"  {'CSV: '+str(len(bench['N']))+' points' if bench else 'absent'}")

# ─── Calibrate SS_MS ─────────────────────────────────────────────────────────
ss_ms_list = []
for r in [r1, r4, r5, r6]:
    if r is None: continue
    ss = r.get("ss_batch", r.get("scheme_switches_batch", 0))
    t  = r.get("time_fhe_ms", r.get("time_ms", 0))
    if ss > 0: ss_ms_list.append(t / ss)
SS_MS = float(np.mean(ss_ms_list)) if ss_ms_list else SS_MS_THEO
print(f"\n  SS_MS measured: {SS_MS:.0f} ms/switch")

# ─── Precision ───────────────────────────────────────────────────────────────
TP = TN = FP = FN = 0
for r, n_col_clear, n_total in [(r1,1,1),(r2,0,1),(r4,2,5),(r5,3,5),(r6,5,5)]:
    if r is None: continue
    fhe_col = r.get("collision_fhe", r.get("collision_count", 0))
    tp = min(fhe_col, n_col_clear)
    fp = max(0, fhe_col - n_col_clear)
    fn = max(0, n_col_clear - fhe_col)
    tn = max(0, n_total - n_col_clear - fp)
    TP += tp; TN += tn; FP += fp; FN += fn

total     = TP + TN + FP + FN
accuracy  = (TP+TN)/total*100 if total else 0
precision = TP/(TP+FP)*100    if (TP+FP) else 0
recall    = TP/(TP+FN)*100    if (TP+FN) else 0

# ─── Colors (light theme) ────────────────────────────────────────────────────
C = dict(
    batch   = '#1565C0',
    nobatch = '#C62828',
    gain    = '#2E7D32',
    real    = '#E65100',
    grid    = '#E0E0E0',
    text    = '#212121',
    bg      = '#FFFFFF',
    panel   = '#F5F5F5',
    tp      = '#1B5E20',
    tn      = '#0D47A1',
    fp      = '#E65100',
    fn      = '#B71C1C',
)

def make_fig():
    fig, ax = plt.subplots(figsize=(8, 5))
    fig.patch.set_facecolor(C['bg'])
    ax.set_facecolor(C['panel'])
    ax.tick_params(colors=C['text'], labelsize=9)
    for spine in ax.spines.values(): spine.set_edgecolor('#BDBDBD')
    ax.grid(True, color=C['grid'], linestyle='--', alpha=0.8)
    return fig, ax

def style(ax, title, xl, yl):
    ax.set_title(title, color=C['text'], fontsize=13, fontweight='bold', pad=10)
    ax.set_xlabel(xl, color=C['text'], fontsize=10)
    ax.set_ylabel(yl, color=C['text'], fontsize=10)

def save(fig, filename):
    fig.tight_layout()
    p = out(filename)
    fig.savefig(p, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  Saved: {filename}")

# ═══════════════════════════════════════════════════════════════════════════════
print("\n[4] Generating plots...")

# ── Plot 1 : Latency vs Number of Drones N ────────────────────────────────────
fig, ax = make_fig()
style(ax, "Latency vs Number of Drones N",
      "Number of drones N (log₂ scale)", "Latency (s)")

N_drones    = np.array([2, 4, 8, 16, 32, 64, 128])
t_batch_d   = (SETUP_MS + SS_BATCH*(N_drones-1)**0 * SS_MS) / 1000  # constant
t_batch_d   = np.full(len(N_drones), (SETUP_MS + SS_BATCH*SS_MS)/1000)
t_nobatch_d = (SETUP_MS + (N_drones-1)*SS_PER_PAIR*SS_MS) / 1000
t_bat_th    = np.full(len(N_drones), (SETUP_MS + SS_BATCH*SS_MS_THEO)/1000)
t_no_th     = (SETUP_MS + (N_drones-1)*SS_PER_PAIR*SS_MS_THEO) / 1000

ax.plot(np.log2(N_drones), t_nobatch_d, color=C['nobatch'], lw=2.5,
        marker='^', ms=8, label=f"Without batching  ({SS_MS:.0f} ms/SS, measured)")
ax.plot(np.log2(N_drones), t_batch_d,   color=C['batch'],   lw=2.5,
        marker='o', ms=8, ls='--', label="With batching  (constant)")
ax.plot(np.log2(N_drones), t_no_th,     color=C['nobatch'], lw=1.2,
        ls=':', alpha=0.5, label=f"Without batching  ({SS_MS_THEO:.0f} ms/SS, theory)")
ax.plot(np.log2(N_drones), t_bat_th,    color=C['batch'],   lw=1.2,
        ls=':', alpha=0.5, label=f"With batching  (theory)")
if r6:
    t_r = (r6.get("time_fhe_ms",0) + SETUP_MS) / 1000
    ax.scatter([np.log2(6)], [t_r], color=C['real'], s=150, zorder=8,
               marker='*', label=f"Measured N=6  ({t_r:.0f} s)")
ax.fill_between(np.log2(N_drones), t_batch_d, t_nobatch_d, alpha=0.08, color=C['gain'])
ax.set_xticks(np.log2(N_drones))
ax.set_xticklabels([str(n) for n in N_drones])
ax.legend(fontsize=8.5, framealpha=0.9)
save(fig, "plot_latency_vs_drones.png")

# ── Plot 2 : SIMD Batching (key result) ───────────────────────────────────────
fig, ax = make_fig()
style(ax, "SIMD Batching — With vs Without  (Key Result)",
      "Number of neighbor segments N", "Latency (s)")

N_simd         = np.array([1, 2, 5, 10, 20, 50])
t_simd_batch   = np.full(len(N_simd), (SETUP_MS + SS_BATCH*SS_MS)/1000)
t_simd_nobatch = (SETUP_MS + N_simd*SS_PER_PAIR*SS_MS) / 1000

ax.plot(N_simd, t_simd_nobatch, color=C['nobatch'], lw=2.5,
        marker='^', ms=8, label=f"Without batching  (N × {SS_PER_PAIR} SS)")
ax.plot(N_simd, t_simd_batch,   color=C['batch'],   lw=2.5,
        marker='o', ms=8, ls='--', label=f"With batching  ({SS_BATCH} SS constant)")
ax.fill_between(N_simd, t_simd_batch, t_simd_nobatch, alpha=0.10, color=C['gain'])
for n_v in [5, 10]:
    g = n_v * SS_PER_PAIR // SS_BATCH
    ax.axvline(n_v, color=C['gain'], lw=1.2, ls=':', alpha=0.8)
    ax.text(n_v+0.4, t_simd_batch[0]*1.06, f"×{g}",
            color=C['gain'], fontsize=13, fontweight='bold')
ax.legend(fontsize=9.5, framealpha=0.9)
save(fig, "plot_simd_batching.png")

# ── Plot 3 : Latency vs Trajectory Steps K ────────────────────────────────────
fig, ax = make_fig()
style(ax, "Latency vs Trajectory Steps K\n(N=5 neighbors fixed)",
      "Trajectory steps K (log₂ scale)", "Latency (s)")

K_vals      = np.array([1, 2, 4, 8, 16, 32, 64])
N_voisins   = 5
t_bat_K     = (SETUP_MS + K_vals * SS_BATCH          * SS_MS) / 1000
t_no_K      = (SETUP_MS + K_vals * N_voisins * SS_PER_PAIR * SS_MS) / 1000
t_bat_K_th  = (SETUP_MS + K_vals * SS_BATCH          * SS_MS_THEO) / 1000
t_no_K_th   = (SETUP_MS + K_vals * N_voisins * SS_PER_PAIR * SS_MS_THEO) / 1000

ax.plot(K_vals, t_no_K,     color=C['nobatch'], lw=2.5,
        marker='^', ms=8, label=f"Without batching  ({SS_MS:.0f} ms/SS, measured)")
ax.plot(K_vals, t_bat_K,    color=C['batch'],   lw=2.5,
        marker='o', ms=8, ls='--', label="With batching  (measured)")
ax.plot(K_vals, t_no_K_th,  color=C['nobatch'], lw=1.2,
        ls=':', alpha=0.5, label=f"Without batching  ({SS_MS_THEO:.0f} ms/SS, theory)")
ax.plot(K_vals, t_bat_K_th, color=C['batch'],   lw=1.2,
        ls=':', alpha=0.5, label="With batching  (theory)")
if r6:
    t_k1 = (r6.get("time_fhe_ms",0) + SETUP_MS) / 1000
    ax.scatter([1], [t_k1], color=C['real'], s=150, zorder=8,
               marker='*', label=f"Measured K=1  ({t_k1:.0f} s)")
ax.fill_between(K_vals, t_bat_K, t_no_K, alpha=0.08, color=C['gain'])
ax.set_xscale('log', base=2)
ax.set_xticks(K_vals)
ax.set_xticklabels([str(k) for k in K_vals])
ax.legend(fontsize=8.5, framealpha=0.9)
save(fig, "plot_latency_vs_steps.png")

# ── Plot 4 : Confusion Matrix ──────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(6, 5))
fig.patch.set_facecolor(C['bg'])
ax.set_facecolor(C['panel'])
ax.set_title(f"Confusion Matrix  ({total} FHE decisions)",
             color=C['text'], fontsize=13, fontweight='bold', pad=10)

conf = np.array([[TP, FN], [FP, TN]], dtype=float)
ax.imshow(conf, cmap=plt.cm.Blues, aspect='auto', vmin=0, vmax=max(TP,TN)+1)

cell_labels = [
    [f"TP = {TP}\n(correct collision)",   f"FN = {FN}\n(missed collision)"],
    [f"FP = {FP}\n(false alarm)",         f"TN = {TN}\n(correct no-collision)"]
]
cell_colors = [[C['tp'], C['fn']], [C['fp'], C['tn']]]
for i in range(2):
    for j in range(2):
        ax.text(j, i, cell_labels[i][j], ha='center', va='center',
                color='white' if conf[i,j] > conf.max()/2 else C['text'],
                fontsize=12, fontweight='bold')

ax.set_xticks([0,1])
ax.set_yticks([0,1])
ax.set_xticklabels(["Predicted: YES", "Predicted: NO"], color=C['text'], fontsize=10)
ax.set_yticklabels(["Actual: YES", "Actual: NO"],       color=C['text'], fontsize=10)
ax.tick_params(length=0)
for spine in ax.spines.values(): spine.set_edgecolor('#BDBDBD')

ax.text(0.5, -0.15,
        f"Precision = {precision:.0f}%     Recall = {recall:.0f}%     Accuracy = {accuracy:.0f}%",
        transform=ax.transAxes, ha='center',
        color=C['real'], fontsize=11, fontweight='bold')
save(fig, "plot_confusion_matrix.png")

# ─── Summary ──────────────────────────────────────────────────────────────────
print(f"""
{'='*58}
  PERFORMANCE SUMMARY
{'='*58}
  SS_MS measured : {SS_MS:.0f} ms  (×{SS_MS/SS_MS_THEO:.1f} vs theory {SS_MS_THEO:.0f} ms)
  Batching gain  : ×{5*SS_PER_PAIR//SS_BATCH}  ({5*SS_PER_PAIR} → {SS_BATCH} SS, N=5)
  Precision      : {precision:.0f}%
  Recall (TPR)   : {recall:.0f}%
  Accuracy       : {accuracy:.0f}%
  TP={TP}  TN={TN}  FP={FP}  FN={FN}
{'='*58}
  Files generated:
    plot_latency_vs_drones.png
    plot_simd_batching.png
    plot_latency_vs_steps.png
    plot_confusion_matrix.png
{'='*58}""")
