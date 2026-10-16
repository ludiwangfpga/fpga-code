# -*- coding: utf-8 -*-
"""r24_analyze.py -- R2.4 serial-vs-bypass: statistics + figures from the xsim CSVs.
Reads  data/{adapt_latency_samples,chain_samples,udp_samples}.csv (next to this script)
Writes data/r24_summary.csv and figures/r24_stage_timeline.{png,pdf},
       figures/r24_latency_hist.{png,pdf}
Run:   python r24_analyze.py   (numpy + matplotlib, Agg); any working directory
"""
import csv, os, statistics as st
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "data")              # CSVs written by the xsim testbenches
FIGDIR = os.path.join(HERE, "figures")          # figures are regenerated, not committed
os.makedirs(FIGDIR, exist_ok=True)
F_CLK = 125e6
NS = 1e9 / F_CLK          # 8 ns / cycle
BUDGET_US = 100.0

def rd(name):
    with open(os.path.join(ROOT, name), newline="") as f:
        return list(csv.DictReader(f))

A = rd("adapt_latency_samples.csv")
B = rd("chain_samples.csv")
C = rd("udp_samples.csv")

def stats(v):
    v = np.asarray(v, dtype=float)
    return dict(n=len(v), min=int(v.min()), median=float(np.median(v)), max=int(v.max()),
                mean=float(v.mean()), sd=float(v.std(ddof=0)))

a_cyc  = [int(r["cycles"]) for r in A]
c_fwd  = [int(r["onchip_cyc_last"]) for r in C]
c_ad   = [int(r["adapt_cycles"]) for r in C]
c_ser  = [int(r["serial_equiv_cycles"]) for r in C]
b_byp0 = [int(r["byp_noadapt_cycles"]) for r in B]
b_byp1 = [int(r["byp_adapt_cycles"]) for r in B]
b_ser  = [int(r["serial_cycles"]) for r in B]
b_req  = [int(r["adapt_req"]) for r in B]
b_ser1 = [c for c, q in zip(b_ser, b_req) if q]
b_ser0 = [c for c, q in zip(b_ser, b_req) if not q]

rows = []
def add(label, v, note=""):
    s = stats(v); s["label"] = label; s["note"] = note
    s["min_us"] = s["min"] * NS / 1e3; s["max_us"] = s["max"] * NS / 1e3
    s["median_us"] = s["median"] * NS / 1e3
    s["max_pct_budget"] = 100.0 * s["max_us"] / BUDGET_US
    rows.append(s); return s

sA  = add("A: adaptation update start->done (deployed adaptive_impedance_v2)", a_cyc, "N=2000 random updates incl. dead-zone / enable=0")
sC0 = add("C: BYPASS forward chain, on-chip op18 counter (deployed udp_fk_cmd)", c_fwd, "command latch -> iq written")
sCa = add("C: adaptation op15 start->done inside deployed udp_fk_cmd", c_ad, "off the critical path")
sCs = add("C: SERIAL-equivalent forward chain = on-chip fwd + adaptation", c_ser, "every beat carries an update")
sB0 = add("B: wrapper BYPASS, adaptation never triggered", b_byp0, "wrapper start -> done")
sB1 = add("B: wrapper BYPASS, adaptation triggered concurrently", b_byp1, "wrapper start -> done")
sBs = add("B: wrapper SERIAL, all beats (update on 48.4 % of beats)", b_ser, "wrapper start -> done")
sB1u= add("B: wrapper SERIAL, beats WITH update", b_ser1, "")
sB0u= add("B: wrapper SERIAL, beats WITHOUT update", b_ser0, "")

with open(os.path.join(ROOT, "r24_summary.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["measurement", "n", "min_cycles", "median_cycles", "max_cycles", "mean_cycles", "sd_cycles",
                "min_us", "median_us", "max_us", "max_pct_of_100us", "note"])
    for s in rows:
        w.writerow([s["label"], s["n"], s["min"], s["median"], s["max"], f'{s["mean"]:.3f}', f'{s["sd"]:.3f}',
                    f'{s["min_us"]:.3f}', f'{s["median_us"]:.3f}', f'{s["max_us"]:.3f}', f'{s["max_pct_budget"]:.2f}', s["note"]])

added = sCs["median"] - sC0["median"]
print("adaptation latency        :", sA)
print("bypass on-chip forward    :", sC0)
print("serial-equivalent forward :", sCs)
print(f"added latency (median)    : {added:.0f} cycles = {added*NS/1e3:.3f} us = {100*added*NS/1e3/BUDGET_US:.2f} % of 100 us")
print(f"serial jitter (with/without update beats, wrapper): {sB1u['min']}-{sB0u['min']} = {sB1u['min']-sB0u['min']} cycles")

# ---------------------------------------------------------------- palette (validated, dataviz skill)
C_FK, C_IMP, C_TIQ, C_ADP, C_LAT = "#2a78d6", "#1baf7a", "#eda100", "#eb6834", "#9a9893"
TXT, TXT2, GRID = "#0b0b0b", "#52514e", "#e6e5e1"
plt.rcParams.update({"font.family": "serif", "font.size": 9, "axes.edgecolor": TXT2,
                     "axes.labelcolor": TXT, "xtick.color": TXT2, "ytick.color": TXT2})

# ---------------------------------------------------------------- Fig 1: stage timeline
fig, ax = plt.subplots(figsize=(7.0, 2.6))
rows_tl = [
    ("Serial reference\n(same law, same I/O)", [("latch", 4, C_LAT), ("FK + J", 422, C_FK), ("adaptation", 172, C_ADP), ("imp. + J$^T$", 119, C_IMP), ("$\\tau\\rightarrow i_q$", 4, C_TIQ)]),
    ("Decoupled bypass\n(deployed)",          [("latch", 4, C_LAT), ("FK + J", 422, C_FK), ("imp. + J$^T$", 119, C_IMP), ("$\\tau\\rightarrow i_q$", 4, C_TIQ)]),
]
y = [1.0, 0.0]
for (name, segs), yy in zip(rows_tl, y):
    x = 0
    for lab, w, col in segs:
        ax.barh(yy, w, left=x, height=0.42, color=col, edgecolor="#fcfcfb", linewidth=2)
        if w >= 100:
            ax.text(x + w / 2, yy, f"{lab}\n{w}", ha="center", va="center", color="#fcfcfb", fontsize=8)
        x += w
    tot = x
    ax.text(tot + 8, yy, f"{tot} cyc = {tot*NS/1e3:.2f} µs", va="center", ha="left", color=TXT, fontsize=8.5)
# adaptation drawn beside the bypass row (off the critical path)
ax.barh(-0.62, 172, left=4, height=0.26, color=C_ADP, alpha=0.9, edgecolor="#fcfcfb", linewidth=2)
ax.text(4 + 172 + 8, -0.62, "adaptation 172 cyc, off the critical path (fixed)", va="center", ha="left", color=TXT2, fontsize=7.5)
ax.set_yticks(y); ax.set_yticklabels([r[0] for r in rows_tl])
ax.set_xlabel("cycles @ 125 MHz (8 ns)")
ax.set_xlim(0, 1000); ax.set_ylim(-1.0, 1.45)
for s in ("top", "right"): ax.spines[s].set_visible(False)
ax.grid(axis="x", color=GRID, linewidth=0.6); ax.set_axisbelow(True)
ax.text(0, 1.38, "latch 4 / $\\tau\\rightarrow i_q$ 4 not labelled (too narrow)", fontsize=7, color=TXT2)
fig.tight_layout()
for ext in ("png", "pdf"):
    fig.savefig(os.path.join(FIGDIR, f"r24_stage_timeline.{ext}"), dpi=300)
plt.close(fig)

# ---------------------------------------------------------------- Fig 2: per-beat latency histogram (wrapper B, 1000 beats)
fig, ax = plt.subplots(figsize=(4.6, 2.8))
vals = sorted(set(b_byp1) | set(b_ser))
xs = np.arange(len(vals)); wbar = 0.36
cnt_b = [b_byp1.count(v) for v in vals]
cnt_s = [b_ser.count(v) for v in vals]
ax.bar(xs - wbar/2, cnt_b, width=wbar, color=C_FK, label="decoupled bypass (deployed)")
ax.bar(xs + wbar/2, cnt_s, width=wbar, color=C_ADP, label="serial reference")
for i, (cb, cs) in enumerate(zip(cnt_b, cnt_s)):
    if cb: ax.text(xs[i] - wbar/2, cb + 12, str(cb), ha="center", fontsize=8, color=TXT)
    if cs: ax.text(xs[i] + wbar/2, cs + 12, str(cs), ha="center", fontsize=8, color=TXT)
ax.set_xticks(xs); ax.set_xticklabels([f"{v} cyc\n{v*NS/1e3:.2f} µs" for v in vals])
ax.set_ylabel("beats (of 1000)"); ax.set_ylim(0, 1150)
ax.set_title("forward-chain latency per beat, random inputs,\nadaptation requested on 48.4 % of beats", fontsize=9)
ax.legend(frameon=False, fontsize=8, loc="upper right")
for s in ("top", "right"): ax.spines[s].set_visible(False)
ax.grid(axis="y", color=GRID, linewidth=0.6); ax.set_axisbelow(True)
fig.tight_layout()
for ext in ("png", "pdf"):
    fig.savefig(os.path.join(FIGDIR, f"r24_latency_hist.{ext}"), dpi=300)
plt.close(fig)
print("figures written to", FIGDIR)
