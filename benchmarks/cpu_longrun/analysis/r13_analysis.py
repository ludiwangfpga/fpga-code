"""R1.3 / R2.2 long-run CPU latency analysis. numpy + matplotlib only.
Writes figures to FIGDIR and prints a JSON blob with all numbers used in the .md."""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "data")
LR = os.path.join(SRC, "longrun")
FIGDIR = os.path.join(HERE, "..", "figures")
os.makedirs(FIGDIR, exist_ok=True)

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif", "serif"],
    "font.size": 9, "axes.labelsize": 9, "axes.titlesize": 9,
    "legend.fontsize": 8, "xtick.labelsize": 8, "ytick.labelsize": 8,
    "mathtext.fontset": "stix", "pdf.fonttype": 42,
})

QUIET_EXCL_S = 180.0      # contaminated window in quiet run
QUIET_N_AT_180 = 507052032  # n at the 180 s checkpoint (quiet_2h.log)
FPGA_US = 4.39
BUDGET_US = 100.0

def load_summary(tag):
    with open(os.path.join(LR, f"{tag}_summary.json")) as f:
        return json.load(f)

def load_hist(tag):
    lo, hi, cnt = [], [], []
    with open(os.path.join(LR, f"{tag}_hist.csv")) as f:
        next(f)
        for line in f:
            a, b, c = line.strip().split(",")
            lo.append(float(a)); hi.append(float(b)); cnt.append(int(c))
    return np.array(lo), np.array(hi), np.array(cnt, dtype=np.int64)

def load_tail(tag):
    d = np.loadtxt(os.path.join(LR, f"{tag}_tail.csv"), delimiter=",", skiprows=1)
    d = d[np.argsort(d[:, 0], kind="stable")]
    return d[:, 0], d[:, 1]

def hist_pct(hi, cnt, p, total):
    """Same convention as Hist::pct in cpu_longrun_main.inc: bin upper edge, conservative."""
    target = p / 100.0 * total
    c = np.cumsum(cnt)
    i = np.searchsorted(c, target, side="left")
    if i >= len(hi):
        return 10000.0
    return hi[i] if np.isfinite(hi[i]) else 10000.0

PCTS = [50, 99, 99.9, 99.99, 99.999, 99.9999]
states = {}
for tag, label in [("quiet_2h", "idle"), ("load_2h", "all-cores-loaded"), ("io_1h", "I/O-loaded")]:
    s = load_summary(tag)
    lo, hi, cnt = load_hist(tag)
    n = int(s["n"]); assert cnt.sum() == n, (tag, cnt.sum(), n)
    t, us = load_tail(tag)
    st = dict(tag=tag, label=label, n=n, elapsed_s=s["elapsed_s"], final=s["final"],
              mean=s["mean_us"], sd=s["sd_us"], max=s["max_us"], max_at_s=s["max_at_elapsed_s"],
              over_4p39=s["over_4p39us"], over_33p46=s["over_33p46us"], over_100=s["over_100us"],
              over_1000=s["over_1000us"], over_10ms=s["over_10ms"], tsc_ghz=s["tsc_ghz"])
    for p in PCTS:
        st[f"p{p}"] = hist_pct(hi, cnt, p, n)
    # consistency check vs summary.json
    for key, p in [("p50_us", 50), ("p999_us", 99.9), ("p9999_us", 99.99), ("p999999_us", 99.9999)]:
        assert abs(s[key] - st[f"p{p}"]) < 1e-6, (tag, key, s[key], st[f"p{p}"])
    st["tail_t"] = t; st["tail_us"] = us; st["hist"] = (lo, hi, cnt)
    st["tail_over_100"] = int((us > 100).sum()); st["tail_over_1000"] = int((us > 1000).sum())
    st["tail_over_33p46"] = int((us > 33.46).sum())
    assert st["tail_over_100"] == st["over_100"], (tag, st["tail_over_100"], st["over_100"])
    assert st["tail_over_1000"] == st["over_1000"]
    states[tag] = st

# ---- quiet: exclude first 180 s ----
q = states["quiet_2h"]
m = q["tail_t"] >= QUIET_EXCL_S
q["excl"] = dict(
    n=q["n"] - QUIET_N_AT_180, elapsed_s=q["elapsed_s"] - QUIET_EXCL_S,
    max=float(q["tail_us"][m].max()), max_at_s=float(q["tail_t"][m][np.argmax(q["tail_us"][m])]),
    over_100=int((q["tail_us"][m] > 100).sum()), over_1000=int((q["tail_us"][m] > 1000).sum()),
    over_33p46=int((q["tail_us"][m] > 33.46).sum()),
    first180_over_100=int((q["tail_us"][~m] > 100).sum()), first180_over_1000=int((q["tail_us"][~m] > 1000).sum()),
    first180_over_33p46=int((q["tail_us"][~m] > 33.46).sum()), first180_max=float(q["tail_us"][~m].max()),
)
# percentile sensitivity: remove first-180 s tail samples (>20 us) from hist bins, scale sub-20 us bins pro rata
lo, hi, cnt = q["hist"]
cnt2 = cnt.astype(np.float64).copy()
frac_keep = 1.0 - QUIET_N_AT_180 / q["n"]
cnt2[hi <= 20.0] *= frac_keep
excl_us = q["tail_us"][~m]
for v in excl_us:
    b = np.searchsorted(hi, v, side="left")  # bin with lo <= v < hi
    if b < len(cnt2):
        cnt2[b] -= 1
cnt2 = np.clip(cnt2, 0, None)
n2 = cnt2.sum()
q["excl"]["pcts_approx"] = {f"p{p}": hist_pct(hi, cnt2, p, n2) for p in PCTS}
# how many samples above the full-run p99.9999 threshold come from the first 180 s
thr = q["p99.9999"]
q["excl"]["n_above_p999999_total"] = int((q["tail_us"] > thr).sum())
q["excl"]["n_above_p999999_first180"] = int((excl_us > thr).sum())

# ---- original 10^6 runs ----
orig = {}
for name in ["quiet", "loaded"]:
    a = np.fromfile(os.path.join(SRC, f"percall_{name}.bin"), dtype="<f8")
    a_s = np.sort(a); n = len(a)
    def P(p): return float(a_s[min(n - 1, int(np.ceil(p / 100 * n)) - 1)])
    orig[name] = dict(n=n, mean=float(a.mean()), sd=float(a.std()), p50=P(50), p99=P(99), p99_9=P(99.9), p99_99=P(99.99),
                      max=float(a.max()), over_4p39=int((a > 4.39).sum()), over_10=int((a > 10).sum()),
                      over_33p46=int((a > 33.46).sum()), over_100=int((a > 100).sum()),
                      duration_s=float(a.sum() * 1e-6), raw=a)
    # running max at 1e6 (=max) for the growth series
    orig[name]["runmax_1e6"] = float(a.max())

# ---- growth of max vs calls ----
def growth(st, t0=0.0):
    rate = st["n"] / st["elapsed_s"]  # calls / s
    t, us = st["tail_t"], st["tail_us"]
    m = t >= t0
    t, us = t[m] - t0, us[m]
    calls = t * rate
    runmax = np.maximum.accumulate(us)
    out = {}
    for k in [1e6, 1e7, 1e8, 1e9, 1e10, 1e11]:
        if k > st["n"] - t0 * rate:
            out[f"{k:.0e}"] = None; continue
        sel = calls <= k
        out[f"{k:.0e}"] = float(us[sel].max()) if sel.any() else float("nan")
    return rate, calls, runmax, out

growth_res = {}
for tag in ["quiet_2h", "load_2h", "io_1h"]:
    rate, calls, runmax, out = growth(states[tag])
    growth_res[tag] = dict(rate=rate, series=out, calls=calls, runmax=runmax)
rate_q, calls_qx, runmax_qx, out_qx = growth(q, t0=QUIET_EXCL_S)
growth_res["quiet_2h_excl180"] = dict(rate=rate_q, series=out_qx, calls=calls_qx, runmax=runmax_qx)

# ================= FIGURES =================
colors = {"quiet_2h": "#1f77b4", "load_2h": "#d62728", "io_1h": "#2ca02c"}
names = {"quiet_2h": "CPU, idle",
         "load_2h": "CPU, all cores loaded",
         "io_1h": "CPU, I/O loaded"}

# --- Fig 1: CCDF ---
fig, ax = plt.subplots(figsize=(3.5, 2.7))
for tag in ["quiet_2h", "load_2h", "io_1h"]:
    lo, hi, cnt = states[tag]["hist"]; n = states[tag]["n"]
    ccdf = (n - np.cumsum(cnt)) / n
    fin = np.isfinite(hi)
    xs, ys = hi[fin], ccdf[fin]
    # beyond the last finite bin edge, use the exact per-event tail log instead of the overflow bin
    edge = hi[fin].max()
    tl = np.sort(states[tag]["tail_us"]); tl = tl[tl > edge]
    assert len(tl) == n - np.cumsum(cnt)[fin][-1], (tag, len(tl))
    xs = np.concatenate([xs, tl]); ys = np.concatenate([ys, (len(tl) - 1 - np.arange(len(tl))) / n])
    k = ys > 0
    ax.step(xs[k], ys[k], where="post", color=colors[tag], lw=1.0, label=names[tag])
# original idle 10^6 for reference
a = np.sort(orig["quiet"]["raw"]); n = len(a)
ccdf = 1.0 - np.arange(1, n + 1) / n
k = ccdf > 0
ax.plot(a[k], ccdf[k], color="0.45", lw=0.8, ls=":", label="CPU, idle ($10^6$-call run)")
ax.axvline(FPGA_US, color="k", lw=1.0, ls="--", label="FPGA, 4.39 $\\mu$s (fixed)")
ax.axvline(BUDGET_US, color="0.3", lw=0.8, ls="-.", label="100 $\\mu$s budget")
ax.axhline(1e-6, color="0.7", lw=0.6, ls=":")
ax.text(0.33, 1.4e-6, "$10^{-6}$ (resolution of a $10^6$-call run)", fontsize=6.5, color="0.4")
ax.set_xscale("log"); ax.set_yscale("log")
ax.set_xlim(0.3, 4e4); ax.set_ylim(2e-11, 1.5)
ax.set_xlabel("Per-call control-law latency $x$ ($\\mu$s)")
ax.set_ylabel("$P(\\mathrm{latency} > x)$")
ax.grid(True, which="major", lw=0.3, alpha=0.5)
ax.legend(loc="upper right", frameon=True, framealpha=0.9, edgecolor="none", fontsize=6.5)
fig.tight_layout(pad=0.3)
fig.savefig(os.path.join(FIGDIR, "fig_r13_ccdf.png"), dpi=300)
fig.savefig(os.path.join(FIGDIR, "fig_r13_ccdf.pdf"))
plt.close(fig)

# --- Fig 2: tail-event time series ---
fig, axes = plt.subplots(3, 1, figsize=(3.5, 4.6), sharex=False)
panel_titles = {"quiet_2h": "(a) CPU, idle", "load_2h": "(b) CPU, all cores loaded",
                "io_1h": "(c) CPU, I/O loaded"}
for ax, tag in zip(axes, ["quiet_2h", "load_2h", "io_1h"]):
    t, us = states[tag]["tail_t"], states[tag]["tail_us"]
    ax.scatter(t / 3600.0, us, s=1.2, c=colors[tag], alpha=0.35, lw=0, rasterized=True)
    ax.axhline(BUDGET_US, color="0.3", lw=0.8, ls="-.")
    ax.axhline(1000, color="0.3", lw=0.8, ls=":")
    ax.axhline(FPGA_US, color="k", lw=0.8, ls="--")
    ax.set_yscale("log"); ax.set_ylim(3, 5e4)
    ax.set_xlim(0, states[tag]["elapsed_s"] / 3600.0)
    ax.set_ylabel("latency ($\\mu$s)")
    ax.set_title(panel_titles[tag], loc="left", fontsize=8)
    ax.grid(True, lw=0.3, alpha=0.4)
    if tag == "quiet_2h":
        ax.axvspan(0, QUIET_EXCL_S / 3600.0, color="0.85", lw=0)
        ax.text(QUIET_EXCL_S / 3600.0 + 0.02, 2.2e4, "excl. <180 s", fontsize=6.5, va="top", color="0.3")
    ax.text(ax.get_xlim()[1] * 0.995, 115, "100 $\\mu$s", fontsize=6.5, ha="right", va="bottom", color="0.3")
    ax.text(ax.get_xlim()[1] * 0.995, 1150, "1 ms", fontsize=6.5, ha="right", va="bottom", color="0.3")
    ax.text(ax.get_xlim()[1] * 0.995, 4.9, "FPGA 4.39 $\\mu$s", fontsize=6.5, ha="right", va="bottom")
axes[-1].set_xlabel("elapsed time (h)")
fig.tight_layout(pad=0.3, h_pad=0.6)
fig.savefig(os.path.join(FIGDIR, "fig_r13_tail_timeseries.png"), dpi=300)
fig.savefig(os.path.join(FIGDIR, "fig_r13_tail_timeseries.pdf"))
plt.close(fig)

# --- Fig 3: running max vs calls ---
fig, ax = plt.subplots(figsize=(3.5, 2.5))
for tag in ["quiet_2h", "load_2h", "io_1h"]:
    g = growth_res[tag]
    ax.step(g["calls"], g["runmax"], where="post", color=colors[tag], lw=1.0, label=names[tag])
g = growth_res["quiet_2h_excl180"]
ax.step(g["calls"], g["runmax"], where="post", color=colors["quiet_2h"], lw=1.0, ls="--", label="CPU, idle (restart at 180 s)")
# original 10^6
a = orig["quiet"]["raw"]; rm = np.maximum.accumulate(a)
ax.plot(np.arange(1, len(a) + 1), rm, color="0.45", lw=0.8, ls=":", label="CPU, idle ($10^6$-call run)")
ax.axhline(FPGA_US, color="k", lw=0.8, ls="--"); ax.text(1.2e3, 5.0, "FPGA 4.39 $\\mu$s", fontsize=6.5)
ax.axhline(BUDGET_US, color="0.3", lw=0.8, ls="-."); ax.text(1.2e3, 115, "100 $\\mu$s budget", fontsize=6.5)
ax.set_xscale("log"); ax.set_yscale("log")
ax.set_xlim(1e3, 3e10); ax.set_ylim(3, 5e4)
ax.set_xlabel("number of calls observed"); ax.set_ylabel("running maximum ($\\mu$s)")
ax.grid(True, lw=0.3, alpha=0.4)
ax.legend(loc="upper left", frameon=False, fontsize=6.2)
fig.tight_layout(pad=0.3)
fig.savefig(os.path.join(FIGDIR, "fig_r13_running_max.png"), dpi=300)
fig.savefig(os.path.join(FIGDIR, "fig_r13_running_max.pdf"))
plt.close(fig)

# ================= REPORT NUMBERS =================
def clean(st):
    return {k: v for k, v in st.items() if k not in ("tail_t", "tail_us", "hist")}
report = {
    "states": {k: clean(v) for k, v in states.items()},
    "orig": {k: {kk: vv for kk, vv in v.items() if kk != "raw"} for k, v in orig.items()},
    "growth": {k: dict(rate=v["rate"], series=v["series"]) for k, v in growth_res.items()},
}
# extra: counts > 10 us, > 20 us for orig
for name in ["quiet", "loaded"]:
    a = orig[name]["raw"]
    report["orig"][name]["over_20"] = int((a > 20).sum())
# quiet: biggest events list
for tag in ["quiet_2h", "load_2h", "io_1h"]:
    t, us = states[tag]["tail_t"], states[tag]["tail_us"]
    idx = np.argsort(us)[::-1][:8]
    report["states"][tag]["top8"] = [(float(t[i]), float(us[i])) for i in idx]
    # inter-event spacing of >100 us events
    tt = np.sort(t[us > 100])
    if len(tt) > 1:
        d = np.diff(tt)
        report["states"][tag]["gap100_median_s"] = float(np.median(d)); report["states"][tag]["gap100_max_s"] = float(d.max())
        # per-10-min bucket counts of >100 us events
        edges = np.arange(0, states[tag]["elapsed_s"] + 600, 600)
        report["states"][tag]["per10min_over100"] = np.histogram(tt, bins=edges)[0].tolist()
print(json.dumps(report, indent=1, default=float))
