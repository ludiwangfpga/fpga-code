#!/usr/bin/env python3
# traj_analyze.py — 硬件轨迹跟踪实验日志分析出图。
#
# 日志契约(每行, 可含表头行, 健壮解析):
#   t_ms, q0_q16..q5_q16, xd0_q16..xd5_q16
#   t_ms: 真实毫秒时间戳; q*_q16: CAN 实读关节角(Q16.16 弧度);
#   xd*_q16: host 当拍实发的**绝对** x_des(Q16.16, 前3列米/后3列弧度)。
# 处理: 每行 q 经 fk_ref(复用 ../fk_ab_check.py, 已板级逐位验证的双精度参考)算实际位姿;
#       期望 = 日志里的 xd 前 3 轴。
# 输出(写到日志同目录): traj_xy / traj_xyz_t / traj_err_t 各 .png+.pdf,
#       终端打印全程与稳态段(t>5s)逐轴+欧氏 RMS/max(mm), 末行 sentinel TRAJ_ANALYZE_OK。
#
# 用法: python traj_analyze.py <log.csv>
#       python traj_analyze.py --selftest    （退化自测, 见文件末尾说明）
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))  # tools/ —— 复用 fk_ab_check.fk_ref, 勿复制代码
from fk_ab_check import fk_ref

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SC = 65536.0

# ---------- 解析 ----------

def parse_log(path):
    """健壮解析: 跳过表头/注释/坏行; 逗号或空白分隔; 取每行前 13 个数。"""
    rows = []
    with open(path, "r", encoding="utf-8-sig", errors="replace") as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            parts = [p for p in ln.replace(",", " ").split() if p]
            if len(parts) < 13:
                continue
            try:
                vals = [float(p) for p in parts[:13]]
            except ValueError:
                continue  # 表头行
            rows.append(vals)
    if not rows:
        raise SystemExit("TRAJ_ANALYZE_FAIL: 日志无有效数据行: %s" % path)
    return rows

# ---------- 计算 ----------

def compute(rows):
    """返回 dict: t(s), 期望/实际位置(mm, 各3列), 逐轴误差与误差范数(mm)。"""
    t = []
    des = [[], [], []]
    act = [[], [], []]
    err = [[], [], []]
    en = []
    for v in rows:
        t.append(v[0] / 1000.0)
        q = [int(round(x)) for x in v[1:7]]
        pose = fk_ref(q)                      # 双精度参考 FK, 米/弧度
        d2 = 0.0
        for k in range(3):
            dk = v[7 + k] / SC * 1000.0       # 期望, mm
            ak = pose[k] * 1000.0             # 实际, mm
            ek = ak - dk
            des[k].append(dk)
            act[k].append(ak)
            err[k].append(ek)
            d2 += ek * ek
        en.append(math.sqrt(d2))
    return {"t": t, "des": des, "act": act, "err": err, "en": en}

def _rms(vs):
    return math.sqrt(sum(v * v for v in vs) / len(vs))

def print_stats(d):
    axes = "xyz"
    print("==== 轨迹跟踪误差统计 (单位 mm) ====")
    for label, mask in (("全程", None), ("稳态 t>5s", 5.0)):
        idx = [i for i, tt in enumerate(d["t"]) if mask is None or tt > mask]
        if not idx:
            print("%-10s N=0 (跳过)" % label)
            continue
        parts_rms, parts_max = [], []
        for k in range(3):
            vs = [d["err"][k][i] for i in idx]
            parts_rms.append("%s=%.3f" % (axes[k], _rms(vs)))
            parts_max.append("%s=%.3f" % (axes[k], max(abs(v) for v in vs)))
        ns = [d["en"][i] for i in idx]
        print("%-10s (N=%d)" % (label, len(idx)))
        print("  RMS: %s |e|=%.3f" % (" ".join(parts_rms), _rms(ns)))
        print("  MAX: %s |e|=%.3f" % (" ".join(parts_max), max(ns)))

# ---------- 出图 ----------

_STYLE = {
    "font.family": "serif",
    "mathtext.fontset": "dejavuserif",
    "font.size": 10,
    "axes.grid": True,
    "grid.linestyle": ":",
    "grid.linewidth": 0.5,
    "grid.alpha": 0.6,
    "axes.linewidth": 0.8,
    "legend.frameon": False,
    "savefig.dpi": 200,
    "savefig.bbox": "tight",
}
_DES = dict(color="0.4", linestyle="--", linewidth=1.0, label="desired")
_ACT = dict(color="black", linestyle="-", linewidth=1.1, label="actual")

def _save(fig, outdir, stem):
    for ext in ("png", "pdf"):
        fig.savefig(os.path.join(outdir, "%s.%s" % (stem, ext)))
    plt.close(fig)
    print("[出图] %s.png / .pdf" % os.path.join(outdir, stem))

def make_plots(d, outdir):
    plt.rcParams.update(_STYLE)
    t = d["t"]

    # 1) xy 平面路径: 期望虚线 vs 实际实线
    fig, ax = plt.subplots(figsize=(4.2, 4.2))
    ax.plot(d["des"][0], d["des"][1], **_DES)
    ax.plot(d["act"][0], d["act"][1], **_ACT)
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="best")
    _save(fig, outdir, "traj_xy")

    # 2) x/y/z vs t 三子图
    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(5.2, 5.6))
    for k, name in enumerate(("x", "y", "z")):
        axs[k].plot(t, d["des"][k], **_DES)
        axs[k].plot(t, d["act"][k], **_ACT)
        axs[k].set_ylabel("%s (mm)" % name)
    axs[0].legend(loc="best")
    axs[2].set_xlabel("t (s)")
    fig.align_ylabels(axs)
    _save(fig, outdir, "traj_xyz_t")

    # 3) 位置误差范数 vs t
    fig, ax = plt.subplots(figsize=(5.2, 2.8))
    ax.plot(t, d["en"], color="black", linewidth=1.0)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("position error (mm)")
    ax.set_ylim(bottom=0.0)
    _save(fig, outdir, "traj_err_t")

def analyze(log_path):
    rows = parse_log(log_path)
    d = compute(rows)
    outdir = os.path.dirname(os.path.abspath(log_path))
    make_plots(d, outdir)
    print_stats(d)
    return d

# ---------- 退化自测 ----------
# 局限说明: 本仓库无解析逆解, 无法合成"q(t) 恰好跟踪 x_des(t)"的完美跟踪日志。
# 退化做法: 取固定锚点关节角 q0(常数), p0=fk_ref(q0), 日志 q 列恒为 q0,
#   xd 列 = Q16.16(p0) + Δ表(traj_xdes_delta.csv)。
# 此时实际位姿恒为 p0, 误差范数应精确等于 |Δ(t)|(仅差 p0 定点化的 <0.02mm 舍入),
# 可定量校验 fk_ref 接线/Q16.16 换算/误差计算/出图全流程自身正确。

def selftest():
    table = os.path.join(_HERE, "traj_xdes_delta.csv")
    deltas = []
    with open(table, encoding="utf-8") as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            try:
                deltas.append([int(p) for p in ln.split(",")])
            except ValueError:
                continue
    print("==== SELFTEST (退化自测) ====")
    print("[输入] Δ表 %s, %d 行" % (table, len(deltas)))

    q0_rad = [0.1, -0.3, 0.5, -0.2, 0.4, -0.1]
    q0 = [int(round(v * SC)) for v in q0_rad]
    p0 = fk_ref(q0)
    p0_q16 = [int(round(v * SC)) for v in p0]
    print("[锚点] q0(rad) = %s" % q0_rad)
    print("[锚点] p0 = x=%.4f y=%.4f z=%.4f (m)  rx=%.4f ry=%.4f rz=%.4f (rad)"
          % tuple(p0))

    outdir = os.path.join(_HERE, "selftest_out")
    os.makedirs(outdir, exist_ok=True)
    log_path = os.path.join(outdir, "selftest_log.csv")
    with open(log_path, "w", newline="\n", encoding="utf-8") as f:
        f.write("t_ms,q0_q16,q1_q16,q2_q16,q3_q16,q4_q16,q5_q16,"
                "xd0_q16,xd1_q16,xd2_q16,xd3_q16,xd4_q16,xd5_q16\n")  # 表头行, 兼验证健壮解析
        for row in deltas:
            xd = [p0_q16[k] + row[1 + k] for k in range(6)]
            f.write("%d,%s,%s\n" % (row[0],
                                    ",".join(str(v) for v in q0),
                                    ",".join(str(v) for v in xd)))
    print("[合成] %s (恒定 q0 + xd=p0+Δ)" % log_path)

    d = analyze(log_path)

    # 定量校验: 误差范数应等于 |Δ(t)| (来自 Δ 表), 容差 0.05 mm (p0 定点化舍入)
    ok = True
    def chk(name, cond, detail):
        nonlocal ok
        print("[校验] %-34s %s  (%s)" % (name, "OK" if cond else "FAIL", detail))
        if not cond:
            ok = False

    dn = [math.sqrt(sum((row[1 + k] / SC * 1000.0) ** 2 for k in range(3)))
          for row in deltas]  # |Δ(t)|, mm
    chk("行数 = Δ表行数", len(d["en"]) == len(deltas),
        "%d vs %d" % (len(d["en"]), len(deltas)))
    chk("err(0) ≈ 0", d["en"][0] < 0.05, "%.4f mm" % d["en"][0])
    chk("err(60s) ≈ 100 mm", abs(d["en"][-1] - 100.0) < 0.05, "%.4f mm" % d["en"][-1])
    dev = max(abs(d["en"][i] - dn[i]) for i in range(len(dn)))
    chk("max|err(t)-|Δ(t)|| < 0.05 mm", dev < 0.05, "%.5f mm" % dev)
    print("[局限] 无逆解, 未合成 q 真实跟踪的日志; 本自测以恒定 q 的已知误差剖面"
          " err(t)=|Δ(t)| 定量校验脚本全流程(fk_ref/换算/统计/出图)自身正确。")
    if not ok:
        print("TRAJ_ANALYZE_FAIL")
        sys.exit(1)

def main():
    if len(sys.argv) != 2:
        print("用法: python traj_analyze.py <log.csv> | --selftest")
        sys.exit(2)
    if sys.argv[1] == "--selftest":
        selftest()
    else:
        analyze(sys.argv[1])
    print("TRAJ_ANALYZE_OK")

if __name__ == "__main__":
    main()
