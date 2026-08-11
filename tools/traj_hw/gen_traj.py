#!/usr/bin/env python3
# gen_traj.py — 生成硬件轨迹跟踪实验的增量轨迹表 traj_xdes_delta.csv（确定性，与 seed 无关）。
#
# 轨迹定义（相对开跑前锚点位姿 p0 的增量，Δ(0)=0 平滑起步）:
#   五次多项式时间标度 s(t): [0,T]->[0,1], s(0)=s'(0)=s''(0)=0, s(T)=1, s'(T)=s''(T)=0
#     s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5, tau = t/T, T = 60 s
#   theta(t) = 3 * 2*pi * s(t)                    # xy 平面 3 圈
#   dx = r*(cos(theta) - 1), dy = r*sin(theta)    # r = 0.1118 m，圆心在起点 -x 方向 r 处，轨迹从起点出发
#   dz = 0.100 * s(t)                             # z 相对爬升 +100 mm
#   drx = dry = drz = 0                           # 姿态保持恒定
# 采样 100 Hz x 60 s -> 6001 行；数值口径 Q16.16（米/弧度），四舍五入取整。
#
# 用法: python gen_traj.py     （在本目录生成/覆盖 traj_xdes_delta.csv 并自检，末行 GEN_TRAJ_OK）
import math
import os
import sys

T_S = 60.0          # 总时长 s
FS = 100            # 采样率 Hz
N = int(T_S * FS) + 1  # 6001 行（含 t=0 与 t=60s 两端点）
R_M = 0.1118        # 圆半径 m
TURNS = 3           # 圈数
DZ_M = 0.100        # z 相对爬升 m
SC = 65536.0        # Q16.16

def s_quintic(tau):
    """五次多项式 0->1，端点一二阶导为零。"""
    return ((6.0 * tau - 15.0) * tau + 10.0) * tau * tau * tau

def q16(v):
    """Q16.16 定点化，四舍五入（远离零）。"""
    n = int(math.floor(abs(v) * SC + 0.5))
    return n if v >= 0.0 else -n

def gen_rows():
    rows = []
    for k in range(N):
        t_ms = k * 10
        tau = t_ms / (T_S * 1000.0)
        s = s_quintic(tau)
        theta = TURNS * 2.0 * math.pi * s
        dx = R_M * (math.cos(theta) - 1.0)
        dy = R_M * math.sin(theta)
        dz = DZ_M * s
        rows.append((t_ms, q16(dx), q16(dy), q16(dz), 0, 0, 0))
    return rows

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out_csv = os.path.join(here, "traj_xdes_delta.csv")
    rows = gen_rows()
    with open(out_csv, "w", newline="\n", encoding="utf-8") as f:
        # 注释行故意用纯 ASCII: 实验机 host 工程是 GBK 环境, 避免任何编码下显示乱码
        f.write("# delta table rel. to anchor pose p0 (send x_des(t)=p0+delta(t), per-component Q16.16"
                " integer add); units: t_ms=ms, dx/dy/dz=meter(Q16.16 int), drx/dry/drz=rad(Q16.16 int);"
                " 100Hz x 60s = 6001 rows, delta(0)=0 smooth start, attitude deltas all 0\n")
        f.write("t_ms,dx_q16,dy_q16,dz_q16,drx_q16,dry_q16,drz_q16\n")
        for r in rows:
            f.write("%d,%d,%d,%d,%d,%d,%d\n" % r)

    # ---- 自检（读回文件核对） ----
    data = []
    with open(out_csv, encoding="utf-8") as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            parts = ln.split(",")
            try:
                data.append([int(p) for p in parts])
            except ValueError:
                continue  # 列名表头行
    ok = True
    def chk(name, cond, detail):
        nonlocal ok
        print("[自检] %-28s %s  (%s)" % (name, "OK" if cond else "FAIL", detail))
        if not cond:
            ok = False

    chk("行数 = 6001", len(data) == 6001, "实际 %d" % len(data))
    chk("Δ(0) 全零", all(v == 0 for v in data[0][1:]), "row0 = %s" % (data[0],))
    max_dx = max(abs(r[1]) for r in data) / SC
    max_dy = max(abs(r[2]) for r in data) / SC
    chk("|Δx|max ≈ 2r = %.4f m" % (2 * R_M), abs(max_dx - 2 * R_M) < 1e-4, "实际 %.6f m" % max_dx)
    chk("|Δy|max ≈ r = %.4f m" % R_M, abs(max_dy - R_M) < 1e-4, "实际 %.6f m" % max_dy)
    dz_end_expect = q16(DZ_M)
    chk("Δz(60s) = Q16.16(0.100) = %d" % dz_end_expect, data[-1][3] == dz_end_expect,
        "实际 %d (= %.6f m)" % (data[-1][3], data[-1][3] / SC))
    chk("姿态增量恒 0", all(r[4] == r[5] == r[6] == 0 for r in data), "drx/dry/drz")
    s_end = s_quintic(1.0)
    theta_end = TURNS * 2.0 * math.pi * s_end
    chk("θ 端点 = 6π", abs(theta_end - 6.0 * math.pi) < 1e-12,
        "θ(60s) = %.12f = %.6fπ" % (theta_end, theta_end / math.pi))
    chk("t_ms 端点", data[0][0] == 0 and data[-1][0] == 60000,
        "%d..%d" % (data[0][0], data[-1][0]))

    print("输出: %s" % out_csv)
    if not ok:
        print("GEN_TRAJ_FAIL")
        sys.exit(1)
    print("GEN_TRAJ_OK")

if __name__ == "__main__":
    main()
