#!/usr/bin/env python3
# fk_ab_check.py — R1.2 官方复现管线③: RTL dump vs 双精度参考误差统计。
# 参考模型与 tools/fk_golden.py 完全同口径: Q16.16 DH 常量 /65536, 同一欧拉角提取
#   rz=atan2(T10,T00), ry=atan2(-T20,sqrt(T00^2+T10^2)), rx=atan2(T21,T22)。
# 读 sim/fk_ab_vectors.mem + fk_ab_out.txt(tb_fk_ab dump), 输出位置/姿态误差统计。
# 用法: python tools/fk_ab_check.py [dump文件]   (默认仓库根 fk_ab_out.txt)
import math, os, sys

D  = [10879, 0, 0, 12583, 0, 3604]           # DH d (Q16.16)
A  = [3604, 13107, 3670, 0, 0, 0]            # DH a (Q16.16)
AL = [102944, 0, 102944, -102944, 102944, 0] # DH alpha (Q16.16)
SC = 65536.0

def mm(Ma, Mb):
    return [[sum(Ma[i][k] * Mb[k][j] for k in range(4)) for j in range(4)] for i in range(4)]

def fk_ref(qi):
    q = [v / SC for v in qi]
    dm = [x / SC for x in D]; am = [x / SC for x in A]; alm = [x / SC for x in AL]
    T = [[1.0,0,0,0],[0,1.0,0,0],[0,0,1.0,0],[0,0,0,1.0]]
    for i in range(6):
        ct, st = math.cos(q[i]), math.sin(q[i])
        ca, sa = math.cos(alm[i]), math.sin(alm[i])
        Ti = [[ct, -st*ca,  st*sa, am[i]*ct],
              [st,  ct*ca, -ct*sa, am[i]*st],
              [0.0,    sa,     ca,     dm[i]],
              [0.0,   0.0,    0.0,      1.0]]
        T = mm(T, Ti)
    rz = math.atan2(T[1][0], T[0][0])
    ry = math.atan2(-T[2][0], math.sqrt(T[0][0]**2 + T[1][0]**2))
    rx = math.atan2(T[2][1], T[2][2])
    return [T[0][3], T[1][3], T[2][3], rx, ry, rz]

def ang_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi

def pctl(sorted_vals, p):
    n = len(sorted_vals)
    return sorted_vals[min(n - 1, max(0, int(math.ceil(p * n)) - 1))]

def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    dump = sys.argv[1] if len(sys.argv) > 1 else os.path.join(root, "fk_ab_out.txt")
    memf = os.path.join(root, "sim", "fk_ab_vectors.mem")
    words = []
    with open(memf) as f:
        for ln in f:
            ln = ln.strip()
            if ln:
                v = int(ln, 16)
                words.append(v - 0x100000000 if v & 0x80000000 else v)
    cases = [words[i*6:(i+1)*6] for i in range(len(words) // 6)]
    outs = []
    with open(dump) as f:
        for ln in f:
            ln = ln.split()
            if len(ln) == 6:
                outs.append([int(x) for x in ln])
    if len(outs) != len(cases):
        print("FK_AB_CHECK_FAIL: case count mismatch rtl=%d ref=%d" % (len(outs), len(cases)))
        sys.exit(1)
    ax = [0.0, 0.0, 0.0]  # 逐轴 max, mm
    norms = []            # 欧氏范数, mm
    angs = []             # 姿态误差, deg (三轴混合)
    worst = (-1.0, -1)
    for i, (qi, oi) in enumerate(zip(cases, outs)):
        ref = fk_ref(qi)
        rtl = [v / SC for v in oi]
        d3 = [(rtl[k] - ref[k]) * 1000.0 for k in range(3)]  # m -> mm
        for k in range(3):
            ax[k] = max(ax[k], abs(d3[k]))
        nrm = math.sqrt(d3[0]**2 + d3[1]**2 + d3[2]**2)
        norms.append(nrm)
        if nrm > worst[0]:
            worst = (nrm, i)
        for k in range(3, 6):
            angs.append(abs(math.degrees(ang_diff(rtl[k], ref[k]))))
    n = len(norms)
    mean = sum(norms) / n
    sd = math.sqrt(sum((v - mean)**2 for v in norms) / n)
    s = sorted(norms)
    print("==== FK fixed-point vs double reference (N=%d) ====" % n)
    print("pos err per-axis max (mm): |dx|=%.6f |dy|=%.6f |dz|=%.6f" % (ax[0], ax[1], ax[2]))
    print("pos err euclid (mm): max=%.6f mean=%.6f sd=%.6f p95=%.6f p99=%.6f"
          % (s[-1], mean, sd, pctl(s, 0.95), pctl(s, 0.99)))
    print("pos err worst case idx=%d" % worst[1])
    print("ang err (deg): max=%.6f mean=%.6f" % (max(angs), sum(angs) / len(angs)))
    print("FK_AB_CHECK_OK")

if __name__ == "__main__":
    main()
