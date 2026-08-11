#!/usr/bin/env python3
# fk_ab_gen.py — R1.2 官方复现管线①: 生成 1004 例关节角向量 (Q16.16, 小端语义无关, 每行一个 32bit hex)
# 1000 例 seed=42 均匀 [-π,π]^6 + 4 个特殊构型(全零/全π2/±π边界/腕部奇异邻近)。
# 输出 sim/fk_ab_vectors.mem, 供 sim/tb_fk_ab.v $readmemh 使用。
# 用法: python tools/fk_ab_gen.py   (在仓库根目录执行, 或任意目录——路径按脚本位置定位)
import math, os, random

PI_Q = 205887  # round(pi*65536)

def q16(v):
    x = int(round(v * 65536.0))
    return max(-PI_Q, min(PI_Q, x))

def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rng = random.Random(42)
    cases = []
    for _ in range(1000):
        cases.append([q16(rng.uniform(-math.pi, math.pi)) for _ in range(6)])
    cases.append([0, 0, 0, 0, 0, 0])                                        # 全零
    cases.append([102944] * 6)                                              # 全 π/2
    cases.append([205887, -205887, 205887, -205887, 205887, -205887])       # ±π 边界
    cases.append([6554, 13107, -19661, 26214, 7, -32768])                   # 奇异邻近: q4≈1e-4 rad
    out = os.path.join(root, "sim", "fk_ab_vectors.mem")
    with open(out, "w", newline="\n") as f:
        for c in cases:
            for v in c:
                f.write("%08x\n" % (v & 0xFFFFFFFF))
    print("FK_AB_VECTORS_OK %d cases -> %s" % (len(cases), out))

if __name__ == "__main__":
    main()
