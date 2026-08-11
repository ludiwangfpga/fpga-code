#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""force_golden.py — R1.3 力链路全链 Q16.16 定点对拍黄金参考（双精度浮点）

复刻 tools/fk_golden.py 的 1004 例 FK 对拍范式，把它搬到"力链路"上：

    F   = K·(x_des − x) − D·ẋ            (impedance/wrench_calc.v)
    τ   = Jᵀ·F                            (impedance/impedance_torque.v)
    i_q*= sat(τ·KT_INV >> 16, ±2000)      (foc/torque_to_iq.v)
    τ_ext = Kt·i_q,actual − τ_cmd          (V3/src/udp_fk_cmd.v op15 内联, 论文式(8))

用法（在仓库根目录跑）:
    python tools/force_golden.py gen   [--n 1000] [--seed 20260809]
        -> 写 sim/r1_3_vectors.mem (每例 36 个 32bit hex 字) 与 sim/r1_3_meta.txt

    python tools/force_golden.py check
        -> 读 sim/r1_3_vectors.mem + sim/r1_3_rtl_out.mem (由 tb_r1_3_force_precision 导出),
           逐例用双精度重算参考值, 打印误差统计表 + 饱和/溢出计数

    python tools/force_golden.py selftest
        -> 用 fk_golden.py 里已板验的 M4.1/M4.2/M4.3/M5.1 手算样例自检定点镜像模型

设计要点（必须与 RTL 位级一致）:
  * 定点乘 fmul64(a,b) = (a*b) >>> 16  —— 算术右移 = 向 −∞ 取整（截断，非四舍五入）
  * wrench_calc / impedance_torque 的中间量为 64 位, 仅在写回 32 位时 sat32(±2147483647)
  * torque_to_iq 的 fmult 取 temp[47:16]（即 (a*b)>>16 的低 32 位）, 再对 ±2000 饱和,
    输出取 val[15:0] —— 注意该 ±2000 是对 **Q16.16 数值** 比较, 不是对"安培整数"比较
  * τ_ext 用 KT_Q16=3277 直接相乘且 **不右移**, 结果截断到 32 位（无饱和）
"""

import argparse
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fk_golden as fkg  # noqa: E402  复用 DH 常量 / fk() / jac()

SC = 65536.0
SAT32 = 2147483647
INT32_MIN = -2147483648

KT = 0.05                 # 电机力矩常数 N·m/A（论文式(7)(8)）
KT_INV_RTL = 1310720      # foc/torque_to_iq.v 默认参数 = 20.0 的 Q16.16
KT_INV_FIX = 20           # 参数覆写版：整数 20 = 1/Kt，使 i_q* 直接落在"安培"量纲
KT_Q16 = 3277             # V3/src/udp_fk_cmd.v: τ_ext = iq_act*KT_Q16 − τ_cmd
IQ_MAX = 2000
IQ_MIN = -2000

WORDS_IN = 36             # 每例输入字数: q6 + xdes6 + K6 + D6 + xv6 + iqact6
WORDS_OUT = 72            # 每例输出字数: pose6 + J36 + F6 + tau6 + iq_asis6 + iq_fix6 + tauext6

VEC_PATH = os.path.join("sim", "r1_3_vectors.mem")
META_PATH = os.path.join("sim", "r1_3_meta.txt")
OUT_PATH = os.path.join("sim", "r1_3_rtl_out.mem")


# ---------------------------------------------------------------- 定点工具
def to_i32(u):
    u &= 0xFFFFFFFF
    return u - (1 << 32) if u & 0x80000000 else u


def to_u32(i):
    return i & 0xFFFFFFFF


def q16(v):
    """实数 -> Q16.16 整数（四舍五入 + int32 饱和）"""
    i = int(round(v * 65536.0))
    if i > SAT32:
        i = SAT32
    if i < -SAT32:
        i = -SAT32
    return i


def r16(i):
    return i / 65536.0


def fmul64(a, b):
    """RTL fmul64: (a*b) >>> 16, Python 的 >> 对负数即算术右移"""
    return (a * b) >> 16


def sat32(v):
    if v > SAT32:
        return SAT32
    if v < -SAT32:
        return -SAT32
    return v


# ------------------------------------------------- 定点镜像（位级复刻 RTL）
def fx_wrench(xd, p, K, D, xv):
    """impedance/wrench_calc.v 的位级镜像。返回 (F[6], dx_word[6])"""
    F = []
    dx = []
    for i in range(6):
        d = to_i32(xd[i] - p[i])          # RTL 用 32 位差, 会回绕
        dx.append(d)
        kp = fmul64(K[i], d)
        dv = fmul64(D[i], xv[i])
        F.append(sat32(kp - dv))
    return F, dx


def fx_jt(J, F):
    """impedance/impedance_torque.v 的位级镜像：τ[i] = sat32(Σ_r (J[r][i]·F[r])>>16)"""
    tau = []
    for i in range(6):
        acc = 0
        for r in range(6):
            acc += fmul64(J[r * 6 + i], F[r])
        tau.append(sat32(acc))
    return tau


def fx_torque_to_iq(tau, kt_inv):
    """foc/torque_to_iq.v 的位级镜像：fmult 取 temp[47:16], 再对 ±2000 饱和"""
    out = []
    for t in tau:
        temp = t * kt_inv
        v = to_i32(temp >> 16)            # temp[47:16]
        if v > IQ_MAX:
            v = IQ_MAX
        elif v < IQ_MIN:
            v = IQ_MIN
        else:
            v = v & 0xFFFF                # val[15:0]
            v = v - 0x10000 if v & 0x8000 else v
        out.append(v)
    return out


def fx_tau_ext(iq_act, tau_cmd):
    """V3/src/udp_fk_cmd.v op15 内联：tauext <= (sext(iq_act)*KT_Q16) - tau_cmd, 截 32 位"""
    out = []
    raw = []
    for i in range(6):
        exact = iq_act[i] * KT_Q16 - tau_cmd[i]
        raw.append(exact)
        out.append(to_i32(exact))         # 写进 reg[31:0]，无饱和 -> 回绕
    return out, raw


# ------------------------------------------------------ 双精度浮点黄金参考
def ref_chain(xd_w, p_w, K_w, D_w, xv_w, J_w, iq_act):
    """全部输入取 RTL 实际收到的 Q16.16 字 / 65536（与 fk_golden 同口径），
    随后一律用双精度实数运算，仅在 RTL 明确定义的饱和点上做同样的钳位。"""
    xd = [r16(v) for v in xd_w]
    p = [r16(v) for v in p_w]
    K = [r16(v) for v in K_w]
    D = [r16(v) for v in D_w]
    xv = [r16(v) for v in xv_w]
    J = [r16(v) for v in J_w]
    SATR = SAT32 / 65536.0

    F = []
    for i in range(6):
        v = K[i] * (xd[i] - p[i]) - D[i] * xv[i]
        F.append(max(-SATR, min(SATR, v)))

    tau = []
    for i in range(6):
        s = 0.0
        for r in range(6):
            s += J[r * 6 + i] * F[r]
        tau.append(max(-SATR, min(SATR, s)))

    # (a) RTL 自身声明的算术：i_q* = (τ/Kt) 的 Q16.16 数值, 对 ±2000 饱和
    iq_asis = [max(-2000.0, min(2000.0, t / KT * 65536.0)) for t in tau]
    # (b) 论文式(7)口径：i_q* = τ/Kt (安培), 对 ±2000 A 饱和
    iq_phys = [max(-2000.0, min(2000.0, t / KT)) for t in tau]

    tau_ext = [KT * iq_act[i] - tau[i] for i in range(6)]
    return F, tau, iq_asis, iq_phys, tau_ext


# ------------------------------------------------------------------ 采样
def jac_f(qi):
    """用 fk_golden.jac() 拿 Q16.16 J（仅用于 **设计激励幅度**，不用于参考值）"""
    return fkg.jac(qi)


def predict(qi, dx, xvv, K, D):
    """给定实数 Δx/ẋ/K/D，预测 F、τ 幅度（仅用于分层缩放）"""
    Jw = jac_f(qi)
    J = [r16(v) for v in Jw]
    F = [K[i] * dx[i] - D[i] * xvv[i] for i in range(6)]
    tau = [sum(J[r * 6 + i] * F[r] for r in range(6)) for i in range(6)]
    return F, tau


def rand_q(rng):
    return [rng.uniform(-math.pi, math.pi) for _ in range(6)]


def rand_q_singular(rng):
    """近奇异构型：腕部 q4→0（球腕奇异，J 的 3 个转动列退化）"""
    q = rand_q(rng)
    q[4] = rng.choice([0.0, 1e-4, -1e-4, 1e-3, -1e-3, 1e-2, -1e-2])
    if rng.random() < 0.4:
        q[3] = 0.0
    return q


def build_case(rng, stratum):
    """返回 (q_real[6], xdes_real[6], K_real[6], D_real[6], xv_real[6], iq_act[6])"""
    if stratum == "D":
        q = rand_q_singular(rng)
    else:
        q = rand_q(rng)
    qi = [q16(v) for v in q]
    pose = fkg.fk(q)

    # Δx / ẋ 的"形状"
    if stratum == "C":
        dx = [rng.uniform(-2.0, 2.0) for _ in range(6)]
    else:
        dx = [rng.uniform(-0.05, 0.05) for _ in range(3)] + \
             [rng.uniform(-0.30, 0.30) for _ in range(3)]
    xvv = [rng.uniform(-0.5, 0.5) for _ in range(3)] + \
          [rng.uniform(-1.0, 1.0) for _ in range(3)]

    # K / D 的"形状"（对角，平动/转动分档）
    K = [10 ** rng.uniform(1.0, 3.7) for _ in range(3)] + \
        [10 ** rng.uniform(0.0, 2.3) for _ in range(3)]
    D = [10 ** rng.uniform(-1.0, 2.3) for _ in range(3)] + \
        [10 ** rng.uniform(-1.0, 1.3) for _ in range(3)]

    # 按分层缩放 K/D，使 τ（或 F）落在目标带
    Fp, taup = predict(qi, dx, xvv, K, D)
    if stratum == "B":
        target = 10 ** rng.uniform(-5.0, math.log10(1.5e-3))   # 目标 |τ|max，落在 iq 未饱和线性区
        cur = max(abs(t) for t in taup) or 1.0
        s = target / cur
    elif stratum == "A" or stratum == "D":
        target = 10 ** rng.uniform(-3.0, 1.0)                  # 1 mN·m .. 10 N·m 工作区
        cur = max(abs(t) for t in taup) or 1.0
        s = target / cur
    else:  # C: 逼近并跨越 sat32(±32768) 边界
        target = 10 ** rng.uniform(2.0, 4.8)                   # |F|max 100 .. 63 000
        cur = max(abs(f) for f in Fp) or 1.0
        s = target / cur
    K = [min(32767.0, max(-32767.0, v * s)) for v in K]
    D = [min(32767.0, max(-32767.0, v * s)) for v in D]

    xdes = [pose[i] + dx[i] for i in range(6)]

    # i_q,actual：整数（FOC 反馈量），覆盖 ±2000 钳位与 ±2047 满量程边界
    iq_act = []
    for _ in range(6):
        u = rng.random()
        if u < 0.15:
            iq_act.append(rng.choice([0, 1, -1, 2000, -2000, 2047, -2048]))
        else:
            iq_act.append(rng.randint(-2048, 2047))
    return q, xdes, K, D, xvv, iq_act


SPECIALS = [
    # (tag, q, dx, K, D, xv, iq_act) —— dx/K/D/xv 给实数, xdes = fk(q)+dx
    ("S0_home_zero_load",
     [0.0] * 6, [0.0] * 6, [800.0] * 3 + [40.0] * 3, [50.0] * 3 + [5.0] * 3,
     [0.0] * 6, [0] * 6),
    ("S1_nontrivial_paper",
     [34315 / SC, -51472 / SC, 68629 / SC, 0.0, 102944 / SC, 0.0],
     [0.05, -0.03, 0.02, 0.0, 0.1, -0.05],
     [800.0] * 3 + [40.0] * 3, [50.0] * 3 + [5.0] * 3,
     [0.02, 0.01, -0.01, 0.005, -0.02, 0.01], [100, -100, 50, -50, 2000, -2000]),
    ("S2_wrist_singular_q4_0",
     [0.3, -0.5, 0.7, 0.0, 0.0, 0.0], [0.02, 0.02, 0.02, 0.05, 0.05, 0.05],
     [1000.0] * 3 + [50.0] * 3, [20.0] * 3 + [2.0] * 3, [0.1] * 3 + [0.2] * 3,
     [500, -500, 500, -500, 500, -500]),
    ("S3_elbow_stretched",
     [0.0, 0.0, 0.0, 0.0, math.pi / 2, 0.0], [0.03, -0.03, 0.03, 0.1, -0.1, 0.1],
     [2000.0] * 3 + [100.0] * 3, [100.0] * 3 + [10.0] * 3, [0.3] * 6,
     [2047, -2048, 0, 1, -1, 1234]),
    ("S4_joint_limit_pm_pi",
     [math.pi, -math.pi, math.pi, -math.pi, math.pi, -math.pi],
     [0.04, 0.04, -0.04, 0.2, -0.2, 0.2],
     [1500.0] * 3 + [80.0] * 3, [60.0] * 3 + [6.0] * 3, [-0.4] * 3 + [0.8] * 3,
     [-2000, 2000, -1000, 1000, -1, 1]),
    ("S5_F_saturation",
     [0.5, 0.4, -0.6, 0.2, 0.8, -0.3], [2.0] * 6,
     [32767.0] * 6, [0.0] * 6, [0.0] * 6, [2047] * 6),
    ("S6_iq_sat_boundary",
     [0.2, -0.4, 0.6, 0.1, -0.5, 0.3], [0.001] * 3 + [0.001] * 3,
     [0.05] * 3 + [0.05] * 3, [0.0] * 6, [0.0] * 6, [0] * 6),
    ("S7_tauext_overflow_probe",
     [0.7, 0.2, -0.9, 0.4, 1.1, -0.2], [2.0] * 6,
     [32767.0] * 6, [0.0] * 6, [0.0] * 6, [2047, 2047, 2047, -2048, -2048, -2048]),
]


def gen(n_random, seed):
    rng = random.Random(seed)
    strata = ["A"] * int(n_random * 0.40) + ["B"] * int(n_random * 0.30) + \
             ["C"] * int(n_random * 0.20) + ["D"] * int(n_random * 0.10)
    while len(strata) < n_random:
        strata.append("A")

    words = []
    meta = []

    def emit(tag, stratum, q, xdes, K, D, xv, iq_act):
        qi = [q16(v) for v in q]
        w = qi + [q16(v) for v in xdes] + [q16(v) for v in K] + \
            [q16(v) for v in D] + [q16(v) for v in xv] + \
            [max(-32768, min(32767, int(v))) for v in iq_act]
        assert len(w) == WORDS_IN
        words.extend(w)
        meta.append((len(meta), stratum, tag))

    for tag, q, dx, K, D, xv, iq_act in SPECIALS:
        pose = fkg.fk(q)
        xdes = [pose[i] + dx[i] for i in range(6)]
        emit(tag, "S", q, xdes, K, D, xv, iq_act)

    for k, st in enumerate(strata):
        q, xdes, K, D, xv, iq_act = build_case(rng, st)
        emit("R%04d" % k, st, q, xdes, K, D, xv, iq_act)

    os.makedirs("sim", exist_ok=True)
    with open(VEC_PATH, "w") as f:
        for w in words:
            f.write("%08x\n" % to_u32(w))
    with open(META_PATH, "w") as f:
        f.write("# n_cases=%d words_in=%d words_out=%d seed=%d\n" %
                (len(meta), WORDS_IN, WORDS_OUT, seed))
        for idx, st, tag in meta:
            f.write("%d %s %s\n" % (idx, st, tag))
    print("GEN: %d cases (%d specials + %d random), %d words -> %s" %
          (len(meta), len(SPECIALS), len(strata), len(words), VEC_PATH))
    print("GEN_OK")


# ------------------------------------------------------------------ 统计
def pct(sv, p):
    if not sv:
        return 0.0
    k = (len(sv) - 1) * p / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    if lo == hi:
        return sv[int(k)]
    return sv[lo] + (sv[hi] - sv[lo]) * (k - lo)


class Stat(object):
    def __init__(self, name, unit):
        self.name, self.unit = name, unit
        self.abs = []
        self.signed = []

    def add(self, rtl, ref):
        self.abs.append(abs(rtl - ref))
        self.signed.append(rtl - ref)

    def row(self, scale=1.0):
        if not self.abs:
            return None
        s = sorted(self.abs)
        n = len(s)
        mean = sum(s) / n
        var = sum((x - mean) ** 2 for x in s) / n
        ms = sum(self.signed) / n
        return dict(name=self.name, unit=self.unit, n=n,
                    mx=s[-1] * scale, mean=mean * scale, std=math.sqrt(var) * scale,
                    p95=pct(s, 95) * scale, p99=pct(s, 99) * scale,
                    msigned=ms * scale)


def fmt(v):
    if v == 0:
        return "0"
    a = abs(v)
    if a >= 1e4 or a < 1e-3:
        return "%.3e" % v
    return "%.6g" % v


def check():
    if not os.path.exists(VEC_PATH):
        print("MISSING %s — 先跑 gen" % VEC_PATH)
        return 1
    if not os.path.exists(OUT_PATH):
        print("MISSING %s — 先跑 xsim (run_sim_r1_3.bat)" % OUT_PATH)
        return 1
    vin = [to_i32(int(l, 16)) for l in open(VEC_PATH) if l.strip()]
    vout = [to_i32(int(l, 16)) for l in open(OUT_PATH) if l.strip()]
    meta = []
    for line in open(META_PATH):
        if line.startswith("#"):
            continue
        p = line.split()
        meta.append((int(p[0]), p[1], p[2]))
    n = len(vin) // WORDS_IN
    if len(vout) // WORDS_OUT != n:
        print("SIZE MISMATCH: vec %d cases, rtl_out %d cases" % (n, len(vout) // WORDS_OUT))
        return 1

    st_F_lin = Stat("F 平动 (rows 0-2)", "N")
    st_F_rot = Stat("F 转动 (rows 3-5)", "N.m")
    st_tau = Stat("tau = J^T F", "N.m")
    st_iq_asis = Stat("i_q* (as-is, RTL 自身算术)", "LSB")
    st_iq_asis_lin = Stat("i_q* (as-is, 未饱和子集)", "LSB")
    st_iq_fix = Stat("i_q* (KT_INV=20 参数覆写)", "A")
    st_trt_asis = Stat("Kt*i_q* − tau, |tau|<=100 (as-is)", "N.m")
    st_trt_fix = Stat("Kt*i_q* − tau, |tau|<=100 (fix)", "N.m")
    st_tauext = Stat("tau_ext = Kt*iq_act − tau", "N.m")
    st_tauext_ok = Stat("tau_ext (未溢出子集)", "N.m")
    st_tauext_dec = Stat("tau_ext vs 声明常数 Kt=3277/2^16", "N.m")

    per_stratum = {}
    n_Fsat = n_tausat = n_iqsat_asis = n_iqsat_fix = n_extovf = 0
    n_dx_ovf = 0
    n_iq_slice_ovf = 0        # torque_to_iq fmult 取 temp[47:16] 越界 -> 回绕
    n_trt_in = 0              # |tau_ref| <= 2000*Kt = 100 N.m 的样本数
    n_sign_tot = n_sign_bad = 0   # 非零 tau 下 i_q*(as-is) 的符号是否与 tau 同号
    sat_by_stratum = {}       # 分层 as-is 饱和率
    mism_fx = 0          # 定点镜像 vs RTL 不一致（应为 0）
    mism_detail = []

    for c in range(n):
        bi = c * WORDS_IN
        bo = c * WORDS_OUT
        q_w = vin[bi:bi + 6]
        xd_w = vin[bi + 6:bi + 12]
        K_w = vin[bi + 12:bi + 18]
        D_w = vin[bi + 18:bi + 24]
        xv_w = vin[bi + 24:bi + 30]
        iq_act = vin[bi + 30:bi + 36]

        p_w = vout[bo:bo + 6]
        J_w = vout[bo + 6:bo + 42]
        F_w = vout[bo + 42:bo + 48]
        tau_w = vout[bo + 48:bo + 54]
        iqa_w = vout[bo + 54:bo + 60]
        iqf_w = vout[bo + 60:bo + 66]
        ext_w = vout[bo + 66:bo + 72]

        # --- 定点镜像自检：Python 位级模型必须与 RTL 完全相等 ---
        mF, mdx = fx_wrench(xd_w, p_w, K_w, D_w, xv_w)
        mtau = fx_jt(J_w, mF)
        miqa = fx_torque_to_iq(mtau, KT_INV_RTL)
        miqf = fx_torque_to_iq(mtau, KT_INV_FIX)
        mext, mext_raw = fx_tau_ext(iq_act, mtau)
        if (mF != F_w or mtau != tau_w or miqa != iqa_w or
                miqf != iqf_w or mext != ext_w):
            mism_fx += 1
            if len(mism_detail) < 5:
                mism_detail.append((c, F_w, mF, tau_w, mtau, iqa_w, miqa))

        # --- 溢出/饱和统计（用精确整数判断） ---
        for i in range(6):
            if xd_w[i] - p_w[i] != mdx[i]:
                n_dx_ovf += 1
        for i in range(6):
            if abs(F_w[i]) == SAT32:
                n_Fsat += 1
            if abs(tau_w[i]) == SAT32:
                n_tausat += 1
            if abs(iqa_w[i]) == IQ_MAX:
                n_iqsat_asis += 1
            if abs(iqf_w[i]) == IQ_MAX:
                n_iqsat_fix += 1
            if not (INT32_MIN <= mext_raw[i] <= SAT32):
                n_extovf += 1
            # torque_to_iq: fmult 取 temp[47:16] 只有 32 位, |tau*KT_INV|>=2^47 时该切片回绕
            if not (INT32_MIN <= (tau_w[i] * KT_INV_RTL) >> 16 <= SAT32):
                n_iq_slice_ovf += 1

        # --- 双精度参考 ---
        rF, rtau, riq_asis, riq_phys, rext = ref_chain(
            xd_w, p_w, K_w, D_w, xv_w, J_w, iq_act)

        strat = meta[c][1]
        ps = per_stratum.setdefault(strat, [Stat("F", "N"), Stat("tau", "N.m"),
                                            Stat("iq", "LSB"), Stat("ext", "N.m")])
        sb = sat_by_stratum.setdefault(strat, [0, 0])
        for i in range(6):
            sb[1] += 1
            if abs(iqa_w[i]) == IQ_MAX:
                sb[0] += 1
            # 符号一致性只在 |τ| 明显高于量化底噪 (1e-3 N·m ≈ 65 LSB) 时才有意义
            if abs(rtau[i]) > 1e-3:
                n_sign_tot += 1
                if (iqa_w[i] > 0) != (rtau[i] > 0) and iqa_w[i] != 0:
                    n_sign_bad += 1
        for i in range(6):
            fr = r16(F_w[i])
            (st_F_lin if i < 3 else st_F_rot).add(fr, rF[i])
            ps[0].add(fr, rF[i])
            tr = r16(tau_w[i])
            st_tau.add(tr, rtau[i])
            ps[1].add(tr, rtau[i])
            st_iq_asis.add(float(iqa_w[i]), riq_asis[i])
            ps[2].add(float(iqa_w[i]), riq_asis[i])
            if abs(iqa_w[i]) < IQ_MAX and abs(riq_asis[i]) < IQ_MAX:
                st_iq_asis_lin.add(float(iqa_w[i]), riq_asis[i])
            st_iq_fix.add(float(iqf_w[i]), riq_phys[i])
            # 往返：论文式(7) τ = Kt·i_q，理想 Kt·i_q* 应还原 τ_cmd。
            # 只统计"物理正确口径下本就不该被 ±2000 钳位"的样本 (|τ| <= 2000·Kt = 100 N·m)
            if abs(rtau[i]) <= IQ_MAX * KT:
                n_trt_in += 1
                st_trt_asis.add(KT * iqa_w[i], rtau[i])
                st_trt_fix.add(KT * iqf_w[i], rtau[i])
            er = r16(ext_w[i])
            st_tauext.add(er, rext[i])
            ps[3].add(er, rext[i])
            if INT32_MIN <= mext_raw[i] <= SAT32:
                st_tauext_ok.add(er, rext[i])
                # 用 RTL 声明的常数 3277/2^16 而非精确 0.05 做参考 -> 分离"常数量化" 与 "运算误差"
                st_tauext_dec.add(er, r16(KT_Q16) * iq_act[i] - rtau[i])

    # ---------------------------------------------------------------- 输出
    print("")
    print("=" * 100)
    print("R1.3 力链路全链 Q16.16 定点 vs 双精度浮点对拍  —  %d 例 (%d 特殊 + %d 蒙特卡洛), 每例 6 通道"
          % (n, len(SPECIALS), n - len(SPECIALS)))
    print("=" * 100)
    print("定点镜像自检 (Python 位级模型 vs RTL 导出): %s  (不一致例数 = %d)"
          % ("BITEXACT" if mism_fx == 0 else "MISMATCH", mism_fx))
    for d in mism_detail:
        print("   case %d: F rtl=%s mdl=%s | tau rtl=%s mdl=%s | iq rtl=%s mdl=%s" % d)
    print("")
    hdr = ("%-34s %-6s %6s %12s %12s %12s %12s %12s %13s" %
           ("指标", "单位", "N", "最大", "均值", "标准差", "P95", "P99", "均值(带符号)"))
    print(hdr)
    print("-" * len(hdr))
    for s in [st_F_lin, st_F_rot, st_tau, st_iq_asis, st_iq_asis_lin,
              st_iq_fix, st_trt_asis, st_trt_fix, st_tauext, st_tauext_ok,
              st_tauext_dec]:
        r = s.row()
        if r is None:
            print("%-34s %-6s   (无样本)" % (s.name, s.unit))
            continue
        print("%-34s %-6s %6d %12s %12s %12s %12s %12s %13s" %
              (r["name"], r["unit"], r["n"], fmt(r["mx"]), fmt(r["mean"]),
               fmt(r["std"]), fmt(r["p95"]), fmt(r["p99"]), fmt(r["msigned"])))
    print("")
    print("以 LSB(=2^-16) 计的定点量化误差:")
    for s in [st_F_lin, st_F_rot, st_tau, st_tauext_ok]:
        r = s.row(scale=65536.0)
        print("  %-34s 最大 %8.3f LSB  均值 %8.4f LSB  带符号均值 %+8.4f LSB"
              % (r["name"], r["mx"], r["mean"], r["msigned"]))
    print("")
    tot = n * 6
    print("饱和 / 溢出计数 (共 %d 个通道样本):" % tot)
    print("  wrench_calc  sat32 命中 F      : %6d (%.2f%%)" % (n_Fsat, 100.0 * n_Fsat / tot))
    print("  impedance_torque sat32 命中 tau: %6d (%.2f%%)" % (n_tausat, 100.0 * n_tausat / tot))
    print("  torque_to_iq  ±2000 饱和 (as-is)      : %6d (%.2f%%)"
          % (n_iqsat_asis, 100.0 * n_iqsat_asis / tot))
    print("  torque_to_iq  ±2000 饱和 (KT_INV=20)  : %6d (%.2f%%)"
          % (n_iqsat_fix, 100.0 * n_iqsat_fix / tot))
    print("  torque_to_iq  temp[47:16] 切片回绕     : %6d (%.2f%%)"
          % (n_iq_slice_ovf, 100.0 * n_iq_slice_ovf / tot))
    print("  tau_ext 32 位回绕 (无饱和保护)         : %6d (%.2f%%)"
          % (n_extovf, 100.0 * n_extovf / tot))
    print("  Δx = x_des − pose 的 int32 差回绕      : %6d (%.2f%%)"
          % (n_dx_ovf, 100.0 * n_dx_ovf / tot))
    print("")
    print("量纲/门限关键数（推导 + 实测一致）:")
    print("  torque_to_iq as-is  : i_q* = sat(τ·%d/2^16, ±%d) -> 增益 %d LSB/(N·m) = 2^16 × (1/Kt)"
          % (KT_INV_RTL, IQ_MAX, KT_INV_RTL))
    print("                        ±2000 钳位实际在 |τ| > %.6g N·m 触发（论文式(7)口径应为 %.6g N·m）"
          % (IQ_MAX * 65536.0 / KT_INV_RTL / 65536.0, IQ_MAX * KT))
    print("                        temp[47:16] 切片在 |τ| >= 2^47/%d = %.6g N·m 回绕（可致符号翻转）"
          % (KT_INV_RTL, (2 ** 47) / KT_INV_RTL / 65536.0))
    print("  torque_to_iq fix    : KT_INV=20 -> i_q*=trunc(τ/Kt) A, 分辨率 1 A/LSB, 钳位在 ±%.6g N·m"
          % (IQ_MAX * KT))
    print("  τ_ext 常数 Kt       : KT_Q16=%d -> %.9f (精确 %.2f), 相对增益偏差 %+.1f ppm"
          % (KT_Q16, KT_Q16 / 65536.0, KT, (KT_Q16 / 65536.0 / KT - 1) * 1e6))
    print("  |τ|<=2000·Kt=100 N·m 的往返样本数: %d / %d" % (n_trt_in, tot))
    print("  i_q*(as-is) 符号与 τ 同号 (仅 |τ|>1e-3 N·m): %d / %d  (符号错 %d, %.3f%%)"
          % (n_sign_tot - n_sign_bad, n_sign_tot, n_sign_bad,
             100.0 * n_sign_bad / max(1, n_sign_tot)))
    print("")
    print("分层 i_q*(as-is) ±2000 饱和率:")
    for k in sorted(sat_by_stratum):
        s0, s1 = sat_by_stratum[k]
        print("  %-2s : %5d / %5d = %6.2f%%" % (k, s0, s1, 100.0 * s0 / s1))
    print("")
    print("分层误差 (最大值):")
    print("  %-4s %6s %12s %12s %12s %12s" % ("层", "例数", "F(N)", "tau(N.m)", "iq(LSB)", "tau_ext(N.m)"))
    names = {"S": "特殊", "A": "工作区", "B": "小信号", "C": "极限", "D": "近奇异"}
    for k in sorted(per_stratum):
        a, b, cc, d = per_stratum[k]
        print("  %-4s %6d %12s %12s %12s %12s" %
              (k + "(" + names.get(k, "?") + ")", len(a.abs) // 6,
               fmt(a.row()["mx"]), fmt(b.row()["mx"]),
               fmt(cc.row()["mx"]), fmt(d.row()["mx"])))
    print("=" * 100)
    if mism_fx == 0:
        print("R1_3_CHECK_OK")
    else:
        print("R1_3_CHECK_MISMATCH")
    return 0


# ---------------------------------------------------------------- selftest
def selftest():
    """用 fk_golden.py 内既有(已板验)的 M4.x/M5.1 手算样例交叉验证定点镜像"""
    ok = True
    qn = [34315, -51472, 68629, 0, 102944, 0]
    Jrm = fkg.jac(qn)
    pose = [20053, 11578, -8661, -1, -85786, -171571]
    xdes = [23329, 9612, -7350, -1, -79233, -174848]
    K = [52428800] * 3 + [2621440] * 3
    D = [3276800] * 3 + [327680] * 3
    xdot = [1311, 655, -655, 328, -1311, 655]
    F, _ = fx_wrench(xdes, pose, K, D, xdot)
    exp_F = [sat32(fmul64(K[i], xdes[i] - pose[i]) - fmul64(D[i], xdot[i])) for i in range(6)]
    if F != exp_F:
        print("SELFTEST FAIL wrench: %s vs %s" % (F, exp_F))
        ok = False
    tau = fx_jt(Jrm, F)
    exp_tau = [sat32(sum(fmul64(Jrm[r * 6 + i], F[r]) for r in range(6))) for i in range(6)]
    if tau != exp_tau:
        print("SELFTEST FAIL jt: %s vs %s" % (tau, exp_tau))
        ok = False
    print("M4.3 F   =", F)
    print("M4.3 tau =", tau)
    small = [1, -1, 1, 0, 2, -1]
    Ks = [6553600] * 3 + [655360] * 3
    Ds = [655360] * 3 + [65536] * 3
    xds = [6, -6, 3, 0, -6, 3]
    Fs = [sat32(fmul64(Ks[i], small[i]) - fmul64(Ds[i], xds[i])) for i in range(6)]
    ts = fx_jt(Jrm, Fs)
    print("M5.1 tau =", ts)
    print("M5.1 iq(as-is)      =", fx_torque_to_iq(ts, KT_INV_RTL))
    print("M5.1 iq(KT_INV=20)  =", fx_torque_to_iq(ts, KT_INV_FIX))
    print("torque_to_iq 饱和门限: |tau| > %d LSB = %.6g N.m"
          % (IQ_MAX * 65536 // KT_INV_RTL, IQ_MAX * 65536.0 / KT_INV_RTL / 65536.0))
    print("SELFTEST_OK" if ok else "SELFTEST_FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["gen", "check", "selftest"])
    ap.add_argument("--n", type=int, default=1000)
    ap.add_argument("--seed", type=int, default=20260809)
    a = ap.parse_args()
    if a.mode == "gen":
        return gen(a.n, a.seed)
    if a.mode == "check":
        return check()
    return selftest()


if __name__ == "__main__":
    sys.exit(main() or 0)
