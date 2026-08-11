#!/usr/bin/env python3
# fk_golden.py — 用 Q16.16 DH 常量 + 与 fk_engine 相同的 atan2 欧拉约定算黄金位姿。
# 用法: python tools/fk_golden.py
import math
d  = [10879,0,0,12583,0,3604]      # DH d (Q16.16)
a  = [3604,13107,3670,0,0,0]       # DH a (Q16.16)
al = [102944,0,102944,-102944,102944,0]  # DH alpha (Q16.16)
SC = 65536.0
dm=[x/SC for x in d]; am=[x/SC for x in a]; alm=[x/SC for x in al]
def mm(A,B): return [[sum(A[i][k]*B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
def fk(q):
    T=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    for i in range(6):
        ct,st=math.cos(q[i]),math.sin(q[i]); ca,sa=math.cos(alm[i]),math.sin(alm[i])
        Ti=[[ct,-st*ca,st*sa,am[i]*ct],[st,ct*ca,-ct*sa,am[i]*st],[0,sa,ca,dm[i]],[0,0,0,1]]
        T=mm(T,Ti)
    rz=math.atan2(T[1][0],T[0][0]); ry=math.atan2(-T[2][0],math.sqrt(T[0][0]**2+T[1][0]**2)); rx=math.atan2(T[2][1],T[2][2])
    return [T[0][3],T[1][3],T[2][3],rx,ry,rz]
def q16(v): return int(round(v*65536))
def jac(qi):
    q=[v/SC for v in qi]
    T=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]; Ts=[]
    for i in range(6):
        ct,st=math.cos(q[i]),math.sin(q[i]); ca,sa=math.cos(alm[i]),math.sin(alm[i])
        Ti=[[ct,-st*ca,st*sa,am[i]*ct],[st,ct*ca,-ct*sa,am[i]*st],[0,sa,ca,dm[i]],[0,0,0,1]]
        T=mm(T,Ti); Ts.append([r[:] for r in T])
    pe=[Ts[5][0][3],Ts[5][1][3],Ts[5][2][3]]
    def frame(k):
        if k<0: return [0,0,1],[0,0,0]
        return [Ts[k][0][2],Ts[k][1][2],Ts[k][2][2]],[Ts[k][0][3],Ts[k][1][3],Ts[k][2][3]]
    def cross(z,dp): return [z[1]*dp[2]-z[2]*dp[1],z[2]*dp[0]-z[0]*dp[2],z[0]*dp[1]-z[1]*dp[0]]
    J=[[0]*6 for _ in range(6)]
    for c in range(6):
        z,p=frame(c-1); dp=[pe[i]-p[i] for i in range(3)]; v=cross(z,dp)
        J[0][c],J[1][c],J[2][c]=v; J[3][c],J[4][c],J[5][c]=z
    return [q16(J[r][c]) for r in range(6) for c in range(6)]   # row-major 36
if __name__=="__main__":
    # qi 为 FPGA 实际收到的 Q16.16 关节角整数
    for name,qi in [("home",[0,0,0,0,0,0]),
                    ("nontrivial",[34315,-51472,68629,0,102944,0]),
                    ("generic",[22877,11439,-34315,17157,45751,-28598])]:
        p=fk([v/SC for v in qi])
        print(name, "Q16.16:", [q16(v) for v in p])
        print(name, "J(36):", jac(qi))
    # ---- 阻抗 τ 黄金值 (Q16.16). 定点镜像 fmult: (a*b)>>16, 64位累加, sat32. ----
    def fmult_i(ai,bi):
        s=((ai*bi)>>16)&0xFFFFFFFF; return s-0x100000000 if s&0x80000000 else s
    def sat32(v): return 2147483647 if v>2147483647 else (-2147483647 if v<-2147483647 else v)
    qn=[34315,-51472,68629,0,102944,0]              # nontrivial
    Jrm=jac(qn)                                      # 行优先 36 (= gold_j)
    Jq=[[Jrm[r*6+c] for c in range(6)] for r in range(6)]
    def tau_from(Fi): return [sat32(sum(fmult_i(Jq[r][i],Fi[r]) for r in range(6))) for i in range(6)]
    print("M4.1 F=[1,0.5,-0.5,0.25,0.1,-0.1]:")
    print("  Fi  =", [65536,32768,-32768,16384,6554,-6554])
    print("  tau =", tau_from([65536,32768,-32768,16384,6554,-6554]))
    print("M4.1 sat F=30000(all):")
    print("  tau =", tau_from([1966080000]*6))
    # ---- M4.2 刚度: F=fmult(K, x_des-pose), 再 τ=Jᵀ·F ----
    pose=[20053,11578,-8661,-1,-85786,-171571]      # nontrivial 位姿(Q16.16)
    xdes=[23329,9612,-7350,-1,-79233,-174848]
    K=[52428800,52428800,52428800,2621440,2621440,2621440]   # 800,800,800,40,40,40
    F2=[sat32(fmult_i(K[i], xdes[i]-pose[i])) for i in range(6)]
    print("M4.2 stiffness:")
    print("  F2  =", F2)
    print("  tau =", tau_from(F2))
    # 集成(delta 法, Δx=delta 精确): delta/K 小, F~1
    delta=[655,-655,655,0,1311,-655]
    Ki=[6553600,6553600,6553600,655360,655360,655360]        # 100,100,100,10,10,10
    F2i=[sat32(fmult_i(Ki[i], delta[i])) for i in range(6)]
    print("M4.2 integ delta:")
    print("  F   =", F2i)
    print("  tau =", tau_from(F2i))
    # ---- M4.3 阻尼: F=sat(fmult(K,Δx)-fmult(D,ẋ)), 再 τ=Jᵀ·F ----
    D=[3276800,3276800,3276800,327680,327680,327680]        # 50,50,50,5,5,5
    xdot=[1311,655,-655,328,-1311,655]
    F3=[sat32(fmult_i(K[i],xdes[i]-pose[i]) - fmult_i(D[i],xdot[i])) for i in range(6)]
    print("M4.3 damping:")
    print("  F3  =", F3)
    print("  tau =", tau_from(F3))
    # 集成(delta 法): F=fmult(Ki,delta)-fmult(Di,xdoti)
    Di=[655360,655360,655360,65536,65536,65536]             # 10,10,10,1,1,1
    xdoti=[655,-655,328,0,-655,328]
    F3i=[sat32(fmult_i(Ki[i],delta[i]) - fmult_i(Di[i],xdoti[i])) for i in range(6)]
    print("M4.3 integ delta:")
    print("  F   =", F3i)
    print("  tau =", tau_from(F3i))
    # ---- M5.1 torque_to_iq: iq = sat((tau·KT_INV)>>16, ±2000) ----
    KT_INV = 1310720    # 20.0 in Q16.16 (1/Kt, Kt=0.05)
    IQ_MAX = 2000
    def to_iq(taus):
        out=[]
        for t in taus:
            v = (t*KT_INV) >> 16          # 算术右移(Python int 即算术)
            if v >  IQ_MAX: v =  IQ_MAX
            elif v < -IQ_MAX: v = -IQ_MAX
            out.append(v)
        return out
    delta_s=[1,-1,1,0,2,-1]                                       # 小增量(Q16.16 LSB)
    Ks=[6553600,6553600,6553600,655360,655360,655360]             # K=[100,100,100,10,10,10]
    Ds=[655360,655360,655360,65536,65536,65536]                   # D=[10,10,10,1,1,1]
    xds=[6,-6,3,0,-6,3]                                          # ẋ(Q16.16 LSB, 缩小到不饱和)
    Fiq=[sat32(fmult_i(Ks[i],delta_s[i]) - fmult_i(Ds[i],xds[i])) for i in range(6)]
    tiq=tau_from(Fiq)
    print("M5.1 iq test (small delta):")
    print("  delta=",delta_s)
    print("  tau =", tiq)
    print("  iq  =", to_iq(tiq))
