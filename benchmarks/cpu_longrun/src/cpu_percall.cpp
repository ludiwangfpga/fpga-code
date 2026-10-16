// cpu_percall.cpp — CPU 基线「单次调用延迟」测量 (per-call latency, 供尾部/抖动分析)
//
// 与 cpu_baseline.cpp 的关系:
//   * 控制律实现 (control_law 及其全部子函数) 从 cpu_baseline.cpp 逐字复制, 保持位级一致;
//     cpu_baseline.cpp 本身不做任何修改 (它是论文现有 BATCH=100 批平均数据的来源, 必须可复现).
//   * 唯一差别是「测量方式」: 本文件用 __rdtscp 对 **每一次** control_law 调用单独计时,
//     不做任何批平均 —— 批平均会把 100us 尖峰摊薄成 1us, 无法用于尾部 (p99.9/p99.99/max) 分析.
//
// 计时方法 (Intel "How to Benchmark Code Execution Times" 白皮书口径):
//     lfence; rdtsc; lfence   -> t0
//     <被测代码>
//     rdtscp; lfence          -> t1
//   TSC 频率用 QueryPerformanceCounter 在 ~200ms 区间内标定 (启动时打印, 便于复核).
//
// 采样: N = 1,000,000 次单独调用, 预热 10,000 次 (丢弃).
//       每次调用的关节角互不相同 (沿用 cpu_baseline.cpp 的 make_q 递增做法, 覆盖不同 CORDIC 路径),
//       但 q 数组在计时区间之外预生成, 使计时区间内只含 control_law 本身.
//
// 运行环境:
//   cpu_percall.exe          -> quiet  : 被测线程绑核 CPU0 + HIGH_PRIORITY_CLASS, 机器空闲
//   cpu_percall.exe --load   -> loaded : 同上, 且在其它所有逻辑核上各起 1 个满载自旋线程
//
// 输出:
//   控制台: N / TSC 频率 / 计时开销 / mean,SD,min,p50,p95,p99,p99.9,p99.99,max (us) / count(>100us)
//   落盘  : percall_quiet.bin | percall_loaded.bin
//           格式 = 纯二进制, 无文件头, N 个连续 IEEE-754 double (float64), 小端 (x86),
//                  每个元素 = 该次调用的延迟, 单位 us.
//           numpy: np.fromfile(path, dtype='<f8')
//
// 编译口径与 cpu_baseline.cpp 一致: cl /O2 /std:c++17  (不用 /arch:AVX*, 不用 fast-math, 不用 SIMD)

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <vector>
#include <cmath>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <intrin.h>

using q16 = int32_t;
using q64 = int64_t;

// ===============================================================================
// ==== 以下 control_law 及其子函数从 cpu_baseline.cpp 逐字复制, 保持位级一致 ====
// ===============================================================================

// -------------------------------------------------------------------------------
// Q16.16 定点乘 (与 RTL fmult 完全等价: temp = a*b (64bit); fmult = temp[47:16];)
// -------------------------------------------------------------------------------
static inline q16 fmult(q16 a, q16 b) {
    q64 t = (q64)a * (q64)b;
    return (q16)(t >> 16);
}

// 64 位中间结果 (与 wrench_calc / impedance_torque 的 fmul64 等价)
static inline q64 fmul64(q16 a, q16 b) {
    q64 t = (q64)a * (q64)b;
    return t >> 16;
}

// 32 位饱和 (impedance_torque / wrench_calc 的 sat32)
static inline q16 sat32(q64 v) {
    if (v >  (q64)2147483647LL) return  (q16)2147483647;
    if (v < -(q64)2147483647LL) return -(q16)2147483647;
    return (q16)v;
}

// 16 位饱和 (torque_to_iq 的 saturate)
static inline int16_t sat16(q16 v) {
    const q16 IQ_MAX =  2000;
    const q16 IQ_MIN = -2000;
    if (v > IQ_MAX) return (int16_t)IQ_MAX;
    if (v < IQ_MIN) return (int16_t)IQ_MIN;
    return (int16_t)v;
}

// -------------------------------------------------------------------------------
// 常量 (Q16.16, 与 RTL 完全一致)
// -------------------------------------------------------------------------------
static constexpr q16 FIXED_ONE  = 65536;
static constexpr q16 PI_Q       = 205887;   // π
static constexpr q16 PI2_Q      = 102944;   // π/2
static constexpr q16 NEG_PI2_Q  = -102944;
static constexpr q16 NEG_PI_Q   = -205887;
static constexpr q16 CORDIC_K   = 39797;    // 1/K_cordic (Q16.16, 0.6073)

// atan_table[i] = atan(2^-i) in Q16.16, i=0..15
static const q16 ATAN_TABLE[16] = {
    51472, 30386, 16055,  8150,  4091,  2047,  1024,   512,
      256,   128,    64,    32,    16,     8,     4,     2
};

// DH 参数 (Q16.16, 与 part1_config.v 完全一致)
static const q16 DH_D[6]     = { 10879,     0,     0, 12583,     0,  3604 };
static const q16 DH_A[6]     = {  3604, 13107,  3670,     0,     0,     0 };
static const q16 DH_ALPHA[6] = { PI2_Q,    0, PI2_Q, NEG_PI2_Q, PI2_Q, 0 };

// KT_INV (与 torque_to_iq.v 默认值一致, 6 关节全 20.0 Q16.16)
static const q16 KT_INV[6] = { 1310720, 1310720, 1310720, 1310720, 1310720, 1310720 };

// -------------------------------------------------------------------------------
// CORDIC sin/cos (16 迭代, 与 cordic_sincos.v 算法步骤完全一致)
// -------------------------------------------------------------------------------
static inline void cordic_sincos(q16 angle, q16& sin_out, q16& cos_out) {
    q16 x, y, z;
    bool neg_sin = false, neg_cos = false;

    if (angle >= NEG_PI2_Q && angle <= PI2_Q) {
        z = angle;
    } else if (angle > PI2_Q && angle <= PI_Q) {
        z = PI_Q - angle;
        neg_cos = true;
    } else if (angle > PI_Q) {
        z = angle - PI_Q;
        neg_sin = true;
        neg_cos = true;
    } else if (angle < NEG_PI2_Q && angle >= -PI_Q) {
        z = PI_Q + angle;
        neg_sin = true;
        neg_cos = true;
    } else {
        z = angle + (PI_Q << 1);
    }
    x = CORDIC_K;
    y = 0;

    for (int iter = 0; iter < 16; ++iter) {
        q16 x_new, y_new;
        if (z >= 0) {
            x_new = x - (y >> iter);
            y_new = y + (x >> iter);
            z -= ATAN_TABLE[iter];
        } else {
            x_new = x + (y >> iter);
            y_new = y - (x >> iter);
            z += ATAN_TABLE[iter];
        }
        x = x_new;
        y = y_new;
    }

    sin_out = neg_sin ? -y : y;
    cos_out = neg_cos ? -x : x;
}

// -------------------------------------------------------------------------------
// CORDIC atan2 (16 迭代 vectoring 模式, 与 cordic_atan2.v 完全一致)
// -------------------------------------------------------------------------------
static inline void cordic_atan2(q16 x_in, q16 y_in, q16& angle, q16& magnitude) {
    bool x_neg = (x_in < 0);
    bool y_neg = (y_in < 0);

    q16 x = x_neg ? -x_in : x_in;
    q16 y = y_neg ? -y_in : y_in;
    q16 z = 0;

    for (int iter = 0; iter < 16; ++iter) {
        q16 x_new, y_new;
        if (y >= 0) {
            x_new = x + (y >> iter);
            y_new = y - (x >> iter);
            z += ATAN_TABLE[iter];
        } else {
            x_new = x - (y >> iter);
            y_new = y + (x >> iter);
            z -= ATAN_TABLE[iter];
        }
        x = x_new;
        y = y_new;
    }

    magnitude = x;
    if (!x_neg && !y_neg)       angle = z;
    else if (x_neg && !y_neg)   angle = PI_Q - z;
    else if (x_neg && y_neg)    angle = -PI_Q + z;
    else                        angle = -z;
}

// -------------------------------------------------------------------------------
// 4x4 矩阵乘: Tt = Ta * Ti, Q16.16 fmult
// -------------------------------------------------------------------------------
static inline void mat4_mul(const q16 Ta[16], const q16 Ti[16], q16 Tt[16]) {
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            q16 s = 0;
            for (int k = 0; k < 4; ++k) {
                s += fmult(Ta[r*4+k], Ti[k*4+c]);
            }
            Tt[r*4+c] = s;
        }
    }
}

// -------------------------------------------------------------------------------
// FK 引擎 (与 fk_engine.v 完全等价)
// -------------------------------------------------------------------------------
static inline void fk_engine(
    const q16 q[6],
    q16 pose[6],
    q16 frame_z[5][3],
    q16 frame_p[5][3]
){
    q16 Ta[16] = {
        FIXED_ONE, 0, 0, 0,
        0, FIXED_ONE, 0, 0,
        0, 0, FIXED_ONE, 0,
        0, 0, 0, FIXED_ONE
    };

    for (int j = 0; j < 6; ++j) {
        q16 sin_q, cos_q, sin_a, cos_a;
        cordic_sincos(q[j], sin_q, cos_q);
        cordic_sincos(DH_ALPHA[j], sin_a, cos_a);

        q16 Ti[16];
        Ti[ 0] = cos_q;
        Ti[ 1] = -fmult(sin_q, cos_a);
        Ti[ 2] =  fmult(sin_q, sin_a);
        Ti[ 3] =  fmult(DH_A[j], cos_q);

        Ti[ 4] = sin_q;
        Ti[ 5] =  fmult(cos_q, cos_a);
        Ti[ 6] = -fmult(cos_q, sin_a);
        Ti[ 7] =  fmult(DH_A[j], sin_q);

        Ti[ 8] = 0;
        Ti[ 9] = sin_a;
        Ti[10] = cos_a;
        Ti[11] = DH_D[j];

        Ti[12] = 0;
        Ti[13] = 0;
        Ti[14] = 0;
        Ti[15] = FIXED_ONE;

        q16 Tt[16];
        mat4_mul(Ta, Ti, Tt);
        std::memcpy(Ta, Tt, sizeof(Ta));

        if (j < 5) {
            frame_z[j][0] = Ta[ 2];
            frame_z[j][1] = Ta[ 6];
            frame_z[j][2] = Ta[10];
            frame_p[j][0] = Ta[ 3];
            frame_p[j][1] = Ta[ 7];
            frame_p[j][2] = Ta[11];
        }
    }

    pose[0] = Ta[ 3];
    pose[1] = Ta[ 7];
    pose[2] = Ta[11];

    q16 rz, mag1;
    cordic_atan2(Ta[0], Ta[4], rz, mag1);

    q16 sqrt_xy = fmult(mag1, CORDIC_K);
    q16 ry, mag2;
    cordic_atan2(sqrt_xy, -Ta[8], ry, mag2);

    q16 rx, mag3;
    cordic_atan2(Ta[10], Ta[9], rx, mag3);

    pose[3] = rx;
    pose[4] = ry;
    pose[5] = rz;
}

// -------------------------------------------------------------------------------
// 解析雅可比 (与 analytical_jacobian_serial.v 完全一致)
// -------------------------------------------------------------------------------
static inline void compute_jacobian(
    const q16 pose[6],
    const q16 frame_z[5][3],
    const q16 frame_p[5][3],
    q16 J[6][6]
){
    const q16 pe_x = pose[0], pe_y = pose[1], pe_z = pose[2];

    J[0][0] = -pe_y; J[1][0] = pe_x; J[2][0] = 0;
    J[3][0] = 0;     J[4][0] = 0;    J[5][0] = FIXED_ONE;

    for (int col = 1; col <= 5; ++col) {
        int fi = col - 1;
        q16 zx = frame_z[fi][0], zy = frame_z[fi][1], zz = frame_z[fi][2];
        q16 px = frame_p[fi][0], py = frame_p[fi][1], pz = frame_p[fi][2];
        q16 dx = pe_x - px;
        q16 dy = pe_y - py;
        q16 dz = pe_z - pz;

        q16 p0 = fmult(zy, dz);
        q16 p1 = fmult(zz, dy);
        q16 p2 = fmult(zz, dx);
        q16 p3 = fmult(zx, dz);
        q16 p4 = fmult(zx, dy);
        q16 p5 = fmult(zy, dx);

        J[0][col] = p0 - p1;
        J[1][col] = p2 - p3;
        J[2][col] = p4 - p5;
        J[3][col] = zx;
        J[4][col] = zy;
        J[5][col] = zz;
    }
}

// -------------------------------------------------------------------------------
// 笛卡尔阻抗 (与 wrench_calc.v 完全一致)
// -------------------------------------------------------------------------------
static inline void wrench_calc(
    const q16 x_des[6], const q16 pose[6],
    const q16 K[6],     const q16 D[6],
    const q16 xv[6],
    q16 F[6]
){
    q16 dx[6];
    for (int i = 0; i < 6; ++i) dx[i] = x_des[i] - pose[i];
    for (int i = 0; i < 6; ++i) {
        q64 kp = fmul64(K[i], dx[i]);
        q64 dv = fmul64(D[i], xv[i]);
        F[i] = sat32(kp - dv);
    }
}

// -------------------------------------------------------------------------------
// 力矩映射 (与 impedance_torque.v 完全一致)
// -------------------------------------------------------------------------------
static inline void impedance_torque(
    const q16 J[6][6], const q16 F[6],
    q16 tau[6]
){
    for (int i = 0; i < 6; ++i) {
        q64 acc = 0;
        for (int r = 0; r < 6; ++r) {
            acc += fmul64(J[r][i], F[r]);
        }
        tau[i] = sat32(acc);
    }
}

// -------------------------------------------------------------------------------
// τ -> iq (与 torque_to_iq.v 完全一致)
// -------------------------------------------------------------------------------
static inline void torque_to_iq(const q16 tau[6], int16_t iq[6]) {
    for (int i = 0; i < 6; ++i) {
        q16 v = fmult(tau[i], KT_INV[i]);
        iq[i] = sat16(v);
    }
}

// -------------------------------------------------------------------------------
// 完整控制律 (单次): q[6] -> iq[6]
// -------------------------------------------------------------------------------
static inline void control_law(
    const q16 q[6],
    const q16 x_des[6],
    const q16 K[6],
    const q16 D[6],
    const q16 xv[6],
    int16_t iq_out[6],
    q16 tau_out[6]
){
    q16 pose[6];
    q16 frame_z[5][3], frame_p[5][3];
    fk_engine(q, pose, frame_z, frame_p);

    q16 J[6][6];
    compute_jacobian(pose, frame_z, frame_p, J);

    q16 F[6];
    wrench_calc(x_des, pose, K, D, xv, F);

    impedance_torque(J, F, tau_out);

    torque_to_iq(tau_out, iq_out);
}

// ===============================================================================
// ==== 以下为 per-call 计时框架 (本文件新增) ====================================
// ===============================================================================

// rdtsc 计时原语 (Intel 白皮书口径)
static inline uint64_t tsc_begin() {
    _mm_lfence();
    uint64_t t = __rdtsc();
    _mm_lfence();
    return t;
}
static inline uint64_t tsc_end() {
    unsigned aux;
    uint64_t t = __rdtscp(&aux);
    _mm_lfence();
    return t;
}

// ---- CPUID: 检查 Invariant TSC / rdtscp 支持 ----
struct CpuFeat {
    bool has_rdtscp   = false;
    bool has_inv_tsc  = false;
    char brand[64]    = {0};
};
static CpuFeat query_cpu() {
    CpuFeat f;
    int r[4];
    // 扩展叶最大值
    __cpuid(r, 0x80000000);
    unsigned maxext = (unsigned)r[0];
    if (maxext >= 0x80000001) {
        __cpuid(r, 0x80000001);
        f.has_rdtscp = ((r[3] >> 27) & 1) != 0;   // EDX bit27 = RDTSCP
    }
    if (maxext >= 0x80000007) {
        __cpuid(r, 0x80000007);
        f.has_inv_tsc = ((r[3] >> 8) & 1) != 0;   // EDX bit8 = InvariantTSC
    }
    if (maxext >= 0x80000004) {
        int* p = (int*)f.brand;
        __cpuid(p + 0, 0x80000002);
        __cpuid(p + 4, 0x80000003);
        __cpuid(p + 8, 0x80000004);
        f.brand[48] = 0;
    }
    return f;
}

// ---- TSC 频率标定: 用 QPC 在 ~calib_ms 区间内测 TSC 增量 ----
static double calibrate_tsc_hz(double calib_ms, double* out_qpc_hz) {
    LARGE_INTEGER qf;
    QueryPerformanceFrequency(&qf);
    if (out_qpc_hz) *out_qpc_hz = (double)qf.QuadPart;
    const double target_ticks = calib_ms * 1e-3 * (double)qf.QuadPart;

    LARGE_INTEGER q0, q1;
    // 先稳一下: 忙等到 QPC 跳变
    QueryPerformanceCounter(&q0);
    LARGE_INTEGER qtmp;
    do { QueryPerformanceCounter(&qtmp); } while (qtmp.QuadPart == q0.QuadPart);

    QueryPerformanceCounter(&q0);
    uint64_t c0 = tsc_begin();
    do {
        QueryPerformanceCounter(&q1);
    } while ((double)(q1.QuadPart - q0.QuadPart) < target_ticks);
    uint64_t c1 = tsc_end();

    double dt_s   = (double)(q1.QuadPart - q0.QuadPart) / (double)qf.QuadPart;
    double dcyc   = (double)(c1 - c0);
    return dcyc / dt_s;
}

// ---- 满载自旋线程 ----
static volatile LONG g_load_stop = 0;
struct LoadArg { DWORD_PTR mask; };

static DWORD WINAPI load_thread(LPVOID p) {
    LoadArg* a = (LoadArg*)p;
    SetThreadAffinityMask(GetCurrentThread(), a->mask);
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_NORMAL);
    // 纯 ALU 自旋 (依赖链, 不可被优化掉)
    volatile uint64_t sink = 0;
    uint64_t x = (uint64_t)(uintptr_t)a * 0x9E3779B97F4A7C15ull + 1;
    while (InterlockedCompareExchange(&g_load_stop, 0, 0) == 0) {
        for (int i = 0; i < 4096; ++i) {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            x += 0x2545F4914F6CDD1Dull;
        }
        sink = x;
    }
    (void)sink;
    return 0;
}

// ---- 分位数 ----
static double pct_of(const std::vector<double>& sorted, double p) {
    size_t n = sorted.size();
    if (n == 0) return 0.0;
    double pos = (p / 100.0) * (double)(n - 1);
    size_t idx = (size_t)(pos + 0.5);
    if (idx >= n) idx = n - 1;
    return sorted[idx];
}

int main(int argc, char** argv) {
    bool loaded = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--load") == 0) loaded = true;
    }
    const char* env_name = loaded ? "loaded" : "quiet";
    char bin_path[512];
    std::snprintf(bin_path, sizeof(bin_path),
                  "E:\\eth\\udp_add_eth\\benchmarks\\percall_%s.bin", env_name);

    // ---- CPU 特性 ----
    CpuFeat feat = query_cpu();
    SYSTEM_INFO si; GetSystemInfo(&si);
    const int n_logical = (int)si.dwNumberOfProcessors;

    std::printf("==== CPU per-call latency benchmark (env = %s) ====\n", env_name);
    std::printf("CPU            : %s\n", feat.brand[0] ? feat.brand : "(unknown)");
    std::printf("logical procs  : %d\n", n_logical);
    std::printf("RDTSCP support : %s\n", feat.has_rdtscp ? "yes" : "NO (fatal)");
    std::printf("Invariant TSC  : %s\n", feat.has_inv_tsc ? "yes" : "NO (TSC may vary with P-state!)");
    if (!feat.has_rdtscp) {
        std::fprintf(stderr, "BLOCKED: CPU does not support RDTSCP\n");
        return 3;
    }

    // ---- 绑核 + 优先级 ----
    HANDLE proc   = GetCurrentProcess();
    HANDLE thread = GetCurrentThread();
    // 注意: 不设 process affinity (否则 load 线程无法分散到其它核), 只绑测量线程到 CPU0
    if (!SetThreadAffinityMask(thread, (DWORD_PTR)1)) {
        std::fprintf(stderr, "WARN: SetThreadAffinityMask failed\n");
    }
    // 优先级: 先试 REALTIME, 失败退回 HIGH.
    // loaded 模式下强制只用 HIGH —— REALTIME 类是进程级, 19 个 realtime 自旋线程会饿死系统.
    BOOL rt_ok = FALSE;
    if (!loaded) rt_ok = SetPriorityClass(proc, REALTIME_PRIORITY_CLASS);
    if (!rt_ok)  SetPriorityClass(proc, HIGH_PRIORITY_CLASS);
    DWORD cls = GetPriorityClass(proc);
    const char* cls_str = (cls == REALTIME_PRIORITY_CLASS) ? "REALTIME" :
                          (cls == HIGH_PRIORITY_CLASS)     ? "HIGH"     : "other";
    SetThreadPriority(thread, THREAD_PRIORITY_TIME_CRITICAL);
    std::printf("priority class : %s (thread = TIME_CRITICAL), affinity = CPU0\n", cls_str);

    // ---- TSC 频率标定 (~200 ms) ----
    double qpc_hz = 0.0;
    double tsc_hz = calibrate_tsc_hz(200.0, &qpc_hz);
    const double cyc_to_us = 1e6 / tsc_hz;
    std::printf("QPC freq       : %.0f Hz\n", qpc_hz);
    std::printf("TSC freq       : %.6f GHz  (calibrated over ~200 ms vs QPC)\n", tsc_hz / 1e9);
    std::printf("1 TSC cycle    : %.6f ns\n", cyc_to_us * 1e3);

    // ---- 计时开销标定: 空测量区间 ----
    {
        const int NO = 200000;
        std::vector<uint64_t> ov(NO);
        for (int i = 0; i < NO; ++i) {
            uint64_t a = tsc_begin();
            uint64_t b = tsc_end();
            ov[i] = b - a;
        }
        std::sort(ov.begin(), ov.end());
        double omin = (double)ov.front()          * cyc_to_us;
        double omed = (double)ov[NO / 2]          * cyc_to_us;
        double o99  = (double)ov[(size_t)(NO*0.99)] * cyc_to_us;
        double omax = (double)ov.back()           * cyc_to_us;
        std::printf("timer overhead : min=%.5f us (%llu cyc)  p50=%.5f us (%llu cyc)  "
                    "p99=%.5f us  max=%.5f us\n",
                    omin, (unsigned long long)ov.front(),
                    omed, (unsigned long long)ov[NO / 2], o99, omax);
        std::printf("                 (per-call latencies below are RAW, overhead NOT subtracted)\n");
    }

    // ---- 阻抗参数 (与 cpu_baseline.cpp 一致) ----
    q16 x_des[6] = {
        (q16)(0.30 * 65536), (q16)(0.00 * 65536), (q16)(0.30 * 65536),
        0, 0, 0
    };
    q16 K_imp[6] = {
        (q16)(200.0 * 65536), (q16)(200.0 * 65536), (q16)(200.0 * 65536),
        (q16)( 50.0 * 65536), (q16)( 50.0 * 65536), (q16)( 50.0 * 65536)
    };
    q16 D_imp[6] = {
        (q16)(10.0 * 65536), (q16)(10.0 * 65536), (q16)(10.0 * 65536),
        (q16)( 5.0 * 65536), (q16)( 5.0 * 65536), (q16)( 5.0 * 65536)
    };
    q16 xv[6] = { 0, 0, 0, 0, 0, 0 };

    // ---- 预生成输入 (与 cpu_baseline.cpp 的 make_q 公式一致, TOTAL = N) ----
    const int N_WARMUP = 10000;
    const int N        = 1000000;

    std::printf("N samples      : %d single calls (warm-up %d discarded)\n", N, N_WARMUP);
    std::printf("generating %d input q-vectors (%.1f MB) ...\n",
                N, (double)N * 6 * sizeof(q16) / 1048576.0);

    std::vector<q16> q_all((size_t)N * 6);
    {
        const double TOTAL = (double)N;
        for (int i = 0; i < N; ++i) {
            double base = (double)i / TOTAL * 3.0 - 1.5;
            q16* p = &q_all[(size_t)i * 6];
            p[0] = (q16)((base + 0.10) * 65536);
            p[1] = (q16)((base + 0.20) * 65536);
            p[2] = (q16)((base + 0.30) * 65536);
            p[3] = (q16)((base + 0.40) * 65536);
            p[4] = (q16)((base + 0.50) * 65536);
            p[5] = (q16)((base + 0.60) * 65536);
        }
    }
    std::vector<uint64_t> cyc((size_t)N);
    // 预触碰结果缓冲, 避免测量期间缺页
    std::memset(cyc.data(), 0, cyc.size() * sizeof(uint64_t));

    // ---- 起满载线程 ----
    std::vector<HANDLE> loads;
    std::vector<LoadArg> largs;
    if (loaded) {
        largs.resize((size_t)n_logical - 1);
        for (int c = 1; c < n_logical; ++c) {
            largs[(size_t)c - 1].mask = ((DWORD_PTR)1) << c;
            HANDLE h = CreateThread(NULL, 0, load_thread, &largs[(size_t)c - 1], 0, NULL);
            if (h) loads.push_back(h);
        }
        std::printf("load threads   : %d spinning on CPU1..CPU%d\n",
                    (int)loads.size(), n_logical - 1);
        Sleep(500);  // 让负载线程爬满
    } else {
        std::printf("load threads   : none (quiet)\n");
    }

    // ---- Anti-DCE ----
    volatile int64_t accum = 0;
    int16_t iq[6]; q16 tau[6];

    // ---- 预热 ----
    for (int i = 0; i < N_WARMUP; ++i) {
        control_law(&q_all[(size_t)i * 6], x_des, K_imp, D_imp, xv, iq, tau);
        accum += iq[0] + iq[1] + iq[2] + iq[3] + iq[4] + iq[5];
    }

    // ---- 正式采样: 每次调用单独计时 ----
    std::printf("measuring ...\n");
    std::fflush(stdout);
    for (int i = 0; i < N; ++i) {
        const q16* qp = &q_all[(size_t)i * 6];
        uint64_t t0 = tsc_begin();
        control_law(qp, x_des, K_imp, D_imp, xv, iq, tau);
        accum += iq[0] + iq[1] + iq[2] + iq[3] + iq[4] + iq[5];
        uint64_t t1 = tsc_end();
        cyc[(size_t)i] = t1 - t0;
    }

    // ---- 停负载 ----
    if (loaded) {
        InterlockedExchange(&g_load_stop, 1);
        for (HANDLE h : loads) { WaitForSingleObject(h, 5000); CloseHandle(h); }
    }
    SetPriorityClass(proc, NORMAL_PRIORITY_CLASS);

    std::fprintf(stderr, "anti-dce accum=%lld\n", (long long)accum);

    // ---- 转 us + 统计 ----
    std::vector<double> us((size_t)N);
    for (int i = 0; i < N; ++i) us[(size_t)i] = (double)cyc[(size_t)i] * cyc_to_us;

    double sum = 0.0;
    for (double v : us) sum += v;
    double mean = sum / (double)N;
    double var = 0.0;
    for (double v : us) { double d = v - mean; var += d * d; }
    double sd = std::sqrt(var / (double)N);

    int over100 = 0, over10 = 0, over1 = 0;
    for (double v : us) {
        if (v > 100.0) ++over100;
        if (v >  10.0) ++over10;
        if (v >   1.0) ++over1;
    }

    std::vector<double> sorted = us;
    std::sort(sorted.begin(), sorted.end());

    double p50   = pct_of(sorted, 50.0);
    double p95   = pct_of(sorted, 95.0);
    double p99   = pct_of(sorted, 99.0);
    double p999  = pct_of(sorted, 99.9);
    double p9999 = pct_of(sorted, 99.99);
    double mn    = sorted.front();
    double mx    = sorted.back();

    // ---- 落盘: 原始 per-call 延迟 (float64 LE, 无文件头) ----
    FILE* fb = std::fopen(bin_path, "wb");
    if (!fb) {
        std::fprintf(stderr, "FAILED to open %s\n", bin_path);
        return 2;
    }
    size_t wrote = std::fwrite(us.data(), sizeof(double), us.size(), fb);
    std::fclose(fb);
    if (wrote != us.size()) {
        std::fprintf(stderr, "FAILED to write all samples (%zu/%zu)\n", wrote, us.size());
        return 2;
    }

    // ---- 打印 ----
    std::printf("\n---- per-call latency (us), env = %s, N = %d ----\n", env_name, N);
    std::printf("mean    = %10.4f us\n", mean);
    std::printf("SD      = %10.4f us\n", sd);
    std::printf("min     = %10.4f us\n", mn);
    std::printf("p50     = %10.4f us\n", p50);
    std::printf("p95     = %10.4f us\n", p95);
    std::printf("p99     = %10.4f us\n", p99);
    std::printf("p99.9   = %10.4f us\n", p999);
    std::printf("p99.99  = %10.4f us\n", p9999);
    std::printf("max     = %10.4f us\n", mx);
    std::printf("count(>  1 us) = %d  (%.4f %%)\n", over1,   100.0*over1  /N);
    std::printf("count(> 10 us) = %d  (%.4f %%)\n", over10,  100.0*over10 /N);
    std::printf("count(>100 us) = %d  (%.4f %%)\n", over100, 100.0*over100/N);
    std::printf("raw dump = %s  (%d x float64 LE, no header; numpy: np.fromfile(p,dtype='<f8'))\n",
                bin_path, N);

    // ---- ASCII 直方图 (对数尺度, 便于看尾部) ----
    {
        const int NB = 24;
        // bin 边界: 以 min 为起点, 每 bin 乘 1.35
        double lo = (mn > 0.0) ? mn : 0.01;
        std::vector<double> edge(NB + 1);
        edge[0] = lo;
        for (int b = 1; b <= NB; ++b) edge[b] = edge[b-1] * 1.35;
        std::vector<long long> cnt(NB, 0);
        for (double v : us) {
            int b = NB - 1;
            for (int k = 0; k < NB; ++k) {
                if (v < edge[k+1]) { b = k; break; }
            }
            cnt[b]++;
        }
        long long maxc = 0;
        for (long long c : cnt) if (c > maxc) maxc = c;
        std::printf("\nLog-scale histogram (x1.35 per bin):\n");
        for (int b = 0; b < NB; ++b) {
            if (cnt[b] == 0) continue;
            int bar = (maxc > 0) ? (int)((double)cnt[b] * 50.0 / (double)maxc) : 0;
            if (bar == 0 && cnt[b] > 0) bar = 1;
            std::printf("[%9.3f, %9.3f) us | %8lld | ", edge[b], edge[b+1], cnt[b]);
            for (int k = 0; k < bar; ++k) std::putchar('#');
            std::putchar('\n');
        }
    }

    return 0;
}
