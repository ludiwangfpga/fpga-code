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


// ===============================================================================
// ==== 以下为 long-run 计时框架 (cpu_longrun.cpp 新增; 上方全部逐字来自 cpu_percall.cpp) ====
// ===============================================================================
//
// 目的: 回应 26-OJIES-0346 审稿人 R1.3 / R2.2 —— 在 *长时程* 与 *含 I/O 的真实负载* 下
//       测量通用 OS 上同一位级等价控制律的单次调用延迟尾部, 判断是否越过计算预算.
//
// 与 cpu_percall.cpp 的差别只在测量框架:
//   * 按墙钟时长运行 (--minutes), 调用次数不设上限 (10^9 ~ 10^11 量级);
//   * 不保存全部原始样本 (无法容纳), 改为: 直方图 (0~200us 每 10ns 一格, 200us~10ms 每 1us 一格,
//     >10ms 溢出桶) + 阈值计数 + 最大值及其发生时刻 + 所有 >TAIL_LOG_US 的样本逐条落盘;
//   * 每 60 s 落盘一次 checkpoint (summary.json / hist.csv / tail.csv), 中断不丢数据;
//     checkpoint 的写文件发生在计时区间之外, 但会打断被测线程 ~ms 级; 它本身不进入样本.
//   * 三种环境:
//       quiet  : 被测线程绑 CPU0 + REALTIME/TIME_CRITICAL, 机器空闲 (同 cpu_percall quiet)
//       load   : 同上但 HIGH 优先级, 其余逻辑核各 1 个 ALU 自旋线程 (同 cpu_percall --load)
//       loadio : load + 2 个磁盘写/刷/无缓冲读线程 + 1 个 UDP 环回 ping-pong 线程
//                + 1 个 256 MB 分配/写/释放线程 (制造 ISR/DPC/缺页等 OS 侧干扰)
//
// 用法: cpu_longrun.exe --mode quiet|load|loadio --minutes 120 [--out DIR] [--tag NAME]
//        [--tail-us 20] [--checkpoint-s 60]

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#include <string>
#include <ctime>

static volatile LONG g_io_stop = 0;
static char g_io_dir[512] = {0};

// ---- I/O 干扰线程: 磁盘 ----
static DWORD WINAPI disk_thread(LPVOID p) {
    int id = (int)(intptr_t)p;
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_NORMAL);
    char path[600];
    std::snprintf(path, sizeof(path), "%s\\io_%d.bin", g_io_dir, id);
    const SIZE_T CHUNK = (SIZE_T)16 << 20;   // 16 MB, 页对齐 (满足 NO_BUFFERING 扇区对齐)
    char* buf = (char*)VirtualAlloc(NULL, CHUNK, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
    if (!buf) return 1;
    std::memset(buf, 0x5A + id, CHUNK);
    while (InterlockedCompareExchange(&g_io_stop, 0, 0) == 0) {
        HANDLE h = CreateFileA(path, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
        if (h == INVALID_HANDLE_VALUE) { Sleep(20); continue; }
        DWORD w = 0;
        for (int k = 0; k < 4; ++k) WriteFile(h, buf, (DWORD)CHUNK, &w, NULL);   // 64 MB
        FlushFileBuffers(h);
        CloseHandle(h);
        h = CreateFileA(path, GENERIC_READ, 0, NULL, OPEN_EXISTING,
                        FILE_FLAG_NO_BUFFERING | FILE_FLAG_SEQUENTIAL_SCAN, NULL);
        if (h != INVALID_HANDLE_VALUE) {
            DWORD r = 0;
            while (ReadFile(h, buf, (DWORD)CHUNK, &r, NULL) && r > 0) {}
            CloseHandle(h);
        }
    }
    DeleteFileA(path);
    VirtualFree(buf, 0, MEM_RELEASE);
    return 0;
}

// ---- I/O 干扰线程: UDP 环回 ping-pong ----
static DWORD WINAPI udp_thread(LPVOID) {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_NORMAL);
    SOCKET a = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    SOCKET b = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (a == INVALID_SOCKET || b == INVALID_SOCKET) return 1;
    sockaddr_in sa{}; sa.sin_family = AF_INET; sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK); sa.sin_port = 0;
    sockaddr_in sb = sa;
    if (bind(a, (sockaddr*)&sa, sizeof(sa)) != 0) return 2;
    if (bind(b, (sockaddr*)&sb, sizeof(sb)) != 0) return 2;
    int la = sizeof(sa), lb = sizeof(sb);
    getsockname(a, (sockaddr*)&sa, &la);
    getsockname(b, (sockaddr*)&sb, &lb);
    DWORD tmo = 200;
    setsockopt(a, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tmo, sizeof(tmo));
    setsockopt(b, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tmo, sizeof(tmo));
    char pkt[1400]; std::memset(pkt, 0xA5, sizeof(pkt));
    char rcv[1500];
    while (InterlockedCompareExchange(&g_io_stop, 0, 0) == 0) {
        sendto(a, pkt, sizeof(pkt), 0, (sockaddr*)&sb, sizeof(sb));
        sockaddr_in from{}; int fl = sizeof(from);
        recvfrom(b, rcv, sizeof(rcv), 0, (sockaddr*)&from, &fl);
        sendto(b, pkt, sizeof(pkt), 0, (sockaddr*)&sa, sizeof(sa));
        fl = sizeof(from);
        recvfrom(a, rcv, sizeof(rcv), 0, (sockaddr*)&from, &fl);
    }
    closesocket(a); closesocket(b);
    return 0;
}

// ---- I/O 干扰线程: 内存分配/写/释放 ----
static DWORD WINAPI mem_thread(LPVOID) {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_NORMAL);
    const SIZE_T SZ = (SIZE_T)256 << 20;
    while (InterlockedCompareExchange(&g_io_stop, 0, 0) == 0) {
        char* m = (char*)VirtualAlloc(NULL, SZ, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
        if (m) { std::memset(m, 0x3C, SZ); VirtualFree(m, 0, MEM_RELEASE); }
        Sleep(50);
    }
    return 0;
}

// ---- 直方图 ----
struct Hist {
    // 0 .. 200 us: 10 ns/bin -> 20000 bins ; 200 us .. 10 ms: 1 us/bin -> 9800 bins ; overflow
    static const int NFINE = 20000, NCOARSE = 9800;
    std::vector<uint64_t> fine, coarse;
    uint64_t overflow = 0;
    Hist() : fine(NFINE, 0), coarse(NCOARSE, 0) {}
    inline void add(double us) {
        if (us < 200.0) { int b = (int)(us * 100.0); if (b < 0) b = 0; if (b >= NFINE) b = NFINE - 1; ++fine[b]; }
        else if (us < 10000.0) { int b = (int)(us - 200.0); if (b >= NCOARSE) b = NCOARSE - 1; ++coarse[b]; }
        else ++overflow;
    }
    // 分位数 (bin 上边界, 保守)
    double pct(double p, uint64_t total) const {
        if (total == 0) return 0.0;
        double target = p / 100.0 * (double)total;
        uint64_t c = 0;
        for (int i = 0; i < NFINE; ++i) { c += fine[i]; if ((double)c >= target) return (i + 1) * 0.01; }
        for (int i = 0; i < NCOARSE; ++i) { c += coarse[i]; if ((double)c >= target) return 200.0 + (i + 1); }
        return 10000.0;
    }
};

struct Stats {
    uint64_t n = 0;
    double sum = 0.0, sumsq = 0.0;
    double mn = 1e300, mx = 0.0;
    uint64_t mx_index = 0; double mx_elapsed_s = 0.0;
    uint64_t over_1 = 0, over_439 = 0, over_10 = 0, over_3346 = 0, over_100 = 0, over_1000 = 0;
};

struct TailRec { double t_s; double us; };

static void write_checkpoint(const char* out_dir, const char* tag, const char* mode,
                             const Stats& st, const Hist& h, std::vector<TailRec>& tail,
                             double elapsed_s, double tsc_ghz, double tail_us, bool final_flag)
{
    char path[700];
    // summary.json
    std::snprintf(path, sizeof(path), "%s\\%s_summary.json", out_dir, tag);
    FILE* f = std::fopen(path, "w");
    if (f) {
        double mean = st.n ? st.sum / (double)st.n : 0.0;
        double var  = st.n ? st.sumsq / (double)st.n - mean * mean : 0.0;
        if (var < 0) var = 0;
        std::fprintf(f, "{\n");
        std::fprintf(f, "  \"mode\": \"%s\",\n  \"final\": %s,\n  \"elapsed_s\": %.1f,\n  \"tsc_ghz\": %.6f,\n",
                     mode, final_flag ? "true" : "false", elapsed_s, tsc_ghz);
        std::fprintf(f, "  \"n\": %llu,\n  \"mean_us\": %.5f,\n  \"sd_us\": %.5f,\n  \"min_us\": %.5f,\n  \"max_us\": %.4f,\n",
                     (unsigned long long)st.n, mean, std::sqrt(var), st.mn, st.mx);
        std::fprintf(f, "  \"max_at_index\": %llu,\n  \"max_at_elapsed_s\": %.3f,\n",
                     (unsigned long long)st.mx_index, st.mx_elapsed_s);
        std::fprintf(f, "  \"p50_us\": %.3f,\n  \"p99_us\": %.3f,\n  \"p999_us\": %.3f,\n  \"p9999_us\": %.3f,\n  \"p99999_us\": %.3f,\n  \"p999999_us\": %.3f,\n",
                     h.pct(50, st.n), h.pct(99, st.n), h.pct(99.9, st.n), h.pct(99.99, st.n), h.pct(99.999, st.n), h.pct(99.9999, st.n));
        std::fprintf(f, "  \"over_1us\": %llu,\n  \"over_4p39us\": %llu,\n  \"over_10us\": %llu,\n  \"over_33p46us\": %llu,\n  \"over_100us\": %llu,\n  \"over_1000us\": %llu,\n  \"over_10ms\": %llu,\n",
                     (unsigned long long)st.over_1, (unsigned long long)st.over_439, (unsigned long long)st.over_10,
                     (unsigned long long)st.over_3346, (unsigned long long)st.over_100, (unsigned long long)st.over_1000,
                     (unsigned long long)h.overflow);
        std::fprintf(f, "  \"tail_log_threshold_us\": %.1f\n}\n", tail_us);
        std::fclose(f);
    }
    // hist.csv (只写非零 bin)
    std::snprintf(path, sizeof(path), "%s\\%s_hist.csv", out_dir, tag);
    f = std::fopen(path, "w");
    if (f) {
        std::fprintf(f, "bin_lo_us,bin_hi_us,count\n");
        for (int i = 0; i < Hist::NFINE; ++i) if (h.fine[i]) std::fprintf(f, "%.2f,%.2f,%llu\n", i * 0.01, (i + 1) * 0.01, (unsigned long long)h.fine[i]);
        for (int i = 0; i < Hist::NCOARSE; ++i) if (h.coarse[i]) std::fprintf(f, "%.1f,%.1f,%llu\n", 200.0 + i, 201.0 + i, (unsigned long long)h.coarse[i]);
        if (h.overflow) std::fprintf(f, "10000,inf,%llu\n", (unsigned long long)h.overflow);
        std::fclose(f);
    }
    // tail.csv (追加)
    if (!tail.empty()) {
        std::snprintf(path, sizeof(path), "%s\\%s_tail.csv", out_dir, tag);
        bool fresh = false;
        { FILE* t = std::fopen(path, "r"); if (!t) fresh = true; else std::fclose(t); }
        f = std::fopen(path, "a");
        if (f) {
            if (fresh) std::fprintf(f, "elapsed_s,latency_us\n");
            for (const TailRec& r : tail) std::fprintf(f, "%.3f,%.3f\n", r.t_s, r.us);
            std::fclose(f);
        }
        tail.clear();
    }
}

int main(int argc, char** argv) {
    const char* mode = "quiet";
    double minutes = 120.0;
    char out_dir[512]; std::snprintf(out_dir, sizeof(out_dir), "E:\\eth\\udp_add_eth\\benchmarks\\longrun");
    char tag[128] = {0};
    double tail_us = 20.0;
    double ckpt_s = 60.0;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--mode") && i + 1 < argc) mode = argv[++i];
        else if (!std::strcmp(argv[i], "--minutes") && i + 1 < argc) minutes = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--out") && i + 1 < argc) std::snprintf(out_dir, sizeof(out_dir), "%s", argv[++i]);
        else if (!std::strcmp(argv[i], "--tag") && i + 1 < argc) std::snprintf(tag, sizeof(tag), "%s", argv[++i]);
        else if (!std::strcmp(argv[i], "--tail-us") && i + 1 < argc) tail_us = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--checkpoint-s") && i + 1 < argc) ckpt_s = std::atof(argv[++i]);
    }
    // load   : ALU spinners on every other logical core (extreme saturation; froze the host once after 107 min)
    // io     : only the I/O interference threads (disk / UDP loopback / memory), no spinners -> realistic host-in-the-loop load
    // loadio : both
    const bool is_load   = !std::strcmp(mode, "load") || !std::strcmp(mode, "loadio");
    const bool is_loadio = !std::strcmp(mode, "loadio") || !std::strcmp(mode, "io");
    if (!tag[0]) std::snprintf(tag, sizeof(tag), "%s", mode);
    CreateDirectoryA(out_dir, NULL);
    std::snprintf(g_io_dir, sizeof(g_io_dir), "%s", out_dir);

    CpuFeat feat = query_cpu();
    SYSTEM_INFO si; GetSystemInfo(&si);
    const int n_logical = (int)si.dwNumberOfProcessors;

    std::printf("==== CPU long-run per-call latency (mode = %s, %.0f min) ====\n", mode, minutes);
    std::printf("CPU            : %s\n", feat.brand[0] ? feat.brand : "(unknown)");
    std::printf("logical procs  : %d\n", n_logical);
    std::printf("RDTSCP support : %s\n", feat.has_rdtscp ? "yes" : "NO (fatal)");
    std::printf("Invariant TSC  : %s\n", feat.has_inv_tsc ? "yes" : "NO");
    if (!feat.has_rdtscp) { std::fprintf(stderr, "BLOCKED: no RDTSCP\n"); return 3; }

    HANDLE proc = GetCurrentProcess();
    HANDLE thread = GetCurrentThread();
    if (!SetThreadAffinityMask(thread, (DWORD_PTR)1)) std::fprintf(stderr, "WARN: affinity failed\n");
    BOOL rt_ok = FALSE;
    if (!is_load) rt_ok = SetPriorityClass(proc, REALTIME_PRIORITY_CLASS);
    if (!rt_ok)   SetPriorityClass(proc, HIGH_PRIORITY_CLASS);
    DWORD cls = GetPriorityClass(proc);
    SetThreadPriority(thread, THREAD_PRIORITY_TIME_CRITICAL);
    std::printf("priority class : %s (thread = TIME_CRITICAL), affinity = CPU0\n",
                cls == REALTIME_PRIORITY_CLASS ? "REALTIME" : cls == HIGH_PRIORITY_CLASS ? "HIGH" : "other");

    double qpc_hz = 0.0;
    double tsc_hz = calibrate_tsc_hz(200.0, &qpc_hz);
    const double cyc_to_us = 1e6 / tsc_hz;
    std::printf("TSC freq       : %.6f GHz\n", tsc_hz / 1e9);

    {   // 计时开销
        const int NO = 200000;
        std::vector<uint64_t> ov(NO);
        for (int i = 0; i < NO; ++i) { uint64_t a = tsc_begin(); uint64_t b = tsc_end(); ov[i] = b - a; }
        std::sort(ov.begin(), ov.end());
        std::printf("timer overhead : p50=%.5f us  max=%.5f us (NOT subtracted)\n",
                    (double)ov[NO / 2] * cyc_to_us, (double)ov.back() * cyc_to_us);
    }

    q16 x_des[6] = { (q16)(0.30 * 65536), (q16)(0.00 * 65536), (q16)(0.30 * 65536), 0, 0, 0 };
    q16 K_imp[6] = { (q16)(200.0 * 65536), (q16)(200.0 * 65536), (q16)(200.0 * 65536),
                     (q16)( 50.0 * 65536), (q16)( 50.0 * 65536), (q16)( 50.0 * 65536) };
    q16 D_imp[6] = { (q16)(10.0 * 65536), (q16)(10.0 * 65536), (q16)(10.0 * 65536),
                     (q16)( 5.0 * 65536), (q16)( 5.0 * 65536), (q16)( 5.0 * 65536) };
    q16 xv[6] = { 0, 0, 0, 0, 0, 0 };

    const int N_WARMUP = 10000;
    const int NQ = 1000000;   // 预生成输入池, 循环使用
    std::vector<q16> q_all((size_t)NQ * 6);
    for (int i = 0; i < NQ; ++i) {
        double base = (double)i / (double)NQ * 3.0 - 1.5;
        q16* p = &q_all[(size_t)i * 6];
        p[0] = (q16)((base + 0.10) * 65536); p[1] = (q16)((base + 0.20) * 65536);
        p[2] = (q16)((base + 0.30) * 65536); p[3] = (q16)((base + 0.40) * 65536);
        p[4] = (q16)((base + 0.50) * 65536); p[5] = (q16)((base + 0.60) * 65536);
    }

    // ---- 负载线程 ----
    std::vector<HANDLE> loads; std::vector<LoadArg> largs;
    WSADATA wsa; bool wsa_ok = false;
    if (is_load) {
        largs.resize((size_t)n_logical - 1);
        for (int c = 1; c < n_logical; ++c) {
            largs[(size_t)c - 1].mask = ((DWORD_PTR)1) << c;
            HANDLE h = CreateThread(NULL, 0, load_thread, &largs[(size_t)c - 1], 0, NULL);
            if (h) loads.push_back(h);
        }
        std::printf("load threads   : %d spinning on CPU1..CPU%d\n", (int)loads.size(), n_logical - 1);
    }
    std::vector<HANDLE> ios;
    if (is_loadio) {
        wsa_ok = (WSAStartup(MAKEWORD(2, 2), &wsa) == 0);
        for (int k = 0; k < 2; ++k) { HANDLE h = CreateThread(NULL, 0, disk_thread, (LPVOID)(intptr_t)k, 0, NULL); if (h) ios.push_back(h); }
        if (wsa_ok) { HANDLE h = CreateThread(NULL, 0, udp_thread, NULL, 0, NULL); if (h) ios.push_back(h); }
        { HANDLE h = CreateThread(NULL, 0, mem_thread, NULL, 0, NULL); if (h) ios.push_back(h); }
        std::printf("io threads     : %d (2 disk, %d udp loopback, 1 mem)\n", (int)ios.size(), wsa_ok ? 1 : 0);
    }
    if (is_load || is_loadio) Sleep(1000);

    volatile int64_t accum = 0;
    int16_t iq[6]; q16 tau[6];
    for (int i = 0; i < N_WARMUP; ++i) {
        control_law(&q_all[(size_t)i * 6], x_des, K_imp, D_imp, xv, iq, tau);
        accum += iq[0] + iq[1] + iq[2] + iq[3] + iq[4] + iq[5];
    }

    Hist hist; Stats st; std::vector<TailRec> tail; tail.reserve(1 << 16);
    const uint64_t run_cycles  = (uint64_t)(minutes * 60.0 * tsc_hz);
    const uint64_t ckpt_cycles = (uint64_t)(ckpt_s * tsc_hz);
    const uint64_t t_start = tsc_begin();
    uint64_t next_ckpt = t_start + ckpt_cycles;
    uint64_t idx = 0;
    time_t wall0 = std::time(NULL);
    std::printf("start          : %s", std::ctime(&wall0));
    std::printf("measuring ...  (checkpoint every %.0f s -> %s\\%s_*.{json,csv})\n", ckpt_s, out_dir, tag);
    std::fflush(stdout);

    for (;;) {
        const q16* qp = &q_all[(size_t)(idx % (uint64_t)NQ) * 6];
        uint64_t t0 = tsc_begin();
        control_law(qp, x_des, K_imp, D_imp, xv, iq, tau);
        accum += iq[0] + iq[1] + iq[2] + iq[3] + iq[4] + iq[5];
        uint64_t t1 = tsc_end();
        double us = (double)(t1 - t0) * cyc_to_us;
        ++idx;
        hist.add(us);
        st.n++; st.sum += us; st.sumsq += us * us;
        if (us < st.mn) st.mn = us;
        if (us > st.mx) { st.mx = us; st.mx_index = idx; st.mx_elapsed_s = (double)(t1 - t_start) / tsc_hz; }
        if (us > 1.0)     st.over_1++;
        if (us > 4.39)    st.over_439++;
        if (us > 10.0)    st.over_10++;
        if (us > 33.46)   st.over_3346++;
        if (us > 100.0)   st.over_100++;
        if (us > 1000.0)  st.over_1000++;
        if (us > tail_us) tail.push_back({ (double)(t1 - t_start) / tsc_hz, us });

        if ((idx & 0xFFFF) == 0) {
            uint64_t now = t1;
            if (now >= next_ckpt) {
                double el = (double)(now - t_start) / tsc_hz;
                write_checkpoint(out_dir, tag, mode, st, hist, tail, el, tsc_hz / 1e9, tail_us, false);
                std::printf("[ckpt %7.0f s] n=%llu  p50=%.3f  p99.99=%.3f  max=%.2f us  >4.39:%llu  >33.46:%llu  >100:%llu\n",
                            el, (unsigned long long)st.n, hist.pct(50, st.n), hist.pct(99.99, st.n), st.mx,
                            (unsigned long long)st.over_439, (unsigned long long)st.over_3346, (unsigned long long)st.over_100);
                std::fflush(stdout);
                next_ckpt += ckpt_cycles;
            }
            if (now - t_start >= run_cycles) break;
        }
    }
    const uint64_t t_end = tsc_end();
    const double elapsed = (double)(t_end - t_start) / tsc_hz;

    if (is_loadio) { InterlockedExchange(&g_io_stop, 1); for (HANDLE h : ios) { WaitForSingleObject(h, 10000); CloseHandle(h); } if (wsa_ok) WSACleanup(); }
    if (is_load)   { InterlockedExchange(&g_load_stop, 1); for (HANDLE h : loads) { WaitForSingleObject(h, 5000); CloseHandle(h); } }
    SetPriorityClass(proc, NORMAL_PRIORITY_CLASS);

    write_checkpoint(out_dir, tag, mode, st, hist, tail, elapsed, tsc_hz / 1e9, tail_us, true);
    std::fprintf(stderr, "anti-dce accum=%lld\n", (long long)accum);

    double mean = st.sum / (double)st.n;
    std::printf("\n---- long-run per-call latency (us), mode = %s, elapsed = %.1f s, N = %llu ----\n", mode, elapsed, (unsigned long long)st.n);
    std::printf("mean    = %10.4f us\n", mean);
    std::printf("min     = %10.4f us\n", st.mn);
    std::printf("p50     = %10.3f us\n", hist.pct(50, st.n));
    std::printf("p99     = %10.3f us\n", hist.pct(99, st.n));
    std::printf("p99.9   = %10.3f us\n", hist.pct(99.9, st.n));
    std::printf("p99.99  = %10.3f us\n", hist.pct(99.99, st.n));
    std::printf("p99.999 = %10.3f us\n", hist.pct(99.999, st.n));
    std::printf("p99.9999= %10.3f us\n", hist.pct(99.9999, st.n));
    std::printf("max     = %10.3f us  (at call #%llu, t=%.3f s)\n", st.mx, (unsigned long long)st.mx_index, st.mx_elapsed_s);
    std::printf("count(> 4.39 us) = %llu\n", (unsigned long long)st.over_439);
    std::printf("count(>33.46 us) = %llu\n", (unsigned long long)st.over_3346);
    std::printf("count(>  100 us) = %llu\n", (unsigned long long)st.over_100);
    std::printf("count(> 1000 us) = %llu\n", (unsigned long long)st.over_1000);
    std::printf("LONGRUN_DONE %s\n", tag);
    return 0;
}
