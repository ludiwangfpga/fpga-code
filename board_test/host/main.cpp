// Pre-include every header that yolo.h pulls in, so that the access-widening
// macro below never leaks into a system/standard header (MSVC xkeycheck.h
// rejects macroized keywords inside standard headers).
#include <cstdint>
#include <WinSock2.h>
#include <WS2tcpip.h>
// Part 3 (E2 sweep) needs raw op2/op10/op18 frames on the SAME socket that is
// bound to 172.28.109.174:8080, because the FPGA bitstream replies to a fixed
// destination port 8080 (udp_tx cfg, it does not echo the request source port),
// YOLO::~YOLO() never closes the socket (port stays taken for process lifetime),
// and yolo.h exposes no raw-opcode method (op18 has no wrapper).  So a second
// bind to 8080 can never work; we widen access to YOLO's private socket for
// this test file only (yolo.h/yolo.cpp stay untouched).
#define private public
#include "yolo.h"
#undef private
#include <cstdio>
#include <cmath>

// New-bit verification host.
// Part 1: FK precision sweep -- 32 joint vectors (op2 write q / op8 read pose),
//         board pose vs double-precision reference (same DH/Euler as fk_golden.py).
//         Discriminates which RTL is flashed:
//           truncating RTL (old bit)      : mean pos err ~0.09 mm, max ~0.17 mm
//           round-to-nearest RTL (new bit): mean pos err ~0.03 mm, max ~0.08 mm
// Part 2: original full-chain demo (op10/op11/op12/op14/op13) for liveness.
// Protocol / socket code untouched (yolo.cpp identical to the verified archive).

static const int QV[32][6] = {
    {57412,-195589,-92637,-113975,97373,72760},
    {161490,-170088,-32151,-193618,-115858,2205},
    {-194961,-124011,61719,18506,-115116,36757},
    {127416,-203211,125929,81589,-65781,-141865},
    {188269,-67286,-167697,-166062,143089,42712},
    {126468,94598,14918,194817,-50016,21429},
    {135641,48803,148942,31852,84238,-187018},
    {-112045,-86725,-173031,-110030,-164298,-91425},
    {55871,-55659,-53456,-119618,-95953,179803},
    {60957,44937,-135417,94349,-138602,-49637},
    {201573,57648,23450,76020,141178,113650},
    {-111571,-192669,-75992,-95638,-119010,182379},
    {154979,-76311,64006,-42976,170700,-16944},
    {-96816,-104332,25270,-97697,34830,163813},
    {-41424,-115577,204873,3923,-168453,-186486},
    {-160737,52479,120271,-32053,-179728,-48746},
    {204290,11989,193978,148560,-201160,90888},
    {74824,15223,-96016,58045,-159953,-26862},
    {-19055,186870,154767,-97430,241,-132323},
    {169910,152570,-82995,57216,44871,-142952},
    {108095,16215,114731,12499,-205652,-72408},
    {-197867,176692,155948,136572,-79261,-182035},
    {155655,184043,-170617,-5769,-177387,107309},
    {109464,-153019,-10178,20508,-96744,153359},
    {-31650,-118674,16181,94680,-123058,-77530},
    {203890,61716,-25489,7237,-156061,-113363},
    {-66672,36363,-111132,-115207,-176654,53985},
    {-111615,166942,148089,-176710,-107883,69581},
    {0,0,0,0,0,0},
    {102944,102944,102944,102944,102944,102944},
    {205887,-205887,205887,-205887,205887,-205887},
    {6554,13107,-19661,26214,7,-32768},
};
static const double REF[32][3] = {
    {-0.0222865, +0.0576375, +0.2400478},
    {+0.1892420, -0.1393546, +0.2369934},
    {+0.1541539, +0.0106681, -0.1327851},
    {+0.0808908, -0.3269262, +0.0352504},
    {-0.2094676, +0.0310617, +0.1479049},
    {-0.1119370, +0.2801435, +0.4805806},
    {-0.0385956, +0.1223458, +0.5201526},
    {-0.0558630, -0.1655257, +0.1162543},
    {+0.0290219, -0.0472070, -0.0350089},
    {-0.0023557, +0.0748822, +0.2127767},
    {-0.3560566, +0.0656427, +0.3374268},
    {-0.0494578, +0.0303743, +0.2696616},
    {-0.1432098, +0.1167147, -0.1745249},
    {+0.0134538, +0.1541086, -0.1736805},
    {+0.1339399, -0.0958778, -0.0346702},
    {-0.1840735, -0.1385864, +0.4512082},
    {-0.1929612, +0.0016624, +0.3394641},
    {+0.0246408, +0.1198741, +0.1306493},
    {-0.3126872, +0.0937794, +0.0555409},
    {-0.1231314, +0.1065587, +0.2653967},
    {-0.0275568, +0.3493796, +0.3225455},
    {+0.2975250, -0.0085604, +0.1010916},
    {+0.0538900, -0.0490636, +0.0989776},
    {+0.0036955, -0.2067074, +0.1760449},
    {-0.1242753, +0.1238890, -0.0788752},
    {-0.2681657, +0.0039965, +0.2078458},
    {+0.0436310, -0.1149608, +0.1571005},
    {+0.0145570, +0.2883734, +0.1552131},
    {+0.3109894, +0.0000022, -0.0809937},
    {+0.0549936, -0.0010094, +0.5579979},
    {+0.0890049, -0.0000013, +0.0289904},
    {+0.2806594, +0.0281614, -0.0456184},
};

// ---- Part 3 helpers: raw frames for the E2 determinism sweep ----
// Frame layouts copied byte-for-byte from yolo.cpp (op2 31B / op10 103B) and
// from the FPGA op18 spec (24B reply: last/min/max/runs/magic/0, uint32 LE).

static void e2_build_op2(unsigned char tx[31], const int q[6])
{
    tx[0] = 2; tx[1] = 0; tx[2] = 0; tx[3] = 0; tx[4] = 0; tx[5] = 24; tx[6] = 0;
    for (int i = 0; i < 6; ++i) {
        int v = q[i];                                  // QV entries are Q16.16 raw already
        tx[7 + 4 * i] =  v        & 0xFF;
        tx[8 + 4 * i] = (v >> 8)  & 0xFF;
        tx[9 + 4 * i] = (v >> 16) & 0xFF;
        tx[10 + 4 * i] = (v >> 24) & 0xFF;
    }
}

static void e2_build_op10(unsigned char tx[103])
{
    // Fixed impedance payload (values are irrelevant for the cycle count -- that
    // data-independence is exactly what the sweep verifies): x_des ~1-2 mm,
    // K = 100/100/100/10/10/10, D = 10/10/10/1/1/1, xdot = 0, all Q16.16.
    static const int XD[6] = { 66, -66, 66, 0, 131, -66 };
    static const int KK[6] = { 6553600, 6553600, 6553600, 655360, 655360, 655360 };
    static const int DD[6] = { 655360, 655360, 655360, 65536, 65536, 65536 };
    static const int XV[6] = { 0, 0, 0, 0, 0, 0 };
    tx[0] = 10; tx[1] = 0; tx[2] = 0; tx[3] = 0; tx[4] = 0; tx[5] = 96; tx[6] = 0;
    for (int i = 0; i < 6; ++i) {
        int vx = XD[i], vk = KK[i], vd = DD[i], vv = XV[i];
        tx[7 + 4 * i] = vx & 0xFF; tx[8 + 4 * i] = (vx >> 8) & 0xFF; tx[9 + 4 * i] = (vx >> 16) & 0xFF; tx[10 + 4 * i] = (vx >> 24) & 0xFF;
        tx[31 + 4 * i] = vk & 0xFF; tx[32 + 4 * i] = (vk >> 8) & 0xFF; tx[33 + 4 * i] = (vk >> 16) & 0xFF; tx[34 + 4 * i] = (vk >> 24) & 0xFF;
        tx[55 + 4 * i] = vd & 0xFF; tx[56 + 4 * i] = (vd >> 8) & 0xFF; tx[57 + 4 * i] = (vd >> 16) & 0xFF; tx[58 + 4 * i] = (vd >> 24) & 0xFF;
        tx[79 + 4 * i] = vv & 0xFF; tx[80 + 4 * i] = (vv >> 8) & 0xFF; tx[81 + 4 * i] = (vv >> 16) & 0xFF; tx[82 + 4 * i] = (vv >> 24) & 0xFF;
    }
}

// op18 read: [18][addr=0 LE4][len=24 LE2] -> 24B = last,min,max,runs,magic,tail (uint32 LE).
// Returns 1 on success and fills out[6]; 0 after retries.
static int e2_read_op18(SOCKET s, sockaddr_in* fpga, unsigned out[6])
{
    unsigned char rd[7], rx[64];
    int sz = sizeof(*fpga);
    rd[0] = 18; rd[1] = 0; rd[2] = 0; rd[3] = 0; rd[4] = 0; rd[5] = 24; rd[6] = 0;
    for (int att = 0; att < 5; ++att) {
        sendto(s, (char*)rd, 7, 0, (SOCKADDR*)fpga, sizeof(*fpga));
        int n = recvfrom(s, (char*)rx, sizeof(rx), 0, (SOCKADDR*)fpga, &sz);
        if (n >= 24) {
            for (int i = 0; i < 6; ++i)
                out[i] = (unsigned)rx[4*i] | ((unsigned)rx[4*i+1] << 8) | ((unsigned)rx[4*i+2] << 16) | ((unsigned)rx[4*i+3] << 24);
            return 1;
        }
        Sleep(100);
    }
    return 0;
}

// ---- Part 4 helpers: force-chain bit-exact sweep (raw op11/op12 reads) ----
// Read helper: [op][addr=0 LE4][len=24 LE2] (7B) -> 24B reply = 6 x int32 LE.
// Returns 1 on success and fills out[6]; 0 after retries.
static int p4_read6(SOCKET s, sockaddr_in* fpga, unsigned char op, int out[6])
{
    unsigned char rd[7], rx[64];
    int sz = sizeof(*fpga);
    rd[0] = op; rd[1] = 0; rd[2] = 0; rd[3] = 0; rd[4] = 0; rd[5] = 24; rd[6] = 0;
    for (int att = 0; att < 5; ++att) {
        sendto(s, (char*)rd, 7, 0, (SOCKADDR*)fpga, sizeof(*fpga));
        int n = recvfrom(s, (char*)rx, sizeof(rx), 0, (SOCKADDR*)fpga, &sz);
        if (n >= 24) {
            for (int i = 0; i < 6; ++i)
                out[i] = (int)((unsigned)rx[4*i] | ((unsigned)rx[4*i+1] << 8) |
                               ((unsigned)rx[4*i+2] << 16) | ((unsigned)rx[4*i+3] << 24));
            return 1;
        }
        Sleep(100);
    }
    return 0;
}

// RTL-simulated expected values for the force chain, generated by
// V3/sim/tb_force_board.v (round-to-nearest FK RTL) -> force_board_expected.txt.
// Per case: op2(QV[n]) -> op10(e2_build_op10 payload, byte-for-byte the same
// frame Part 4 sends) -> op11 tau (int32 Q16.16 N*m) / op12 iq (int32).
// Note: x_des is ABSOLUTE desired pose (wrench_calc.v: dx = x_des - pose), so the
// fixed near-zero x_des makes dx large and every iq saturates to +/-2000 -- the
// comparison is bit-exact regardless; full 32-bit diversity lives in tau.
static const int TAU_EXP[32][6] = {
    {1657026,-1154870,-1458094,101190,1898588,921611},
    {434500,2232438,1809253,499337,-1786125,1165439},
    {-440199,9130,-125722,-7659,-324259,-480356},
    {-1710117,1187279,1124578,1297120,-1073805,-336738},
    {339986,-798067,-461344,997791,-126392,584020},
    {-1444452,-354508,-668467,-585548,463138,-1336605},
    {-2041755,688577,1075950,-1981951,1095077,-896939},
    {1334080,1271905,1525943,1377535,323935,-246917},
    {-561130,-689997,-737267,996318,808382,-398743},
    {1943460,-323382,-157440,601691,2038930,237358},
    {-777574,-955803,-1038942,422330,495219,-936186},
    {-817465,-1070372,-1421754,-399187,-837861,-960988},
    {242703,28495,160711,-325138,496072,290037},
    {-1959527,1480033,1284008,364596,2079764,-284896},
    {-1235232,-1798664,-1579676,1162624,-1578890,-377672},
    {1419809,1069016,1263092,2018577,616109,-1401317},
    {1499042,-25858,364020,1503963,-1545305,-1406566},
    {1913305,-586401,-457228,-361004,1166925,1310352},
    {1811343,251149,305592,701391,-2441875,700628},
    {1722208,-552871,-945254,25891,-1865003,546703},
    {-2035924,1042464,1338236,-1084273,1662300,1089444},
    {1901151,96620,143973,695611,2020632,-612428},
    {-619503,-147924,-238643,672821,-242852,-702014},
    {-570351,777954,425230,-282927,190849,-733395},
    {-1365879,1200981,1010756,377113,-1283602,-1275810},
    {1139583,-49165,-174925,-1424851,275176,1099234},
    {1582310,-191313,-99867,-1493648,-996832,1199301},
    {-258710,-1665245,-1620909,-50942,1851671,-638160},
    {-2683,-365659,-473136,511,-113105,439},
    {-1030467,-825140,-825109,-1030111,202041,-1029482},
    {-1219,106698,70019,630,30433,-624},
    {-134130,-52035,-185107,-71782,-640879,-71553},
};
static const int IQ_EXP[32][6] = {
    {2000,-2000,-2000,2000,2000,2000},
    {2000,2000,2000,2000,-2000,2000},
    {-2000,2000,-2000,-2000,-2000,-2000},
    {-2000,2000,2000,2000,-2000,-2000},
    {2000,-2000,-2000,2000,-2000,2000},
    {-2000,-2000,-2000,-2000,2000,-2000},
    {-2000,2000,2000,-2000,2000,-2000},
    {2000,2000,2000,2000,2000,-2000},
    {-2000,-2000,-2000,2000,2000,-2000},
    {2000,-2000,-2000,2000,2000,2000},
    {-2000,-2000,-2000,2000,2000,-2000},
    {-2000,-2000,-2000,-2000,-2000,-2000},
    {2000,2000,2000,-2000,2000,2000},
    {-2000,2000,2000,2000,2000,-2000},
    {-2000,-2000,-2000,2000,-2000,-2000},
    {2000,2000,2000,2000,2000,-2000},
    {2000,-2000,2000,2000,-2000,-2000},
    {2000,-2000,-2000,-2000,2000,2000},
    {2000,2000,2000,2000,-2000,2000},
    {2000,-2000,-2000,2000,-2000,2000},
    {-2000,2000,2000,-2000,2000,2000},
    {2000,2000,2000,2000,2000,-2000},
    {-2000,-2000,-2000,2000,-2000,-2000},
    {-2000,2000,2000,-2000,2000,-2000},
    {-2000,2000,2000,2000,-2000,-2000},
    {2000,-2000,-2000,-2000,2000,2000},
    {2000,-2000,-2000,-2000,-2000,2000},
    {-2000,-2000,-2000,-2000,2000,-2000},
    {-2000,-2000,-2000,2000,-2000,2000},
    {-2000,-2000,-2000,-2000,2000,-2000},
    {-2000,2000,2000,2000,2000,-2000},
    {-2000,-2000,-2000,-2000,-2000,-2000},
};

int main()
{
    YOLO yolo;
    yolo.init_udp();

    // ---- Part 1: FK precision sweep (which RTL is on the board?) ----
    printf("=== FK precision sweep: 32 vectors, board pose vs double reference ===\n");
    double pose[6];
    double sum = 0.0, mx = 0.0;
    int worst = -1, link_fail = 0;
    for (int n = 0; n < 32; ++n) {
        double q[6];
        for (int i = 0; i < 6; ++i) q[i] = QV[n][i] / 65536.0;
        yolo.fk_pose(q, pose);
        double dx = pose[0] - REF[n][0];
        double dy = pose[1] - REF[n][1];
        double dz = pose[2] - REF[n][2];
        double emm = sqrt(dx*dx + dy*dy + dz*dz) * 1000.0;   // Euclidean, mm
        if (emm > 10.0) ++link_fail;                          // zeros from timeout etc.
        sum += emm;
        if (emm > mx) { mx = emm; worst = n; }
        printf("case %2d: err = %8.4f mm\n", n, emm);
    }
    double mean = sum / 32.0;
    printf("---- sweep summary: mean = %.4f mm   max = %.4f mm (case %d) ----\n", mean, mx, worst);
    if (link_fail) {
        printf("VERDICT: %d/32 cases look like NO RESPONSE (err > 10 mm) -- link problem, not precision.\n", link_fail);
    } else if (mean < 0.055) {
        printf("VERDICT: ROUND-TO-NEAREST RTL (new bit) -- FK precision upgraded, R1.2 margin restored.\n");
    } else {
        printf("VERDICT: TRUNCATING RTL (old bit still flashed?) -- expected mean ~0.09 mm matches old datapath.\n");
    }

    // ---- Part 2: original full-chain demo (liveness) ----
    const double D2R = 3.14159265358979 / 180.0;
    double q[6]    = { 30*D2R, -45*D2R, 60*D2R, 0, 90*D2R, 0 };
    double K[6]    = { 100, 100, 100, 10, 10, 10 };
    double Dd[6]   = { 10, 10, 10, 1, 1, 1 };
    double xdot[6] = { 0, 0, 0, 0, 0, 0 };
    const double delta[6] = { 0.001,-0.001,0.001, 0,0.002,-0.001 };

    double x_des[6], tau[6] = { 0 };
    int    iq[6] = { 0 }, duty[18] = { 0 };

    yolo.fk_pose(q, pose);
    for (int i = 0; i < 6; ++i) x_des[i] = pose[i] + delta[i];
    int ok = yolo.imp_iq_full(q, x_des, K, Dd, xdot, tau, iq);

    int phi[6] = { 0, 0, 0, 0, 0, 0 };
    int adc[18]; for (int i = 0; i < 18; ++i) adc[i] = 2048;
    yolo.write_phi_adc(phi, adc);
    yolo.read_pwm_duty(duty);

    printf("=== full-chain demo ===\n");
    printf("pose      : x=%.4f y=%.4f z=%.4f m | rx=%.4f ry=%.4f rz=%.4f rad\n",
           pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    printf("tau (Nm)  :"); for (int i = 0; i < 6; ++i) printf(" %+8.4f", tau[i]); printf("\n");
    printf("iq  (ref) :"); for (int i = 0; i < 6; ++i) printf(" %+6d", iq[i]); printf("\n");
    printf("PWM duty  : "); for (int m = 0; m < 6; ++m) printf("m%d=%d/%d/%d ", m, duty[3*m], duty[3*m+1], duty[3*m+2]); printf("\n");
    printf(ok ? "==== full chain OK ====\n" : "==== NO RESPONSE (check link / IP / ARP / firewall) ====\n");

    // ---- Part 3: E2 determinism sweep (on-chip cycle counter, op18) ----
    // One "run" = one full control-law computation = op2 (FK + Jacobian, 422 cyc)
    // followed by op10 (Cartesian impedance 119 cyc + torque-to-iq 4 cyc).
    // The on-chip observer adds the 4 command-FSM latch cycles and PAUSES between
    // the two commands (host/network gap is test I/O, not logic cycles), so every
    // completed chain must count exactly 4 + 422 + 119 + 4 = 549 cycles
    // (paper Table III).  10^5 chains -> min == max == 549 means zero spread.
    printf("\n=== Part 3: E2 determinism sweep (1e5 on-board forward chains, op18) ===\n");
    {
        SOCKET s3 = yolo.hServer;          // reuse the socket bound to 8080 (see note at top)
        sockaddr_in* fa = &yolo.fpgaAddr;
        DWORD tmo3 = 100;                  // short timeout: a lost ACK costs 100 ms, not 500 ms
        setsockopt(s3, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tmo3, sizeof(tmo3));

        unsigned b[6];                     // before: last,min,max,runs,magic,tail
        if (!e2_read_op18(s3, fa, b)) {
            printf("op18 BEFORE: no response -- link problem, sweep aborted.\n");
            return 0;
        }
        printf("op18 before: last=%u min=0x%08X max=%u runs=%u magic=0x%08X tail=%u\n",
               b[0], b[1], b[2], b[3], b[4], b[5]);
        if (b[4] != 0xE2E2E2E2u) {
            printf("MAGIC MISMATCH (got 0x%08X, want 0xE2E2E2E2) -- old bit still flashed? Sweep aborted.\n", b[4]);
            return 0;
        }

        const int REPS = 100000;
        int pairs_ok = 0, fail2 = 0, fail10 = 0;
        unsigned char t2[31], t10[103], rxb[64];
        int sz3 = sizeof(*fa);
        e2_build_op10(t10);
        for (int rep = 0; rep < REPS; ++rep) {
            int a2 = 0, a10 = 0;
            e2_build_op2(t2, QV[rep % 32]);     // joint angles cycle through the 32 vectors
            sendto(s3, (char*)t2, 31, 0, (SOCKADDR*)fa, sizeof(*fa));
            if (recvfrom(s3, (char*)rxb, sizeof(rxb), 0, (SOCKADDR*)fa, &sz3) > 0) a2 = 1; else ++fail2;
            sendto(s3, (char*)t10, 103, 0, (SOCKADDR*)fa, sizeof(*fa));
            if (recvfrom(s3, (char*)rxb, sizeof(rxb), 0, (SOCKADDR*)fa, &sz3) > 0) a10 = 1; else ++fail10;
            if (a2 && a10) ++pairs_ok;
            if ((rep + 1) % 10000 == 0)
                printf("  progress %6d / %d   (op2 ACK fail %d, op10 ACK fail %d)\n",
                       rep + 1, REPS, fail2, fail10);
        }

        unsigned a[6];                     // after
        if (!e2_read_op18(s3, fa, a)) {
            printf("op18 AFTER: no response -- link died mid-sweep?\n");
            return 0;
        }
        printf("op18 after : last=%u min=%u max=%u runs=%u magic=0x%08X tail=%u\n",
               a[0], a[1], a[2], a[3], a[4], a[5]);

        unsigned d_runs = a[3] - b[3];
        printf("summary    : chains sent=%d  both-ACK ok=%d  runs delta=%u  min=%u  max=%u\n",
               REPS, pairs_ok, d_runs, a[1], a[2]);
        if (a[1] == 549u && a[2] == 549u && d_runs == (unsigned)pairs_ok)
            printf("VERDICT: DETERMINISM CONFIRMED -- zero spread over 1e5 on-board executions.\n");
        else if (a[1] != a[2])
            printf("VERDICT: FAIL -- spread detected: min=%u max=%u (diff=%u).\n", a[1], a[2], a[2] - a[1]);
        else if (a[1] != 549u)
            printf("VERDICT: FAIL -- constant but not 549: min=max=%u.\n", a[1]);
        else
            printf("VERDICT: INCONCLUSIVE -- min=max=549 but runs delta %u != both-ACK pairs %d (ACK loss on the wire?).\n",
                   d_runs, pairs_ok);
    }

    // ---- Part 4: force-chain bit-exact sweep (op2->op10->op11 tau / op12 iq) ----
    // Same 32 joint vectors as Part 1 (pose already bit-verified on board) and the
    // SAME fixed op10 payload as Part 3 (e2_build_op10, byte-for-byte).  Board tau
    // and iq are compared bit-exactly against the RTL simulation expectations
    // (TAU_EXP/IQ_EXP above, from V3/sim/tb_force_board.v).  32/32 exact upgrades
    // Table VI's carrier from FK-only to the full force chain FK->J->F->tau->iq.
    printf("\n=== Part 4: force-chain bit-exact sweep (32 cases, op11 tau / op12 iq) ===\n");
    {
        SOCKET s4 = yolo.hServer;          // same socket note as Part 3
        sockaddr_in* fa4 = &yolo.fpgaAddr;
        unsigned char t2[31], t10[103], rxb[64];
        int sz4 = sizeof(*fa4);
        int tau_got[6], iq_got[6];
        int case_ok[32];
        int exact = 0, chain_fail = 0;
        e2_build_op10(t10);
        for (int n = 0; n < 32; ++n) {
            case_ok[n] = 0;
            e2_build_op2(t2, QV[n]);
            sendto(s4, (char*)t2, 31, 0, (SOCKADDR*)fa4, sizeof(*fa4));
            int a2 = (recvfrom(s4, (char*)rxb, sizeof(rxb), 0, (SOCKADDR*)fa4, &sz4) > 0);
            sendto(s4, (char*)t10, 103, 0, (SOCKADDR*)fa4, sizeof(*fa4));
            int a10 = (recvfrom(s4, (char*)rxb, sizeof(rxb), 0, (SOCKADDR*)fa4, &sz4) > 0);
            int r11 = p4_read6(s4, fa4, 11, tau_got);
            int r12 = p4_read6(s4, fa4, 12, iq_got);
            if (!a2 || !a10 || !r11 || !r12) {
                printf("case %2d: NO RESPONSE (op2 %d op10 %d op11 %d op12 %d)\n", n, a2, a10, r11, r12);
                ++chain_fail;
                continue;
            }
            int tau_ok = 1, iq_ok = 1;
            for (int i = 0; i < 6; ++i) {
                if (tau_got[i] != TAU_EXP[n][i]) tau_ok = 0;
                if (iq_got[i]  != IQ_EXP[n][i])  iq_ok = 0;
            }
            printf("case %2d: tau %s iq %s\n", n, tau_ok ? "MATCH" : "DIFF", iq_ok ? "MATCH" : "DIFF");
            if (!tau_ok)
                for (int i = 0; i < 6; ++i)
                    if (tau_got[i] != TAU_EXP[n][i])
                        printf("   tau[%d] exp=0x%08X got=0x%08X\n", i, (unsigned)TAU_EXP[n][i], (unsigned)tau_got[i]);
            if (!iq_ok)
                for (int i = 0; i < 6; ++i)
                    if (iq_got[i] != IQ_EXP[n][i])
                        printf("   iq [%d] exp=0x%08X got=0x%08X\n", i, (unsigned)IQ_EXP[n][i], (unsigned)iq_got[i]);
            if (tau_ok && iq_ok) { case_ok[n] = 1; ++exact; }
        }
        printf("---- Part 4 summary: %d/32 exact", exact);
        if (chain_fail) printf("  (%d case(s) no-response)", chain_fail);
        printf(" ----\n");
        if (exact == 32) {
            printf("VERDICT: FORCE-CHAIN BIT-EXACT ON BOARD -- Table VI carrier upgraded.\n");
        } else {
            printf("VERDICT: FORCE-CHAIN NOT BIT-EXACT -- diff/failed cases:");
            for (int n = 0; n < 32; ++n) if (!case_ok[n]) printf(" %d", n);
            printf("\n");
        }
    }

    // ---- Part 5: R2.1 on-board -- adaptive_en 0/1 cycle equality ----
    // Phase A runs 20000 forward chains with adaptive_en still 0 (op15 was never
    // sent since bitstream load); phase B repeats them after op15(enable=1).
    // Claims verified on real silicon:
    //   (1) op18 min == max == 549 across BOTH phases (counters are cumulative,
    //       so a single deviating chain in either phase would split min/max);
    //   (2) adaptation is genuinely ACTIVE in phase B: op17 equals the RTL-sim
    //       18 words and tau(QV[0]) changes from tauA (=TAU_EXP[0]) to tauB.
    // Expected integers come from V3/sim/tb_r21_board.v (RTL simulation ->
    // r21_board_expected.txt); the op15 payload below is byte-for-byte the tb's
    // (enable=1, q_err=0.015625 rad, Ks_nom=150 N/m, Ds_nom=10 N*s/m, Q16.16).
    // NOTE: expectations assume a FRESH bitstream load. Parts 1-4 never send
    // op15/op16, so adaptive_en=0 and K_env/P sit at reset priors when Part 5
    // starts. Re-running the host without reprogramming leaves adaptive_en=1
    // and a mutated K_env -- the tauA/op17 compares would then legitimately DIFF.
    printf("\n=== Part 5: R2.1 on-board -- adaptive_en 0/1 cycle equality ===\n");
    {
        static const int P5_QE = 1024;       // q_err  = 0.015625 rad  (Q16.16)
        static const int P5_KN = 9830400;    // Ks_nom = 150.0 N/m
        static const int P5_DN = 655360;     // Ds_nom = 10.0 N*s/m
        static const int P5_ADAPT_EXP[18] = {   // op17: K_env_est x6, Ks_out x6, Ds_out x6
            42342630, 77713240, 81527470, 61913364, 39304041, 51593355,
             2282257,  1243504,  1185327,  1560838,  2458697,  1873046,
              152150,    82900,    79020,   104050,   163910,   124860 };
        static const int P5_TAUB_EXP[6] = {     // op11 after op15, chain QV[0]+op10
            4735077, -3314153, -3369290, 1168912, 5093184, 2597363 };

        SOCKET s5 = yolo.hServer;          // same socket note as Part 3
        sockaddr_in* fa5 = &yolo.fpgaAddr;
        unsigned char t2[31], t10[103], t15[80], rxb[128];
        int sz5 = sizeof(*fa5);
        int p5_fail = 0;
        e2_build_op10(t10);

        unsigned base5[6];
        if (!e2_read_op18(s5, fa5, base5)) { printf("Part 5: op18 base read no response -- aborted.\n"); return 0; }
        if (base5[4] != 0xE2E2E2E2u) { printf("Part 5: op18 magic bad (0x%08X) -- aborted.\n", base5[4]); return 0; }

        // ---- phase A: 20000 chains, adaptive_en = 0 ----
        const int P5REPS = 20000;
        int okA = 0;
        for (int rep = 0; rep < P5REPS; ++rep) {
            int a2 = 0, a10 = 0;
            e2_build_op2(t2, QV[rep % 32]);
            sendto(s5, (char*)t2, 31, 0, (SOCKADDR*)fa5, sizeof(*fa5));
            if (recvfrom(s5, (char*)rxb, sizeof(rxb), 0, (SOCKADDR*)fa5, &sz5) > 0) a2 = 1;
            sendto(s5, (char*)t10, 103, 0, (SOCKADDR*)fa5, sizeof(*fa5));
            if (recvfrom(s5, (char*)rxb, sizeof(rxb), 0, (SOCKADDR*)fa5, &sz5) > 0) a10 = 1;
            if (a2 && a10) ++okA;
            if ((rep + 1) % 5000 == 0) printf("  phase A progress %5d / %d\n", rep + 1, P5REPS);
        }
        unsigned mA[6];
        if (!e2_read_op18(s5, fa5, mA)) { printf("Part 5: op18 read after phase A failed -- aborted.\n"); return 0; }
        printf("phase A (adaptive_en=0): chains=%d ok=%d op18 min=%u max=%u runs+=%u\n",
               P5REPS, okA, mA[1], mA[2], mA[3] - base5[3]);
        if (mA[1] != 549u || mA[2] != 549u) {
            printf("phase A op18 FAIL: min=%u max=%u (want 549/549)\n", mA[1], mA[2]);
            p5_fail = 1;
        }

        // ---- tauA: chain QV[0] + op10, read op11 (must equal TAU_EXP[0]) ----
        int tauA[6] = { 0 }, tauB[6] = { 0 };
        e2_build_op2(t2, QV[0]);
        sendto(s5, (char*)t2, 31, 0, (SOCKADDR*)fa5, sizeof(*fa5));
        int aA2 = (recvfrom(s5, (char*)rxb, sizeof(rxb), 0, (SOCKADDR*)fa5, &sz5) > 0);
        sendto(s5, (char*)t10, 103, 0, (SOCKADDR*)fa5, sizeof(*fa5));
        int aA10 = (recvfrom(s5, (char*)rxb, sizeof(rxb), 0, (SOCKADDR*)fa5, &sz5) > 0);
        int rA11 = p4_read6(s5, fa5, 11, tauA);
        int tauA_ok = (aA2 && aA10 && rA11);
        if (tauA_ok)
            for (int i = 0; i < 6; ++i) if (tauA[i] != TAU_EXP[0][i]) tauA_ok = 0;
        printf("tauA vs expected: %s\n", tauA_ok ? "MATCH" : "DIFF");
        if (!tauA_ok) {
            if (!aA2 || !aA10 || !rA11)
                printf("   (link: op2 %d op10 %d op11 %d)\n", aA2, aA10, rA11);
            for (int i = 0; i < 6; ++i)
                if (tauA[i] != TAU_EXP[0][i])
                    printf("   tauA[%d] exp=0x%08X got=0x%08X\n", i, (unsigned)TAU_EXP[0][i], (unsigned)tauA[i]);
            p5_fail = 1;   // state leak or payload mismatch -- see NOTE above
        }

        // ---- op15 (enable=1) + op17 compare ----
        // Frame: [15][addr=0 LE4][len=73 LE2][enable][q_err x6][Ks_nom x6][Ds_nom x6] = 80B.
        t15[0] = 15; t15[1] = 0; t15[2] = 0; t15[3] = 0; t15[4] = 0; t15[5] = 73; t15[6] = 0;
        t15[7] = 1;                                       // enable byte -> adaptive_en=1
        for (int i = 0; i < 6; ++i) {
            t15[ 8 + 4*i] =  P5_QE        & 0xFF; t15[ 9 + 4*i] = (P5_QE >> 8)  & 0xFF;
            t15[10 + 4*i] = (P5_QE >> 16) & 0xFF; t15[11 + 4*i] = (P5_QE >> 24) & 0xFF;
            t15[32 + 4*i] =  P5_KN        & 0xFF; t15[33 + 4*i] = (P5_KN >> 8)  & 0xFF;
            t15[34 + 4*i] = (P5_KN >> 16) & 0xFF; t15[35 + 4*i] = (P5_KN >> 24) & 0xFF;
            t15[56 + 4*i] =  P5_DN        & 0xFF; t15[57 + 4*i] = (P5_DN >> 8)  & 0xFF;
            t15[58 + 4*i] = (P5_DN >> 16) & 0xFF; t15[59 + 4*i] = (P5_DN >> 24) & 0xFF;
        }
        sendto(s5, (char*)t15, 80, 0, (SOCKADDR*)fa5, sizeof(*fa5));
        int ack15 = (recvfrom(s5, (char*)rxb, sizeof(rxb), 0, (SOCKADDR*)fa5, &sz5) >= 18 && rxb[0] == 0xAA);

        int adapt_got[18] = { 0 }; int r17 = 0;
        {
            unsigned char rd[7];
            rd[0] = 17; rd[1] = 0; rd[2] = 0; rd[3] = 0; rd[4] = 0; rd[5] = 72; rd[6] = 0;
            for (int att = 0; att < 5 && !r17; ++att) {
                sendto(s5, (char*)rd, 7, 0, (SOCKADDR*)fa5, sizeof(*fa5));
                int n = recvfrom(s5, (char*)rxb, sizeof(rxb), 0, (SOCKADDR*)fa5, &sz5);
                if (n >= 72) {
                    for (int i = 0; i < 18; ++i)
                        adapt_got[i] = (int)((unsigned)rxb[4*i] | ((unsigned)rxb[4*i+1] << 8) |
                                             ((unsigned)rxb[4*i+2] << 16) | ((unsigned)rxb[4*i+3] << 24));
                    r17 = 1;
                } else Sleep(100);
            }
        }
        int op17_ok = (ack15 && r17);
        if (op17_ok)
            for (int i = 0; i < 18; ++i) if (adapt_got[i] != P5_ADAPT_EXP[i]) op17_ok = 0;
        printf("op15 enable=1 ACK %s; op17 vs expected: %s%s\n",
               ack15 ? "ok" : "FAIL", op17_ok ? "MATCH" : "DIFF",
               op17_ok ? "  -> adaptation ACTIVE" : "");
        if (!op17_ok) {
            if (!r17) printf("   (op17 no response)\n");
            else for (int i = 0; i < 18; ++i)
                if (adapt_got[i] != P5_ADAPT_EXP[i])
                    printf("   op17[%2d] exp=0x%08X got=0x%08X\n", i, (unsigned)P5_ADAPT_EXP[i], (unsigned)adapt_got[i]);
            p5_fail = 1;
        }

        // ---- phase B: 20000 chains, adaptive_en = 1 ----
        int okB = 0;
        for (int rep = 0; rep < P5REPS; ++rep) {
            int a2 = 0, a10 = 0;
            e2_build_op2(t2, QV[rep % 32]);
            sendto(s5, (char*)t2, 31, 0, (SOCKADDR*)fa5, sizeof(*fa5));
            if (recvfrom(s5, (char*)rxb, sizeof(rxb), 0, (SOCKADDR*)fa5, &sz5) > 0) a2 = 1;
            sendto(s5, (char*)t10, 103, 0, (SOCKADDR*)fa5, sizeof(*fa5));
            if (recvfrom(s5, (char*)rxb, sizeof(rxb), 0, (SOCKADDR*)fa5, &sz5) > 0) a10 = 1;
            if (a2 && a10) ++okB;
            if ((rep + 1) % 5000 == 0) printf("  phase B progress %5d / %d\n", rep + 1, P5REPS);
        }
        unsigned mB[6];
        if (!e2_read_op18(s5, fa5, mB)) { printf("Part 5: op18 read after phase B failed -- aborted.\n"); return 0; }
        printf("phase B (adaptive_en=1): chains=%d ok=%d op18 min=%u max=%u runs+=%u\n",
               P5REPS, okB, mB[1], mB[2], mB[3] - mA[3]);
        if (mB[1] != 549u || mB[2] != 549u) {
            printf("phase B op18 FAIL: min=%u max=%u (want 549/549)\n", mB[1], mB[2]);
            p5_fail = 1;
        }

        // ---- tauB: chain QV[0] + op10, read op11 (RTL-sim expected, != tauA) ----
        e2_build_op2(t2, QV[0]);
        sendto(s5, (char*)t2, 31, 0, (SOCKADDR*)fa5, sizeof(*fa5));
        int aB2 = (recvfrom(s5, (char*)rxb, sizeof(rxb), 0, (SOCKADDR*)fa5, &sz5) > 0);
        sendto(s5, (char*)t10, 103, 0, (SOCKADDR*)fa5, sizeof(*fa5));
        int aB10 = (recvfrom(s5, (char*)rxb, sizeof(rxb), 0, (SOCKADDR*)fa5, &sz5) > 0);
        int rB11 = p4_read6(s5, fa5, 11, tauB);
        int tauB_ok = (aB2 && aB10 && rB11);
        if (tauB_ok)
            for (int i = 0; i < 6; ++i) if (tauB[i] != P5_TAUB_EXP[i]) tauB_ok = 0;
        int tauBA_diff = 0;
        for (int i = 0; i < 6; ++i) if (tauB[i] != tauA[i]) tauBA_diff = 1;
        printf("tauB vs expected: %s ; tauB != tauA: %s\n",
               tauB_ok ? "MATCH" : "DIFF", tauBA_diff ? "CONFIRMED" : "FAIL");
        if (!tauB_ok) {
            if (!aB2 || !aB10 || !rB11)
                printf("   (link: op2 %d op10 %d op11 %d)\n", aB2, aB10, rB11);
            for (int i = 0; i < 6; ++i)
                if (tauB[i] != P5_TAUB_EXP[i])
                    printf("   tauB[%d] exp=0x%08X got=0x%08X\n", i, (unsigned)P5_TAUB_EXP[i], (unsigned)tauB[i]);
            p5_fail = 1;
        }
        if (!tauBA_diff) p5_fail = 1;

        if (!p5_fail)
            printf("VERDICT: R2.1 ON-BOARD -- forward-chain cycles invariant to adaptive_en (549/549), adaptation active.\n");
        else
            printf("VERDICT: R2.1 ON-BOARD FAIL -- see DIFF/FAIL lines above.\n");
    }
    return 0;
}
