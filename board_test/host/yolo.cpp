#include "yolo.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdint>

// Minimal UDP client for the FPGA FK / impedance / FOC pipeline.
// (YOLO object-detection code + OpenCV dependency removed; see WORKFLOW notes.)

YOLO::YOLO() {}
YOLO::~YOLO() {}

void YOLO::init_udp()
{
    WORD sockVersion = MAKEWORD(2, 2);
    WSADATA wsadata;
    if (WSAStartup(sockVersion, &wsadata)) { printf("WSAStartup failed\n"); return; }
    hServer = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (hServer == INVALID_SOCKET) { printf("socket failed\n"); return; }
    sockaddr_in addrServer;
    const char LocalIP[] = "172.28.109.174";          // host (this PC)
    addrServer.sin_family = AF_INET;
    addrServer.sin_port = htons(8080);
    addrServer.sin_addr.S_un.S_addr = inet_addr(LocalIP);
    int nRet = bind(hServer, (sockaddr*)&addrServer, sizeof(addrServer));
    if (nRet == SOCKET_ERROR) { printf("socket bind failed\n"); closesocket(hServer); WSACleanup(); return; }
    fpgaAddr.sin_family = AF_INET;                     // FPGA
    fpgaAddr.sin_port = htons(8081);
    inet_pton(AF_INET, "172.28.109.173", &fpgaAddr.sin_addr);
}
// FK data-path loopback test (Milestone 1: 24B echo). Reply == request.
void YOLO::fk_echo(const uint32_t in[6], uint32_t out[6])
{
    int sz = sizeof(fpgaAddr);
    unsigned char tx[31];
    unsigned char rd[7];
    unsigned char rx[64];

    DWORD tmo = 500;
    setsockopt(hServer, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tmo, sizeof(tmo));

    // write: [2][addr=0 LE4][len=24 LE2][in0..in5 LE4]
    tx[0] = 2;
    tx[1] = 0; tx[2] = 0; tx[3] = 0; tx[4] = 0;
    tx[5] = 24; tx[6] = 0;
    for (int i = 0; i < 6; ++i) {
        tx[7 + 4 * i] =  in[i]        & 0xFF;
        tx[8 + 4 * i] = (in[i] >> 8)  & 0xFF;
        tx[9 + 4 * i] = (in[i] >> 16) & 0xFF;
        tx[10 + 4 * i] = (in[i] >> 24) & 0xFF;
    }

    // read: [8][addr=8 LE4][len=24 LE2]
    rd[0] = 8;
    rd[1] = 8; rd[2] = 0; rd[3] = 0; rd[4] = 0;
    rd[5] = 24; rd[6] = 0;

    for (int attempt = 0; attempt < 40; ++attempt) {
        sendto(hServer, (char*)tx, 31, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        int n = recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz);
        if (n <= 0) { Sleep(200); continue; }

        sendto(hServer, (char*)rd, 7, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        n = recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz);
        if (n >= 24) {
            for (int i = 0; i < 6; ++i) {
                out[i] =  (uint32_t)(unsigned char)rx[4 * i]
                       | ((uint32_t)(unsigned char)rx[4 * i + 1] << 8)
                       | ((uint32_t)(unsigned char)rx[4 * i + 2] << 16)
                       | ((uint32_t)(unsigned char)rx[4 * i + 3] << 24);
            }
            return;
        }
        Sleep(200);
    }
    printf("fk_echo: no response after retries (check link/IP/ARP/firewall)\n");
    for (int i = 0; i < 6; ++i) out[i] = 0xFFFFFFFFu;
}

// FK stage-A position read (Milestone 2.1). q_rad[6] -> xyz[3] (meters), sent[3] (sentinel rx/ry/rz).
void YOLO::fk_pos(const double q_rad[6], double xyz[3], uint32_t sent[3])
{
    int sz = sizeof(fpgaAddr);
    unsigned char tx[31];
    unsigned char rd[7];
    unsigned char rx[64];

    DWORD tmo = 500;
    setsockopt(hServer, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tmo, sizeof(tmo));

    // write: [2][addr=0 LE4][len=24 LE2][q0..q5 Q16.16 LE]
    tx[0] = 2;
    tx[1] = 0; tx[2] = 0; tx[3] = 0; tx[4] = 0;
    tx[5] = 24; tx[6] = 0;
    for (int i = 0; i < 6; ++i) {
        double sc = q_rad[i] * 65536.0;
        int v = (int)(sc + (sc < 0 ? -0.5 : 0.5));   // round to nearest, Q16.16
        tx[7 + 4 * i] =  v        & 0xFF;
        tx[8 + 4 * i] = (v >> 8)  & 0xFF;
        tx[9 + 4 * i] = (v >> 16) & 0xFF;
        tx[10 + 4 * i] = (v >> 24) & 0xFF;
    }

    // read: [8][addr=8 LE4][len=24 LE2]
    rd[0] = 8;
    rd[1] = 8; rd[2] = 0; rd[3] = 0; rd[4] = 0;
    rd[5] = 24; rd[6] = 0;

    for (int attempt = 0; attempt < 40; ++attempt) {
        sendto(hServer, (char*)tx, 31, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        if (recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz) <= 0) { Sleep(200); continue; }

        sendto(hServer, (char*)rd, 7, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        if (recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz) >= 24) {
            for (int i = 0; i < 3; ++i) {
                int v = (int)((unsigned)rx[4*i] | ((unsigned)rx[4*i+1] << 8) | ((unsigned)rx[4*i+2] << 16) | ((unsigned)rx[4*i+3] << 24));
                xyz[i] = v / 65536.0;
            }
            for (int i = 0; i < 3; ++i) {
                sent[i] = (unsigned)rx[12+4*i] | ((unsigned)rx[13+4*i] << 8) | ((unsigned)rx[14+4*i] << 16) | ((unsigned)rx[15+4*i] << 24);
            }
            return;
        }
        Sleep(200);
    }
    printf("fk_pos: no response after retries (check link/IP/ARP/firewall)\n");
    xyz[0] = xyz[1] = xyz[2] = 0.0; sent[0] = sent[1] = sent[2] = 0;
}

// FK full pose read (Milestone 2.2). q_rad[6] -> pose[6]: x,y,z (m) + rx,ry,rz (rad), all Q16.16.
void YOLO::fk_pose(const double q_rad[6], double pose[6])
{
    int sz = sizeof(fpgaAddr);
    unsigned char tx[31];
    unsigned char rd[7];
    unsigned char rx[64];

    DWORD tmo = 500;
    setsockopt(hServer, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tmo, sizeof(tmo));

    tx[0] = 2; tx[1] = 0; tx[2] = 0; tx[3] = 0; tx[4] = 0; tx[5] = 24; tx[6] = 0;
    for (int i = 0; i < 6; ++i) {
        double sc = q_rad[i] * 65536.0;
        int v = (int)(sc + (sc < 0 ? -0.5 : 0.5));
        tx[7 + 4 * i] =  v        & 0xFF;
        tx[8 + 4 * i] = (v >> 8)  & 0xFF;
        tx[9 + 4 * i] = (v >> 16) & 0xFF;
        tx[10 + 4 * i] = (v >> 24) & 0xFF;
    }
    rd[0] = 8; rd[1] = 8; rd[2] = 0; rd[3] = 0; rd[4] = 0; rd[5] = 24; rd[6] = 0;

    for (int attempt = 0; attempt < 40; ++attempt) {
        sendto(hServer, (char*)tx, 31, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        if (recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz) <= 0) { Sleep(200); continue; }
        sendto(hServer, (char*)rd, 7, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        if (recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz) >= 24) {
            for (int i = 0; i < 6; ++i) {
                int v = (int)((unsigned)rx[4*i] | ((unsigned)rx[4*i+1] << 8) | ((unsigned)rx[4*i+2] << 16) | ((unsigned)rx[4*i+3] << 24));
                pose[i] = v / 65536.0;
            }
            return;
        }
        Sleep(200);
    }
    printf("fk_pose: no response after retries (check link/IP/ARP/firewall)\n");
    for (int i = 0; i < 6; ++i) pose[i] = 0.0;
}

// M3.1 path check: send op9, expect a 144-byte ramp byte[i]=i. Prints OK/FAIL.
void YOLO::fk_jac_pathcheck()
{
    int sz = sizeof(fpgaAddr);
    unsigned char rd[7], rx[256];
    DWORD tmo = 500;
    setsockopt(hServer, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tmo, sizeof(tmo));
    rd[0] = 9; rd[1] = 0; rd[2] = 0; rd[3] = 0; rd[4] = 0; rd[5] = 144; rd[6] = 0;
    for (int att = 0; att < 40; ++att) {
        sendto(hServer, (char*)rd, 7, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        int n = recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz);
        if (n >= 144) {
            int ok = 1;
            for (int i = 0; i < 144; i++) if ((unsigned char)rx[i] != (unsigned char)i) ok = 0;
            printf(ok ? "op9 ramp OK (144B)\n" : "op9 ramp FAIL\n");
            return;
        }
        Sleep(200);
    }
    printf("op9: no response after retries\n");
}

// M3.2: write 6 joint angles (op2), read the 6x6 Jacobian (op9, 144B). J[36] row-major, Q16.16->double.
void YOLO::fk_jacobian(const double q_rad[6], double J[36])
{
    int sz = sizeof(fpgaAddr);
    unsigned char tx[31], rd[7], rx[256];
    DWORD tmo = 500;
    setsockopt(hServer, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tmo, sizeof(tmo));
    tx[0] = 2; tx[1] = 0; tx[2] = 0; tx[3] = 0; tx[4] = 0; tx[5] = 24; tx[6] = 0;
    for (int i = 0; i < 6; ++i) {
        double sc = q_rad[i] * 65536.0;
        int v = (int)(sc + (sc < 0 ? -0.5 : 0.5));
        tx[7 + 4 * i] =  v        & 0xFF; tx[8 + 4 * i] = (v >> 8)  & 0xFF;
        tx[9 + 4 * i] = (v >> 16) & 0xFF; tx[10 + 4 * i] = (v >> 24) & 0xFF;
    }
    rd[0] = 9; rd[1] = 0; rd[2] = 0; rd[3] = 0; rd[4] = 0; rd[5] = 144; rd[6] = 0;
    for (int att = 0; att < 40; ++att) {
        sendto(hServer, (char*)tx, 31, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        if (recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz) <= 0) { Sleep(200); continue; }
        sendto(hServer, (char*)rd, 7, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        if (recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz) >= 144) {
            for (int k = 0; k < 36; ++k) {
                int v = (int)((unsigned)rx[4*k] | ((unsigned)rx[4*k+1] << 8) | ((unsigned)rx[4*k+2] << 16) | ((unsigned)rx[4*k+3] << 24));
                J[k] = v / 65536.0;
            }
            return;
        }
        Sleep(200);
    }
    printf("fk_jacobian: no response after retries (check link/IP/ARP/firewall)\n");
    for (int k = 0; k < 36; ++k) J[k] = 0.0;
}

// Benchmark helper: op2 write q + recv ACK only (no read-back). Returns 1 on ACK, 0 on timeout.
int YOLO::fk_write(const double q_rad[6])
{
    int sz = sizeof(fpgaAddr);
    unsigned char tx[31], rx[64];
    DWORD tmo = 500;
    setsockopt(hServer, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tmo, sizeof(tmo));
    tx[0] = 2; tx[1] = 0; tx[2] = 0; tx[3] = 0; tx[4] = 0; tx[5] = 24; tx[6] = 0;
    for (int i = 0; i < 6; ++i) {
        double sc = q_rad[i] * 65536.0;
        int v = (int)(sc + (sc < 0 ? -0.5 : 0.5));
        tx[7 + 4 * i] =  v        & 0xFF; tx[8 + 4 * i] = (v >> 8)  & 0xFF;
        tx[9 + 4 * i] = (v >> 16) & 0xFF; tx[10 + 4 * i] = (v >> 24) & 0xFF;
    }
    for (int att = 0; att < 40; ++att) {
        sendto(hServer, (char*)tx, 31, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        if (recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz) > 0) return 1;
        Sleep(200);
    }
    return 0;
}

// M4.1 impedance: op2 write q + op10 write F + op11 read tau (6 joint torques, Q16.16).
int YOLO::imp_torque_from_F(const double q_rad[6], const double F[6], double tau[6])
{
    int sz = sizeof(fpgaAddr);
    unsigned char tx[31], txF[31], rd[7], rx[64];
    DWORD tmo = 500;
    setsockopt(hServer, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tmo, sizeof(tmo));
    // op2 write q
    tx[0] = 2; tx[1] = 0; tx[2] = 0; tx[3] = 0; tx[4] = 0; tx[5] = 24; tx[6] = 0;
    for (int i = 0; i < 6; ++i) {
        double sc = q_rad[i] * 65536.0;
        int v = (int)(sc + (sc < 0 ? -0.5 : 0.5));
        tx[7 + 4 * i] =  v        & 0xFF; tx[8 + 4 * i] = (v >> 8)  & 0xFF;
        tx[9 + 4 * i] = (v >> 16) & 0xFF; tx[10 + 4 * i] = (v >> 24) & 0xFF;
    }
    // op10 write F
    txF[0] = 10; txF[1] = 0; txF[2] = 0; txF[3] = 0; txF[4] = 0; txF[5] = 24; txF[6] = 0;
    for (int i = 0; i < 6; ++i) {
        double sc = F[i] * 65536.0;
        int v = (int)(sc + (sc < 0 ? -0.5 : 0.5));
        txF[7 + 4 * i] =  v        & 0xFF; txF[8 + 4 * i] = (v >> 8)  & 0xFF;
        txF[9 + 4 * i] = (v >> 16) & 0xFF; txF[10 + 4 * i] = (v >> 24) & 0xFF;
    }
    // op11 read tau
    rd[0] = 11; rd[1] = 11; rd[2] = 0; rd[3] = 0; rd[4] = 0; rd[5] = 24; rd[6] = 0;
    for (int att = 0; att < 40; ++att) {
        sendto(hServer, (char*)tx, 31, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        if (recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz) <= 0) { Sleep(200); continue; }
        sendto(hServer, (char*)txF, 31, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        if (recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz) <= 0) { Sleep(200); continue; }
        sendto(hServer, (char*)rd, 7, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        if (recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz) >= 24) {
            for (int i = 0; i < 6; ++i) {
                int v = (int)((unsigned)rx[4*i] | ((unsigned)rx[4*i+1] << 8) | ((unsigned)rx[4*i+2] << 16) | ((unsigned)rx[4*i+3] << 24));
                tau[i] = v / 65536.0;
            }
            return 1;
        }
        Sleep(200);
    }
    printf("imp_torque_from_F: no response after retries (check link/IP/ARP/firewall)\n");
    for (int i = 0; i < 6; ++i) tau[i] = 0.0;
    return 0;
}

// M4.2 stiffness: op2 write q + op10 write x_des,K (48B) + op11 read tau.
int YOLO::imp_torque_stiffness(const double q_rad[6], const double x_des[6], const double K[6], double tau[6])
{
    int sz = sizeof(fpgaAddr);
    unsigned char tx[31], txC[55], rd[7], rx[64];
    DWORD tmo = 500;
    setsockopt(hServer, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tmo, sizeof(tmo));
    // op2 write q
    tx[0] = 2; tx[1] = 0; tx[2] = 0; tx[3] = 0; tx[4] = 0; tx[5] = 24; tx[6] = 0;
    for (int i = 0; i < 6; ++i) {
        double sc = q_rad[i] * 65536.0;
        int v = (int)(sc + (sc < 0 ? -0.5 : 0.5));
        tx[7 + 4 * i] = v & 0xFF; tx[8 + 4 * i] = (v >> 8) & 0xFF;
        tx[9 + 4 * i] = (v >> 16) & 0xFF; tx[10 + 4 * i] = (v >> 24) & 0xFF;
    }
    // op10 write x_des(7..30) + K(31..54), len=48
    txC[0] = 10; txC[1] = 0; txC[2] = 0; txC[3] = 0; txC[4] = 0; txC[5] = 48; txC[6] = 0;
    for (int i = 0; i < 6; ++i) {
        double sx = x_des[i] * 65536.0;
        int vx = (int)(sx + (sx < 0 ? -0.5 : 0.5));
        txC[7 + 4 * i] = vx & 0xFF; txC[8 + 4 * i] = (vx >> 8) & 0xFF;
        txC[9 + 4 * i] = (vx >> 16) & 0xFF; txC[10 + 4 * i] = (vx >> 24) & 0xFF;
        double sk = K[i] * 65536.0;
        int vk = (int)(sk + (sk < 0 ? -0.5 : 0.5));
        txC[31 + 4 * i] = vk & 0xFF; txC[32 + 4 * i] = (vk >> 8) & 0xFF;
        txC[33 + 4 * i] = (vk >> 16) & 0xFF; txC[34 + 4 * i] = (vk >> 24) & 0xFF;
    }
    // op11 read tau
    rd[0] = 11; rd[1] = 11; rd[2] = 0; rd[3] = 0; rd[4] = 0; rd[5] = 24; rd[6] = 0;
    for (int att = 0; att < 40; ++att) {
        sendto(hServer, (char*)tx, 31, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        if (recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz) <= 0) { Sleep(200); continue; }
        sendto(hServer, (char*)txC, 55, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        if (recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz) <= 0) { Sleep(200); continue; }
        sendto(hServer, (char*)rd, 7, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        if (recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz) >= 24) {
            for (int i = 0; i < 6; ++i) {
                int v = (int)((unsigned)rx[4*i] | ((unsigned)rx[4*i+1] << 8) | ((unsigned)rx[4*i+2] << 16) | ((unsigned)rx[4*i+3] << 24));
                tau[i] = v / 65536.0;
            }
            return 1;
        }
        Sleep(200);
    }
    printf("imp_torque_stiffness: no response after retries (check link/IP/ARP/firewall)\n");
    for (int i = 0; i < 6; ++i) tau[i] = 0.0;
    return 0;
}

// M4.3 full impedance: op2 write q + op10 write x_des,K,D,xdot (96B) + op11 read tau.
int YOLO::imp_torque_full(const double q_rad[6], const double x_des[6], const double K[6], const double D[6], const double xdot[6], double tau[6])
{
    int sz = sizeof(fpgaAddr);
    unsigned char tx[31], txC[103], rd[7], rx[64];
    DWORD tmo = 500;
    setsockopt(hServer, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tmo, sizeof(tmo));
    // op2 write q
    tx[0] = 2; tx[1] = 0; tx[2] = 0; tx[3] = 0; tx[4] = 0; tx[5] = 24; tx[6] = 0;
    for (int i = 0; i < 6; ++i) {
        double sc = q_rad[i] * 65536.0;
        int v = (int)(sc + (sc < 0 ? -0.5 : 0.5));
        tx[7 + 4 * i] = v & 0xFF; tx[8 + 4 * i] = (v >> 8) & 0xFF;
        tx[9 + 4 * i] = (v >> 16) & 0xFF; tx[10 + 4 * i] = (v >> 24) & 0xFF;
    }
    // op10 write x_des(7..30)+K(31..54)+D(55..78)+xdot(79..102), len=96
    txC[0] = 10; txC[1] = 0; txC[2] = 0; txC[3] = 0; txC[4] = 0; txC[5] = 96; txC[6] = 0;
    for (int i = 0; i < 6; ++i) {
        double sx = x_des[i] * 65536.0; int vx = (int)(sx + (sx < 0 ? -0.5 : 0.5));
        txC[7 + 4 * i] = vx & 0xFF; txC[8 + 4 * i] = (vx >> 8) & 0xFF; txC[9 + 4 * i] = (vx >> 16) & 0xFF; txC[10 + 4 * i] = (vx >> 24) & 0xFF;
        double sk = K[i] * 65536.0; int vk = (int)(sk + (sk < 0 ? -0.5 : 0.5));
        txC[31 + 4 * i] = vk & 0xFF; txC[32 + 4 * i] = (vk >> 8) & 0xFF; txC[33 + 4 * i] = (vk >> 16) & 0xFF; txC[34 + 4 * i] = (vk >> 24) & 0xFF;
        double sd = D[i] * 65536.0; int vd = (int)(sd + (sd < 0 ? -0.5 : 0.5));
        txC[55 + 4 * i] = vd & 0xFF; txC[56 + 4 * i] = (vd >> 8) & 0xFF; txC[57 + 4 * i] = (vd >> 16) & 0xFF; txC[58 + 4 * i] = (vd >> 24) & 0xFF;
        double sv = xdot[i] * 65536.0; int vv = (int)(sv + (sv < 0 ? -0.5 : 0.5));
        txC[79 + 4 * i] = vv & 0xFF; txC[80 + 4 * i] = (vv >> 8) & 0xFF; txC[81 + 4 * i] = (vv >> 16) & 0xFF; txC[82 + 4 * i] = (vv >> 24) & 0xFF;
    }
    // op11 read tau
    rd[0] = 11; rd[1] = 11; rd[2] = 0; rd[3] = 0; rd[4] = 0; rd[5] = 24; rd[6] = 0;
    for (int att = 0; att < 40; ++att) {
        sendto(hServer, (char*)tx, 31, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        if (recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz) <= 0) { Sleep(200); continue; }
        sendto(hServer, (char*)txC, 103, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        if (recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz) <= 0) { Sleep(200); continue; }
        sendto(hServer, (char*)rd, 7, 0, (SOCKADDR*)&fpgaAddr, sizeof(fpgaAddr));
        if (recvfrom(hServer, (char*)rx, sizeof(rx), 0, (SOCKADDR*)&fpgaAddr, &sz) >= 24) {
            for (int i = 0; i < 6; ++i) {
                int v = (int)((unsigned)rx[4*i] | ((unsigned)rx[4*i+1] << 8) | ((unsigned)rx[4*i+2] << 16) | ((unsigned)rx[4*i+3] << 24));
                tau[i] = v / 65536.0;
            }
            return 1;
        }
        Sleep(200);
    }
    printf("imp_torque_full: no response after retries (check link/IP/ARP/firewall)\n");
    for (int i = 0; i < 6; ++i) tau[i] = 0.0;
    return 0;
}

// M5.1 FOC bridge: op2 q + op10 x_des,K,D,xdot(96B) + op11 read tau + op12 read iq.
int YOLO::imp_iq_full(const double q_rad[6], const double x_des[6], const double K[6], const double D[6], const double xdot[6], double tau[6], int iq[6])
{
    int sz = sizeof(fpgaAddr);
    unsigned char tx[31], txC[103], rdT[7], rdI[7], rx[64];
    DWORD tmo = 500;
    setsockopt(hServer, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tmo, sizeof(tmo));
    tx[0]=2; tx[1]=0; tx[2]=0; tx[3]=0; tx[4]=0; tx[5]=24; tx[6]=0;
    for (int i=0;i<6;++i){ double sc=q_rad[i]*65536.0; int v=(int)(sc+(sc<0?-0.5:0.5));
        tx[7+4*i]=v&0xFF; tx[8+4*i]=(v>>8)&0xFF; tx[9+4*i]=(v>>16)&0xFF; tx[10+4*i]=(v>>24)&0xFF; }
    txC[0]=10; txC[1]=0; txC[2]=0; txC[3]=0; txC[4]=0; txC[5]=96; txC[6]=0;
    for (int i=0;i<6;++i){
        double sx=x_des[i]*65536.0; int vx=(int)(sx+(sx<0?-0.5:0.5));
        txC[7+4*i]=vx&0xFF; txC[8+4*i]=(vx>>8)&0xFF; txC[9+4*i]=(vx>>16)&0xFF; txC[10+4*i]=(vx>>24)&0xFF;
        double sk=K[i]*65536.0; int vk=(int)(sk+(sk<0?-0.5:0.5));
        txC[31+4*i]=vk&0xFF; txC[32+4*i]=(vk>>8)&0xFF; txC[33+4*i]=(vk>>16)&0xFF; txC[34+4*i]=(vk>>24)&0xFF;
        double sd=D[i]*65536.0; int vd=(int)(sd+(sd<0?-0.5:0.5));
        txC[55+4*i]=vd&0xFF; txC[56+4*i]=(vd>>8)&0xFF; txC[57+4*i]=(vd>>16)&0xFF; txC[58+4*i]=(vd>>24)&0xFF;
        double sv=xdot[i]*65536.0; int vv=(int)(sv+(sv<0?-0.5:0.5));
        txC[79+4*i]=vv&0xFF; txC[80+4*i]=(vv>>8)&0xFF; txC[81+4*i]=(vv>>16)&0xFF; txC[82+4*i]=(vv>>24)&0xFF;
    }
    rdT[0]=11; rdT[1]=11; rdT[2]=0; rdT[3]=0; rdT[4]=0; rdT[5]=24; rdT[6]=0;
    rdI[0]=12; rdI[1]=0;  rdI[2]=0; rdI[3]=0; rdI[4]=0; rdI[5]=24; rdI[6]=0;
    for (int att=0; att<40; ++att) {
        sendto(hServer,(char*)tx,31,0,(SOCKADDR*)&fpgaAddr,sizeof(fpgaAddr));
        if (recvfrom(hServer,(char*)rx,sizeof(rx),0,(SOCKADDR*)&fpgaAddr,&sz)<=0){ Sleep(200); continue; }
        sendto(hServer,(char*)txC,103,0,(SOCKADDR*)&fpgaAddr,sizeof(fpgaAddr));
        if (recvfrom(hServer,(char*)rx,sizeof(rx),0,(SOCKADDR*)&fpgaAddr,&sz)<=0){ Sleep(200); continue; }
        sendto(hServer,(char*)rdT,7,0,(SOCKADDR*)&fpgaAddr,sizeof(fpgaAddr));
        if (recvfrom(hServer,(char*)rx,sizeof(rx),0,(SOCKADDR*)&fpgaAddr,&sz)>=24) {
            for (int i=0;i<6;++i){ int v=(int)((unsigned)rx[4*i]|((unsigned)rx[4*i+1]<<8)|((unsigned)rx[4*i+2]<<16)|((unsigned)rx[4*i+3]<<24)); tau[i]=v/65536.0; }
            sendto(hServer,(char*)rdI,7,0,(SOCKADDR*)&fpgaAddr,sizeof(fpgaAddr));
            if (recvfrom(hServer,(char*)rx,sizeof(rx),0,(SOCKADDR*)&fpgaAddr,&sz)>=24) {
                for (int i=0;i<6;++i){ int v=(int)((unsigned)rx[4*i]|((unsigned)rx[4*i+1]<<8)|((unsigned)rx[4*i+2]<<16)|((unsigned)rx[4*i+3]<<24)); iq[i]=v; }
                return 1;
            }
        }
        Sleep(200);
    }
    printf("imp_iq_full: no response after retries (check link/IP/ARP/firewall)\n");
    for (int i=0;i<6;++i){ tau[i]=0.0; iq[i]=0; }
    return 0;
}

// M5.3 FOC: op13 read 6-motor x 3-phase PWM duty (18 x int16 LE, 10-bit center 512). Monitor only.
int YOLO::read_pwm_duty(int duty[18])
{
    int sz = sizeof(fpgaAddr);
    unsigned char rd[7], rx[64];
    DWORD tmo = 500;
    setsockopt(hServer, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tmo, sizeof(tmo));
    rd[0]=13; rd[1]=0; rd[2]=0; rd[3]=0; rd[4]=0; rd[5]=36; rd[6]=0;
    for (int att=0; att<40; ++att) {
        sendto(hServer,(char*)rd,7,0,(SOCKADDR*)&fpgaAddr,sizeof(fpgaAddr));
        if (recvfrom(hServer,(char*)rx,sizeof(rx),0,(SOCKADDR*)&fpgaAddr,&sz) >= 36) {
            for (int i=0;i<18;++i) duty[i] = (int)((unsigned)rx[2*i] | ((unsigned)rx[2*i+1] << 8));
            return 1;
        }
        Sleep(200);
    }
    printf("read_pwm_duty: no response after retries (check link/IP/ARP/firewall)\n");
    for (int i=0;i<18;++i) duty[i]=0;
    return 0;
}

// M5.3 (all-over-UDP): op14 write 6 encoder phi + 6-motor x 3-phase ADC (24 x int16 LE, 48B payload) to feed FOC.
int YOLO::write_phi_adc(const int phi[6], const int adc[18])
{
    int sz = sizeof(fpgaAddr);
    unsigned char tx[55], rx[64];
    DWORD tmo = 500;
    setsockopt(hServer, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tmo, sizeof(tmo));
    tx[0]=14; tx[1]=0; tx[2]=0; tx[3]=0; tx[4]=0; tx[5]=48; tx[6]=0;       // op14, len=48
    for (int i=0;i<6;++i){  int v=phi[i]; tx[7+2*i]=v&0xFF;  tx[8+2*i]=(v>>8)&0xFF; }    // phi 7..18
    for (int i=0;i<18;++i){ int v=adc[i]; tx[19+2*i]=v&0xFF; tx[20+2*i]=(v>>8)&0xFF; }   // adc 19..54
    for (int att=0; att<40; ++att) {
        sendto(hServer,(char*)tx,55,0,(SOCKADDR*)&fpgaAddr,sizeof(fpgaAddr));
        if (recvfrom(hServer,(char*)rx,sizeof(rx),0,(SOCKADDR*)&fpgaAddr,&sz) > 0) return 1;  // ACK
        Sleep(200);
    }
    printf("write_phi_adc: no response after retries (check link/IP/ARP/firewall)\n");
    return 0;
}
