#ifndef YOLO_CPP
#define YOLO_CPP
#pragma once
#include <cstdint>
#include <WinSock2.h>
#include <WS2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

// Minimal UDP client for the FPGA FK / impedance / FOC pipeline.
// FPGA 172.28.109.173:8081  <->  host 172.28.109.174:8080  (init_udp binds these).
// (Original YOLO object-detection members/methods + OpenCV removed during cleanup.)
class YOLO
{
public:
	YOLO();
	~YOLO();

	void init_udp();

	// FK data-path loopback (Milestone 1: 24B echo). in[6] -> out[6] (reply == request).
	void fk_echo(const uint32_t in[6], uint32_t out[6]);

	// FK stage-A (Milestone 2.1): q_rad[6] -> xyz[3] (meters) + sent[3] (sentinel rx/ry/rz).
	void fk_pos(const double q_rad[6], double xyz[3], uint32_t sent[3]);

	// FK full pose (Milestone 2.2): q_rad[6] -> pose[6] = x,y,z (m) + rx,ry,rz (rad).
	void fk_pose(const double q_rad[6], double pose[6]);

	// M3.1 path check: op9 should return a 144B ramp byte[i]=i. Prints OK/FAIL.
	void fk_jac_pathcheck();

	// M3.2: q_rad[6] -> J[36] = 6x6 geometric Jacobian, row-major (rows 0-2 linear m, 3-5 angular).
	void fk_jacobian(const double q_rad[6], double J[36]);

	// Benchmark helper: op2 write q + recv ACK only (no read-back). Returns 1 on ACK, 0 on timeout.
	int fk_write(const double q_rad[6]);

	// Impedance M4.1: op2 write q + op10 write F + op11 read tau (joint torques). 1 on success, 0 on timeout.
	int imp_torque_from_F(const double q_rad[6], const double F[6], double tau[6]);

	// Impedance M4.2 stiffness: op2 write q + op10 write x_des,K (48B) + op11 read tau. tau=J^T(K*(x_des-x_cur)). 1/0.
	int imp_torque_stiffness(const double q_rad[6], const double x_des[6], const double K[6], double tau[6]);

	// Impedance M4.3 full: op2 write q + op10 write x_des,K,D,xdot (96B) + op11 read tau. tau=J^T(K*(x_des-x_cur)-D*xdot). 1/0.
	int imp_torque_full(const double q_rad[6], const double x_des[6], const double K[6], const double D[6], const double xdot[6], double tau[6]);

	// FOC M5.1: op2 + op10(96B) + op11 read tau(double) + op12 read iq(signed16). iq=sat((tau*KT_INV)>>16,+/-2000). 1/0.
	int imp_iq_full(const double q_rad[6], const double x_des[6], const double K[6], const double D[6], const double xdot[6], double tau[6], int iq[6]);

	// FOC M5.3: op13 read 6-motor x 3-phase PWM duty (18 x int16 LE, each 10-bit, center 512). Monitor only. 1/0.
	int read_pwm_duty(int duty[18]);

	// FOC M5.3 (all-over-UDP): op14 write 6 encoder phi + 6-motor x 3-phase ADC current (24 x int16 LE, 48B) to feed FOC. 1/0.
	int write_phi_adc(const int phi[6], const int adc[18]);

private:

	SOCKET hServer;
	struct sockaddr_in fpgaAddr;

};

#endif
