#!/usr/bin/env python3
# e6_rtt.py -- E6: host-FPGA UDP round-trip characterization on the deployed bitstream.
#
# Measures, in lockstep (one outstanding request), with time.perf_counter_ns():
#   Test 1: op8 read-pose single-command RTT, N=100000
#   Test 2: full host-in-the-loop impedance cycle op2+op10+op11+op12, N=10000
# Prints min/p50/mean/p90/p99/p99.9/max in us, dumps raw samples to CSV.
#
# Prereqs: PC NIC static 172.28.109.174/24 wired to the board; firewall allows
# python.exe (first run pops a prompt); no other program bound to port 8080
# (close the VS host console first).
import socket, struct, sys, time

FPGA = ("172.28.109.173", 8081)
HOST = ("172.28.109.174", 8080)
N_OP8 = 100000
N_CYC = 10000

def frame_read(op, rdlen):
    return bytes([op, 0, 0, 0, 0, rdlen & 0xFF, (rdlen >> 8) & 0xFF])

def frame_op2(q6):
    b = bytearray([2, 0, 0, 0, 0, 24, 0])
    for v in q6:
        b += struct.pack("<i", v)
    return bytes(b)

def frame_op10():
    # x_des ~1-2mm, K=100/100/100/10/10/10, D=10/10/10/1/1/1, xdot=0 (Q16.16)
    xd = [66, -66, 66, 0, 131, -66]
    kk = [6553600]*3 + [655360]*3
    dd = [655360]*3 + [65536]*3
    xv = [0]*6
    b = bytearray([10, 0, 0, 0, 0, 96, 0])
    for arr in (xd, kk, dd, xv):
        for v in arr:
            b += struct.pack("<i", v)
    return bytes(b)

Q_DEMO = [34315, -51472, 68629, 0, 102944, 0]   # 30/-45/60/0/90/0 deg in Q16.16

def xact(s, tx, exp_len):
    """one blocking request->reply; returns (rtt_ns, ok)"""
    t0 = time.perf_counter_ns()
    s.sendto(tx, FPGA)
    try:
        rx, _ = s.recvfrom(2048)
    except socket.timeout:
        return (0, False)
    t1 = time.perf_counter_ns()
    return (t1 - t0, len(rx) >= exp_len)

def pctl(sorted_v, p):
    n = len(sorted_v)
    i = min(n - 1, max(0, int(p * n + 0.5) - 1))
    return sorted_v[i]

def report(tag, samples_ns, timeouts):
    v = sorted(samples_ns)
    us = [x / 1000.0 for x in v]
    mean = sum(us) / len(us)
    print("%s: N=%d timeouts=%d" % (tag, len(us), timeouts))
    print("  RTT(us): min=%.1f p50=%.1f mean=%.1f p90=%.1f p99=%.1f p99.9=%.1f max=%.1f"
          % (us[0], pctl(us, 0.50), mean, pctl(us, 0.90), pctl(us, 0.99), pctl(us, 0.999), us[-1]))
    return us

def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(HOST)
    except OSError as e:
        print("BIND FAILED on %s:%d -- set NIC to 172.28.109.174 and close the VS host console. (%s)" % (HOST[0], HOST[1], e))
        return 1
    s.settimeout(0.5)

    # warmup: resolve ARP, wake the path, sanity-check reply lengths
    f8 = frame_read(8, 24)
    ok_warm = 0
    for _ in range(20):
        _, ok = xact(s, f8, 24)
        ok_warm += 1 if ok else 0
    if ok_warm == 0:
        print("NO RESPONSE in warmup -- check link / IP / ARP / firewall / bit flashed.")
        return 1
    print("warmup ok (%d/20 replies)" % ok_warm)

    # ---- Test 1: op8 single-command RTT ----
    print("=== E6 Test 1: op8 single-command RTT, N=%d ===" % N_OP8)
    t1s, to1 = [], 0
    for n in range(N_OP8):
        rtt, ok = xact(s, f8, 24)
        if ok:
            t1s.append(rtt)
        else:
            to1 += 1
        if (n + 1) % 20000 == 0:
            print("  progress %d/%d" % (n + 1, N_OP8))
    us1 = report("op8 RTT", t1s, to1)

    # ---- Test 2: full host-in-loop impedance cycle ----
    print("=== E6 Test 2: full cycle op2+op10+op11+op12, N=%d ===" % N_CYC)
    f2, f10 = frame_op2(Q_DEMO), frame_op10()
    f11, f12 = frame_read(11, 24), frame_read(12, 24)
    t2s, to2 = [], 0
    for n in range(N_CYC):
        t0 = time.perf_counter_ns()
        ok = True
        for tx, el in ((f2, 18), (f10, 18), (f11, 24), (f12, 24)):
            _, o = xact(s, tx, el)
            ok = ok and o
        t1 = time.perf_counter_ns()
        if ok:
            t2s.append(t1 - t0)
        else:
            to2 += 1
        if (n + 1) % 2000 == 0:
            print("  progress %d/%d" % (n + 1, N_CYC))
    us2 = report("host-in-loop cycle", t2s, to2)

    with open("e6_rtt_op8.csv", "w") as f:
        f.write("rtt_us\n")
        for x in us1:
            f.write("%.3f\n" % x)
    with open("e6_cycle.csv", "w") as f:
        f.write("cycle_us\n")
        for x in us2:
            f.write("%.3f\n" % x)
    print("CSV saved: e6_rtt_op8.csv, e6_cycle.csv (cwd)")
    print("E6_RTT_OK")
    return 0

if __name__ == "__main__":
    sys.exit(main())
