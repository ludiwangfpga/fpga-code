# Single-FPGA Compliant-Control Chain for a 6-DOF Manipulator

Complete Vivado sources, build scripts, verification host program, the deployed
bitstream, and the measured data for a compliant-control datapath that runs
entirely inside one FPGA.

## 1. What this is

The design computes the whole follower-side compliant-control chain of a 6-DOF
manipulator on a single Xilinx Zynq-7100 (`xc7z100ffg900-2`), in **one 125 MHz
clock domain**, as pure RTL: there is no soft processor, no operating system and
no software step anywhere on the datapath. Every control quantity is a Q16.16
fixed-point integer, little-endian on the wire. A host PC issues commands and
reads results over raw UDP.

On-chip signal flow:

```
RGMII PHY <-> GMII/AXIS <-> MAC <-> ARP / IP / UDP            rtl/eth/  (protocol stack, reused unchanged)
      |
      +--> udp_fk_cmd            opcode parser + reply generator          rtl/top/udp_fk_cmd.v
             |
             +--> forward kinematics, DH chain with CORDIC sin/cos/atan2  rtl/impedance/fk_engine.v
             +--> analytical Jacobian, serialised                         rtl/impedance/analytical_jacobian_serial.v
             +--> Cartesian impedance   F = K (x_des - x) - D xdot
                                        tau = J^T F                      rtl/impedance/cart_impedance.v
             +--> torque to current     iq* = sat(tau / Kt)               rtl/foc/torque_to_iq.v
             +--> six FOC current loops (Clarke / Park / PI / SVPWM)      rtl/foc/
             +--> per-joint RLS environment-stiffness estimation with an
                  inverse-proportional K / D adaptation law               rtl/impedance/adaptive_impedance_v2.v
             +--> pure-observer cycle counter (opcode 18) that timestamps
                  every forward chain and keeps last / min / max / runs
```

The synthesis top level is `impedance_control_top` (Ethernet pins only; the FOC
phase currents, encoder angles and PWM duties are fed and read back over UDP for
the verification campaign, so no motor-side pins are bound in this build).

Two properties the design is built around:

- **Fixed latency.** One forward chain (write joint angles, then read joint
  torques and currents) is 549 clock cycles, always: 4 command-FSM latch cycles
  + 422 FK/Jacobian + 119 Cartesian impedance + 4 torque-to-current. The on-chip
  observer measured min = max = 549 over 10^5 consecutive on-board executions,
  and the count is identical whether stiffness adaptation is enabled or not.
- **Bit-exact reproducibility.** The board reproduces the RTL simulation
  word for word: host Part 4 compares 32 on-board torque and current vectors
  against expectations emitted by the RTL testbench and reports 32/32 exact.

## 2. Repository layout

| Path | Content |
|---|---|
| `rtl/eth/` | Ethernet protocol stack: RGMII, GMII/AXIS bridges, MAC, ARP, IP, UDP, plus `arbiter/`, `hash/`, `mem/` helpers. Reused unchanged; the application never sees a packet header. |
| `rtl/impedance/` | FK engine, CORDIC sin/cos and atan2, analytical Jacobian, wrench and torque computation, Cartesian impedance, adaptive stiffness estimator, fixed-point helpers. |
| `rtl/foc/` | FOC current loops: Clarke/Park transforms, PI controllers, SVPWM, Cartesian-to-polar, torque-to-current, per-motor channel and the six-channel array. |
| `rtl/top/` | `impedance_control_top.v` (synthesis top), `udp_fk_cmd.v` (command plane), `top_udp_add.v` (earlier FK-only top, kept for reference). |
| `constraints/` | `top_udp_add.xdc`: pin assignment, I/O standards, RGMII interface clocks. |
| `ip/clk_wiz_0/` | Clocking-wizard IP configuration (`.xci`). The build scripts regenerate the IP output products. |
| `scripts/` | Vivado 2019.2 build scripts and one-command xsim verification pipelines. |
| `sim/` | All testbenches, stimulus `.mem` files, and the RTL-generated expectation files consumed by the host program. |
| `tools/` | Python reference models and measurement tools: double-precision FK and force-chain golden models, UDP round-trip probe, trajectory-experiment kit. |
| `board_test/bitstream/` | The deployed bitstream and its timing and utilization reports (build of record). |
| `board_test/host/` | Visual Studio host program that performs the on-board verification (Parts 1-5). |
| `board_test/data/` | Data produced by the tests: RTL dump, board console log, measured robot trajectory. See `board_test/data/README.md`. |
| `benchmarks/cpu_longrun/` | CPU baseline: per-call latency of the bit-level-equivalent control law on a Windows host (two 10^6-call runs and three long runs of 60–120 min), with source, raw data and analysis script. See `benchmarks/cpu_longrun/README.md`. |

`board_test/` deliberately keeps the three things you need for a hardware run
side by side: the bitstream you flash, the host you build, and the data those
runs produced.

## 3. Building the bitstream

Vivado **2019.2**, target part `xc7z100ffg900-2`. All flows are single-process,
non-project mode: `launch_runs` / `launch_simulation` spawn child processes that
fail with a broken pipe on some Windows installations, so the scripts call
`read_verilog` -> `synth_design` -> ... directly instead. Run everything **from
the repository root**.

```bat
set PATH=C:\Xilinx\Vivado\2019.2\bin;%PATH%

vivado -mode batch -source scripts\create_project.tcl   :: optional GUI project -> expect PROJECT_CREATED_OK
vivado -mode batch -source scripts\run_synth.tcl        :: synthesis only        -> expect SYNTH_OK
vivado -mode batch -source scripts\run_impl.tcl         :: synth, opt, place, route, bitstream -> expect BITSTREAM_OK
vivado -mode batch -source scripts\report_util.tcl      :: utilization from the routed checkpoint -> expect UTIL_DONE
```

The success criterion is the **sentinel string** at the end of the log, not the
exit code. `run_impl.tcl` writes `top_udp_add.bit`, `top_udp_add_routed.dcp` and
`timing_summary.rpt` to the repository root; `report_util.tcl` adds
`util_flat.rpt` / `util_hier.rpt`. Those paths are git-ignored — the copies of
record are the ones committed under `board_test/bitstream/`.

### Build of record

`board_test/bitstream/top_udp_add.bit` was produced by exactly these scripts on
2026-08-10 and is the bitstream every board test in this repository ran on.

| Metric | Value |
|---|---|
| WNS / WHS / WPWS | **+0.043 ns / +0.053 ns / +0.264 ns** (0 failing endpoints of 72080 / 72044 / 28658) |
| Slice LUTs | 28575 of 277400 (10.30 %) |
| Slice registers | 27949 of 554800 (5.04 %) |
| Block RAM tiles | 25 of 755 (3.31 %) |
| DSP48E1 | 666 of 2020 (32.97 %) |

Timing is met with strictly positive slack on every corner reported.

### Constraint scope

`constraints/top_udp_add.xdc` assigns pins and I/O standards and creates the two
125 MHz RGMII interface clocks. It intentionally does **not** contain a
`create_clock` for the `pl_clk` board oscillator or RGMII
`set_input_delay` / `set_output_delay` values: those depend on the specific
carrier board and PHY datasheet. Simulation, synthesis and bitstream generation
are unaffected; add the board-level values when deploying on other hardware.
`run_impl.tcl` also downgrades the `UCIO-1` and `NSTD-1` DRC checks to warnings,
because this build leaves the motor-side FOC ports unbound on purpose.

## 4. UDP command plane

The application module `rtl/top/udp_fk_cmd.v` sees only the UDP payload; the
stack strips and builds all headers. Every request has a 7-byte header followed
by payload:

```
byte 0      opcode
bytes 1-4   address  (parsed, not enforced; opcode and length are authoritative)
bytes 5-6   length
bytes 7...  payload
```

All multi-byte fields are little-endian; all physical quantities are Q16.16
(metres, radians, newtons, newton-metres) unless stated otherwise.

| Opcode | Direction | Payload in | Reply | Meaning |
|---|---|---|---|---|
| 2 | write | 24 B: `q0..q5` (Q16.16 rad) | 18 B ACK, first byte `0xAA` | Latch joint angles, start FK + Jacobian, latch pose and `J` on completion |
| 8 | read | — | 24 B | Pose: bytes 0-11 `x, y, z` (m), bytes 12-23 `rx, ry, rz` (rad) |
| 9 | read | — | 144 B | Jacobian `J`, 36 words row-major `J00..J55` (rows 0-2 linear, rows 3-5 angular) |
| 10 | write | 96 B: `x_des0..5`, `K0..5`, `D0..5`, `xdot0..5` | 18 B ACK | Compute `F = K (x_des - x) - D xdot`, `tau = J^T F`, then `iq* = sat(tau * KT_INV >> 16, +/-2000)` |
| 11 | read | — | 24 B | Joint torques `tau0..tau5` (Q16.16 N*m) |
| 12 | read | — | 24 B | Current references `iq0..iq5` (6 x int32, the 16-bit iq sign-extended) |
| 13 | read | — | 36 B | PWM duty snapshot: 6 motors x 3 phases (18 x int16, 10-bit duty centred on 512) |
| 14 | write | 48 B: `phi0..5` + ADC currents, 6 motors x 3 phases (24 x int16) | 18 B ACK | Feed encoder angles and phase currents to the FOC array |
| 15 | write | 73 B: `enable` (1 B) + `q_err0..5` + `Ks_nom0..5` + `Ds_nom0..5` | 18 B ACK | Compute `tau_ext = Kt * iq_actual - tau_cmd` on chip, run the RLS environment-stiffness estimator and the inverse adaptation law, latch `Ks_out` / `Ds_out` / `K_env`. With `enable = 1` the impedance stage uses the adapted gains instead of the values from opcode 10 |
| 16 | write | 48 B: `K_env_prior0..5` + `P_prior0..5` | 18 B ACK | Inject RLS priors (initial stiffness estimate and covariance) |
| 17 | read | — | 72 B | `K_env_est0..5`, `Ks_out0..5`, `Ds_out0..5` |
| 18 | read | — | 24 B | Cycle statistics: `last`, `min`, `max`, `runs`, magic `0xE2E2E2E2`, `0`. Pure observer: it reads existing signals and gates nothing |
| 3 | write | — | 18 B ACK | Start / liveness ping |
| 1, 7, other | — | — | none | Payload consumed, no reply |

A packet flagged with a stream error is discarded without a reply. The latched
pose, Jacobian, torques and adaptive state persist between packets, which is why
a write opcode and its matching read opcode can be separate UDP frames.

The opcode-18 measurement window is defined so that host and network gaps do not
inflate it: it opens on the last byte of an opcode-2 frame, pauses when FK
finishes, resumes on the last byte of the following opcode-10 frame, and closes
when the current reference is written. That is the 549-cycle figure above.

**Reply lengths are fixed on purpose.** The `udp_tx` stage transmits exactly
`cfg_udp_len` bytes and only completes a frame when it observes `valid && last`
while already in its write state, so a reply must hold `valid` high for its whole
length and raise `last` only on the final byte. Single-cycle replies are
swallowed and the transmitter stalls. This is why even a bare acknowledgement is
18 bytes. `sim/tb_udp_fk_cmd.v` contains a receiver model that reproduces this
contract faithfully, including the stall, so the behaviour stays locked down;
update that model before changing the transmit timing.

## 5. Network configuration

The network parameters are hard-coded in `rtl/top/impedance_control_top.v` (and
identically in `rtl/top/top_udp_add.v`). They are **not** runtime-configurable;
changing them means rebuilding the bitstream.

| Parameter | Value |
|---|---|
| FPGA IP : port | `172.28.109.173 : 8081` |
| Host IP : port | `172.28.109.174 : 8080` |
| FPGA source MAC | `11:22:33:44:55:66` |
| Topology | direct wired link, same /24, layer-2 only (no router) |

The FPGA always replies to the fixed destination port 8080 rather than echoing
the request source port, so exactly one program may bind port 8080 at a time.
Set the host NIC to static `172.28.109.174/24` and allow UDP 8080 through the
firewall.

## 6. Running the board test

1. **Flash** `board_test/bitstream/top_udp_add.bit` (Vivado Hardware Manager, or
   any JTAG loader). Use a fresh load: Part 5 assumes the adaptive state sits at
   its reset priors, and Parts 1-4 never send opcode 15 or 16.
2. **Build the host**: open `board_test/host/pcie_cpp.sln` in Visual Studio
   (toolset v143, x64) and build. It is a plain Winsock UDP program; `yolo.cpp` /
   `yolo.h` hold the transport and the per-opcode helpers, `main.cpp` the five
   test parts.
3. **Configure the network** as in section 5 and run the executable. The five
   parts execute back to back on one socket.

| Part | What it exercises | Line that decides pass or fail |
|---|---|---|
| 1 | FK precision on board: 32 joint vectors, opcode 2 then opcode 8, board pose against a double-precision reference | `VERDICT: ROUND-TO-NEAREST RTL (new bit) -- FK precision upgraded, R1.2 margin restored.` |
| 2 | Full-chain liveness: opcodes 10, 11, 12, 14, 13 in sequence, printing pose, torque, current reference and PWM duties | `==== full chain OK ====` |
| 3 | Determinism: 10^5 on-board forward chains, on-chip counter read via opcode 18 before and after | `VERDICT: DETERMINISM CONFIRMED -- zero spread over 1e5 on-board executions.` |
| 4 | Force chain bit-exactness: 32 cases, board torques and currents against RTL-generated expectations | `VERDICT: FORCE-CHAIN BIT-EXACT ON BOARD -- Table VI carrier upgraded.` |
| 5 | Cycle invariance to adaptation: 2 x 20000 chains with `adaptive_en` = 0 and 1, both 549 cycles, adapted torques verified different from the non-adapted ones | `VERDICT: R2.1 ON-BOARD -- forward-chain cycles invariant to adaptive_en (549/549), adaptation active.` |

The expectations Parts 4 and 5 compare against are compiled into `main.cpp` as
integer tables; they were emitted by `sim/tb_force_board.v` and
`sim/tb_r21_board.v` running the real `udp_fk_cmd` RTL against real UDP byte
streams (`sim/force_board_expected.txt`, `sim/r21_board_expected.txt`).

The console output captured on 2026-08-10 is archived in
`board_test/data/board_run_log_20260810.txt`.

## 7. Tools and one-command reproductions

The xsim pipelines need `xvlog` / `xelab` / `xsim` on `PATH` (same Vivado 2019.2
injection as section 3) plus Python 3. **Run them from the repository root**, for
example `scripts\run_fk_ab.bat`; they resolve their sources relative to the
repository root, not to `scripts\`.

| Command | What it does | Expected sentinels |
|---|---|---|
| `scripts\run_fk_ab.bat` | FK accuracy sweep: generate 1004 joint vectors, run them through the RTL FK engine in xsim, compare against a double-precision reference | `FK_AB_VECTORS_OK`, `FK_AB_DUMP_OK 1004`, `FK_AB_CHECK_OK` |
| `scripts\run_sim.bat` | Module regressions: `udp_fk_cmd` (including the faithful transmit-contract model), Cartesian impedance, FOC | `ALL TESTS PASSED` (three times) |
| `scripts\run_sim_v3.bat` | Adaptive closed loop: RLS estimation, inverse gain law, dead zone, priors | `ALL TESTS PASSED` |
| `scripts\run_sim_r2_1_fwd.bat` | Cycle invariance on the real `udp_fk_cmd` RTL driven by real UDP payloads (opcodes 16, 15, 2, 10, 11, 12), `adaptive_en` 0 versus 1 | `R2_1_FWD_CYCLES_EQ_OK`, `ALL TESTS PASSED` |
| `scripts\run_sim_r2_1.bat` | Same claim, module-level cross-check | `R2_1_FORWARD_CYCLES_EQ_OK`, `ALL TESTS PASSED` |
| `scripts\run_sim_r1_1.bat` | Forward-chain cycle measurement per stage (FK/Jacobian 422, impedance 119, torque-to-current 4) | `R1_1_CYCLES_OK`, `ALL TESTS PASSED` |
| `scripts\run_sim_r1_3.bat` | Force-chain fixed-point accuracy over 1008 cases against a double-precision reference and a bit-exact fixed-point mirror, then a FOC current-tracking sweep. Set `PYTHONIOENCODING=utf-8` first on non-UTF-8 consoles | `GEN_OK`, `R1_3_DUMP_OK`, `R1_3_CHECK_OK`, `R1_3_FOC_TRACK_OK` |

The FK sweep of record, reproduced from the RTL in this repository:

```
pos err euclid (mm): max=0.080889 mean=0.027369 sd=0.012504 p95=0.050842 p99=0.064380
ang err (deg):       max=0.151704 mean=0.004967
```

`sim/tb_force_board.v` and `sim/tb_r21_board.v` have no wrapper script; they are
run directly when the host expectation tables need regenerating, for example:

```bat
xvlog rtl\impedance\part1_config.v rtl\impedance\cordic_sincos.v rtl\impedance\cordic_atan2.v ^
      rtl\impedance\fk_engine.v rtl\impedance\analytical_jacobian_serial.v ^
      rtl\impedance\impedance_torque.v rtl\impedance\wrench_calc.v rtl\impedance\cart_impedance.v ^
      rtl\impedance\adaptive_impedance_v2.v rtl\foc\torque_to_iq.v rtl\top\udp_fk_cmd.v ^
      sim\tb_force_board.v
xelab tb_force_board -s tb_fb -timescale 1ns/1ps
xsim tb_fb -runall
```

Each writes its expectation file into the current directory; move it into `sim/`
to replace the committed copy.

### Python tools

Python 3 only, no third-party packages. The source comments and some of the
diagnostic output of these tools are in Chinese, as are the comments in the RTL;
set `PYTHONIOENCODING=utf-8` if your console codepage is not UTF-8.

| File | Purpose |
|---|---|
| `tools/fk_ab_gen.py` | Generates the FK stimulus: 1000 uniform random configurations (seed 42, deterministic) plus 4 special ones (all zero, all pi/2, the +/-pi boundary, near a wrist singularity) into `sim/fk_ab_vectors.mem`. |
| `tools/fk_ab_check.py` | Double-precision FK reference; reads the stimulus and an RTL dump and reports per-axis, Euclidean and angular error statistics. Optional argument: path to the dump. |
| `tools/fk_golden.py` | Standalone golden FK model, same DH constants and Euler convention, for spot-checking single poses. |
| `tools/force_golden.py` | Force-chain reference: `gen` emits the 1008-case stimulus (seed 20260809, deterministic), `check` compares an RTL dump against both a double-precision model and a bit-exact fixed-point mirror. |
| `tools/e6_rtt.py` | Host-to-FPGA UDP round-trip characterisation on a running board: read-pose round trip and the full host-in-the-loop cycle, reporting min / p50 / mean / p90 / p99 / p99.9 / max in microseconds and dumping raw samples as CSV. |
| `tools/traj_hw/gen_traj.py` | Regenerates the trajectory table `traj_xdes_delta.csv`: an xy circle of radius 111.8 mm for three turns plus a 100 mm z climb, quintic time scaling, constant attitude, 100 Hz for 60 s. Deterministic. Sentinel `GEN_TRAJ_OK`. |
| `tools/traj_hw/traj_analyze.py` | Analyses a trajectory log and produces plots plus RMS and maximum error statistics. Sentinel `TRAJ_ANALYZE_OK`. `--selftest` exercises the plotting path on a sample it generates itself, so it needs no robot data. |
| `tools/traj_hw/README_experiment.md` | Procedure for the hardware trajectory experiment: anchoring, the log format the analysis script expects, and the dry-run checklist. **This file is written in Chinese**; the rest of the documentation is English. |

## 8. Where the data comes from

No data in this repository was synthesized or hand-authored. Each artifact states
its carrier explicitly, simulation results are never presented as hardware
measurements, and where a value was not captured the file says so instead of
carrying a number.

| Artifact | Carrier | Kind |
|---|---|---|
| `board_test/data/fk_ab_out.txt` | xsim on `rtl/impedance/fk_engine.v`, 1004 cases | RTL simulation. Reproduces bit-identically from the sources here via `scripts\run_fk_ab.bat`. |
| `sim/fk_ab_vectors.mem` | `tools/fk_ab_gen.py`, seed 42 | Deterministic generated stimulus. |
| `sim/r1_3_vectors.mem`, `sim/r1_3_meta.txt` | `tools/force_golden.py gen`, seed 20260809 | Deterministic generated stimulus. |
| `sim/r1_3_rtl_out.mem` | xsim on the real force chain | RTL simulation dump. |
| `sim/force_board_expected.txt`, `sim/r21_board_expected.txt` | xsim on the real `udp_fk_cmd` driven by UDP byte streams | RTL simulation; used as the on-board expectation tables. |
| `board_test/bitstream/timing_summary.rpt`, `util_flat.rpt`, `util_hier.rpt` | Vivado 2019.2 on the routed design | Tool reports. |
| `board_test/data/board_run_log_20260810.txt` | Host Parts 1-5 on the flashed bitstream | **On-board measurement.** The four numeric lines of the Part 2 demo were not archived verbatim and are marked as such in the file. |
| `board_test/data/trajectory_measured_100Hz_60s.csv` | Host-in-the-loop run on the physical arm, 6001 rows at 100 Hz over 60 s | **Physical robot measurement.** |
| `tools/traj_hw/traj_xdes_delta.csv` | `tools/traj_hw/gen_traj.py` | Deterministic generated command table (the reference the measured log is compared against). |
| `benchmarks/cpu_longrun/data/` | `benchmarks/cpu_longrun/src/` on an i9-10900 / Windows 10 host, timed per call with `rdtscp` | **Host measurement.** Raw per-call samples (10^6-call runs) plus full histograms and per-event tails (long runs). |

Two categories of measurement are not archived here: the raw UDP round-trip
samples produced by `tools/e6_rtt.py`, and the contact-force sensor logs from the
on-robot contact trials. Both live on the experiment machine. See
`board_test/data/README.md` for the per-file index.

## 9. License and citation

MIT, see `LICENSE`. Copyright (c) 2026 the authors.

The paper describing this design is **under review**; citation details will be
added once it is published.
