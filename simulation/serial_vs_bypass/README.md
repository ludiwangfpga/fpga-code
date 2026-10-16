# Serial versus bypass adaptation — RTL simulation

**Kind: RTL simulation (Vivado 2019.2 xsim).** Nothing in this folder was measured
on the board. The testbenches compile the deployed RTL of this repository
(`rtl/`, the sources of `board_test/bitstream/top_udp_add.bit`) unchanged; the
only new logic is a thin test wrapper, `r24_chain_wrap.v`, that composes those
existing modules in a serial topology.

This is the "Serial versus bypass" comparison in Section 5 of the paper.

## Question

In the deployed design the adaptation update (`adaptive_impedance_v2`, triggered
by opcode 15) runs beside the forward chain (FK + Jacobian, Cartesian impedance,
torque-to-current), not inside it. What would a serial design cost, i.e. one with
the same control law, the same Q16.16 format and the same input/output boundary,
in which the forward chain waits for the adaptation update before it applies the
new gains?

## Experiments

| Step | Testbench | Device under test | Stimulus |
|---|---|---|---|
| A | `tb_r24_adapt_latency.v` | `rtl/impedance/adaptive_impedance_v2.v` | 2000 random updates (seed 20261014): K_env 100–10000 N/m; 80 % active, 10 % inside the 1 mm dead zone, 10 % `adaptive_enable = 0`; a random RLS prior every 50 updates |
| B | `tb_r24_chain.v` + `r24_chain_wrap.v` | three wrapper instances in lock-step on identical random inputs: bypass without adaptation, bypass with concurrent adaptation (the deployed topology), and serial (`FK -> wait for adaptation -> latch Ks/Ds -> impedance -> torque-to-current`) | 1000 random beats; an adaptation update is requested on a random 48.4 % of them (p = 0.5) |
| C | `tb_r24_udp_serial_vs_bypass.v` | the real command FSM `rtl/top/udp_fk_cmd.v`, driven by UDP payloads (op15, op2, op10, op18), with the op18 on-chip cycle counter read back | 1000 random beats |

Cycle convention (the same as the on-chip op18 counter): the start cycle counts
as 1, and counting runs through the cycle in which `done` is seen. 1 cycle = 8 ns
at 125 MHz.

## Results (recomputed from the CSVs in `data/`)

| Quantity | min | median | max | SD |
|---|---|---|---|---|
| A: adaptation update, start to done | 172 | 172 | 172 | 0 |
| B: bypass, no adaptation (wrapper boundary) | 547 | 547 | 547 | 0 |
| B: bypass, concurrent adaptation (wrapper boundary) | 547 | 547 | 547 | 0 |
| B: serial, all 1000 beats (wrapper boundary) | 547 | 547 | 719 | 86.0 |
| B: serial, the 484 beats with an update | 719 | 719 | 719 | 0 |
| B: serial, the 516 beats without an update | 547 | 547 | 547 | 0 |
| C: bypass forward chain, on-chip op18 counter (`cyc_min = cyc_max = 549`, `runs = 1000`) | 549 | 549 | 549 | 0 |
| C: adaptation inside `udp_fk_cmd` (op15) | 172 | 172 | 172 | 0 |
| C: serial-equivalent forward chain = op18 count + adaptation | 721 | 721 | 721 | 0 |

In step B the pre-clamp torques τ (all 6 channels) and the current references of
the concurrent-bypass and serial instances are bit-identical in 1000 of 1000 beats.
They differ from the no-adaptation instance in 918 of 1000 beats, so the adaptation
does change the output. These figures come from the xsim console (`xsim_b.log`);
the CSV keeps only channel 0.

How to read the two boundaries:

- The wrapper in step B leaves out the 4-cycle command latch of `udp_fk_cmd`, so
  its bypass chain is 547 cycles, not 549. A serial chain that was actually run
  takes 719 cycles on beats with an update and 547 cycles on beats without one.
- In step C the deployed FSM is not modified. The 721-cycle figure there is the
  measured op18 count (549) plus the measured adaptation latency (172). It is a
  derived sum, not a run of a serial FSM.
- At both boundaries the serial design costs exactly 172 cycles (1.376 µs, 1.38 %
  of a 100 µs budget) per beat with an update. With updates on about half the
  beats, its standard deviation is about 86 cycles. The bypass design shows zero
  spread in every case.

Because the deployed divider runs a fixed number of iterations, the update
latency does not depend on the data. The only variation a serial design would add
is the 172-cycle difference between beats with and without an update. The
decoupling therefore buys a forward-chain cycle count that does not depend on
whether or when adaptation runs. It does not change the control output: the
torques are bit-identical.

## Files

| File | Content |
|---|---|
| `run_r24.bat` | Compiles and runs steps A, B and C with xvlog / xelab / xsim. It uses relative paths to `../../rtl` and writes the CSVs to `data/`. |
| `tb_r24_adapt_latency.v`, `tb_r24_chain.v`, `r24_chain_wrap.v`, `tb_r24_udp_serial_vs_bypass.v` | The testbenches and the serial/bypass wrapper |
| `r24_analyze.py` | Recomputes the statistics into `data/r24_summary.csv` and draws two figures into `figures/` (git-ignored) |
| `data/adapt_latency_samples.csv` | Step A, one row per update |
| `data/chain_samples.csv` | Step B, one row per beat |
| `data/udp_samples.csv` | Step C, one row per beat |
| `data/r24_summary.csv` | The statistics above |

## Rerun

Run these from a Windows command prompt. Vivado 2019.2 is not assumed to be on PATH.

```bat
set PATH=C:\Xilinx\Vivado\2019.2\bin;%PATH%
simulation\serial_vs_bypass\run_r24.bat
python simulation\serial_vs_bypass\r24_analyze.py
```

Expected sentinels in `xsim_a.log`, `xsim_b.log` and `xsim_c.log`:

```
R24_A_SUMMARY min=172 median=172 max=172 sd=0.000000 divmin=51 divmax=51
R24_A_OK
R24_B_SUMMARY bypA=547/547 bypB=547/547 ser=547/547/719 sdser=85.955957 ser1=719/719 ser0=547/547
R24_B_OK
R24_C_SUMMARY fwd=549/549 chipmin=549 chipmax=549 adapt=172/172 ser=721/721/721
R24_C_OK
```

Each step simulates in a few seconds. The random seeds are fixed, so a rerun
reproduces the committed CSVs byte for byte; this was checked on 2026-10-16 from
this folder. `r24_analyze.py` needs numpy and matplotlib.
