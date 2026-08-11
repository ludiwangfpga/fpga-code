# Test data index

Everything in this folder was produced by running the design — either in RTL
simulation or on the board. Each row states the carrier explicitly; nothing here
is synthetic or hand-authored data.

| File | Carrier | What it is |
|---|---|---|
| `fk_ab_out.txt` | **RTL simulation** (xsim, `sim/tb_fk_ab.v` on `rtl/impedance/fk_engine.v`) | Raw pose dump for the 1004-case forward-kinematics accuracy sweep, 1004 lines x 6 Q16.16 integers (x, y, z, rx, ry, rz). Re-created bit-identically by `scripts/run_fk_ab.bat`, which writes it to the repository root; this copy is the archived dump of record. |
| `board_run_log_20260810.txt` | **On-board measurement** (host Parts 1-5 against `../bitstream/top_udp_add.bit`) | Console summary and VERDICT lines captured on 2026-08-10. The four numeric lines of the Part 2 full-chain demo were not archived verbatim and are marked `to be pasted by the author`. |
| `trajectory_measured_100Hz_60s.csv` | **Physical robot measurement** (host-in-the-loop run on the real arm) | Measured trajectory log, 6001 rows at 100 Hz over 60 s. Columns: `t_ms` plus six Q16.16 integers, the measured pose delta relative to the anchor pose `p0` (dx, dy, dz in metres; drx, dry, drz in radians) — the same schema and time base as the commanded table `tools/traj_hw/traj_xdes_delta.csv`, so commanded and achieved can be compared row by row. Original filename on the experiment machine: `real.csv`. |

## Expectation files live in `sim/`, not here

The board-side expectations that host Parts 4 and 5 compare against are RTL
simulation products and are kept next to the testbenches that emit them:

| Expectation file | Emitted by | Consumed by |
|---|---|---|
| `sim/force_board_expected.txt` | `sim/tb_force_board.v` (real `udp_fk_cmd` RTL driven by UDP byte streams) | host Part 4 (compiled into `board_test/host/main.cpp` as integer tables) |
| `sim/r21_board_expected.txt` | `sim/tb_r21_board.v` (same, adaptive path) | host Part 5 |

Both testbenches write their output file into the current working directory, so
run them from the repository root and move the result into `sim/` if you
regenerate them.

## Not included

- Raw UDP round-trip samples (`e6_rtt_op8.csv`, `e6_cycle.csv`): produced on the
  experiment PC by `tools/e6_rtt.py`; not archived in this repository.
- Contact-force sensor logs: recorded on the experiment PC with a physical force
  sensor; not archived in this repository.
