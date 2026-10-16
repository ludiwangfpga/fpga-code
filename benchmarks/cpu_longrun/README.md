# CPU per-call latency of the control law (general-purpose OS baseline)

Raw data, source code and analysis for the CPU baseline that the FPGA's fixed
549-cycle (4.39 µs) forward chain is compared against.

The same control law as the RTL (Q16.16, bit-level equivalent) runs single-threaded
in C++ on a desktop Windows host. Every call is timed individually with `rdtscp`
on the invariant TSC. Batch averaging is not used, because it would hide the tail.

## Host

- Intel Core i9-10900 (10 cores / 20 threads), invariant TSC calibrated at 2.808 GHz
- Windows 10. The REALTIME priority class was requested; the OS granted HIGH.
- MSVC 2022, `/O2 /std:c++17`, no `/arch:AVX`, no fast-math
- The ≈10 ns timer overhead is not subtracted.

## Runs

| Run | Files | Duration | Calls | Median | Max | >100 µs | >1 ms | >10 ms |
|---|---|---|---|---|---|---|---|---|
| Idle, short | `data/percall_quiet.bin` | 0.34 s | 10^6 | 0.329 µs | 33.46 µs | 0 | 0 | 0 |
| Loaded, short | `data/percall_loaded.bin` | 0.68 s | 10^6 | 0.669 µs | 17.85 µs | 0 | 0 | 0 |
| Idle, long | `data/longrun/quiet_2h_*` | 120 min | 2.04×10^10 | 0.33 µs | 1487.5 µs | 1091 | 7 | 0 |
| All cores loaded | `data/longrun/load_2h_*` | 107 min | 8.19×10^9 | 0.76 µs | 29.35 ms | 1075 | 55 | 14 |
| I/O loaded | `data/longrun/io_1h_*` | 60 min | 8.87×10^9 | 0.34 µs | 1473.4 µs | 2878 | 5 | 0 |

- **Idle long run.** The first 180 s overlapped with interactive use of the host. The paper excludes them and reports the remaining 117 min: 1.99×10^10 calls, max 1457 µs, 1061 calls >100 µs, 6 calls >1 ms. `analysis/r13_analysis.py` applies this exclusion (`QUIET_EXCL_S = 180`). The table above shows the unfiltered file contents.
- **All-cores-loaded run.** It stopped at 107 min because the host became unresponsive after its last checkpoint. All data up to that checkpoint are complete.
- **I/O-loaded run.** It runs disk, UDP and memory traffic, with no ALU spinner threads.

## File formats

- `percall_*.bin` — raw per-call latency, little-endian float64, in µs, one value per call (10^6 values).
- `*_summary.json` — call count, elapsed time, mean/SD, percentiles, maximum and its time stamp, and counts above 4.39 / 33.46 / 100 / 1000 / 10 000 µs. Written at every 60 s checkpoint; the file is the final checkpoint.
- `*_hist.csv` — `lo_us,hi_us,count`, a histogram of every call. The last bin is open-ended.
- `*_tail.csv` — `t_s,us`, every call above the tail threshold with its time stamp (exact values, not binned).
- `*.log` — console output of the run.

## Reproduce

```
cd src
build.bat           (from an "x64 Native Tools Command Prompt for VS 2022")
cpu_percall.exe           idle short run
cpu_percall.exe --load    loaded short run
run_longrun.bat           three long runs (about 5 h in total)
```

`cpu_longrun.cpp` is the first 488 lines of `cpu_percall.cpp` (control law and timer primitives, verbatim) followed by `cpu_longrun_main.inc`. `cpu_percall.cpp` writes its `.bin` output to the absolute path set in its `main()`; edit that path before running.

## Analysis

```
cd analysis
python r13_analysis.py > ../figures/analysis_report.json
```

The script needs numpy and matplotlib. It regenerates the CCDF and running-maximum figures in `figures/` and prints every number used in the paper's latency tables as JSON.
