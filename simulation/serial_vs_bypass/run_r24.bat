@echo off
rem ============================================================================
rem run_r24.bat -- serial-vs-bypass RTL simulation (standalone xsim flow,
rem   Vivado 2019.2, no project, no launch_simulation).  Can be started from
rem   any directory; it changes into its own folder and compiles the deployed
rem   RTL from ..\..\rtl (the sources of board_test\bitstream\top_udp_add.bit).
rem
rem   Requires xvlog / xelab / xsim on PATH, e.g.
rem       set PATH=VIVADO_DIR\bin;%PATH%
rem   where VIVADO_DIR is the Vivado 2019.2 installation folder.
rem
rem   Usage: run_r24.bat        (steps a, b, c)
rem          run_r24.bat b      (single step: a | b | c)
rem   Writes data\adapt_latency_samples.csv (a), data\chain_samples.csv (b),
rem          data\udp_samples.csv (c); then run: python r24_analyze.py
rem   Expect sentinels: R24_A_OK  R24_B_OK  R24_C_OK  R24_ALL_DONE
rem   (judge success by the sentinels in xsim_*.log, not by exit codes)
rem ============================================================================
cd /d "%~dp0"
set R=..\..\rtl
if not exist data mkdir data
if exist xsim.dir rmdir /s /q xsim.dir
if "%1"=="" goto step_a
goto step_%1

:step_a
rem ---- A: adaptation update latency distribution (deployed adaptive_impedance_v2) ----
call xvlog %R%\impedance\adaptive_impedance_v2.v tb_r24_adapt_latency.v -log xvlog_a.log
if errorlevel 1 ( echo XVLOG_A_FAILED & exit /b 1 )
call xelab tb_r24_adapt_latency -s sim_a -timescale 1ns/1ps -log xelab_a.log
if errorlevel 1 ( echo XELAB_A_FAILED & exit /b 1 )
call xsim sim_a -runall -log xsim_a.log
if errorlevel 1 ( echo XSIM_A_FAILED & exit /b 1 )
if not "%1"=="" goto done

:step_b
rem ---- B: wrapper-level serial vs bypass (existing deployed modules composed) ----
call xvlog %R%\impedance\part1_config.v %R%\impedance\cordic_sincos.v %R%\impedance\cordic_atan2.v %R%\impedance\fk_engine.v %R%\impedance\analytical_jacobian_serial.v %R%\impedance\impedance_torque.v %R%\impedance\wrench_calc.v %R%\impedance\cart_impedance.v %R%\impedance\adaptive_impedance_v2.v %R%\foc\torque_to_iq.v r24_chain_wrap.v tb_r24_chain.v -log xvlog_b.log
if errorlevel 1 ( echo XVLOG_B_FAILED & exit /b 1 )
call xelab tb_r24_chain -s sim_b -timescale 1ns/1ps -log xelab_b.log
if errorlevel 1 ( echo XELAB_B_FAILED & exit /b 1 )
call xsim sim_b -runall -log xsim_b.log
if errorlevel 1 ( echo XSIM_B_FAILED & exit /b 1 )
if not "%1"=="" goto done

:step_c
rem ---- C: deployed command FSM (udp_fk_cmd) with the op18 on-chip cycle counter ----
call xvlog %R%\impedance\part1_config.v %R%\impedance\cordic_sincos.v %R%\impedance\cordic_atan2.v %R%\impedance\fk_engine.v %R%\impedance\analytical_jacobian_serial.v %R%\impedance\impedance_torque.v %R%\impedance\wrench_calc.v %R%\impedance\cart_impedance.v %R%\impedance\adaptive_impedance_v2.v %R%\foc\torque_to_iq.v %R%\top\udp_fk_cmd.v tb_r24_udp_serial_vs_bypass.v -log xvlog_c.log
if errorlevel 1 ( echo XVLOG_C_FAILED & exit /b 1 )
call xelab tb_r24_udp_serial_vs_bypass -s sim_c -timescale 1ns/1ps -log xelab_c.log
if errorlevel 1 ( echo XELAB_C_FAILED & exit /b 1 )
call xsim sim_c -runall -log xsim_c.log
if errorlevel 1 ( echo XSIM_C_FAILED & exit /b 1 )
if not "%1"=="" goto done

:done
echo R24_ALL_DONE
