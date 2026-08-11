@echo off
rem ============================================================================
rem R2.1 hard metric: forward-chain cycle counts must be IDENTICAL for
rem adaptive_en = 0 and adaptive_en = 1, over the same joint-angle inputs.
rem   DUT   : real RTL rtl\top\udp_fk_cmd.v
rem           (FK+Jac -> cart_impedance -> torque_to_iq), driven by real UDP
rem           payloads op16 / op15 / op2 / op10 / op11 / op12.
rem   Expect: R2_1_FWD_CYCLES_EQ_OK   and   ALL TESTS PASSED
rem   (Chinese notes: see header comment of sim\tb_r2_1_fwd_cycles.v)
rem ============================================================================
cd /d "%~dp0.."
if exist xsim.dir rmdir /s /q xsim.dir
call xvlog rtl\impedance\part1_config.v rtl\impedance\cordic_sincos.v rtl\impedance\cordic_atan2.v rtl\impedance\fk_engine.v rtl\impedance\analytical_jacobian_serial.v rtl\impedance\impedance_torque.v rtl\impedance\wrench_calc.v rtl\impedance\cart_impedance.v rtl\impedance\adaptive_impedance_v2.v rtl\foc\torque_to_iq.v rtl\top\udp_fk_cmd.v sim\tb_r2_1_fwd_cycles.v -log xvlog_r2_1_fwd.log
if errorlevel 1 ( echo XVLOG_FAILED & exit /b 1 )
call xelab tb_r2_1_fwd_cycles -s tb_r2_1_fwd -timescale 1ns/1ps -log xelab_r2_1_fwd.log
if errorlevel 1 ( echo XELAB_FAILED & exit /b 1 )
call xsim tb_r2_1_fwd -runall -log xsim_r2_1_fwd.log
if errorlevel 1 ( echo XSIM_FAILED & exit /b 1 )
