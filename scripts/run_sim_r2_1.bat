@echo off
rem R2.1 拍数对照: adaptive_en=0/1 前向链拍数严格相等(论文 R2.1 契约).
cd /d "%~dp0.."
if exist xsim.dir rmdir /s /q xsim.dir
call xvlog rtl/impedance/adaptive_impedance_v2.v rtl/impedance/wrench_calc.v rtl/impedance/impedance_torque.v rtl/impedance/cart_impedance.v rtl/foc/torque_to_iq.v sim/adaptive_loop_v3.v sim/tb_r2_1_cycles.v -log xvlog_r2_1.log
if errorlevel 1 ( echo XVLOG_FAILED & exit /b 1 )
call xelab tb_r2_1_cycles -s tb_r2_1 -timescale 1ns/1ps -log xelab_r2_1.log
if errorlevel 1 ( echo XELAB_FAILED & exit /b 1 )
call xsim tb_r2_1 -runall -log xsim_r2_1.log
if errorlevel 1 ( echo XSIM_FAILED & exit /b 1 )
