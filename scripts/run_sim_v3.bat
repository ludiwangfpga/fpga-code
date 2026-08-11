@echo off
rem V3 临时验证: adaptive_loop_v3 (自适应阻抗 M6.1+6.2+6.3 合一) 独立模块级仿真.
rem 复制 V2 工程进 V3 后, 源文件已归位到树内 (rtl/impedance/ rtl/foc/ sim/). 期望 ALL TESTS PASSED.
cd /d "%~dp0.."
if exist xsim.dir rmdir /s /q xsim.dir
call xvlog rtl/impedance/adaptive_impedance_v2.v rtl/impedance/wrench_calc.v rtl/impedance/impedance_torque.v rtl/impedance/cart_impedance.v rtl/foc/torque_to_iq.v sim/adaptive_loop_v3.v sim/tb_adaptive_loop_v3.v -log xvlog_v3.log
if errorlevel 1 ( echo XVLOG_FAILED & exit /b 1 )
call xelab tb_adaptive_loop_v3 -s tb_v3 -timescale 1ns/1ps -log xelab_v3.log
if errorlevel 1 ( echo XELAB_FAILED & exit /b 1 )
call xsim tb_v3 -runall -log xsim_v3.log
if errorlevel 1 ( echo XSIM_FAILED & exit /b 1 )
