@echo off
rem R1.1 拍数测量: FK+Jac(analytical_jacobian_serial) -> cart_imp -> torque_to_iq.
cd /d "%~dp0.."
if exist xsim.dir rmdir /s /q xsim.dir
call xvlog rtl\impedance\part1_config.v rtl\impedance\cordic_sincos.v rtl\impedance\cordic_atan2.v rtl\impedance\fk_engine.v rtl\impedance\analytical_jacobian_serial.v rtl\impedance\impedance_torque.v rtl\impedance\wrench_calc.v rtl\impedance\cart_impedance.v rtl\foc\torque_to_iq.v sim\tb_r1_1_cycles.v -log xvlog_r1_1.log
if errorlevel 1 ( echo XVLOG_FAILED & exit /b 1 )
call xelab tb_r1_1_cycles -s tb_r1_1 -timescale 1ns/1ps -log xelab_r1_1.log
if errorlevel 1 ( echo XELAB_FAILED & exit /b 1 )
call xsim tb_r1_1 -runall -log xsim_r1_1.log
if errorlevel 1 ( echo XSIM_FAILED & exit /b 1 )
