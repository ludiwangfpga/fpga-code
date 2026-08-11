@echo off
cd /d "%~dp0.."
if exist xsim.dir rmdir /s /q xsim.dir
call xvlog rtl\impedance\part1_config.v rtl\impedance\cordic_sincos.v rtl\impedance\cordic_atan2.v rtl\impedance\fk_engine.v rtl\impedance\analytical_jacobian_serial.v rtl\impedance\impedance_torque.v rtl\impedance\wrench_calc.v rtl\impedance\cart_impedance.v rtl\impedance\adaptive_impedance_v2.v rtl\foc\torque_to_iq.v rtl\foc\sincos.v rtl\foc\cartesian2polar.v rtl\foc\clark_tr.v rtl\foc\park_tr.v rtl\foc\pi_controller.v rtl\foc\hold_detect.v rtl\foc\svpwm.v rtl\foc\foc_top.v rtl\foc\foc_channel.v rtl\top\udp_fk_cmd.v sim\foc_motor_model.v sim\tb_impedance.v sim\tb_udp_fk_cmd.v sim\tb_foc.v -log xvlog_sim.log
if errorlevel 1 ( echo XVLOG_FAILED & exit /b 1 )
call xelab tb_impedance -s tb_imp -timescale 1ns/1ps -log xelab_imp.log
if errorlevel 1 ( echo XELAB_IMP_FAILED & exit /b 1 )
call xsim tb_imp -runall -log xsim_imp.log
if errorlevel 1 ( echo XSIM_IMP_FAILED & exit /b 1 )
call xelab tb_udp_fk_cmd -s tb_sim -timescale 1ns/1ps -log xelab_sim.log
if errorlevel 1 ( echo XELAB_FAILED & exit /b 1 )
call xsim tb_sim -runall -log xsim_sim.log
if errorlevel 1 ( echo XSIM_FAILED & exit /b 1 )
call xelab tb_foc -s tb_foc -timescale 1ns/1ps -log xelab_foc.log
if errorlevel 1 ( echo XELAB_FOC_FAILED & exit /b 1 )
call xsim tb_foc -runall -log xsim_foc.log
if errorlevel 1 ( echo XSIM_FOC_FAILED & exit /b 1 )
