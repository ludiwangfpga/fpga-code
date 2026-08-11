@echo off
rem =====================================================================
rem R1.3 力链路全链 Q16.16 定点对拍（一键复现）
rem   1) python 生成 1008 例激励向量  -> sim\r1_3_vectors.mem
rem   2) xsim 跑定点 RTL 全链导出结果 -> sim\r1_3_rtl_out.mem
rem   3) python 双精度参考逐例对拍     -> 误差统计表
rem 期望 sentinel: GEN_OK / R1_3_DUMP_OK / R1_3_CHECK_OK
rem 需先注入 Vivado 2019.2 到 PATH:
rem   set PATH=C:\Xilinx\Vivado\2019.2\bin;%%PATH%%
rem =====================================================================
cd /d "%~dp0.."

echo [1/3] generating stimulus vectors ...
python tools\force_golden.py gen --n 1000 --seed 20260809
if errorlevel 1 ( echo GEN_FAILED & exit /b 1 )

echo [2/3] running xsim fixed-point RTL ...
if exist xsim.dir rmdir /s /q xsim.dir
call xvlog rtl\impedance\part1_config.v rtl\impedance\cordic_sincos.v rtl\impedance\cordic_atan2.v rtl\impedance\fk_engine.v rtl\impedance\analytical_jacobian_serial.v rtl\impedance\impedance_torque.v rtl\impedance\wrench_calc.v rtl\impedance\cart_impedance.v rtl\foc\torque_to_iq.v sim\tb_r1_3_force_precision.v -log xvlog_r1_3.log
if errorlevel 1 ( echo XVLOG_FAILED & exit /b 1 )
call xelab tb_r1_3_force_precision -s tb_r1_3 -timescale 1ns/1ps -log xelab_r1_3.log
if errorlevel 1 ( echo XELAB_FAILED & exit /b 1 )
call xsim tb_r1_3 -runall -log xsim_r1_3.log
if errorlevel 1 ( echo XSIM_FAILED & exit /b 1 )

echo [3/4] comparing against double-precision reference ...
python tools\force_golden.py check
if errorlevel 1 ( echo CHECK_FAILED & exit /b 1 )

echo [4/4] FOC current-loop i_q tracking sweep ...
call xvlog rtl\foc\sincos.v rtl\foc\cartesian2polar.v rtl\foc\clark_tr.v rtl\foc\park_tr.v rtl\foc\pi_controller.v rtl\foc\hold_detect.v rtl\foc\svpwm.v rtl\foc\foc_top.v rtl\foc\foc_channel.v sim\foc_motor_model.v sim\tb_r1_3_foc_track.v -log xvlog_r1_3_foc.log
if errorlevel 1 ( echo XVLOG_FOC_FAILED & exit /b 1 )
call xelab tb_r1_3_foc_track -s tb_r1_3_foc -timescale 1ns/1ps -log xelab_r1_3_foc.log
if errorlevel 1 ( echo XELAB_FOC_FAILED & exit /b 1 )
call xsim tb_r1_3_foc -runall -log xsim_r1_3_foc.log
if errorlevel 1 ( echo XSIM_FOC_FAILED & exit /b 1 )
