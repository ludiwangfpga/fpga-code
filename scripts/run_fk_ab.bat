@echo off
rem R1.2 official reproduction pipeline: fk_engine accuracy A/B measurement.
rem   gen vectors -> xsim dump -> compare against double-precision reference.
rem   Expect: FK_AB_VECTORS_OK / FK_AB_DUMP_OK / FK_AB_CHECK_OK
cd /d "%~dp0.."
python tools\fk_ab_gen.py
if errorlevel 1 ( echo GEN_FAILED & exit /b 1 )
if exist xsim.dir rmdir /s /q xsim.dir
call xvlog rtl\impedance\part1_config.v rtl\impedance\cordic_sincos.v rtl\impedance\cordic_atan2.v rtl\impedance\fk_engine.v sim\tb_fk_ab.v -log xvlog_fk_ab.log
if errorlevel 1 ( echo XVLOG_FAILED & exit /b 1 )
call xelab tb_fk_ab -s tb_fk_ab -timescale 1ns/1ps -log xelab_fk_ab.log
if errorlevel 1 ( echo XELAB_FAILED & exit /b 1 )
call xsim tb_fk_ab -runall -log xsim_fk_ab.log
if errorlevel 1 ( echo XSIM_FAILED & exit /b 1 )
python tools\fk_ab_check.py
if errorlevel 1 ( echo CHECK_FAILED & exit /b 1 )
