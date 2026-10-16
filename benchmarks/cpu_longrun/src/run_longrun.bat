@echo off
rem Reproduce the three long runs. Output goes to ..\data\longrun\TAG_*.
rem Checkpoints are written every 60 s, so the runs can be stopped at any time.
setlocal
cd /d %~dp0
if not exist ..\data\longrun mkdir ..\data\longrun
cpu_longrun.exe --mode quiet --minutes 120 --tag quiet_2h --out ..\data\longrun > ..\data\longrun\quiet_2h.log 2>&1
cpu_longrun.exe --mode load  --minutes 120 --tag load_2h  --out ..\data\longrun > ..\data\longrun\load_2h.log  2>&1
cpu_longrun.exe --mode io    --minutes 60  --tag io_1h    --out ..\data\longrun > ..\data\longrun\io_1h.log    2>&1
endlocal
