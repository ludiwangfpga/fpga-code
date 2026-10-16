@echo off
rem Build both benchmarks from a "x64 Native Tools Command Prompt for VS 2022".
rem Flags: /O2 /std:c++17, no /arch:AVX, no fast-math.
setlocal
cd /d %~dp0
cl /nologo /O2 /std:c++17 /EHsc /utf-8 /Fe:cpu_percall.exe cpu_percall.cpp || exit /b 1
rem cpu_longrun.cpp = first 488 lines of cpu_percall.cpp (control law + timer primitives, verbatim) + cpu_longrun_main.inc
cl /nologo /O2 /std:c++17 /EHsc /utf-8 /Fe:cpu_longrun.exe cpu_longrun.cpp || exit /b 2
echo BUILD_OK
endlocal
