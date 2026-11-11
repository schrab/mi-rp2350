@echo off
rem Build helper for rack-uimcu on this machine (Windows + Git Bash host).
rem
rem WHY THIS EXISTS: idf.py's __main__ silently skips main() whenever the
rem MSYSTEM env var is present (the "MSys/Mingw is no longer supported"
rem warning) ??? and Git Bash injects MSYSTEM into EVERY child process, so
rem idf.py invoked from a Git Bash shell prints the warning and exits 0
rem without doing anything. Running the build through cmd.exe (which never
rem sets MSYSTEM) is the reliable path.
rem
rem Usage (from any shell):
rem   cmd /c tools\build_uimcu.bat            (default: build)
rem   cmd /c tools\build_uimcu.bat flash COM22
rem   cmd /c tools\build_uimcu.bat build
setlocal
rem Git Bash injects MSYSTEM into child processes (cmd inherits it) ??? the
rem idf.py guard keys on that var, so clear it here first.
set MSYSTEM=
set MSYSTEM_CARCH=
set MSYSTEM_CHOST=
set MSYSTEM_PREFIX=
set IDF_PATH=C:\Users\schra\esp\v5.4.2
set IDF_PYTHON_ENV_PATH=C:\Users\schra\.espressif\python_env\idf5.4_py3.13_env
set TOOLS=C:\Users\schra\.espressif\tools
set PATH=%IDF_PYTHON_ENV_PATH%;%IDF_PYTHON_ENV_PATH%\Scripts;%IDF_PATH%\tools;%TOOLS%\cmake\3.30.2\bin;%TOOLS%\ninja\1.12.1;%TOOLS%\xtensa-esp-elf\esp-14.2.0_20241119\xtensa-esp-elf\bin;%TOOLS%\riscv32-esp-elf\esp-14.2.0_20241119\riscv32-esp-elf\bin;%PATH%
set IDF_TARGET=esp32c3
cd /d %~dp0..\rack-uimcu
if "%~1"=="" (
    "%IDF_PYTHON_ENV_PATH%\Scripts\python.exe" "%IDF_PATH%\tools\idf.py" build
) else (
    "%IDF_PYTHON_ENV_PATH%\Scripts\python.exe" "%IDF_PATH%\tools\idf.py" %*
)
endlocal
