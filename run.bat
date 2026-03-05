@echo off
setlocal
chcp 65001 >nul

echo Starting OR_Learner project build...

if exist out\build\x64 (
    echo Cleaning CMake cache...
    rmdir /s /q out\build\x64
)

if not exist out mkdir out

cmake -S . -B out/build/x64 -G "Visual Studio 17 2022" -A x64
@REM cmake --build out/build/x64 --config Debug
cmake --build out/build/x64 --config Release

endlocal