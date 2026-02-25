@echo off
setlocal

echo Starting OR_Learner project build...

if exist out (
    echo Cleaning CMake cache...
    rmdir /s /q out
) else (
    mkdir out
)

cmake -S . -B out/build/x64-Debug -G "Visual Studio 17 2022" -A x64
cmake --build out/build/x64-Debug --config Release

endlocal