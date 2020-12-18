REM installer DOG SER VER NUM
rmdir installer_build /s/q
mkdir installer_build
cd installer_build

cmake -GNinja ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DCMAKE_PREFIX_PATH=%CONDA_PREFIX%\Library ^
    -DQT_QMAKE_EXECUTABLE=%CONDA_PREFIX%\Library\bin\qmake.exe ^
    -DCMAKE_TOOLCHAIN_FILE=%CONDA_PREFIX%\h2o.cmake ^
    -DH2O_BUILD_SHARED=ON ^
    -DH2O_BUILD_TEST=OFF ^
    -DH2O_BUILD_DOG=%1 ^
    -DH2O_BUILD_SERIAL=%2 ^
    -DH2O_VERSION_NAME=%3 ^
    ../

ninja

cd ..

robocopy ./data/resources ./installer_build/bin/resources /E
robocopy ./depends/dog/bin ./installer_build/bin /E

set dog=""
if "%1" == "ON" set dog="dog"
if "%2" == "ON" set dog="ser"

ISCC /dMyAppName=%3 /dMyAppVersion=%4 /dMyAppDog=%dog% installer.iss
