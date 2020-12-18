mkdir build_vs
cd build_vs

cmake -G "Visual Studio 16 2019" -A x64 ^
    -DCMAKE_BUILD_TYPE=RelWithDebInfo ^
    -DCMAKE_TOOLCHAIN_FILE=%CONDA_PREFIX%\h2o.cmake ^
    -DH2O_BUILD_SHARED=ON ^
    -DH2O_BUILD_TEST=ON ^
    -DH2O_BUILD_DOG=OFF ^
    -DH2O_BUILD_SERIAL=ON ^
    ../

cd ..
