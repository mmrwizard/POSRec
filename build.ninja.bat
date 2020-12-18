mkdir build
cd build

cmake -GNinja ^
    -DCMAKE_BUILD_TYPE=RelWithDebInfo ^
    -DCMAKE_TOOLCHAIN_FILE=%CONDA_PREFIX%\h2o.cmake ^
    -DH2O_BUILD_SHARED=ON ^
    -DH2O_BUILD_TEST=ON ^
    -DH2O_BUILD_DOG=OFF ^
    -DH2O_BUILD_SERIAL=OFF ^
    -DH2O_VERSION_NAME=kinsar ^
    ../

cd ..
