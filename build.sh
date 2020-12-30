mkdir build
cd build

    #   -DCMAKE_SYSROOT=$CONDA_PREFIX/$HOST/sysroot \
    #   -DTIFF_LIBRARY=$CONDA_PREFIX/lib/libtiff.so \
    #    \
    
cmake -GNinja \
      -DH2O_BUILD_DOG=OFF \
      -DH2O_BUILD_SERIAL=OFF \
      -DCMAKE_PREFIX_PATH=$CONDA_PREFIX \
      -DCMAKE_INSTALL_PREFIX=./install \
      -DCMAKE_SYSROOT=$CONDA_PREFIX/$HOST/sysroot \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DOpenGL_GL_PREFERENCE=LEGACY \
      -DOPENGL_gl_LIBRARY=$CONDA_PREFIX/$HOST/sysroot/usr/lib64/libGL.so \
      -DLaszip_LIBRARY_RELEASE=$CONDA_PREFIX/lib/liblaszip.so \
      -DLaszip_LIBRARY_DEBUG=$CONDA_PREFIX/lib/liblaszip.so \
      -DOpenMP_gomp_LIBRARY=$CONDA_PREFIX/lib/libgomp.so \
      -DTIFF_LIBRARY_RELEASE=$CONDA_PREFIX/lib/libtiff.so \
      -DTIFF_LIBRARY_DEBUG=$CONDA_PREFIX/lib/libtiff.so \
      -DOpenMP_pthread_LIBRARY=$CONDA_PREFIX/$HOST/sysroot/usr/lib/libpthread.so \
      ../

cd ..
