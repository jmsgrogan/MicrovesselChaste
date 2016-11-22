
mkdir $PREFIX/build
cd $PREFIX/build

#CC=$PREFIX/bin/cc
#CXX=$PREFIX/bin/c++
export LIBRARY_PATH=$PREFIX/lib
export INCLUDE_PATH=$PREFIX/include

cmake .. \
    -DPYTHON_EXECUTABLE:FILEPATH=$PYTHON \
    -DCMAKE_LIBRARY_PATH=$PREFIX/lib \
    -DCMAKE_PREFIX_PATH=$PREFIX \
    -DHDF5_C_COMPILER_EXECUTABLE:FILEPATH=$PREFIX/bin/h5pcc \
    -DBUILD_SHARED_LIBS:BOOL=ON \
    -DBUILD_TESTING:BOOL=OFF \
    -DCMAKE_INSTALL_PREFIX=$PREFIX \
    -DBOOST_ROOT=$PREFIX \
    -DChaste_ENABLE_TESTING=ON \
    -DChaste_UPDATE_PROVENANCE=OFF \
    -DChaste_ENABLE_heart_TESTING=OFF \
    -DChaste_ENABLE_lung_TESTING=OFF \
    -DChaste_ENABLE_crypt_TESTING=OFF \
    -DChaste_ENABLE_global_TESTING=OFF \
    -DChaste_ENABLE_linalg_TESTING=OFF \
    -DChaste_ENABLE_io_TESTING=OFF \
    -DChaste_ENABLE_mesh_TESTING=OFF \
    -DChaste_ENABLE_ode_TESTING=OFF \
    -DChaste_ENABLE_pde_TESTING=OFF \
    -DChaste_ENABLE_cell_based_TESTING=OFF \
    -DChaste_ENABLE_continuum_mechanics_TESTING=OFF \
    -DChaste_ENABLE_project_PyChaste_TESTING=OFF \
    -DChaste_ENABLE_project_MicrovesselChaste_TESTING=OFF \
    -DChaste_ERROR_ON_WARNING=OFF \
    -DVTK_DIR=$PREFIX \
    -DXERCESC_LIBRARY=$LIBRARY_PATH/libxerces-c.so \
    -DXERCESC_INCLUDE=$INCLUDE_PATH \
    -DXSD_EXECUTABLE=$PREFIX/bin/xsd \
    $SRC_DIR

make chaste_project_PyChaste -j $CPU_COUNT
make project_PyChaste_Python -j $CPU_COUNT
make chaste_project_MicrovesselChaste -j $CPU_COUNT
make project_MicrovesselChaste_Python -j $CPU_COUNT
make install -j $CPU_COUNT
cp projects/PyChaste/libchaste_project_PyChaste.so $LIBRARY_PATH/chaste/
cd projects/PyChaste/python
python setup.py install --prefix=$PREFIX
cd $PREFIX/build
cp projects/MicrovesselChaste/libchaste_project_MicrovesselChaste.so $LIBRARY_PATH/chaste/
cd projects/MicrovesselChaste/python
python setup.py install --prefix=$PREFIX

