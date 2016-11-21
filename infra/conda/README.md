# How to build the conda package

The `microvessel-chaste` conda package is on the channel`jmsgrogan` on the anaconda cloud. This directory contains build scripts for the package and all neccessary dependency packages. Assuming conda has been suitably set up, each package can be re-built by doing:

```bash
cd <package-name>
conda build .
```

If you are happy with the package it can be uploaded to the anaconda cloud channel by doing:

```bash
anaconda upload <path-to_package>
```

where `<path-to_package>` will be shown in the console after a successful build. Additional steps are needed for the `microvessel-chaste` package, detailed below.

## Preparing conda for the build

To set up conda for the build do:

```bash
wget https://repo.continuum.io/miniconda/Miniconda2-latest-Linux-x86_64.sh
chmod 777 Miniconda2-latest-Linux-x86_64.sh
./Miniconda2-latest-Linux-x86_64.sh
conda update conda
conda install conda-build
conda install anaconda-client
conda config --add channels conda-forge 
conda config --add channels jmsgrogan
```

## Preparing Chaste for the build

At the moment due to issues in the build setup it is neccesary to edit the Chaste source before building the package. 

* Remove `lung`: the lung component does not support VTK 7. It needs to be removed from the main CMakeLists.txt before building Chaste. 
* Remove `apps`: the contents of `apps/src` in the Chaste build directory must be removed. This is due to an issue with libcurl and ssl in the conda environment, which is being investigated.
* Set the path to source: while most packages are automatically grabbed from their version control systems the path to the Chaste source must be manually specified in `chaste/meta.yaml` relative to this directory.

## Working with the package

```bash
conda install microvessel-chaste
```

then in a python session:

```python
import chaste
import microvessel_chaste
```

## Troubleshooting

* Missing librt.so: this is a problem with the version of VTK in conda-forge, make sure VTK is being pulled from the jmsgrogan channel.
* Missing defines in libcurl.so: this happens when Chaste executables are built. Make sure no Chaste apps or tests are being built by modifying cmake variables and CMakeLists.txt appropriately.
* Missing liblapack.so when loading the package in Python after building: libatlas is needed on the system. Do `sudo apt-get install libatlas-base-dev` on Ubuntu, for example. 

