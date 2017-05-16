# MicrovesselChaste - Multi-Scale Agent-Based Modelling with Microvessels

This is an add-on for agent-based modelling of microvessels with [Chaste](http://www.cs.ox.ac.uk/chaste/). See the [project website](https://jmsgrogan.github.io/MicrovesselChaste/) for more details.

## Release Notes:

### 3.4.3

#### geometry
* Added labelling for edges and polygons. Labels can now be written to VTK files.
* Dropped voronoi generator based on tetgen. May replace with VTK version in future.

#### mesh
* Major refactor to present homogeneous interface to both regular grids and unstructured meshes. 
* Initial support for parallel solution storage on grids.
* Added a grid calculator for generating and storing maps of discrete entities to grid locations.
* Significant performance improvements in the generation of disrete entity maps.

#### pde
* Major refactor of finite element and finite difference solvers. Now includes parabolic PDE solvers.
* The density map now plays a fundamental role in relating discrete entity densitities to sink and source
strengths in PDEs.
* Easier application of boundary conditions through geometry labelling.

#### vessel
* Significant performance improvements in merging coincident nodes.

#### python
* Simplify automatic wrapper generation

#### infra
* Add travis CI
* Keep up with Chaste modifications

### 3.4.2

* First public release



## Install from Source (Linux Only)

The project can be used directly as a typical C++ Chaste project. First, Chaste dependencies need to be built following the [Chaste Install Guide](https://chaste.cs.ox.ac.uk/trac/wiki/InstallGuides/InstallGuide). 

The project only supports a specific development version of Chaste. It will eventually match the next Chaste release.

This can be obtained by doing:

```bash
git clone --branch paper/MicrovesselChaste  https://chaste.cs.ox.ac.uk/git/chaste.git $CHASTE_SOURCE_DIR
```

The project code itself can be obtained by doing: 

```bash
git clone https://github.com/jmsgrogan/MicrovesselChaste.git $MICROVESSEL_PROJECT_SOURCE_DIR
```

The Microvessel project code needs to be included in the main Chaste source. This can be done with a symbolic link:

```bash
cd $CHASTE_SOURCE_DIR/projects
ln -s $MICROVESSEL_PROJECT_SOURCE_DIR
```

The C++ libraries can be built using the [Chaste CMake build system](https://chaste.cs.ox.ac.uk/trac/wiki/ChasteGuides/CmakeBuildGuide). First, create a build directory outside the source tree and proceed as:

```bash
cd $CHASTE_BUILD_DIR
cmake $CHASTE_SOURCE_DIR
make project_MicrovesselChaste -j $NUM_AVAILABLE_CPUS
```

This will build the C++ library and all tests. To avoid building tests do:

```bash
make chaste_project_MicrovesselChaste -j $NUM_AVAILABLE_CPUS
```

as the final command. The [Chaste CMake build system guide](https://chaste.cs.ox.ac.uk/trac/wiki/ChasteGuides/CmakeBuildGuide) should be consulted for options related to generating optimized builds, running other types of test and installation as a system library.

To build the Python package it is necessary to first build PyChaste following the instructions [here](https://jmsgrogan.github.io/PyChaste/documentation/installation.html). The MicrovesselChaste package can then be built in a similar way.

```bash
cd $BUILD_DIR
make project_MicrovesselChaste_Python
``` 

The Python package `microvessel-chaste` will be in `$BUILD_DIR` under `projects/MicrovesselChaste/python`. You can either add `$BUILD_DIR/projects/MicrovesselChaste/python` to your PYTHONPATH or do:

```bash
cd $BUILD_DIR/projects/MicrovesselChaste/python
python setup.py install
``` 
