# MicrovesselChaste - Multi-Scale Agent-Based Modelling with Microvessels

This is an add-on for agent-based modelling of microvessels with [Chaste](http://www.cs.ox.ac.uk/chaste/). 

# Features 

* 2D and 3D on- and off-lattice agent based models of cells and microvessels
* Integration with [Chaste](http://www.cs.ox.ac.uk/chaste/)
* Blood flow with haematocrit partitioning, structural adaptation and regression
* Sprouting angiogenesis
* Nutrient transport PDEs solves using finite difference or finite element methods

See the [project wiki](https://github.com/jmsgrogan/MicrovesselChaste/wiki) for some example problems.

# Installation 

The project can be used directly as a typical C++ Chaste project. First, Chaste dependencies need to be built following the [Chaste Install Guide](https://chaste.cs.ox.ac.uk/trac/wiki/InstallGuides/InstallGuide). 

The project only supports the development version of Chaste. This can be obtained by doing:

```bash
git clone https://chaste.cs.ox.ac.uk/git/chaste.git $CHASTE_SOURCE_DIR
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
make project_Microvessel -j $NUM_AVAILABLE_CPUS
```

This will build the C++ library and all tests. To avoid building tests do:

```bash
make chaste_project_Microvessel -j $NUM_AVAILABLE_CPUS
```

as the final command. The [Chaste CMake build system guide](https://chaste.cs.ox.ac.uk/trac/wiki/ChasteGuides/CmakeBuildGuide) should be consulted for options related to generating optimized builds, running other types of test and installation as a system library.