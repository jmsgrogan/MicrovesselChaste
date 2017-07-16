---
layout: page-full-width 
title: Test Solving Pdes Tutorial
---
This tutorial is automatically generated from the file test/tutorials//TestSolvingPdesTutorial.hpp.
Note that the code is given in full at the bottom of the page.



# Solving PDEs in the Microvessel Project
This tutorial demonstrates methods for solving PDEs in the Microvessel Project. It is noted
that the way to set up PDEs differs from that of Cell Based Chaste, although the same solver
can be used behind the scenes.

The following is covered:
 * Solving a linear reaction-diffusion PDE with finite differences.
 * Solving a linear reaction-diffusion PDE with finite finite elements and discrete sinks and sources.
 * Solving a non-linear reaction-diffusion PDE with finite differences.
 * Solving a linear reaction-diffusion PDE using Green's functions.
 * Interacting with regular grids and finite element meshes.
 
# The Test
Start by introducing the necessary header files. The first contain functionality for setting up unit tests,
smart pointer tools and output management.

```cpp
#include <cxxtest/TestSuite.h>
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "SmartPointers.hpp"
#include "OutputFileHandler.hpp"
#include "FileFinder.hpp"
```

Dimensional analysis.

```cpp
#include "Vertex.hpp"
#include "UnitCollection.hpp"
#include "Owen11Parameters.hpp"
#include "GenericParameters.hpp"
#include "ParameterCollection.hpp"
#include "BaseUnits.hpp"
```

Geometry tools.

```cpp
#include "MappableGridGenerator.hpp"
#include "Part.hpp"
```

Vessel networks.

```cpp
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
```

Grids and PDEs.

```cpp
#include "DiscreteContinuumMesh.hpp"
#include "VtkMeshWriter.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"
#include "DiscreteSource.hpp"
#include "VesselBasedDiscreteSource.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde.hpp"
```

This should appear last.

```cpp
#include "PetscSetupAndFinalize.hpp"
class TestSolvingPdesLiteratePaper : public AbstractCellBasedWithTimingsTestSuite
{
public:
```

## Test 1 - Linear Reaction Diffusion With Finite Differences =
In the first example we will solve a steady-state linear reaction diffusion
PDE with finite differences.

```cpp
    void TestLinearReactionDiffusionPdeWithFiniteDifferences() throw(Exception)
    {
        auto p_handler =
                std::make_shared<OutputFileHandler>("TestSolvingPdesLiteratePaper/TestLinearReactionDiffusionPdeWithFiniteDifferences");
```

We will work in microns

```cpp
        QLength reference_length(1_um);
        BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
```

Set up a simulation domain, which will be a cuboid.

```cpp
        QLength domain_width(100_um);
        QLength domain_height(100_um);
        QLength domain_depth(20_um);
        auto p_domain = Part<3>::Create();
        p_domain->AddCuboid(domain_width, domain_height, domain_depth);
```

Make a regular grid on the domain

```cpp
        auto p_grid = RegularGrid<3>::Create();
        p_grid->GenerateFromPart(p_domain, 10.0*reference_length);
```

Set up a PDE, we will model oxygen diffusion.

```cpp
        auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<3>::Create();
        QDiffusivity oxygen_diffusivity(1.e-6*unit::metre_squared_per_second);
        p_oxygen_pde->SetIsotropicDiffusionConstant(oxygen_diffusivity);
```

Add continuum sink term for cells

```cpp
        QRate oxygen_consumption_rate(1.e-6*unit::per_second);
        p_oxygen_pde->SetContinuumLinearInUTerm(-oxygen_consumption_rate);
```

Add a Dirichlet boundary condition on the left face of the domain.

```cpp
        p_domain->AddAttributeToPolygonIfFound(Vertex<3>(0.0_um, domain_height/2.0, domain_depth/2.0), "boundary_1", 1.0);

        auto p_left_face_boundary = DiscreteContinuumBoundaryCondition<3>::Create();
        p_left_face_boundary->SetType(BoundaryConditionType::POLYGON);
        p_left_face_boundary->SetDomain(p_domain);
        p_left_face_boundary->SetValue(10e-3_M);
        p_left_face_boundary->SetLabel("boundary_1");
```

Set up the PDE solvers for the oxygen problem

```cpp
        auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<3>::Create();
        p_oxygen_solver->SetPde(p_oxygen_pde);
        p_oxygen_solver->SetGrid(p_grid);
        p_oxygen_solver->AddBoundaryCondition(p_left_face_boundary);
        p_oxygen_solver->SetLabel("oxygen");
        p_oxygen_solver->SetFileHandler(p_handler);
        p_oxygen_solver->SetFileName("fd_solution.vti");
        p_oxygen_solver->SetWriteSolution(true);
        p_oxygen_solver->Setup();
        p_oxygen_solver->Solve();
    }
};

```


# Code 
The full code is given below


## File name `TestSolvingPdesTutorial.hpp` 

```cpp
#include <cxxtest/TestSuite.h>
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "SmartPointers.hpp"
#include "OutputFileHandler.hpp"
#include "FileFinder.hpp"
#include "Vertex.hpp"
#include "UnitCollection.hpp"
#include "Owen11Parameters.hpp"
#include "GenericParameters.hpp"
#include "ParameterCollection.hpp"
#include "BaseUnits.hpp"
#include "MappableGridGenerator.hpp"
#include "Part.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "VtkMeshWriter.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"
#include "DiscreteSource.hpp"
#include "VesselBasedDiscreteSource.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde.hpp"
#include "PetscSetupAndFinalize.hpp"
class TestSolvingPdesLiteratePaper : public AbstractCellBasedWithTimingsTestSuite
{
public:
    void TestLinearReactionDiffusionPdeWithFiniteDifferences() throw(Exception)
    {
        auto p_handler =
                std::make_shared<OutputFileHandler>("TestSolvingPdesLiteratePaper/TestLinearReactionDiffusionPdeWithFiniteDifferences");
        QLength reference_length(1_um);
        BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
        QLength domain_width(100_um);
        QLength domain_height(100_um);
        QLength domain_depth(20_um);
        auto p_domain = Part<3>::Create();
        p_domain->AddCuboid(domain_width, domain_height, domain_depth);
        auto p_grid = RegularGrid<3>::Create();
        p_grid->GenerateFromPart(p_domain, 10.0*reference_length);
        auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<3>::Create();
        QDiffusivity oxygen_diffusivity(1.e-6*unit::metre_squared_per_second);
        p_oxygen_pde->SetIsotropicDiffusionConstant(oxygen_diffusivity);
        QRate oxygen_consumption_rate(1.e-6*unit::per_second);
        p_oxygen_pde->SetContinuumLinearInUTerm(-oxygen_consumption_rate);
        p_domain->AddAttributeToPolygonIfFound(Vertex<3>(0.0_um, domain_height/2.0, domain_depth/2.0), "boundary_1", 1.0);

        auto p_left_face_boundary = DiscreteContinuumBoundaryCondition<3>::Create();
        p_left_face_boundary->SetType(BoundaryConditionType::POLYGON);
        p_left_face_boundary->SetDomain(p_domain);
        p_left_face_boundary->SetValue(10e-3_M);
        p_left_face_boundary->SetLabel("boundary_1");
        auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<3>::Create();
        p_oxygen_solver->SetPde(p_oxygen_pde);
        p_oxygen_solver->SetGrid(p_grid);
        p_oxygen_solver->AddBoundaryCondition(p_left_face_boundary);
        p_oxygen_solver->SetLabel("oxygen");
        p_oxygen_solver->SetFileHandler(p_handler);
        p_oxygen_solver->SetFileName("fd_solution.vti");
        p_oxygen_solver->SetWriteSolution(true);
        p_oxygen_solver->Setup();
        p_oxygen_solver->Solve();
    }
};

```

