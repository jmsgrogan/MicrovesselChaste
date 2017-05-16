/*

Copyright (c) 2005-2017, University of Oxford.
All rights reserved.

University of Oxford means the Chancellor, Masters and Scholars of the
University of Oxford, having an administrative office at Wellington
Square, Oxford OX1 2JD, UK.

This file is part of Chaste.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of the University of Oxford nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef TESTDISCRETECONTINUUMBOUNDARYCONDITIONS_HPP_
#define TESTDISCRETECONTINUUMBOUNDARYCONDITIONS_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include "SmartPointers.hpp"
#include "Part.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"
#include "SimpleLinearEllipticFiniteElementSolver.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "OutputFileHandler.hpp"
#include "RegularGrid.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "VesselBasedDiscreteSource.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestDiscreteContinuumBoundaryCondition : public CxxTest::TestSuite
{

public:

    void TestSimpleLinearEllipticFiniteDifferenceSolver() throw(Exception)
    {
        // Set up the vessel network
        units::quantity<unit::length> vessel_length(100.0*unit::microns);
        units::quantity<unit::length> reference_length(1.0*unit::microns);
        VesselNetworkGenerator<3> generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length, DimensionalChastePoint<3>());

        // Set up the grid
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(vessel_length, vessel_length, vessel_length, DimensionalChastePoint<3>());
        DimensionalChastePoint<3> translation_vector(-vessel_length/(2.0*reference_length),
                                                     -vessel_length/(2.0*reference_length), 0.0, reference_length);
        p_domain->Translate(translation_vector);
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        units::quantity<unit::length> spacing(10.0*unit::microns);
        p_grid->GenerateFromPart(p_domain, spacing);

        // Choose the PDE
        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<3> > p_pde = DiscreteContinuumLinearEllipticPde<3>::Create();
        units::quantity<unit::diffusivity> diffusivity(0.0033 * unit::metre_squared_per_second);
        units::quantity<unit::concentration_flow_rate> consumption_rate(-2.e-7 * unit::mole_per_metre_cubed_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetContinuumConstantInUTerm(consumption_rate);

        // Set up the discrete source
        boost::shared_ptr<VesselBasedDiscreteSource<3> > p_vessel_source_lin = VesselBasedDiscreteSource<3>::Create();
        p_vessel_source_lin->SetLinearInUValue(-1.e3*unit::per_second);
        boost::shared_ptr<VesselBasedDiscreteSource<3> > p_vessel_source_const = VesselBasedDiscreteSource<3>::Create();
        p_vessel_source_const->SetConstantInUValue(40.e-7* unit::mole_per_metre_cubed_per_second);
        p_pde->AddDiscreteSource(p_vessel_source_lin);
        p_pde->AddDiscreteSource(p_vessel_source_const);

        boost::shared_ptr<DiscreteContinuumBoundaryCondition<3> > p_boundary = DiscreteContinuumBoundaryCondition<3>::Create();
        p_boundary->SetValue(1.0*unit::mole_per_metre_cubed);

        // Set up and run the solver
        SimpleLinearEllipticFiniteDifferenceSolver<3> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        solver.SetVesselNetwork(p_network);
        solver.AddBoundaryCondition(p_boundary);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestDiscreteContinuumBoundaryCondition/FiniteDifference"));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.Solve();
    }

    void TestSimpleLinearEllipticFiniteElementSolver() throw(Exception)
    {
        // Set up the vessel network
        units::quantity<unit::length> vessel_length(100.0*unit::microns);
        units::quantity<unit::length> reference_length(1.0*unit::microns);
        VesselNetworkGenerator<3> generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length, DimensionalChastePoint<3>());

        // Set up the grid
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(vessel_length, vessel_length, vessel_length, DimensionalChastePoint<3>());
        DimensionalChastePoint<3> translation_vector(-vessel_length/(2.0*reference_length),
                                                     -vessel_length/(2.0*reference_length), 0.0, reference_length);
        p_domain->Translate(translation_vector);

        boost::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_domain);
        units::quantity<unit::length> spacing(10.0*unit::microns);
        p_mesh_generator->SetMaxElementArea(spacing*spacing*spacing);
        p_mesh_generator->Update();

        // Choose the PDE
        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<3> > p_pde = DiscreteContinuumLinearEllipticPde<3>::Create();
        units::quantity<unit::diffusivity> diffusivity(0.0033 * unit::metre_squared_per_second);
        units::quantity<unit::concentration_flow_rate> consumption_rate(-2.e-7 * unit::mole_per_metre_cubed_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetContinuumConstantInUTerm(consumption_rate);

        // Set up the discrete source
        boost::shared_ptr<DiscreteContinuumBoundaryCondition<3> > p_boundary = DiscreteContinuumBoundaryCondition<3>::Create();
        p_boundary->SetValue(1.0*unit::mole_per_metre_cubed);

        // Set up and run the solver
        SimpleLinearEllipticFiniteElementSolver<3> solver;
        solver.SetGrid(p_mesh_generator->GetMesh());
        solver.SetPde(p_pde);
        solver.SetVesselNetwork(p_network);
        solver.AddBoundaryCondition(p_boundary);
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestDiscreteContinuumBoundaryCondition/FiniteElement"));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.Solve();
    }
};

#endif /*TESTDISCRETECONTINUUMBOUNDARYCONDITIONS_HPP_*/
