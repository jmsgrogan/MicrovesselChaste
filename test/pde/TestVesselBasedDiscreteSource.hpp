/*

Copyright (c) 2005-2016, University of Oxford.
 All rights reserved.

 University of Oxford means the Chancellor, Masters and Scholars of the
 University of Oxford, having an administrative office at Wellington
 Square, Oxford OX1 2JD, UK.

 This file is part of Chaste.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the abovea copyright notice,
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

#ifndef TESTVESSELBASEDDISCRETESOURCE_HPP_
#define TESTVESSELBASEDDISCRETESOURCE_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include "LinearSteadyStateDiffusionReactionPde.hpp"
#include "SmartPointers.hpp"
#include "Part.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde.hpp"
#include "FiniteDifferenceSolver.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "OutputFileHandler.hpp"
#include "RegularGrid.hpp"
#include "VesselBasedDiscreteSource.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestVesselBasedDiscreteSource : public CxxTest::TestSuite
{

public:

    void TestGridFunction() throw(Exception)
    {
        // Set up the vessel network
        units::quantity<unit::length> vessel_length = 100 * 1.e-6 * unit::metres;
        VesselNetworkGenerator<3> generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length,
                                                                                        DimensionalChastePoint<3>(0.0, 0.0, 0.0));

        // Set up the grid
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(vessel_length,
                            vessel_length,
                            vessel_length,
                            DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        DimensionalChastePoint<3> translation_vector(-vessel_length/(2.0* 1.e-6 * unit::metres),
                                                     -vessel_length/(2.0* 1.e-6 * unit::metres), 0.0, 1.e-6 * unit::metres);
        p_domain->Translate(translation_vector);
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        p_grid->GenerateFromPart(p_domain, 10.0 * 1.e-6 * unit::metres);

        // Choose the PDE
        boost::shared_ptr<LinearSteadyStateDiffusionReactionPde<3> > p_pde = LinearSteadyStateDiffusionReactionPde<3>::Create();
        units::quantity<unit::diffusivity> diffusivity(0.0033 * unit::metre_squared_per_second);
        units::quantity<unit::concentration_flow_rate> consumption_rate(-2.e-7 * unit::mole_per_metre_cubed_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetContinuumConstantInUTerm(consumption_rate);

        // Set up the discrete source
        boost::shared_ptr<VesselBasedDiscreteSource<3> > p_vessel_source_lin = VesselBasedDiscreteSource<3>::Create();
        p_vessel_source_lin->SetLinearInUValue(-1.e3*unit::per_second);

        boost::shared_ptr<VesselBasedDiscreteSource<3> > p_vessel_source_const = VesselBasedDiscreteSource<3>::Create();
        p_vessel_source_const->SetConstantInUValue(40.e3* unit::mole_per_metre_cubed_per_second);

        p_pde->AddDiscreteSource(p_vessel_source_lin);
        p_pde->AddDiscreteSource(p_vessel_source_const);

        // Set up and run the solver
        FiniteDifferenceSolver<3> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        solver.SetVesselNetwork(p_network);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestVesselBasedDiscreteSource/TestGridFunction", false));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.Solve();
    }
};

#endif /*TESTVESSELBASEDDISCRETESOURCE_HPP_*/
