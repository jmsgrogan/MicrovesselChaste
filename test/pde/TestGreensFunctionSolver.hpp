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

#ifndef TESTSIMPLELINEARELLIPTICGREENSFUNCTIONSOLVER_HPP_
#define TESTSIMPLELINEARELLIPTICGREENSFUNCTIONSOLVER_HPP_

#include <cxxtest/TestSuite.h>
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "SimpleLinearEllipticGreensFunctionSolver.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "RegularGrid.hpp"
#include "UnitCollection.hpp"
#include "VesselNetworkPropertyManager.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestGreenFunctionSolver : public CxxTest::TestSuite
{

public:

    void TestSingleVessel3d()
    {
        QLength vessel_length = 2.0e-6*unit::metres;
        VesselNetworkGenerator<3> generator;
        DimensionalChastePoint<3> centre(0.5, 0.5, 0.0);

        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length, centre, 14.0);
        VesselNetworkPropertyManager<3>::SetSegmentRadii(p_network, 0.05*1.e-6*unit::metres);

        // Set up the grid
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(1.0e-6*unit::metres, 1.0e-6*unit::metres, 2.0e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        p_grid->GenerateFromPart(p_domain, 0.1*1.e-6*unit::metres);

        // Choose the PDE
        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<3> > p_pde = DiscreteContinuumLinearEllipticPde<3>::Create();
        units::quantity<unit::diffusivity> diffusivity(0.0033 * unit::metre_squared_per_second);
        units::quantity<unit::concentration_flow_rate> consumption_rate(-2.e-7 * unit::mole_per_metre_cubed_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetContinuumConstantInUTerm(consumption_rate);

        SimpleLinearEllipticGreensFunctionSolver<3> solver;
        solver.SetVesselNetwork(p_network);
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestSimpleLinearEllipticGreensFunctionSolver/TestSingleVessel3d", false));
        solver.SetFileHandler(p_output_file_handler);
        solver.Setup();
        solver.SetWriteSolution(true);
        solver.Solve();
    }
};

#endif /*TESTSIMPLELINEARELLIPTICGREENSFUNCTIONSOLVER_HPP_*/
