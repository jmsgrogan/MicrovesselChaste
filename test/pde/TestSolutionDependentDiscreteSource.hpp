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

#ifndef TESTSOLUTIONDEPENDENTDISCRETESOURCE_HPP_
#define TESTSOLUTIONDEPENDENTDISCRETESOURCE_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "SmartPointers.hpp"
#include "Part.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "OutputFileHandler.hpp"
#include "RegularGrid.hpp"
#include "DiscreteSource.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "SolutionDependentDiscreteSource.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestSolutionDependentDiscreteSource : public CxxTest::TestSuite
{

public:

    void TestSimpleLinearEllipticFiniteDifferenceSolver() throw(Exception)
    {
        // Solve two problems on the same grid
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        QLength domain_length(100.0*unit::microns);
        p_domain->AddCuboid(domain_length, domain_length, domain_length, DimensionalChastePoint<3>());
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        QLength spacing(10.0*unit::microns);
        p_grid->GenerateFromPart(p_domain, spacing);

        // Choose the PDE
        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<3> > p_pde1 = DiscreteContinuumLinearEllipticPde<3>::Create();
        units::quantity<unit::diffusivity> diffusivity(0.0033 * unit::metre_squared_per_second);
        units::quantity<unit::concentration_flow_rate> consumption_rate(-2.e-7 * unit::mole_per_metre_cubed_per_second);
        p_pde1->SetIsotropicDiffusionConstant(diffusivity);
        p_pde1->SetContinuumConstantInUTerm(consumption_rate);

        boost::shared_ptr<DiscreteContinuumBoundaryCondition<3> > p_boundary = DiscreteContinuumBoundaryCondition<3>::Create();
        p_boundary->SetValue(1.0*unit::mole_per_metre_cubed);

        // Set up and run the first solver
        SimpleLinearEllipticFiniteDifferenceSolver<3> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde1);
        solver.AddBoundaryCondition(p_boundary);
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestSolutionDependentDiscreteSource/TestSimpleLinearEllipticFiniteDifferenceSolver"));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.Solve();

        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<3> > p_pde2 = DiscreteContinuumLinearEllipticPde<3>::Create();
        p_pde2->SetIsotropicDiffusionConstant(diffusivity);
        p_pde2->SetContinuumConstantInUTerm(consumption_rate);

        // Set up the discrete source
        boost::shared_ptr<SolutionDependentDiscreteSource<3> > p_solution_dependent_source = SolutionDependentDiscreteSource<3>::Create();
        p_solution_dependent_source->SetConstantInUSinkRatePerSolutionQuantity(0.1*unit::per_second);
        p_solution_dependent_source->SetSolution(solver.GetConcentrations(p_grid));
        p_pde2->AddDiscreteSource(p_solution_dependent_source);

        boost::shared_ptr<DiscreteContinuumBoundaryCondition<3> > p_boundary2 = DiscreteContinuumBoundaryCondition<3>::Create();
        p_boundary2->SetValue(1.0*unit::mole_per_metre_cubed);

        SimpleLinearEllipticFiniteDifferenceSolver<3> solver2;
        solver2.SetGrid(p_grid);
        solver2.SetPde(p_pde2);
        solver2.AddBoundaryCondition(p_boundary2);
        solver2.SetFileHandler(p_output_file_handler);
        solver2.SetWriteSolution(true);
        solver2.Solve();
    }
};

#endif /*TESTSOLUTIONDEPENDENTDISCRETESOURCE_HPP_*/
