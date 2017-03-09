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

#ifndef TESTSIMPLEPARABOLICFINITEDIFFERENCESOLVER_HPP_
#define TESTSIMPLEPARABOLICFINITEDIFFERENCESOLVER_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include <boost/lexical_cast.hpp>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "Part.hpp"
#include "UnitCollection.hpp"
#include "ParabolicDiffusionReactionPde.hpp"
#include "SimpleParabolicFiniteDifferenceSolver.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestSimpleParabolicFiniteDifferenceSolver : public AbstractCellBasedWithTimingsTestSuite
{
public:

    void TestBox() throw(Exception)
    {
        BaseUnits::Instance()->SetReferenceConcentrationScale(1.e-6*unit::mole_per_metre_cubed);
        BaseUnits::Instance()->SetReferenceTimeScale(3600.0*unit::seconds);

        // Set up the mesh
        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(1000e-6*unit::metres, 1000e-6*unit::metres, DimensionalChastePoint<2>(0.0, 0.0, 0.0));

        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        p_grid->GenerateFromPart(p_domain, 100.0e-6*unit::metres);

        // Choose the PDE
        boost::shared_ptr<ParabolicDiffusionReactionPde<2> > p_pde = ParabolicDiffusionReactionPde<2>::Create();
        units::quantity<unit::diffusivity> vegf_diffusivity(6.94e-11 * unit::metre_squared_per_second);
        units::quantity<unit::rate> vegf_decay_rate((-0.8/3600.0) * unit::per_second);
        p_pde->SetIsotropicDiffusionConstant(vegf_diffusivity);
        p_pde->SetContinuumLinearInUTerm(vegf_decay_rate);

        units::quantity<unit::concentration> initial_vegf_concentration(3.93e-4*unit::mole_per_metre_cubed);
        std::vector<double> initial_condition(p_grid->GetNumberOfLocations(), double(initial_vegf_concentration/(1.e-6*unit::mole_per_metre_cubed)));

        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(1.0, 1); // Force 1 hour increments
        SimpleParabolicFiniteDifferenceSolver<2> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        solver.UpdateSolution(initial_condition);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestSimpleParabolicFiniteDifferenceSolver/Box", false));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);

        for(unsigned idx=0; idx<10; idx++)
        {
            solver.SetFileName("output_nl_fd_" + boost::lexical_cast<std::string>(idx));
            solver.Solve();
        }
    }
};

#endif /*TESTSIMPLEPARABOLICFINITEDIFFERENCESOLVER_HPP_*/
