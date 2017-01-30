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

#ifndef TESTTIMEDEPENDENTFINITEDIFFERENCESOLVER_HPP_
#define TESTTIMEDEPENDENTFINITEDIFFERENCESOLVER_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include <boost/lexical_cast.hpp>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "Part.hpp"
#include "UnitCollection.hpp"
#include "CoupledVegfPelletDiffusionReactionPde.hpp"
#include "FiniteDifferenceSolver.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestTimeDependentFiniteDifferenceSolver : public AbstractCellBasedWithTimingsTestSuite
{
public:

    void TestBox() throw(Exception)
    {
        BaseUnits::Instance()->SetReferenceConcentrationScale(1.e-6*unit::mole_per_metre_cubed);
        BaseUnits::Instance()->SetReferenceTimeScale(3600.0*unit::seconds);

        // Set up the mesh
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(2.0e-3*unit::metres, 1.24e-3*unit::metres, 100.0e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0, 0.0));

        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        p_grid->GenerateFromPart(p_domain, 20.0e-6*unit::metres);

        // Choose the PDE
        boost::shared_ptr<CoupledVegfPelletDiffusionReactionPde<3> > p_pde = CoupledVegfPelletDiffusionReactionPde<3>::Create();
        units::quantity<unit::diffusivity> vegf_diffusivity(6.94e-11 * unit::metre_squared_per_second);
        units::quantity<unit::rate> vegf_decay_rate((-0.8/3600.0) * unit::per_second);
        p_pde->SetIsotropicDiffusionConstant(vegf_diffusivity);
        p_pde->SetContinuumLinearInUTerm(vegf_decay_rate);

        units::quantity<unit::concentration> initial_vegf_concentration(3.93e-4*unit::mole_per_metre_cubed);
        p_pde->SetMultiplierValue(initial_vegf_concentration);

        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(1.0, 1); // Force 1 hour increments
        FiniteDifferenceSolver<3> solver;
        solver.SetGrid(p_grid);
        solver.SetParabolicPde(p_pde);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestTimeDependentFiniteDifferenceSolver/Box", false));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);

        for(unsigned idx=0; idx<120; idx++) // 5 days
        {
            solver.SetFileName("output_nl_fd_" + boost::lexical_cast<std::string>(idx));
            solver.Solve();
        }
    }
};

#endif /*TESTTIMEDEPENDENTFINITEDIFFERENCESOLVER_HPP_*/