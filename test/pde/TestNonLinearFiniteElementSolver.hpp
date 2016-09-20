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

#ifndef TESTNONLINEARFINITEELEMENTSOLVER_HPP_
#define TESTNONLINEARFINITEELEMENTSOLVER_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include <boost/lexical_cast.hpp>
#include "FiniteElementSolver.hpp"
#include "UblasIncludes.hpp"
#include "Part.hpp"
#include "Vertex.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde.hpp"
#include "LinearSteadyStateDiffusionReactionPde.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "SmartPointers.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "PetscSetupAndFinalize.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"

class TestNonLinearFiniteElementSolver : public AbstractCellBasedWithTimingsTestSuite
{
public:

    void TestBox() throw(Exception)
    {
        // Set up the mesh
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(4.0e-6*unit::metres, 4.0e-6*unit::metres, 4.0e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        boost::shared_ptr<DiscreteContinuumMeshGenerator<3, 3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3, 3>::Create();
        p_mesh_generator->SetDomain(p_domain);
        p_mesh_generator->SetMaxElementArea(1.0);
        p_mesh_generator->Update();

        // Choose the PDE
        boost::shared_ptr<LinearSteadyStateDiffusionReactionPde<3> > p_linear_pde = LinearSteadyStateDiffusionReactionPde<3>::Create();
        units::quantity<unit::diffusivity> diffusivity(1.e-6 * unit::metre_squared_per_second);
        units::quantity<unit::rate> consumption_rate(-2.e-5 * unit::per_second);
        p_linear_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_linear_pde->SetContinuumLinearInUTerm(consumption_rate);

        boost::shared_ptr<MichaelisMentenSteadyStateDiffusionReactionPde<3> > p_non_linear_pde = MichaelisMentenSteadyStateDiffusionReactionPde<3>::Create();
        p_non_linear_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_non_linear_pde->SetContinuumLinearInUTerm(consumption_rate);

        // Choose the Boundary conditions
        boost::shared_ptr<DiscreteContinuumBoundaryCondition<3> > p_outer_boundary_condition = DiscreteContinuumBoundaryCondition<3>::Create();
        units::quantity<unit::concentration> boundary_concentration(1.0 * unit::mole_per_metre_cubed);
        p_outer_boundary_condition->SetValue(boundary_concentration);

        FiniteElementSolver<3> solver;
        solver.SetMesh(p_mesh_generator->GetMesh());
        solver.SetPde(p_linear_pde);
        solver.SetNonLinearPde(p_non_linear_pde);
        solver.AddBoundaryCondition(p_outer_boundary_condition);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestNonLinearFiniteElementSolver/Box", false));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetFileName("output_nl_t");
        solver.SetWriteSolution(true);
        solver.SetUseSimpleNetonSolver(true);
        solver.Solve();
    }
};

#endif /*TESTNONLINEARFINITEELEMENTSOLVER_HPP_*/
