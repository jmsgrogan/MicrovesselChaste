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

#ifndef TESTCOUPLEDVEGFODESYSTEM_HPP_
#define TESTCOUPLEDVEGFODESYSTEM_HPP_

#include <cxxtest/TestSuite.h>
#include "TetrahedralMesh.hpp"
#include "TrianglesMeshReader.hpp"
#include "BoundaryConditionsContainer.hpp"
#include "CoupledInterfaceOdePdeSolver.hpp"

#include "PetscSetupAndFinalize.hpp"

/**
 * A simple parabolic PDE used in tests.
 */
template <int SPACE_DIM>
class HeatEquation : public AbstractLinearParabolicPde<SPACE_DIM>
{

public:
    double ComputeSourceTerm(const ChastePoint<SPACE_DIM>& , double, Element<SPACE_DIM,SPACE_DIM>* pElement=NULL)
    {
        return 0.0;
    }

    c_matrix<double, SPACE_DIM, SPACE_DIM> ComputeDiffusionTerm(const ChastePoint<SPACE_DIM>& , Element<SPACE_DIM,SPACE_DIM>* pElement=NULL)
    {
        return identity_matrix<double>(SPACE_DIM);
    }

    double ComputeDuDtCoefficientFunction(const ChastePoint<SPACE_DIM>& )
    {
        return 1;
    }
};


class TestCoupledVegfOdeSystem : public CxxTest::TestSuite
{
public:
    void TestExplicitSolver() throw (Exception)
    {
        TetrahedralMesh<2,2> mesh;
        mesh.ConstructRegularSlabMesh(2.0 /*h*/, 50.0 /*width*/, 18.0 /*height*/);

        HeatEquation<2> pde;

        // Set up BCs u=0 on entire boundary
        BoundaryConditionsContainer<2,2,1> bcc;

        // Hijack the Neumann boundary condition flags
        TetrahedralMesh<2,2>::BoundaryElementIterator surf_iter = mesh.GetBoundaryElementIteratorBegin();
        ConstBoundaryCondition<2>* p_left_wall_boundary_condition = new ConstBoundaryCondition<2>(1.0);
        while (surf_iter != mesh.GetBoundaryElementIteratorEnd())
        {
            unsigned node_index = (*surf_iter)->GetNodeGlobalIndex(0);
            double x = mesh.GetNode(node_index)->GetPoint()[0];
            double y = mesh.GetNode(node_index)->GetPoint()[1];
            if (x < 0.2 and y>0.0)
            {
                bcc.AddNeumannBoundaryCondition(*surf_iter, p_left_wall_boundary_condition);
            }
            surf_iter++;
        }

        CoupledInterfaceOdePdeSolver<2> solver(&mesh, &bcc, &pde);

        /* The interface is exactly the same as the `SimpleLinearParabolicSolver`. */
        solver.SetTimeStep(1.0);
        solver.SetTimes(0.0, 100.0);

        std::vector<double> init_cond(mesh.GetNumNodes(), 0.0);
        Vec initial_condition = PetscTools::CreateVec(init_cond);
        solver.SetInitialCondition(initial_condition);

        solver.SetOutputDirectoryAndPrefix("CoupledVegfOdeSystem","results");
        solver.SetOutputToVtk(true);
        solver.SetPrintingTimestepMultiple(10);

        Vec result = solver.Solve();
        ReplicatableVector result_repl(result);

        // Tidy up
        PetscTools::Destroy(initial_condition);
        PetscTools::Destroy(result);
    }
};

#endif // TESTCOUPLEDVEGFODESYSTEM_HPP_
