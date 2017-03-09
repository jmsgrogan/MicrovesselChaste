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

#ifndef TESTGENERALNEUMANNBC_HPP_
#define TESTGENERALNEUMANNBC_HPP_

#include <cxxtest/TestSuite.h>
#include "UblasIncludes.hpp"
//#include "SimpleLinearEllipticSolver.hpp"
#include "SimpleLinearEllipticSolverWithRobinBc.hpp"
#include "TrianglesMeshReader.hpp"
#include "TetrahedralMesh.hpp"
#include "BoundaryConditionsContainer.hpp"
#include "ConstBoundaryCondition.hpp"
#include "OutputFileHandler.hpp"
#include "VtkMeshWriter.hpp"
#include "VesselNetworkGenerator.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "PetscSetupAndFinalize.hpp"

class TestGeneralNeumannBc : public CxxTest::TestSuite
{

public:
    void TestSolvingEllipticPde() throw(Exception)
    {
        double vessel_length = 100.0;
//        VesselNetworkGenerator<3> generator;
//        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length);
//        p_network->GetVessels()[0]->GetStartNode()->SetRadius(10.0);
//        p_network->GetVessels()[0]->GetEndNode()->SetRadius(10.0);
//
//        boost::shared_ptr<Part<3> > p_part = Part<3>::Create();
//        boost::shared_ptr<Polygon> p_circle = p_part->AddCircle(100.0);
//        p_part->Extrude(p_circle, 100.0);
//        p_part->AddVesselNetwork(p_network, true);
//
//        boost::shared_ptr<DiscreteContinuumMesh<3> > p_mesh = PlcMesh<3>::Create();
//        p_mesh->GenerateFromPart(p_part, 100.0);
//
//        // Choose the PDE
//        boost::shared_ptr<HybridLinearEllipticPde<3> > p_pde = HybridLinearEllipticPde<3>::Create();
//        p_pde->SetDiffusionConstant(0.0033);
//        p_pde->SetConstantInUTerm(-1.e-5);
//
//        BoundaryConditionsContainer<3,3,1> bcc;
//
//        double p_wall = 40.0;
//
////        ConstBoundaryCondition<3>* p_outer_boundary_condition = new ConstBoundaryCondition<3>(0.0);
////        TetrahedralMesh<3,3>::BoundaryNodeIterator iter = p_mesh->GetBoundaryNodeIteratorBegin();
////        while (iter < p_mesh->GetBoundaryNodeIteratorEnd())
////        {
////            double x = (*iter)->GetPoint()[0];
////            double y = (*iter)->GetPoint()[1];
////            double radius = std::sqrt(x*x + y*y) / 2.0;
////            if (radius > 49.0)
////            {
////                bcc.AddDirichletBoundaryCondition(*iter, p_outer_boundary_condition);
////            }
////            iter++;
////        }
//
//        /* Now we create Neumann boundary conditions for the ''surface elements'' on x=1 and y=1. Note that
//         * Dirichlet boundary conditions are defined on nodes, whereas Neumann boundary conditions are
//         * defined on surface elements. Note also that the natural boundary condition statement for this
//         * PDE is (D grad u).n = g(x) (where n is the outward-facing surface normal), and g(x) is a prescribed
//         * function, ''not'' something like du/dn=g(x). Hence the boundary condition we are specifying is
//         * (D grad u).n = 0.
//         *
//         * '''Important note for 1D:''' This means that if we were solving 2u,,xx,,=f(x) in 1D, and
//         * wanted to specify du/dx=1 on the LHS boundary, the Neumann boundary value we have to specify is
//         * -2, as n=-1 (outward facing normal) so (D gradu).n = -2 when du/dx=1.
//         *
//         * To define Neumann bcs, we reuse the zero boundary condition object defined above, but apply it
//         * at surface elements.  We loop over these using another iterator provided by the mesh class.
//         */
//        TetrahedralMesh<3,3>::BoundaryElementIterator surf_iter
//            = p_mesh->GetBoundaryElementIteratorBegin();
//
//        double k_term = 0.5;
//        ConstBoundaryCondition<3>* p_inner_boundary_condition = new ConstBoundaryCondition<3>(k_term * p_wall/2.0);
//        while (surf_iter != p_mesh->GetBoundaryElementIteratorEnd())
//        {
//            unsigned node_index = (*surf_iter)->GetNodeGlobalIndex(0);
//            double x = p_mesh->GetNode(node_index)->GetPoint()[0];
//            double y = p_mesh->GetNode(node_index)->GetPoint()[1];
//            double radius = std::sqrt(x*x + y*y) / 2.0;
//            if (radius < 11.0)
//            {
//                bcc.AddNeumannBoundaryCondition(*surf_iter, p_inner_boundary_condition);
//            }
//            surf_iter++;
//        }
//
//        //SimpleLinearEllipticSolver<2,2> solver(&mesh, &pde, &bcc);
//        SimpleLinearEllipticSolverWithRobinBc<3,3> solver(p_mesh.get(), p_pde.get(), &bcc);
//        Vec result = solver.Solve();
//
//        ReplicatableVector result_repl(result);
//        std::vector<double> vec_result;
//        for(unsigned idx=0; idx<result_repl.GetSize(); idx++)
//        {
//            vec_result.push_back(result_repl[idx]);
//        }
//
//        OutputFileHandler output_file_handler("TestGeneralNeumannBcs");
//
//        VtkMeshWriter <3, 3> mesh_writer(output_file_handler.GetRelativePath(), "result", false);
//        mesh_writer.AddPointData("result", vec_result);
//        mesh_writer.WriteFilesUsingMesh(*p_mesh);
//
//        /* All PETSc {{{Vec}}}s should be destroyed when they are no longer needed, or you will have a memory leak. */
//        PetscTools::Destroy(result);
    }
};

#endif /*TESTGENERALNEUMANNBC_HPP_*/
