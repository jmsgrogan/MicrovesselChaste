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
#include "CoupledLumpedSystemFiniteElementSolver.hpp"
#include "CoupledVegfPelletDiffusionReactionPde.hpp"
#include "UnitCollection.hpp"
#include "MappableGridGenerator.hpp"
#include "Part.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"
#include "MultiFormatMeshWriter.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestCoupledLumpedSystemFiniteElementSolver : public CxxTest::TestSuite
{
public:
    void TestExplicitSolver()
    {
        // Set up the mesh
        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(2000e-6*unit::metres, 1000e-6*unit::metres, DimensionalChastePoint<2>(0.0, 0.0, 0.0));

        DiscreteContinuumMeshGenerator<2> mesh_generator;
        mesh_generator.SetDomain(p_domain);
        mesh_generator.SetMaxElementArea(1e7*(units::pow<3>(1.e-6*unit::metres)));
        mesh_generator.Update();
        boost::shared_ptr<DiscreteContinuumMesh<2> > p_mesh = mesh_generator.GetMesh();

        boost::shared_ptr<CoupledVegfPelletDiffusionReactionPde<2> > p_pde =
                CoupledVegfPelletDiffusionReactionPde<2>::Create();
        units::quantity<unit::diffusivity> vegf_diffusivity(6.94e-11 * unit::metre_squared_per_second);
        units::quantity<unit::rate> vegf_decay_rate((-0.8/3600.0) * unit::per_second);
        p_pde->SetIsotropicDiffusionConstant(vegf_diffusivity);
        p_pde->SetContinuumLinearInUTerm(vegf_decay_rate);

        units::quantity<unit::concentration> initial_vegf_concentration(3.93e-4*unit::mole_per_metre_cubed);
        p_pde->SetMultiplierValue(initial_vegf_concentration);

        // Set up robin BC on top plane
        std::vector<DimensionalChastePoint<2> > boundary_points;
        TetrahedralMesh<2,2>::BoundaryElementIterator surf_iter = p_mesh->GetBoundaryElementIteratorBegin();
        while (surf_iter != p_mesh->GetBoundaryElementIteratorEnd())
        {
            unsigned node_index = (*surf_iter)->GetNodeGlobalIndex(0);
            double y = p_mesh->GetNode(node_index)->GetPoint()[1];
            if (y>999.0)
            {
                boundary_points.push_back(DimensionalChastePoint<2>(p_mesh->GetNode(node_index)->GetPoint()[0],
                        p_mesh->GetNode(node_index)->GetPoint()[0], 0.0,
                        p_mesh->GetReferenceLengthScale()));
            }
            surf_iter++;
        }
        boost::shared_ptr<DiscreteContinuumBoundaryCondition<2> > p_boundary_condition =
                DiscreteContinuumBoundaryCondition<2>::Create();
        units::quantity<unit::concentration> boundary_concentration(1.0* unit::mole_per_metre_cubed);
        p_boundary_condition->SetValue(boundary_concentration);
        p_boundary_condition->SetType(BoundaryConditionType::POINT);
        p_boundary_condition->SetIsRobin(true);
        p_boundary_condition->SetPoints(boundary_points);

        boost::shared_ptr<CoupledLumpedSystemFiniteElementSolver<2> > p_solver =
                CoupledLumpedSystemFiniteElementSolver<2>::Create();
        p_solver->SetGrid(p_mesh);
        p_solver->SetPde(p_pde);
        p_solver->AddBoundaryCondition(p_boundary_condition);
        p_solver->SetReferenceConcentration(1.e-9*unit::mole_per_metre_cubed);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestCoupledLumpedSystemFiniteElementSolver/Plane"));
        p_solver->SetFileHandler(p_output_file_handler);
        p_solver->SetWriteSolution(true);
        p_solver->SetTargetTimeIncrement(1.0);
        p_solver->SetStartTime(0.0);
        p_solver->SetEndTime(100.0);
        p_solver->Solve();
    }

//    void xTestSolveOnSphere()
//    {
//        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestCoupledLumpedSystemFiniteElementSolver/Sphere"));
//
//        units::quantity<unit::length> reference_length(1.0 * unit::microns);
//        units::quantity<unit::time> reference_time(1.0* unit::hours);
//        BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
//        BaseUnits::Instance()->SetReferenceTimeScale(reference_time);
//        BaseUnits::Instance()->SetReferenceConcentrationScale(1.e-9*unit::mole_per_metre_cubed);
//
//        MappableGridGenerator hemisphere_generator;
//        units::quantity<unit::length> radius(1400.0 * unit::microns);
//        units::quantity<unit::length> thickness(100.0 * unit::microns);
//        unsigned num_divisions_x = 20;
//        unsigned num_divisions_y = 20;
//        double azimuth_angle = 1.0 * M_PI;
//        double polar_angle = 0.999 * M_PI;
//        boost::shared_ptr<Part<3> > p_domain = hemisphere_generator.GenerateHemisphere(radius/reference_length,
//                                                                                         thickness/reference_length,
//                                                                                         num_divisions_x,
//                                                                                         num_divisions_y,
//                                                                                         azimuth_angle,
//                                                                                         polar_angle);
//
//        p_domain->Write(p_handler->GetOutputDirectoryFullPath()+"cornea.vtp", GeometryFormat::VTP);
//
//        boost::shared_ptr<Part<3> > p_vegf_domain = Part<3> ::Create();
//        units::quantity<unit::length> cylinder_radius(300.0*unit::microns);
//        units::quantity<unit::length> cylinder_height(50.0*unit::microns);
//        DimensionalChastePoint<3> pellet_base(0.0, 0.0, 1300.0);
//        DimensionalChastePoint<3> pellet_centre(0.0, 0.0, 1350.0);
//
//        p_vegf_domain->AddCylinder(cylinder_radius, cylinder_height, pellet_base);
//
//        // Rotate the part
//        std::vector<boost::shared_ptr<Polygon<3> > > polygons = p_vegf_domain->GetPolygons();
//        c_vector<double, 3> rotation_axis;
//        rotation_axis[0] = 0.0;
//        rotation_axis[1] = 1.0;
//        rotation_axis[2] = 0.0;
//        double rotation_angle = 0.0;
//        p_vegf_domain->RotateAboutAxis(rotation_axis, rotation_angle);
//        pellet_centre.RotateAboutAxis(rotation_axis, rotation_angle);
//        p_vegf_domain->Write(p_handler->GetOutputDirectoryFullPath()+"initial_vegf_domain.vtp");
//
//        // Add the pellet domain to the cornea
//        p_domain->AppendPart(p_vegf_domain);
//        p_domain->AddHoleMarker(pellet_centre);
//        p_domain->Write(p_handler->GetOutputDirectoryFullPath()+"merged_cornea.vtp", GeometryFormat::VTP);
//
//        DiscreteContinuumMeshGenerator<3> mesh_generator;
//        mesh_generator.SetDomain(p_domain);
//        mesh_generator.SetMaxElementArea(1e7*(units::pow<3>(1.e-6*unit::metres))); // 1e4 for 'good' mesh
//        mesh_generator.Update();
//        boost::shared_ptr<DiscreteContinuumMesh<3> > p_mesh = mesh_generator.GetMesh();
//        MultiFormatMeshWriter<3> mesh_writer;
//        mesh_writer.SetFilename(p_handler->GetOutputDirectoryFullPath()+"cornea_mesh");
//        mesh_writer.SetMesh(p_mesh);
//        mesh_writer.SetOutputFormat(MeshFormat::VTU);
//        mesh_writer.Write();
//
//        HeatEquation<3> pde;
//
//        BoundaryConditionsContainer<3,3,1> bcc;
//
//        // Hijack the Neumann boundary condition flags
//        TetrahedralMesh<3,3>::BoundaryElementIterator surf_iter = p_mesh->GetBoundaryElementIteratorBegin();
//        ConstBoundaryCondition<3>* p_pellet_bound_condition = new ConstBoundaryCondition<3>(1.0);
//        while (surf_iter != p_mesh->GetBoundaryElementIteratorEnd())
//        {
//            unsigned node_index = (*surf_iter)->GetNodeGlobalIndex(0);
//            double x = p_mesh->GetNode(node_index)->GetPoint()[0];
//            double y = p_mesh->GetNode(node_index)->GetPoint()[1];
//            double z = p_mesh->GetNode(node_index)->GetPoint()[2];
//            DimensionalChastePoint<3> loc(x, y, z);
//            for(unsigned idx=0; idx<polygons.size(); idx++)
//            {
//                if(polygons[idx]->ContainsPoint(loc))
//                {
//                    bcc.AddNeumannBoundaryCondition(*surf_iter, p_pellet_bound_condition);
//                }
//            }
//            surf_iter++;
//        }
//
//        CoupledInterfaceOdePdeSolver<3> solver(p_mesh.get(), &bcc, &pde);
//
//        /* The interface is exactly the same as the `SimpleLinearParabolicSolver`. */
//        solver.SetTimeStep(10000.0);
//        solver.SetTimes(0.0, 1000000.0);
//
//        std::vector<double> init_cond(p_mesh->GetNumNodes(), 0.0);
//        Vec initial_condition = PetscTools::CreateVec(init_cond);
//        solver.SetInitialCondition(initial_condition);
//
//        solver.SetOutputDirectoryAndPrefix("TestCoupledLumpedSystemFiniteElementSolver","results");
//        solver.SetOutputToVtk(true);
//        solver.SetPrintingTimestepMultiple(10);
//
//        Vec result = solver.Solve();
//        ReplicatableVector result_repl(result);
//
//        // Tidy up
//        PetscTools::Destroy(initial_condition);
//        PetscTools::Destroy(result);
//    }
};

#endif // TESTCOUPLEDVEGFODESYSTEM_HPP_
