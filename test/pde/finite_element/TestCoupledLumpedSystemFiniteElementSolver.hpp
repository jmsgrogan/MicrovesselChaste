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

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestCoupledLumpedSystemFiniteElementSolver : public CxxTest::TestSuite
{
    /**
     * Diffusion only in 1D with Robin Boundary on the left
     * From Crank Diffusion text
     */
    double SolveRobinBoundary(double x, double t, double D, double k, double c_0)
    {
        double h = k/D;
        double L = std::sqrt(t*D);
        double erfc1 = erfc(x/(2.0*L));
        double erfc2 = erfc(x/(2.0*L)+h*L);
        double expterm = std::exp(h*x + h*h*D*t);
        double c = c_0*(erfc1-expterm*erfc2);
        return c;
    }

public:

    void TestPlaneSlowRelease() throw(Exception)
    {
        BaseUnits::Instance()->SetReferenceLengthScale(1.0*unit::metres);
        BaseUnits::Instance()->SetReferenceConcentrationScale(1.0*unit::mole_per_metre_cubed);
        BaseUnits::Instance()->SetReferenceTimeScale(1.0*unit::seconds);

        // Set up the mesh
        std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(10.0*unit::metres, 100.0*unit::metres, DimensionalChastePoint<2>(0.0, 0.0, 0.0));
        p_domain->AddAttributeToEdgeIfFound(DimensionalChastePoint<2>(5.0, 100.0, 0, 1.0*unit::metres), "Top Boundary", 1.0);
        TS_ASSERT(p_domain->EdgeHasAttribute(DimensionalChastePoint<2>(5.0, 100.0, 0, 1.0*unit::metres), "Top Boundary"));

        DiscreteContinuumMeshGenerator<2> mesh_generator;
        mesh_generator.SetDomain(p_domain);
        mesh_generator.SetMaxElementArea(4.0*(Qpow3(1.0*unit::metres)));
        mesh_generator.Update();
        std::shared_ptr<DiscreteContinuumMesh<2> > p_mesh = mesh_generator.GetMesh();

        std::shared_ptr<CoupledVegfPelletDiffusionReactionPde<2> > p_pde =
                CoupledVegfPelletDiffusionReactionPde<2>::Create();
        QDiffusivity vegf_diffusivity(2.3* unit::metre_squared_per_second);
        p_pde->SetIsotropicDiffusionConstant(vegf_diffusivity);
        QConcentration initial_vegf_concentration(1.0*unit::mole_per_metre_cubed);
        p_pde->SetCurrentVegfInPellet(initial_vegf_concentration);
        p_pde->SetCorneaPelletPermeability(1.0*unit::metre_per_second);
        p_pde->SetPelletFreeDecayRate(0.0*unit::per_second);
        p_pde->SetPelletBindingConstant(1.0);
        p_pde->SetPelletVolume(1.0*unit::metres*1.0*unit::metres*1.0*unit::metres);

        // Set up robin BC on top plane
        vtkSmartPointer<vtkPoints> p_boundary_points = vtkSmartPointer<vtkPoints>::New();
        TetrahedralMesh<2,2>::BoundaryElementIterator surf_iter = p_mesh->GetBoundaryElementIteratorBegin();
        while (surf_iter != p_mesh->GetBoundaryElementIteratorEnd())
        {
            unsigned node_index = (*surf_iter)->GetNodeGlobalIndex(0);
            double y = p_mesh->GetNode(node_index)->GetPoint()[1];
            if (y>99.9)
            {
                p_boundary_points->InsertNextPoint(p_mesh->GetNode(node_index)->GetPoint()[0],
                        p_mesh->GetNode(node_index)->GetPoint()[1], 0.0);
            }
            surf_iter++;
        }
        std::shared_ptr<DiscreteContinuumBoundaryCondition<2> > p_boundary_condition =
                DiscreteContinuumBoundaryCondition<2>::Create();
        QConcentration boundary_concentration(1.0* unit::mole_per_metre_cubed);
        p_boundary_condition->SetValue(boundary_concentration);
        p_boundary_condition->SetType(BoundaryConditionType::EDGE);
        p_boundary_condition->SetIsRobin(true);
        p_boundary_condition->SetLabel("Top Boundary");
        p_boundary_condition->SetDomain(p_domain);

        std::shared_ptr<CoupledLumpedSystemFiniteElementSolver<2> > p_solver =
                CoupledLumpedSystemFiniteElementSolver<2>::Create();
        p_solver->SetGrid(p_mesh);
        p_solver->SetPde(p_pde);
        p_solver->AddBoundaryCondition(p_boundary_condition);

        auto p_output_file_handler =
        		std::make_shared<OutputFileHandler>("TestCoupledLumpedSystemFiniteElementSolver/PlaneSlowRelease");

        p_solver->SetFileHandler(p_output_file_handler);
        p_solver->SetWriteSolution(true);
        p_solver->SetTargetTimeIncrement(0.1);
        p_solver->SetUseCoupling(false);
        p_solver->SetStartTime(0.0);
        p_solver->SetEndTime(10.0);
        p_solver->SetWriteIntermediateSolutions(true, 10);
        p_solver->Solve();

        // Test the intermediate solutions
        vtkSmartPointer<vtkPoints> p_sample_points = vtkSmartPointer<vtkPoints>::New();
        for(unsigned idx=0; idx<11; idx++)
        {
            p_sample_points->InsertNextPoint(5.0, 100.0-idx*10.0, 0.0);
        }

        std::vector<std::pair<std::vector<double>, double> > intermediate_solutions =
                p_solver->rGetIntermediateSolutions();
        double diff_nondim = vegf_diffusivity*(1.0*unit::seconds)/(1.0*unit::metres*1.0*unit::metres);
        double conc_nondim = initial_vegf_concentration/(1.0*unit::mole_per_metre_cubed);
        double perm_nondim = p_pde->GetCorneaPelletPermeability()*(1.0*unit::seconds)/(1.0*unit::metres);

        for(unsigned idx=0; idx<intermediate_solutions.size();idx++)
        {
            double time = intermediate_solutions[idx].second;
            if(time>10.0)
            {
                p_solver->UpdateSolution(intermediate_solutions[idx].first);
                std::vector<QConcentration > solution = p_solver->GetConcentrations(p_sample_points);
                for(unsigned jdx=0; jdx<11; jdx++)
                {
                    QLength x = double(jdx)*10.0*unit::metres;
                    double x_nondim = x/(1.0*unit::metres);
                    double c_analytical_nondim = SolveRobinBoundary(x_nondim, time, diff_nondim, perm_nondim, conc_nondim);
                    double c_numerical_nondim = solution[jdx]/(1.0*unit::mole_per_metre_cubed);
                    TS_ASSERT_DELTA(c_analytical_nondim, c_numerical_nondim, 2.e-2)
                }
            }
        }
    }

    void TestSolveOnCircle()
    {
        auto p_handler =
        		std::make_shared<OutputFileHandler>("TestCoupledLumpedSystemFiniteElementSolver/Circle");

        QLength reference_length(1.0 * unit::microns);
        QTime reference_time(1.0* unit::hours);
        BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
        BaseUnits::Instance()->SetReferenceTimeScale(reference_time);
        BaseUnits::Instance()->SetReferenceConcentrationScale(1.e-9*unit::mole_per_metre_cubed);

        QLength radius(1300.0 * unit::microns);
        QLength cylinder_radius(300.0*unit::microns);
        QLength pellet_spacing(700.0 * unit::microns);
        QLength delta = pellet_spacing-radius+cylinder_radius;

        std::shared_ptr<Part<2> > p_domain = Part<2> ::Create();
        p_domain->AddCircle(radius, DimensionalChastePoint<2>(0.0, 0.0, 0.0));
        std::shared_ptr<Polygon<2> > p_polygon = p_domain->AddCircle(cylinder_radius,
                DimensionalChastePoint<2>(0.0, -delta/reference_length, 0.0, reference_length));
        p_polygon->AddAttributeToAllEdges("Inner Boundary", 1.0);
        p_domain->AddHoleMarker(DimensionalChastePoint<2>(0.0, 0.0, 0.0, reference_length));
        p_domain->Write(p_handler->GetOutputDirectoryFullPath()+"cornea.vtp", GeometryFormat::VTP);

        DiscreteContinuumMeshGenerator<2> mesh_generator;
        mesh_generator.SetDomain(p_domain);
        mesh_generator.SetMaxElementArea(1e4*(Qpow3(1_um))); // 1e4 for 'good' mesh
        std::vector<DimensionalChastePoint<2> > holes;
        holes.push_back(DimensionalChastePoint<2>(0.0, -delta/reference_length, 0.0, reference_length));
        mesh_generator.SetHoles(holes);
        mesh_generator.Update();

        std::shared_ptr<DiscreteContinuumMesh<2> > p_mesh = mesh_generator.GetMesh();
        MultiFormatMeshWriter<2> mesh_writer;
        mesh_writer.SetFileName(p_handler->GetOutputDirectoryFullPath()+"cornea_mesh");
        mesh_writer.SetMesh(p_mesh);
        mesh_writer.SetOutputFormat(MeshFormat::VTU);
        mesh_writer.Write();

        // Set the BCs
        std::shared_ptr<DiscreteContinuumBoundaryCondition<2> > p_boundary_condition =
                DiscreteContinuumBoundaryCondition<2>::Create();
        QConcentration boundary_concentration(1.0* unit::mole_per_metre_cubed);
        p_boundary_condition->SetValue(boundary_concentration);
        p_boundary_condition->SetType(BoundaryConditionType::EDGE);
        p_boundary_condition->SetIsRobin(true);
        p_boundary_condition->SetDomain(p_domain);
        p_boundary_condition->SetLabel("Inner Boundary");

        // Choose the PDE
        std::shared_ptr<CoupledVegfPelletDiffusionReactionPde<2> > p_pde = CoupledVegfPelletDiffusionReactionPde<2>::Create();
        QDiffusivity vegf_diffusivity(6.94e-11 * unit::metre_squared_per_second);
        QRate vegf_decay_rate((-0.8/3600.0) * unit::per_second);
        p_pde->SetIsotropicDiffusionConstant(vegf_diffusivity);
        p_pde->SetContinuumLinearInUTerm(vegf_decay_rate);
        QConcentration initial_vegf_concentration(3.93e-4*unit::mole_per_metre_cubed);
        p_pde->SetCurrentVegfInPellet(initial_vegf_concentration);

        CoupledLumpedSystemFiniteElementSolver<2> solver;
        solver.SetGrid(p_mesh);
        solver.SetPde(p_pde);
        solver.SetFileHandler(p_handler);
        solver.SetWriteSolution(true);
        solver.AddBoundaryCondition(p_boundary_condition);
        solver.SetTargetTimeIncrement(0.01);
        solver.SetStartTime(0.0);
        solver.SetEndTime(1.0);
        solver.SetWriteIntermediateSolutions(true, 20);
        p_domain->Write(p_handler->GetOutputDirectoryFullPath()+"merged_cornea2.vtp", GeometryFormat::VTP);
        solver.Solve();

        vtkSmartPointer<vtkPoints> p_sample_points = vtkSmartPointer<vtkPoints>::New();
        QLength cell_spacing(40.0*unit::microns);
        QLength sampling_radius = radius-0.2e-6*unit::metres;
        unsigned num_cells = cell_spacing/(2.0*M_PI*sampling_radius) +1u;
        double sweep_angle = 2.0*M_PI/num_cells;

        for(unsigned idx=0; idx<num_cells; idx++)
        {
            double this_angle = double(idx)*sweep_angle;
            double x_coord = (sampling_radius/reference_length)*std::sin(this_angle);
            double y_coord = (sampling_radius/reference_length)*std::cos(this_angle);
            p_sample_points->InsertNextPoint(x_coord, y_coord, 0.0);
        }
    }

    void TestSolveOnSphere()
    {
        auto p_handler =
        		std::make_shared<OutputFileHandler>("TestCoupledLumpedSystemFiniteElementSolver/Sphere");

        QLength reference_length(1.0 * unit::microns);
        QTime reference_time(1.0* unit::hours);
        BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
        BaseUnits::Instance()->SetReferenceTimeScale(reference_time);
        BaseUnits::Instance()->SetReferenceConcentrationScale(1.e-9*unit::mole_per_metre_cubed);

        MappableGridGenerator<3> hemisphere_generator;
        QLength radius(1400.0 * unit::microns);
        QLength thickness(100.0 * unit::microns);
        unsigned num_divisions_x = 20;
        unsigned num_divisions_y = 20;
        double azimuth_angle = 1.0 * M_PI;
        double polar_angle = 0.999 * M_PI;
        std::shared_ptr<Part<3> > p_domain = hemisphere_generator.GenerateHemisphere(radius,
                                                                                         thickness,
                                                                                         num_divisions_x,
                                                                                         num_divisions_y,
                                                                                         azimuth_angle,
                                                                                         polar_angle);
        p_domain->Write(p_handler->GetOutputDirectoryFullPath()+"cornea.vtp", GeometryFormat::VTP);

        std::shared_ptr<Part<3> > p_vegf_domain = Part<3> ::Create();
        QLength cylinder_radius(300.0*unit::microns);
        QLength cylinder_height(40.0*unit::microns);
        DimensionalChastePoint<3> pellet_base(0.0, 0.0, 1305.0);
        DimensionalChastePoint<3> pellet_centre(0.0, 0.0, 1325.0);
        p_vegf_domain->AddCylinder(cylinder_radius, cylinder_height, pellet_base);

        // Rotate the part
        std::vector<std::shared_ptr<Polygon<3> > > polygons = p_vegf_domain->GetPolygons();
        c_vector<double, 3> rotation_axis;
        rotation_axis[0] = 0.0;
        rotation_axis[1] = 1.0;
        rotation_axis[2] = 0.0;
        double rotation_angle = M_PI/25.0;
        p_domain->RotateAboutAxis(rotation_axis, rotation_angle);
        //p_vegf_domain->RotateAboutAxis(rotation_axis, rotation_angle);
        //pellet_centre.RotateAboutAxis(rotation_axis, rotation_angle);
        p_vegf_domain->Write(p_handler->GetOutputDirectoryFullPath()+"initial_vegf_domain.vtp");

        // Add the pellet domain to the cornea
        p_domain->AppendPart(p_vegf_domain);
        p_domain->AddHoleMarker(pellet_centre);
        p_domain->Write(p_handler->GetOutputDirectoryFullPath()+"merged_cornea.vtp", GeometryFormat::VTP);

        DiscreteContinuumMeshGenerator<3> mesh_generator;
        mesh_generator.SetDomain(p_domain);
        mesh_generator.SetMaxElementArea(1e6*(Qpow3(1_um))); // 1e4 for 'good' mesh
        mesh_generator.Update();

        std::shared_ptr<DiscreteContinuumMesh<3> > p_mesh = mesh_generator.GetMesh();
        MultiFormatMeshWriter<3> mesh_writer;
        mesh_writer.SetFileName(p_handler->GetOutputDirectoryFullPath()+"cornea_mesh");
        mesh_writer.SetMesh(p_mesh);
        mesh_writer.SetOutputFormat(MeshFormat::VTU);
        mesh_writer.Write();

        // Set the BCs
        for(unsigned jdx=0;jdx<polygons.size();jdx++)
        {
            p_vegf_domain->AddAttributeToPolygons("Boundary", 1.0);
        }
        std::shared_ptr<DiscreteContinuumBoundaryCondition<3> > p_boundary_condition =
                DiscreteContinuumBoundaryCondition<3>::Create();
        QConcentration boundary_concentration(1.0* unit::mole_per_metre_cubed);
        p_boundary_condition->SetValue(boundary_concentration);
        p_boundary_condition->SetType(BoundaryConditionType::POLYGON);
        p_boundary_condition->SetIsRobin(true);
        p_boundary_condition->SetDomain(p_domain);
        p_boundary_condition->SetLabel("Boundary");

        // Choose the PDE
        std::shared_ptr<CoupledVegfPelletDiffusionReactionPde<3> > p_pde = CoupledVegfPelletDiffusionReactionPde<3>::Create();
        QDiffusivity vegf_diffusivity(6.94e-11 * unit::metre_squared_per_second);
        QRate vegf_decay_rate((-0.8/3600.0) * unit::per_second);
        p_pde->SetIsotropicDiffusionConstant(vegf_diffusivity);
        p_pde->SetContinuumLinearInUTerm(vegf_decay_rate);
        QConcentration initial_vegf_concentration(3.93e-1*unit::mole_per_metre_cubed);
        p_pde->SetCurrentVegfInPellet(initial_vegf_concentration);
        p_pde->SetPelletBindingConstant(100.0);
        p_pde->SetCorneaPelletPermeability(0.002*p_pde->GetCorneaPelletPermeability());

        CoupledLumpedSystemFiniteElementSolver<3> solver;
        solver.SetGrid(p_mesh);
        solver.SetPde(p_pde);
        solver.SetFileHandler(p_handler);
        solver.SetWriteSolution(true);
        solver.AddBoundaryCondition(p_boundary_condition);
        solver.SetTargetTimeIncrement(0.1);
        solver.SetStartTime(0.0);
        solver.SetEndTime(10.0);
        solver.SetWriteIntermediateSolutions(true, 10.0);
        p_domain->Write(p_handler->GetOutputDirectoryFullPath()+"merged_cornea2.vtp", GeometryFormat::VTP);
        solver.Solve();
    }
};

#endif // TESTCOUPLEDVEGFODESYSTEM_HPP_
