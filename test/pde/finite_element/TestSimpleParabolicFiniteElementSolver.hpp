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

#ifndef TESTSIMPLEPARABOLICFINITEELEMENTSOLVER_HPP_
#define TESTSIMPLEPARABOLICFINITEELEMENTSOLVER_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include <boost/lexical_cast.hpp>
#include "SmartPointers.hpp"
#include "ParabolicDiffusionReactionPde.hpp"
#include "SimpleParabolicFiniteElementSolver.hpp"
#include "UblasIncludes.hpp"
#include "Part.hpp"
#include "DimensionalChastePoint.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"
#include "Owen11Parameters.hpp"
#include "BaseUnits.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestSimpleParabolicFiniteElementSolver : public CxxTest::TestSuite
{
public:

    void TestPlane()
    {
        // Set up the mesh
        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(1.0*unit::metres, 1.0*unit::metres, DimensionalChastePoint<2>(0.0, 0.0, 0.0));

        DiscreteContinuumMeshGenerator<2> mesh_generator;
        mesh_generator.SetDomain(p_domain);
        mesh_generator.SetMaxElementArea(0.01*(units::pow<3>(1.0*unit::metres)));
        mesh_generator.Update();
        boost::shared_ptr<DiscreteContinuumMesh<2> > p_mesh = mesh_generator.GetMesh();

        boost::shared_ptr<ParabolicDiffusionReactionPde<2> > p_pde =
                ParabolicDiffusionReactionPde<2>::Create();
        units::quantity<unit::diffusivity> diffusivity(1.0 * unit::metre_squared_per_second);
        units::quantity<unit::rate> decay_rate(-0.5 * unit::per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetContinuumLinearInUTerm(decay_rate);

        // BC on right plane
        vtkSmartPointer<vtkPoints> p_boundary_points = vtkSmartPointer<vtkPoints>::New();
        TetrahedralMesh<2,2>::BoundaryElementIterator surf_iter = p_mesh->GetBoundaryElementIteratorBegin();
        while (surf_iter != p_mesh->GetBoundaryElementIteratorEnd())
        {
            unsigned node_index = (*surf_iter)->GetNodeGlobalIndex(0);
            double x = p_mesh->GetNode(node_index)->GetPoint()[0];
            if (x>0.999)
            {
                p_boundary_points->InsertNextPoint(p_mesh->GetNode(node_index)->GetPoint()[0],
                        p_mesh->GetNode(node_index)->GetPoint()[1], 0.0);
            }
            surf_iter++;
        }
        boost::shared_ptr<DiscreteContinuumBoundaryCondition<2> > p_boundary_condition =
                DiscreteContinuumBoundaryCondition<2>::Create();
        units::quantity<unit::concentration> boundary_concentration(1.0* unit::mole_per_metre_cubed);
        p_boundary_condition->SetValue(boundary_concentration);
        p_boundary_condition->SetType(BoundaryConditionType::POINT);
        p_boundary_condition->SetPoints(p_boundary_points);

        boost::shared_ptr<SimpleParabolicFiniteElementSolver<2> > p_solver =
                SimpleParabolicFiniteElementSolver<2>::Create();
        p_solver->SetGrid(p_mesh);
        p_solver->SetPde(p_pde);
        p_solver->AddBoundaryCondition(p_boundary_condition);
        p_solver->SetReferenceConcentration(1.e-9*unit::mole_per_metre_cubed);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestSimpleParabolicFiniteElementSolver/Plane"));
        p_solver->SetFileHandler(p_output_file_handler);
        p_solver->SetWriteSolution(true);
        p_solver->SetTargetTimeIncrement(1.0);
        p_solver->SetStartTime(0.0);
        p_solver->SetEndTime(100.0);
        p_solver->Solve();
    }

    void Test3dKroghCylinderNetworkSurface() throw(Exception)
    {
        // Set up the vessel network
        units::quantity<unit::length> micron_length_scale = 1.e-6*unit::metres;
        BaseUnits::Instance()->SetReferenceLengthScale(micron_length_scale);
        BaseUnits::Instance()->SetReferenceTimeScale(3600.0*unit::seconds);

        units::quantity<unit::length> vessel_length = 100.0 * micron_length_scale;
        VesselNetworkGenerator<3> generator;
        DimensionalChastePoint<3> centre(vessel_length/(2.0*micron_length_scale), vessel_length/(2.0*micron_length_scale), 0.0, micron_length_scale);
        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length, centre);

        // Set up the mesh
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(vessel_length, vessel_length, vessel_length, DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        p_domain->AddVesselNetwork(p_network, true);
        boost::shared_ptr<DiscreteContinuumMeshGenerator<3, 3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3, 3>::Create();
        p_mesh_generator->SetDomain(p_domain);
        p_mesh_generator->SetMaxElementArea(500.0*units::pow<3>(micron_length_scale));
        p_mesh_generator->Update();

        // Choose the PDE
        boost::shared_ptr<ParabolicDiffusionReactionPde<3> > p_pde = ParabolicDiffusionReactionPde<3>::Create();
        units::quantity<unit::diffusivity> diffusivity(Owen11Parameters::mpVegfDiffusivity->GetValue());
        units::quantity<unit::rate> consumption_rate(-Owen11Parameters::mpVegfDecayRate->GetValue("User"));
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetContinuumLinearInUTerm(consumption_rate);
        p_pde->SetReferenceConcentration(1.e-9*unit::mole_per_metre_cubed);

        // Choose the Boundary conditions
        boost::shared_ptr<DiscreteContinuumBoundaryCondition<3> > p_vessel_ox_boundary_condition = DiscreteContinuumBoundaryCondition<3>::Create();
        units::quantity<unit::concentration> boundary_concentration(300.e-9*unit::mole_per_metre_cubed);
        p_vessel_ox_boundary_condition->SetValue(boundary_concentration);
        p_vessel_ox_boundary_condition->SetType(BoundaryConditionType::VESSEL_VOLUME);
        p_vessel_ox_boundary_condition->SetSource(BoundaryConditionSource::PRESCRIBED);
        p_vessel_ox_boundary_condition->SetNetwork(p_network);

        // Set up and run the solver
        SimpleParabolicFiniteElementSolver<3> solver;
        solver.SetGrid(p_mesh_generator->GetMesh());
        solver.SetPde(p_pde);
        solver.AddBoundaryCondition(p_vessel_ox_boundary_condition);
        solver.SetReferenceConcentration(1.e-9*unit::mole_per_metre_cubed);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestSimpleParabolicFiniteElementSolver/KroghCylinder3dSurface", false));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        p_solver->SetTargetTimeIncrement(1.0);
        p_solver->SetStartTime(0.0);
        p_solver->SetEndTime(100.0);
        solver.Solve();
    }
};

#endif /*TESTSimpleParabolicFiniteElementSolver_HPP_*/
