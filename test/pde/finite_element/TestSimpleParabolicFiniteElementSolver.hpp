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

#include "SimpleParabolicFiniteElementSolver.hpp"
#include "SmartPointers.hpp"
#include "ParabolicDiffusionReactionPde.hpp"
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
private:

    /**
     * Hill 1928 Fixed bounary on both ends, diffusion only.
     */
    double Solve1DParabolic(double x, double D, double t)
    {
        double b = 0.5;
        double c = 1.0;
        for(unsigned idx=0; idx<10; idx++)
        {
            double n = 2*idx + 1;
            c = c - (4.0/M_PI)*(1.0/n)*std::exp(-n*n*D*M_PI*M_PI*t/(4.0*b*b))*std::sin(n*M_PI*x/(2.0*b));
        }
        return c;
    }

public:

    void TestPlane()
    {
        // Set up the mesh
        BaseUnits::Instance()->SetReferenceLengthScale(1.0*unit::metres);
        BaseUnits::Instance()->SetReferenceConcentrationScale(1.0*unit::mole_per_metre_cubed);
        BaseUnits::Instance()->SetReferenceTimeScale(1.0*unit::seconds);

        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(1.0*unit::metres, 1.0*unit::metres, DimensionalChastePoint<2>(0.0, 0.0, 0.0));

        DiscreteContinuumMeshGenerator<2> mesh_generator;
        mesh_generator.SetDomain(p_domain);
        mesh_generator.SetMaxElementArea(0.005*(units::pow<3>(1.0*unit::metres)));
        mesh_generator.Update();
        boost::shared_ptr<DiscreteContinuumMesh<2> > p_mesh = mesh_generator.GetMesh();

        boost::shared_ptr<ParabolicDiffusionReactionPde<2> > p_pde =
                ParabolicDiffusionReactionPde<2>::Create();
        units::quantity<unit::diffusivity> diffusivity(1.0 * unit::metre_squared_per_second);
        units::quantity<unit::rate> decay_rate(-0.5 * unit::per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        //p_pde->SetContinuumLinearInUTerm(decay_rate);

        // BC on right plane
        vtkSmartPointer<vtkPoints> p_boundary_points = vtkSmartPointer<vtkPoints>::New();
        TetrahedralMesh<2,2>::BoundaryElementIterator surf_iter = p_mesh->GetBoundaryElementIteratorBegin();
        while (surf_iter != p_mesh->GetBoundaryElementIteratorEnd())
        {
            unsigned node_index = (*surf_iter)->GetNodeGlobalIndex(0);
            double x = p_mesh->GetNode(node_index)->GetPoint()[0];
            if (x>0.999 or x<0.001)
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
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestSimpleParabolicFiniteElementSolver/Plane"));
        p_solver->SetFileHandler(p_output_file_handler);
        p_solver->SetWriteSolution(true);
        p_solver->SetTargetTimeIncrement(0.0005);
        p_solver->SetStartTime(0.0);
        p_solver->SetEndTime(0.5);
        p_solver->SetWriteIntermediateSolutions(true, 100);
        p_solver->Solve();

        // Test the intermediate solutions
        vtkSmartPointer<vtkPoints> p_sample_points = vtkSmartPointer<vtkPoints>::New();
        for(unsigned idx=0; idx<11; idx++)
        {
            p_sample_points->InsertNextPoint(double(idx)*0.1, 0.5, 0.0);
        }

        std::vector<std::pair<std::vector<double>, double> > intermediate_solutions =
                p_solver->rGetIntermediateSolutions();
        double diff_nondim = diffusivity*(1.0*unit::seconds)/(1.0*unit::metres*1.0*unit::metres);
        for(unsigned idx=0; idx<intermediate_solutions.size();idx++)
        {
            double time = intermediate_solutions[idx].second;
            p_solver->UpdateSolution(intermediate_solutions[idx].first);
            std::vector<units::quantity<unit::concentration> > solution = p_solver->GetConcentrations(p_sample_points);
            for(unsigned jdx=0; jdx<11; jdx++)
            {
                units::quantity<unit::length> x = double(jdx)*0.1*unit::metres;
                double x_nondim = x/(1.0*unit::metres);
                double c_analytical_nondim = Solve1DParabolic(x_nondim, diff_nondim, time);
                double c_numerical_nondim = solution[jdx]/boundary_concentration;
                TS_ASSERT_DELTA(c_analytical_nondim, c_numerical_nondim, 1.e-2)
            }
        }
    }

    void Test3dKroghCylinderNetworkSurface() throw(Exception)
    {
        // Set up the vessel network
        units::quantity<unit::length> micron_length_scale = 1.e-6*unit::metres;
        BaseUnits::Instance()->SetReferenceLengthScale(micron_length_scale);
        BaseUnits::Instance()->SetReferenceTimeScale(3600.0*unit::seconds);
        BaseUnits::Instance()->SetReferenceConcentrationScale(1.e-9*unit::mole_per_metre_cubed);

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

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestSimpleParabolicFiniteElementSolver/KroghCylinder3dSurface", false));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.SetTargetTimeIncrement(0.001);
        solver.SetStartTime(0.0);
        solver.SetEndTime(0.1);
        solver.SetWriteIntermediateSolutions(true, 10);
        solver.Solve();
    }
};

#endif /*TESTSimpleParabolicFiniteElementSolver_HPP_*/
