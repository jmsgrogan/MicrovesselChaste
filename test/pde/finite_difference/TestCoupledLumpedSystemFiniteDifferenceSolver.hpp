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

#ifndef TESTCOUPLEDLUMPEDSYSTEMFINITEDIFFERENCESOLVER_HPP_
#define TESTCOUPLEDLUMPEDSYSTEMFINITEDIFFERENCESOLVER_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include <math.h>
#include <boost/lexical_cast.hpp>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "Part.hpp"
#include "UnitCollection.hpp"
#include "CoupledVegfPelletDiffusionReactionPde.hpp"
#include "CoupledLumpedSystemFiniteDifferenceSolver.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "CoupledLumpedSystemFiniteElementSolver.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestCoupledLumpedSystemFiniteDifferenceSolver : public AbstractCellBasedWithTimingsTestSuite
{

    /**
     * Diffusion only in 1D with Robin Boundary on the left
     * From Crank Diffusion text. Only suitable until field
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
        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(10.0*unit::metres, 100.0*unit::metres, DimensionalChastePoint<2>(0.0, 0.0, 0.0));

        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        p_grid->GenerateFromPart(p_domain, 5.0*unit::metres);

        // Choose the PDE
        boost::shared_ptr<CoupledVegfPelletDiffusionReactionPde<2> > p_pde = CoupledVegfPelletDiffusionReactionPde<2>::Create();
        units::quantity<unit::diffusivity> vegf_diffusivity(1.0 * unit::metre_squared_per_second);
        p_pde->SetIsotropicDiffusionConstant(vegf_diffusivity);

        units::quantity<unit::concentration> initial_vegf_concentration(1.0*unit::mole_per_metre_cubed);
        p_pde->SetCurrentVegfInPellet(initial_vegf_concentration);
        p_pde->SetCorneaPelletPermeability(1.0*unit::metre_per_second);
        p_pde->SetPelletFreeDecayRate(0.0*unit::per_second);
        p_pde->SetPelletBindingConstant(1.0);
        p_pde->SetPelletSurfaceArea(1.0*unit::metres_squared);
        p_pde->SetPelletVolume(1.0*unit::metres*1.0*unit::metres*1.0*unit::metres);

        CoupledLumpedSystemFiniteDifferenceSolver<2> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestCoupledLumpedSystemFiniteDifferenceSolver/PlaneSlowRelease"));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.SetTargetTimeIncrement(0.1);
        solver.SetUseCoupling(false);
        solver.SetStartTime(0.0);
        solver.SetEndTime(100.0);
        solver.SetWriteIntermediateSolutions(true, 100);
        solver.Solve();

        // Test the intermediate solutions
        vtkSmartPointer<vtkPoints> p_sample_points = vtkSmartPointer<vtkPoints>::New();
        for(unsigned idx=0; idx<11; idx++)
        {
            p_sample_points->InsertNextPoint(5.0, 100.0-idx*10.0, 0.0);
        }

        std::vector<std::pair<std::vector<double>, double> > intermediate_solutions = solver.rGetIntermediateSolutions();
        double diff_nondim = vegf_diffusivity*(1.0*unit::seconds)/(1.0*unit::metres*1.0*unit::metres);
        for(unsigned idx=0; idx<intermediate_solutions.size();idx++)
        {
            double time = intermediate_solutions[idx].second;
            if(time>10.0)
            {
                solver.UpdateSolution(intermediate_solutions[idx].first);
                std::vector<units::quantity<unit::concentration> > solution = solver.GetConcentrations(p_sample_points);
                for(unsigned jdx=0; jdx<11; jdx++)
                {
                    units::quantity<unit::length> x = double(jdx)*10.0*unit::metres;
                    double x_nondim = x/(1.0*unit::metres);
                    double c_analytical_nondim = SolveRobinBoundary(x_nondim, time, diff_nondim, 1.0, 1.0);
                    double c_numerical_nondim = solution[jdx]/initial_vegf_concentration;
                    TS_ASSERT_DELTA(c_analytical_nondim, c_numerical_nondim, 2.e-2)
                }
            }
        }
    }

    void TestPlane() throw(Exception)
    {
        BaseUnits::Instance()->SetReferenceLengthScale(1.e-6*unit::metres);
        BaseUnits::Instance()->SetReferenceConcentrationScale(1.e-9*unit::mole_per_metre_cubed);
        BaseUnits::Instance()->SetReferenceTimeScale(3600.0*unit::seconds);

        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(2000e-6*unit::metres, 1000e-6*unit::metres, DimensionalChastePoint<2>(0.0, 0.0, 0.0));
        p_domain->AddAttributeToEdgeIfFound(DimensionalChastePoint<2>(1000.0, 1000.0, 0, 1e-6*unit::metres), "Top Boundary", 1.0);
        TS_ASSERT(p_domain->EdgeHasAttribute(DimensionalChastePoint<2>(1000.0, 1000.0, 0, 1e-6*unit::metres), "Top Boundary"));

        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        p_grid->GenerateFromPart(p_domain, 50.0e-6*unit::metres);

        // Choose the PDE
        boost::shared_ptr<CoupledVegfPelletDiffusionReactionPde<2> > p_pde = CoupledVegfPelletDiffusionReactionPde<2>::Create();
        units::quantity<unit::diffusivity> vegf_diffusivity(6.94e-11 * unit::metre_squared_per_second);
        units::quantity<unit::rate> vegf_decay_rate((-0.8/3600.0) * unit::per_second);
        p_pde->SetIsotropicDiffusionConstant(vegf_diffusivity);
        p_pde->SetContinuumLinearInUTerm(vegf_decay_rate);
        units::quantity<unit::concentration> initial_vegf_concentration(3.93e-1*unit::mole_per_metre_cubed);
        p_pde->SetCurrentVegfInPellet(initial_vegf_concentration);
        p_pde->SetPelletBindingConstant(100.0);
        p_pde->SetPelletSurfaceArea(2000e-6*unit::metres*1e-6*unit::metres);
        p_pde->SetCorneaPelletPermeability(0.002*p_pde->GetCorneaPelletPermeability());

        double target_time = 0.01;
        double end_time = 24.0;

        // Solve the finite difference problem
        CoupledLumpedSystemFiniteDifferenceSolver<2> fd_solver;
        fd_solver.SetGrid(p_grid);
        fd_solver.SetPde(p_pde);
        MAKE_PTR_ARGS(OutputFileHandler, p_fd_output_file_handler, ("TestCoupledLumpedSystemFiniteDifferenceSolver/Plane"));
        fd_solver.SetFileHandler(p_fd_output_file_handler);
        fd_solver.SetWriteSolution(true);
        fd_solver.SetTargetTimeIncrement(target_time);
        fd_solver.SetStartTime(0.0);
        fd_solver.SetUseCoupling(true);
        fd_solver.SetEndTime(end_time);
        fd_solver.SetWriteIntermediateSolutions(true, 20);
        fd_solver.Solve();

        // Solve the finite element problem
        DiscreteContinuumMeshGenerator<2> mesh_generator;
        mesh_generator.SetDomain(p_domain);
        mesh_generator.SetMaxElementArea(2e3*(units::pow<3>(1.e-6*unit::metres)));
        mesh_generator.Update();
        boost::shared_ptr<DiscreteContinuumMesh<2> > p_mesh = mesh_generator.GetMesh();

        // Set up robin BC on top plane
        boost::shared_ptr<DiscreteContinuumBoundaryCondition<2> > p_boundary_condition =
                DiscreteContinuumBoundaryCondition<2>::Create();
        units::quantity<unit::concentration> boundary_concentration(1.0* unit::mole_per_metre_cubed);
        p_boundary_condition->SetValue(boundary_concentration);
        p_boundary_condition->SetType(BoundaryConditionType::EDGE);
        p_boundary_condition->SetIsRobin(true);
        p_boundary_condition->SetLabel("Top Boundary");
        p_boundary_condition->SetDomain(p_domain);

        // Reset vegf in pellet!
        p_pde->SetCurrentVegfInPellet(initial_vegf_concentration);

        CoupledLumpedSystemFiniteElementSolver<2> solver;
        solver.SetGrid(p_mesh);
        solver.SetPde(p_pde);
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestCoupledLumpedSystemFiniteElementSolver/Plane"));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.AddBoundaryCondition(p_boundary_condition);
        solver.SetTargetTimeIncrement(target_time);
        solver.SetStartTime(0.0);
        solver.SetUseCoupling(true);
        solver.SetEndTime(end_time);
        solver.SetWriteIntermediateSolutions(true, 20);
        solver.Solve();
    }
};

#endif /*TESTCOUPLEDLUMPEDSYSTEMFINITEDIFFERENCESOLVER_HPP_*/
