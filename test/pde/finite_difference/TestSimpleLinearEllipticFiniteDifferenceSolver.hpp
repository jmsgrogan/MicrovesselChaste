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

#ifndef TESTSIMPLELINEARELLIPTICFINITEDIFFERENCESOLVER_HPP_
#define TESTSIMPLELINEARELLIPTICFINITEDIFFERENCESOLVER_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "SmartPointers.hpp"
#include "Part.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "OutputFileHandler.hpp"
#include "RegularGrid.hpp"
#include "BaseUnits.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestSimpleLinearEllipticFiniteDifferenceSolver : public CxxTest::TestSuite
{

public:

    void TestRectangleDomain() throw(Exception)
    {
        // Set up the grid
        BaseUnits::Instance()->SetReferenceLengthScale(1.0*unit::metres);
        BaseUnits::Instance()->SetReferenceConcentrationScale(1.0*unit::mole_per_metre_cubed);
        BaseUnits::Instance()->SetReferenceTimeScale(1.0*unit::seconds);

        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(5.0*unit::metres,
                               5.0*unit::metres,
                               DimensionalChastePoint<2>(0.0, 0.0, 0.0));
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        p_grid->GenerateFromPart(p_domain, 1.0*unit::metres);

        // Choose the PDE
        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<2> > p_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
        units::quantity<unit::diffusivity> diffusivity(1.0* unit::metre_squared_per_second);
        units::quantity<unit::concentration_flow_rate> consumption_rate(-0.05 * unit::mole_per_metre_cubed_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetContinuumConstantInUTerm(consumption_rate);

        // Prescribe a value on the domain's left boundary
        boost::shared_ptr<DiscreteContinuumBoundaryCondition<2> > p_boundary_condition = DiscreteContinuumBoundaryCondition<2>::Create();
        units::quantity<unit::concentration> boundary_concentration(1.0* unit::mole_per_metre_cubed);
        p_boundary_condition->SetValue(boundary_concentration);
        p_boundary_condition->SetType(BoundaryConditionType::POINT);
        vtkSmartPointer<vtkPoints> p_boundary_points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkPoints> p_points = p_grid->GetLocations();
        for(unsigned idx=0; idx<p_points->GetNumberOfPoints(); idx++)
        {
            if(p_points->GetPoint(idx)[0]==0.0)
            {
                p_boundary_points->InsertNextPoint(p_points->GetPoint(idx));
            }
        }
        p_boundary_condition->SetPoints(p_boundary_points);

        // Set up and run the solver
        SimpleLinearEllipticFiniteDifferenceSolver<2> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        solver.AddBoundaryCondition(p_boundary_condition);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestSimpleLinearEllipticFiniteDifferenceSolver/RectangleDomain", true));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.Solve();

        std::vector<units::quantity<unit::concentration> > solution = solver.GetConcentrations();

        // Analytical c = k*x*x/(2*D) - k*x*w/D+c_0
        for(unsigned idx=0; idx<6; idx++)
        {
            units::quantity<unit::length> x = double(idx)*1.0*unit::metres;
            units::quantity<unit::length> w = 5.0*unit::metres;
            units::quantity<unit::concentration> c = -consumption_rate*x*x/(2.0*diffusivity)-
                    x*-consumption_rate*w/diffusivity + boundary_concentration;
            double norm_analytical = c/(1.0* unit::mole_per_metre_cubed);
            double norm_numerical = solution[idx]/(1.0* unit::mole_per_metre_cubed);
            TS_ASSERT_DELTA(norm_analytical, norm_numerical, 1.e-6)
        }
    }

    void TestCuboidalDomain() throw(Exception)
    {
        // Set up the grid
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(200.0*1.e-6*unit::metres,
                            10.0*1.e-6*unit::metres,
                            10.0*1.e-6*unit::metres,
                            DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        p_grid->GenerateFromPart(p_domain, 10.0*1.e-6*unit::metres);

        // Choose the PDE
        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<3> > p_pde = DiscreteContinuumLinearEllipticPde<3>::Create();
        units::quantity<unit::diffusivity> diffusivity(1.e-3 * unit::metre_squared_per_second);
        units::quantity<unit::concentration_flow_rate> consumption_rate(-20.0 * unit::mole_per_metre_cubed_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetContinuumConstantInUTerm(consumption_rate);

        // Prescribe a value on the domain boundaries
        boost::shared_ptr<DiscreteContinuumBoundaryCondition<3> > p_boundary_condition = DiscreteContinuumBoundaryCondition<3>::Create();
        units::quantity<unit::concentration> boundary_concentration(1.e-3 * unit::mole_per_metre_cubed);
        p_boundary_condition->SetValue(boundary_concentration);
        p_boundary_condition->SetType(BoundaryConditionType::POINT);
        vtkSmartPointer<vtkPoints> p_boundary_points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkPoints> p_points = p_grid->GetLocations();
        for(unsigned idx=0; idx<p_points->GetNumberOfPoints(); idx++)
        {
            if(p_points->GetPoint(idx)[0]==0.0)
            {
                p_boundary_points->InsertNextPoint(p_points->GetPoint(idx));
            }
        }
        p_boundary_condition->SetPoints(p_boundary_points);
        // Set up and run the solver

        SimpleLinearEllipticFiniteDifferenceSolver<3> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        solver.AddBoundaryCondition(p_boundary_condition);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestSimpleLinearEllipticFiniteDifferenceSolver/CuboidalDomain", true));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.Solve();

        std::vector<units::quantity<unit::concentration> > solution = solver.GetConcentrations();

        // Analytical c = k*x*x/(2*D) - k*x*w/D+c_0
        for(unsigned idx=0; idx<6; idx++)
        {
            units::quantity<unit::length> x = double(idx)*10.0e-6*unit::metres;
            units::quantity<unit::length> w = 200.0e-6*unit::metres;
            units::quantity<unit::concentration> c = -consumption_rate*x*x/(2.0*diffusivity)-
                    x*-consumption_rate*w/diffusivity + boundary_concentration;
            double norm_analytical = c/(1.0* unit::mole_per_metre_cubed);
            double norm_numerical = solution[idx]/(1.0* unit::mole_per_metre_cubed);
            TS_ASSERT_DELTA(norm_analytical, norm_numerical, 1.e-6)
        }
    }

    void TestWithVesselBoundaryConditions() throw(Exception)
    {
        // Set up the vessel network
        units::quantity<unit::length> vessel_length = 100.0 * 1.e-6 * unit::metres;
        VesselNetworkGenerator<3> generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length,
                                                                                        DimensionalChastePoint<3>(0.0, 0.0, 0.0));

        // Set up the grid
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(vessel_length, vessel_length, vessel_length, DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        p_grid->GenerateFromPart(p_domain, 10.0*1.e-6*unit::metres);

        // Choose the PDE
        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<3> > p_pde = DiscreteContinuumLinearEllipticPde<3>::Create();
        units::quantity<unit::diffusivity> diffusivity(0.0033 * unit::metre_squared_per_second);
        units::quantity<unit::rate> consumption_rate(-2.e6 * unit::per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetContinuumLinearInUTerm(consumption_rate);

        // Set up the boundary condition
        boost::shared_ptr<DiscreteContinuumBoundaryCondition<3> > p_vessel_boundary_condition = DiscreteContinuumBoundaryCondition<3>::Create();
        units::quantity<unit::concentration> boundary_concentration(1.e-3 * unit::mole_per_metre_cubed);
        p_vessel_boundary_condition->SetValue(boundary_concentration);
        p_vessel_boundary_condition->SetType(BoundaryConditionType::VESSEL_LINE);
        p_vessel_boundary_condition->SetSource(BoundaryConditionSource::PRESCRIBED);

        // Set up and run the solver
        SimpleLinearEllipticFiniteDifferenceSolver<3> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        solver.AddBoundaryCondition(p_vessel_boundary_condition);
        solver.SetVesselNetwork(p_network);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestSimpleLinearEllipticFiniteDifferenceSolver/WithVessels", true));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.Solve();
    }
};

#endif /*TESTSIMPLELINEARELLIPTICFINITEDIFFERENCESOLVER_HPP_*/
