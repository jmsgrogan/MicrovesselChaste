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

#ifndef TESTDISCRETESOURCE_HPP_
#define TESTDISCRETESOURCE_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "SmartPointers.hpp"
#include "Part.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"
#include "SimpleNonLinearEllipticFiniteDifferenceSolver.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "OutputFileHandler.hpp"
#include "RegularGrid.hpp"
#include "DiscreteSource.hpp"
#include "DimensionalChastePoint.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "FunctionMap.hpp"
#include "SimpleLinearEllipticFiniteElementSolver.hpp"
#include "VtkMeshWriter.hpp"
#include "GridCalculator.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestDiscreteSource : public CxxTest::TestSuite
{

public:

    void TestGridFunction() throw(Exception)
    {
        units::quantity<unit::length> length(100.0*unit::microns);

        // Set up the grid
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(length, length, length, DimensionalChastePoint<3>());
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        units::quantity<unit::length> grid_spacing(5.0*unit::microns);
        p_grid->GenerateFromPart(p_domain, grid_spacing);

        boost::shared_ptr<GridCalculator<3> > p_grid_calc = GridCalculator<3>::Create();
        p_grid_calc->SetGrid(p_grid);

        // Set up the discrete source
        std::vector<DimensionalChastePoint<3> > linear_consumption_points;
        linear_consumption_points.push_back(DimensionalChastePoint<3>(50.0, 50.0, 50.0, 1.e-6 * unit::metres));
        boost::shared_ptr<DiscreteSource<3> > p_linear_point_source = DiscreteSource<3>::Create();

        p_linear_point_source->SetLinearInUValue(1.e3 * unit::per_second);
        p_linear_point_source->SetPoints(linear_consumption_points);
        p_linear_point_source->SetGridCalculator(p_grid_calc);

        boost::shared_ptr<DiscreteSource<3> > p_const_point_source = DiscreteSource<3>::Create();
        units::quantity<unit::concentration_flow_rate> consumption_rate(-2.e-4 * unit::mole_per_metre_cubed_per_second);
        p_const_point_source->SetConstantInUValue(consumption_rate);
        std::vector<DimensionalChastePoint<3> > constant_consumption_points;
        constant_consumption_points.push_back(DimensionalChastePoint<3>(25.0, 25.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(75.0, 25.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(75.0, 75.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(25.0, 75.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(25.0, 25.0, 75.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(75.0, 25.0, 75.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(75.0, 75.0, 75.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(25.0, 75.0, 75.0, 1.e-6 * unit::metres));
        p_const_point_source->SetPoints(constant_consumption_points);
        p_const_point_source->SetGridCalculator(p_grid_calc);

        // Set up a function map
        FunctionMap<3> solver;
        solver.SetGrid(p_grid);

        // Get the source values at each point on the grid
        std::vector<units::quantity<unit::rate> > point_rates = p_linear_point_source->GetLinearInUValues();
        std::vector<units::quantity<unit::concentration_flow_rate> > point_conc_rates = p_const_point_source->GetConstantInUValues();
        std::vector<double> solution;
        for(unsigned idx=0; idx<p_grid_calc->GetGrid()->GetNumberOfLocations(); idx++)
        {
            solution.push_back(double(point_rates[idx].value() + point_conc_rates[idx].value()));
        }

        solver.UpdateSolution(solution);
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestDiscreteSource/TestGridFunction", true));
        solver.SetFileHandler(p_output_file_handler);
        solver.Write();
    }

    void TestMeshFunction() throw(Exception)
    {
        units::quantity<unit::length> length(100.0*unit::microns);

        // Set up the grid
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(length, length, length, DimensionalChastePoint<3>());

        boost::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_domain);
        p_mesh_generator->SetMaxElementArea(units::pow<3>(0.1*length));
        p_mesh_generator->Update();

        boost::shared_ptr<GridCalculator<3> > p_grid_calc = GridCalculator<3>::Create();
        p_grid_calc->SetGrid(p_mesh_generator->GetMesh());

        // Set up the discrete source
        std::vector<DimensionalChastePoint<3> > linear_consumption_points;
        linear_consumption_points.push_back(DimensionalChastePoint<3>(50.0, 50.0, 50.0, 1.e-6 * unit::metres));
        boost::shared_ptr<DiscreteSource<3> > p_linear_point_source = DiscreteSource<3>::Create();
        p_linear_point_source->SetLinearInUValue(1.e3 * unit::per_second);
        p_linear_point_source->SetPoints(linear_consumption_points);
        p_linear_point_source->SetGridCalculator(p_grid_calc);

        boost::shared_ptr<DiscreteSource<3> > p_const_point_source = DiscreteSource<3>::Create();
        units::quantity<unit::concentration_flow_rate> consumption_rate(-2.e-4 * unit::mole_per_metre_cubed_per_second);
        p_const_point_source->SetConstantInUValue(consumption_rate);
        std::vector<DimensionalChastePoint<3> > constant_consumption_points;
        constant_consumption_points.push_back(DimensionalChastePoint<3>(25.0, 25.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(75.0, 25.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(75.0, 75.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(25.0, 75.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(25.0, 25.0, 75.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(75.0, 25.0, 75.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(75.0, 75.0, 75.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(25.0, 75.0, 75.0, 1.e-6 * unit::metres));
        p_const_point_source->SetPoints(constant_consumption_points);
        p_const_point_source->SetGridCalculator(p_grid_calc);

        // Set up a function map
        SimpleLinearEllipticFiniteElementSolver<3> solver;
        solver.SetGrid(p_mesh_generator->GetMesh());

        // Get the source values at each point on the grid
        std::vector<units::quantity<unit::rate> > point_rates = p_linear_point_source->GetLinearInUValues();
        std::vector<units::quantity<unit::concentration_flow_rate> > point_conc_rates = p_const_point_source->GetConstantInUValues();
        std::vector<double> solution;
        for(unsigned idx=0; idx<point_conc_rates.size(); idx++)
        {
            solution.push_back(double(point_rates[idx].value() + point_conc_rates[idx].value()));
        }
        solver.UpdateElementSolution(solution);
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestDiscreteSource/TestMeshFunction", false));
        solver.SetFileHandler(p_output_file_handler);
        solver.Write();
    }

    void TestLinearGridPde() throw(Exception)
    {
        units::quantity<unit::length> length(100.0*unit::microns);

        // Set up the grid
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(length, length, length, DimensionalChastePoint<3>());
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        units::quantity<unit::length> grid_spacing(5.0*unit::microns);
        p_grid->GenerateFromPart(p_domain, grid_spacing);

        // Choose the PDE
        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<3> > p_pde = DiscreteContinuumLinearEllipticPde<3>::Create();
        units::quantity<unit::diffusivity> diffusivity(0.0033 * unit::metre_squared_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);

        // Set up the discrete source
        std::vector<DimensionalChastePoint<3> > linear_consumption_points;
        linear_consumption_points.push_back(DimensionalChastePoint<3>(50.0, 50.0, 50.0, 1.e-6 * unit::metres));
        boost::shared_ptr<DiscreteSource<3> > p_linear_point_source = DiscreteSource<3>::Create();
        p_linear_point_source->SetLinearInUValue(-1.0 * unit::per_second);
        p_linear_point_source->SetPoints(linear_consumption_points);

        boost::shared_ptr<DiscreteSource<3> > p_const_point_source = DiscreteSource<3>::Create();
        units::quantity<unit::concentration_flow_rate> consumption_rate(2.e-4 * unit::mole_per_metre_cubed_per_second);
        p_const_point_source->SetConstantInUValue(consumption_rate);
        std::vector<DimensionalChastePoint<3> > constant_consumption_points;
        constant_consumption_points.push_back(DimensionalChastePoint<3>(25.0, 25.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(75.0, 25.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(75.0, 75.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(25.0, 75.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(25.0, 25.0, 75.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(75.0, 25.0, 75.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(75.0, 75.0, 75.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(25.0, 75.0, 75.0, 1.e-6 * unit::metres));
        p_const_point_source->SetPoints(constant_consumption_points);

        p_pde->AddDiscreteSource(p_const_point_source);
        p_pde->AddDiscreteSource(p_linear_point_source);

        boost::shared_ptr<DiscreteContinuumBoundaryCondition<3> > p_boundary2 = DiscreteContinuumBoundaryCondition<3>::Create();
        p_boundary2->SetValue(3.e-6*unit::mole_per_metre_cubed);

        // Set up and run the solver
        SimpleLinearEllipticFiniteDifferenceSolver<3> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        solver.AddBoundaryCondition(p_boundary2);
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestDiscreteSource/TestLinearGridPde", false));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.Solve();
    }

    void TestNonLinearGridPde() throw(Exception)
    {
        units::quantity<unit::length> length(100.0*unit::microns);

        // Set up the grid
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(length, length, length, DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        units::quantity<unit::length> grid_spacing(5.0*unit::microns);
        p_grid->GenerateFromPart(p_domain, grid_spacing);

        // Choose the PDE
        boost::shared_ptr<MichaelisMentenSteadyStateDiffusionReactionPde<3> > p_pde = MichaelisMentenSteadyStateDiffusionReactionPde<3>::Create();
        units::quantity<unit::diffusivity> diffusivity(0.0033 * unit::metre_squared_per_second);

        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetMichaelisMentenThreshold(2.0 * unit::mole_per_metre_cubed);

        // Set up the discrete source
        std::vector<DimensionalChastePoint<3> > linear_consumption_points;
        linear_consumption_points.push_back(DimensionalChastePoint<3>(50.0, 50.0, 50.0, 1.e-6 * unit::metres));
        boost::shared_ptr<DiscreteSource<3> > p_linear_point_source = DiscreteSource<3>::Create();
        p_linear_point_source->SetLinearInUValue(-1.0 * unit::per_second);
        p_linear_point_source->SetPoints(linear_consumption_points);

        boost::shared_ptr<DiscreteSource<3> > p_const_point_source = DiscreteSource<3>::Create();
        units::quantity<unit::concentration_flow_rate> consumption_rate(2.e-4 * unit::mole_per_metre_cubed_per_second);
        p_const_point_source->SetConstantInUValue(consumption_rate);
        std::vector<DimensionalChastePoint<3> > constant_consumption_points;
        constant_consumption_points.push_back(DimensionalChastePoint<3>(25.0, 25.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(75.0, 25.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(75.0, 75.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(25.0, 75.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(25.0, 25.0, 75.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(75.0, 25.0, 75.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(75.0, 75.0, 75.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<3>(25.0, 75.0, 75.0, 1.e-6 * unit::metres));
        p_const_point_source->SetPoints(constant_consumption_points);

        boost::shared_ptr<DiscreteContinuumBoundaryCondition<3> > p_boundary2 = DiscreteContinuumBoundaryCondition<3>::Create();
        p_boundary2->SetValue(3.e-6*unit::mole_per_metre_cubed);

        p_pde->AddDiscreteSource(p_linear_point_source);
        p_pde->AddDiscreteSource(p_const_point_source);

        // Set up and run the solver
        SimpleNonLinearEllipticFiniteDifferenceSolver<3> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        solver.AddBoundaryCondition(p_boundary2);
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestDiscreteSource/TestNonLinearGridPde", false));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.Solve();
    }
};

#endif /*TESTDISCRETESOURCE_HPP_*/
