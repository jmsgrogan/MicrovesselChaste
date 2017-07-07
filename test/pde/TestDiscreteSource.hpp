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

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestDiscreteSource : public CxxTest::TestSuite
{

public:

    void TestGridFunction() throw(Exception)
    {
        QLength length(100.0*unit::microns);

        // Set up the grid
        std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(length, length, DimensionalChastePoint<2>());

        std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        QLength grid_spacing(5.0*unit::microns);
        p_grid->GenerateFromPart(p_domain, grid_spacing);

        // Set up a density map
        std::shared_ptr<DensityMap<2> > p_density_map = DensityMap<2>::Create();
        p_density_map->SetGrid(p_grid);

        // Set up the discrete source
        std::vector<DimensionalChastePoint<2> > linear_consumption_points;
        linear_consumption_points.push_back(DimensionalChastePoint<2>(50.0, 50.0, 0.0, 1_um));
        std::shared_ptr<DiscreteSource<2> > p_linear_point_source = DiscreteSource<2>::Create();

        p_linear_point_source->SetLinearInUValue(1.0 * unit::per_second);
        p_linear_point_source->SetPoints(linear_consumption_points);
        p_linear_point_source->SetDensityMap(p_density_map);

        std::shared_ptr<DiscreteSource<2> > p_const_point_source = DiscreteSource<2>::Create();
        QConcentrationFlowRate consumption_rate(2.0 * unit::mole_per_metre_cubed_per_second);
        p_const_point_source->SetConstantInUValue(consumption_rate);
        std::vector<DimensionalChastePoint<2> > constant_consumption_points;
        constant_consumption_points.push_back(DimensionalChastePoint<2>(25.0, 25.0, 0.0, 1_um));
        constant_consumption_points.push_back(DimensionalChastePoint<2>(75.0, 25.0, 0.0, 1_um));
        constant_consumption_points.push_back(DimensionalChastePoint<2>(75.0, 75.0, 0.0, 1_um));
        constant_consumption_points.push_back(DimensionalChastePoint<2>(25.0, 75.0, 0.0, 1_um));
        p_const_point_source->SetPoints(constant_consumption_points);
        p_const_point_source->SetDensityMap(p_density_map);

        // Set up a function map
        FunctionMap<2> solver;
        solver.SetGrid(p_grid);

        // Get the source values at each point on the grid
        std::vector<QRate > point_rates = p_linear_point_source->GetLinearInUValues();
        std::vector<QConcentrationFlowRate > point_conc_rates = p_const_point_source->GetConstantInUValues();
        std::vector<double> solution;
        for(unsigned idx=0; idx<p_density_map->GetGridCalculator()->GetGrid()->GetNumberOfPoints(); idx++)
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
        QLength length(100.0*unit::microns);

        // Set up the grid
        std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(length, length, DimensionalChastePoint<2>());

        std::shared_ptr<DiscreteContinuumMeshGenerator<2> > p_mesh_generator = DiscreteContinuumMeshGenerator<2>::Create();
        p_mesh_generator->SetDomain(p_domain);
        p_mesh_generator->SetMaxElementArea(Qpow3(0.02*length));
        p_mesh_generator->Update();

        std::shared_ptr<DensityMap<2> > p_density_map = DensityMap<2>::Create();
        p_density_map->SetGrid(p_mesh_generator->GetMesh());

        // Set up the discrete source
        std::vector<DimensionalChastePoint<2> > linear_consumption_points;
        linear_consumption_points.push_back(DimensionalChastePoint<2>(50.0, 50.0, 0.0, 1_um));
        std::shared_ptr<DiscreteSource<2> > p_linear_point_source = DiscreteSource<2>::Create();
        p_linear_point_source->SetLinearInUValue(1.0 * unit::per_second);
        p_linear_point_source->SetPoints(linear_consumption_points);
        p_linear_point_source->SetDensityMap(p_density_map);

        std::shared_ptr<DiscreteSource<2> > p_const_point_source = DiscreteSource<2>::Create();
        QConcentrationFlowRate consumption_rate(2.0 * unit::mole_per_metre_cubed_per_second);
        p_const_point_source->SetConstantInUValue(consumption_rate);
        std::vector<DimensionalChastePoint<2> > constant_consumption_points;
        constant_consumption_points.push_back(DimensionalChastePoint<2>(25.0, 25.0, 25.0, 1_um));
        constant_consumption_points.push_back(DimensionalChastePoint<2>(75.0, 25.0, 25.0, 1_um));
        constant_consumption_points.push_back(DimensionalChastePoint<2>(75.0, 75.0, 25.0, 1_um));
        constant_consumption_points.push_back(DimensionalChastePoint<2>(25.0, 75.0, 25.0, 1_um));
        p_const_point_source->SetPoints(constant_consumption_points);
        p_const_point_source->SetDensityMap(p_density_map);

        // Set up a function map
        FunctionMap<2> solver;
        solver.SetGrid(p_mesh_generator->GetMesh());

        // Get the source values at each point on the grid
        std::vector<QRate > point_rates = p_linear_point_source->GetLinearInUValues();
        std::vector<QConcentrationFlowRate > point_conc_rates = p_const_point_source->GetConstantInUValues();
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
        BaseUnits::Instance()->SetReferenceLengthScale(1.0*unit::metres);
        BaseUnits::Instance()->SetReferenceConcentrationScale(1.0*unit::mole_per_metre_cubed);

        QLength length(100.0*unit::metres);

        // Set up the grid
        std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(length, length, DimensionalChastePoint<2>());
        std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        QLength grid_spacing(5.0*unit::metres);
        p_grid->GenerateFromPart(p_domain, grid_spacing);

        // Choose the PDE
        std::shared_ptr<DiscreteContinuumLinearEllipticPde<2> > p_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
        QDiffusivity diffusivity(0.0033 * unit::metre_squared_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);

        // Set up the discrete source
        std::vector<DimensionalChastePoint<2> > linear_consumption_points;
        linear_consumption_points.push_back(DimensionalChastePoint<2>(50.0, 50.0, 0.0, 1.0 * unit::metres));
        std::shared_ptr<DiscreteSource<2> > p_linear_point_source = DiscreteSource<2>::Create();
        p_linear_point_source->SetLinearInUValue(-1.0 * unit::per_second);
        p_linear_point_source->SetPoints(linear_consumption_points);

        std::shared_ptr<DiscreteSource<2> > p_const_point_source = DiscreteSource<2>::Create();
        QConcentrationFlowRate consumption_rate(2.0 * unit::mole_per_metre_cubed_per_second);
        p_const_point_source->SetConstantInUValue(consumption_rate);
        std::vector<DimensionalChastePoint<2> > constant_consumption_points;
        constant_consumption_points.push_back(DimensionalChastePoint<2>(25.0, 25.0, 0.0, 1.0 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<2>(75.0, 25.0, 0.0, 1.0 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<2>(75.0, 75.0, 0.0, 1.0 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<2>(25.0, 75.0, 0.0, 1.0 * unit::metres));
        p_const_point_source->SetPoints(constant_consumption_points);

        p_pde->AddDiscreteSource(p_const_point_source);
        p_pde->AddDiscreteSource(p_linear_point_source);

        std::shared_ptr<DiscreteContinuumBoundaryCondition<2> > p_boundary2 = DiscreteContinuumBoundaryCondition<2>::Create();
        p_boundary2->SetValue(3.0*unit::mole_per_metre_cubed);

        // Set up and run the solver
        SimpleLinearEllipticFiniteDifferenceSolver<2> solver;
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
        QLength length(100.0*unit::microns);

        // Set up the grid
        std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(length, length, DimensionalChastePoint<2>(0.0, 0.0, 0.0));
        std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        QLength grid_spacing(5.0*unit::microns);
        p_grid->GenerateFromPart(p_domain, grid_spacing);

        // Choose the PDE
        std::shared_ptr<MichaelisMentenSteadyStateDiffusionReactionPde<2> > p_pde =
                MichaelisMentenSteadyStateDiffusionReactionPde<2>::Create();
        QDiffusivity diffusivity(0.0033 * unit::metre_squared_per_second);

        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetMichaelisMentenThreshold(2.0 * unit::mole_per_metre_cubed);

        // Set up the discrete source
        std::vector<DimensionalChastePoint<2> > linear_consumption_points;
        linear_consumption_points.push_back(DimensionalChastePoint<2>(50.0, 50.0, 50.0, 1_um));
        std::shared_ptr<DiscreteSource<2> > p_linear_point_source = DiscreteSource<2>::Create();
        p_linear_point_source->SetLinearInUValue(-1.0 * unit::per_second);
        p_linear_point_source->SetPoints(linear_consumption_points);

        std::shared_ptr<DiscreteSource<2> > p_const_point_source = DiscreteSource<2>::Create();
        QConcentrationFlowRate consumption_rate(2.e-4 * unit::mole_per_metre_cubed_per_second);
        p_const_point_source->SetConstantInUValue(consumption_rate);
        std::vector<DimensionalChastePoint<2> > constant_consumption_points;
        constant_consumption_points.push_back(DimensionalChastePoint<2>(25.0, 25.0, 25.0, 1_um));
        constant_consumption_points.push_back(DimensionalChastePoint<2>(75.0, 25.0, 25.0, 1_um));
        constant_consumption_points.push_back(DimensionalChastePoint<2>(75.0, 75.0, 25.0, 1_um));
        constant_consumption_points.push_back(DimensionalChastePoint<2>(25.0, 75.0, 25.0, 1_um));
        p_const_point_source->SetPoints(constant_consumption_points);

        std::shared_ptr<DiscreteContinuumBoundaryCondition<2> > p_boundary2 = DiscreteContinuumBoundaryCondition<2>::Create();
        p_boundary2->SetValue(3.e-6*unit::mole_per_metre_cubed);
        p_pde->AddDiscreteSource(p_linear_point_source);
        p_pde->AddDiscreteSource(p_const_point_source);

        // Set up and run the solver
        SimpleNonLinearEllipticFiniteDifferenceSolver<2> solver;
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
