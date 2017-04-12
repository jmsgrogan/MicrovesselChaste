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

#ifndef TESTVESSELBASEDDISCRETESOURCE_HPP_
#define TESTVESSELBASEDDISCRETESOURCE_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include "SmartPointers.hpp"
#include "Part.hpp"
#include "FunctionMap.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"
#include "SimpleLinearEllipticFiniteElementSolver.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "OutputFileHandler.hpp"
#include "RegularGrid.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "VesselBasedDiscreteSource.hpp"
#include "DensityMap.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestVesselBasedDiscreteSource : public CxxTest::TestSuite
{

public:

    void TestGridFunction() throw(Exception)
    {
        units::quantity<unit::length> vessel_length(100.0*unit::microns);
        units::quantity<unit::length> reference_length(1.0*unit::microns);
        VesselNetworkGenerator<2> generator;
        boost::shared_ptr<VesselNetwork<2> > p_network = generator.GenerateSingleVessel(vessel_length, DimensionalChastePoint<2>());

        // Set up the grid
        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(vessel_length, vessel_length, DimensionalChastePoint<2>());
        DimensionalChastePoint<2> translation_vector(-vessel_length/(2.0*reference_length),
                                                     -vessel_length/(2.0*reference_length), 0.0, reference_length);
        p_domain->Translate(translation_vector);
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        units::quantity<unit::length> spacing(10.0*unit::microns);
        p_grid->GenerateFromPart(p_domain, spacing);

        // Set up a density map
        boost::shared_ptr<DensityMap<2> > p_density_map = DensityMap<2>::Create();
        p_density_map->SetVesselNetwork(p_network);
        p_density_map->SetGrid(p_grid);

        // Set up the discrete source
        boost::shared_ptr<VesselBasedDiscreteSource<2> > p_vessel_source_lin = VesselBasedDiscreteSource<2>::Create();
        p_vessel_source_lin->SetLinearInUValue(1.0*unit::per_second);
        p_vessel_source_lin->SetDensityMap(p_density_map);

        boost::shared_ptr<VesselBasedDiscreteSource<2> > p_vessel_source_const = VesselBasedDiscreteSource<2>::Create();
        p_vessel_source_const->SetConstantInUValue(2.0* unit::mole_per_metre_cubed_per_second);
        p_vessel_source_const->SetDensityMap(p_density_map);

        // Set up a function map
        FunctionMap<2> solver;
        solver.SetGrid(p_grid);

        // Get the source values at each point on the grid
        std::vector<units::quantity<unit::rate> > point_rates = p_vessel_source_lin->GetLinearInUValues();
        std::vector<units::quantity<unit::concentration_flow_rate> > point_conc_rates = p_vessel_source_const->GetConstantInUValues();
        std::vector<double> solution;
        for(unsigned idx=0; idx<p_density_map->GetGridCalculator()->GetGrid()->GetNumberOfPoints(); idx++)
        {
            solution.push_back(double(point_rates[idx].value() + point_conc_rates[idx].value()));
        }

        solver.UpdateSolution(solution);
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestVesselBasedDiscreteSource/TestGridFunction", true));
        solver.SetFileHandler(p_output_file_handler);
        solver.Write();
    }

    void xTestMeshFunction() throw(Exception)
    {
        units::quantity<unit::length> vessel_length(100.0*unit::microns);
        units::quantity<unit::length> reference_length(1.0*unit::microns);
        VesselNetworkGenerator<2> generator;
        boost::shared_ptr<VesselNetwork<2> > p_network = generator.GenerateSingleVessel(vessel_length, DimensionalChastePoint<2>());

        // Set up the grid
        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(vessel_length, vessel_length, DimensionalChastePoint<2>());
        DimensionalChastePoint<2> translation_vector(-vessel_length/(2.0*reference_length),
                                                     -vessel_length/(2.0*reference_length), 0.0, reference_length);
        p_domain->Translate(translation_vector);

        // Set up the grid
        boost::shared_ptr<DiscreteContinuumMeshGenerator<2> > p_mesh_generator = DiscreteContinuumMeshGenerator<2>::Create();
        p_mesh_generator->SetDomain(p_domain);
        p_mesh_generator->SetMaxElementArea(units::pow<3>(0.02*vessel_length));
        p_mesh_generator->Update();

        // Set up a density map
        boost::shared_ptr<DensityMap<2> > p_density_map = DensityMap<2>::Create();
        p_density_map->SetVesselNetwork(p_network);
        p_density_map->SetGrid(p_mesh_generator->GetMesh());

        // Set up the discrete source
        std::vector<DimensionalChastePoint<2> > linear_consumption_points;
        linear_consumption_points.push_back(DimensionalChastePoint<2>(50.0, 50.0, 0.0, 1.e-6 * unit::metres));
        boost::shared_ptr<DiscreteSource<2> > p_linear_point_source = DiscreteSource<2>::Create();
        p_linear_point_source->SetLinearInUValue(1.0 * unit::per_second);
        p_linear_point_source->SetPoints(linear_consumption_points);
        p_linear_point_source->SetDensityMap(p_density_map);

        boost::shared_ptr<DiscreteSource<2> > p_const_point_source = DiscreteSource<2>::Create();
        units::quantity<unit::concentration_flow_rate> consumption_rate(2.0 * unit::mole_per_metre_cubed_per_second);
        p_const_point_source->SetConstantInUValue(consumption_rate);
        std::vector<DimensionalChastePoint<2> > constant_consumption_points;
        constant_consumption_points.push_back(DimensionalChastePoint<2>(25.0, 25.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<2>(75.0, 25.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<2>(75.0, 75.0, 25.0, 1.e-6 * unit::metres));
        constant_consumption_points.push_back(DimensionalChastePoint<2>(25.0, 75.0, 25.0, 1.e-6 * unit::metres));
        p_const_point_source->SetPoints(constant_consumption_points);
        p_const_point_source->SetDensityMap(p_density_map);

        // Set up a function map
        FunctionMap<2> solver;
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
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestVesselBasedDiscreteSource/TestMeshFunction", false));
        solver.SetFileHandler(p_output_file_handler);
        solver.Write();
    }

    void xTestSimpleLinearEllipticFiniteDifferenceSolver() throw(Exception)
    {
        // Set up the vessel network
        units::quantity<unit::length> vessel_length(100.0*unit::microns);
        units::quantity<unit::length> reference_length(1.0*unit::microns);
        VesselNetworkGenerator<3> generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length, DimensionalChastePoint<3>());

        // Set up the grid
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(vessel_length, vessel_length, vessel_length, DimensionalChastePoint<3>());
        DimensionalChastePoint<3> translation_vector(-vessel_length/(2.0*reference_length),
                                                     -vessel_length/(2.0*reference_length), 0.0, reference_length);
        p_domain->Translate(translation_vector);
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        units::quantity<unit::length> spacing(10.0*unit::microns);
        p_grid->GenerateFromPart(p_domain, spacing);

        // Choose the PDE
        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<3> > p_pde = DiscreteContinuumLinearEllipticPde<3>::Create();
        units::quantity<unit::diffusivity> diffusivity(0.0033 * unit::metre_squared_per_second);
        units::quantity<unit::concentration_flow_rate> consumption_rate(-2.e-7 * unit::mole_per_metre_cubed_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetContinuumConstantInUTerm(consumption_rate);

        // Set up the discrete source
        boost::shared_ptr<VesselBasedDiscreteSource<3> > p_vessel_source_lin = VesselBasedDiscreteSource<3>::Create();
        p_vessel_source_lin->SetLinearInUValue(-1.e3*unit::per_second);
        boost::shared_ptr<VesselBasedDiscreteSource<3> > p_vessel_source_const = VesselBasedDiscreteSource<3>::Create();
        p_vessel_source_const->SetConstantInUValue(40.e-7* unit::mole_per_metre_cubed_per_second);

        p_pde->AddDiscreteSource(p_vessel_source_lin);
        p_pde->AddDiscreteSource(p_vessel_source_const);

        boost::shared_ptr<DiscreteContinuumBoundaryCondition<3> > p_boundary2 = DiscreteContinuumBoundaryCondition<3>::Create();
        p_boundary2->SetValue(1.0*unit::mole_per_metre_cubed);

        // Set up and run the solver
        SimpleLinearEllipticFiniteDifferenceSolver<3> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        solver.SetVesselNetwork(p_network);
        solver.AddBoundaryCondition(p_boundary2);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestVesselBasedDiscreteSource/TestSimpleLinearEllipticFiniteDifferenceSolver"));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.Solve();
    }

    void xTestSimpleLinearEllipticFiniteElementSolver() throw(Exception)
    {
        // Set up the vessel network
        units::quantity<unit::length> vessel_length(100.0*unit::microns);
        units::quantity<unit::length> reference_length(1.0*unit::microns);
        VesselNetworkGenerator<3> generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length, DimensionalChastePoint<3>());

        // Set up the grid
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(vessel_length, vessel_length, vessel_length, DimensionalChastePoint<3>());
        DimensionalChastePoint<3> translation_vector(-vessel_length/(2.0*reference_length),
                                                     -vessel_length/(2.0*reference_length), 0.0, reference_length);
        p_domain->Translate(translation_vector);

        boost::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_domain);
        units::quantity<unit::length> spacing(10.0*unit::microns);
        p_mesh_generator->SetMaxElementArea(spacing*spacing*spacing);
        p_mesh_generator->Update();

        // Choose the PDE
        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<3> > p_pde = DiscreteContinuumLinearEllipticPde<3>::Create();
        units::quantity<unit::diffusivity> diffusivity(0.0033 * unit::metre_squared_per_second);
        units::quantity<unit::concentration_flow_rate> consumption_rate(-2.e-7 * unit::mole_per_metre_cubed_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetContinuumConstantInUTerm(consumption_rate);

        // Set up the discrete source
        boost::shared_ptr<VesselBasedDiscreteSource<3> > p_vessel_source_lin = VesselBasedDiscreteSource<3>::Create();
        p_vessel_source_lin->SetLinearInUValue(-1.e3*unit::per_second);
        boost::shared_ptr<VesselBasedDiscreteSource<3> > p_vessel_source_const = VesselBasedDiscreteSource<3>::Create();
        p_vessel_source_const->SetConstantInUValue(40.e-7* unit::mole_per_metre_cubed_per_second);

        p_pde->AddDiscreteSource(p_vessel_source_lin);
        p_pde->AddDiscreteSource(p_vessel_source_const);

        boost::shared_ptr<DiscreteContinuumBoundaryCondition<3> > p_boundary2 = DiscreteContinuumBoundaryCondition<3>::Create();
        p_boundary2->SetValue(1.0*unit::mole_per_metre_cubed);

        // Set up and run the solver
        SimpleLinearEllipticFiniteElementSolver<3> solver;
        solver.SetGrid(p_mesh_generator->GetMesh());
        solver.SetPde(p_pde);
        solver.SetVesselNetwork(p_network);
        solver.AddBoundaryCondition(p_boundary2);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestVesselBasedDiscreteSource/TestSimpleLinearEllipticFiniteElementSolver"));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.Solve();
    }
};

#endif /*TESTVESSELBASEDDISCRETESOURCE_HPP_*/
