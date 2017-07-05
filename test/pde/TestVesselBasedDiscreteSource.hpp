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
#include "Secomb04Parameters.hpp"
#include "GenericParameters.hpp"
#include "Owen11Parameters.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestVesselBasedDiscreteSource : public CxxTest::TestSuite
{

public:

    void TestGridFunction() throw(Exception)
    {

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler,
                ("TestVesselBasedDiscreteSource/TestGridFunction", true));

        QLength vessel_length(100.0*unit::microns);
        QLength reference_length(1.0*unit::microns);
        VesselNetworkGenerator<2> generator;
        std::shared_ptr<VesselNetwork<2> > p_network =
                generator.GenerateSingleVessel(vessel_length, DimensionalChastePoint<2>());
        p_network->GetVessels()[0]->GetFlowProperties()->SetHaematocrit(0.4);

        // Set up the grid
        std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(vessel_length, vessel_length, DimensionalChastePoint<2>());
        DimensionalChastePoint<2> translation_vector(-vessel_length/(2.0*reference_length),
                                                     0.0, 0.0, reference_length);
        p_domain->Translate(translation_vector);

        std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        QLength spacing(10.0*unit::microns);
        p_grid->GenerateFromPart(p_domain, spacing);

        // Set up a density map
        std::shared_ptr<DensityMap<2> > p_density_map = DensityMap<2>::Create();
        p_density_map->SetVesselNetwork(p_network);
        p_density_map->SetGrid(p_grid);

        // Set up the discrete source
        std::shared_ptr<VesselBasedDiscreteSource<2> > p_vessel_source_lin = VesselBasedDiscreteSource<2>::Create();
        p_vessel_source_lin->SetLinearInUValue(1.0*unit::per_second);
        p_vessel_source_lin->SetDensityMap(p_density_map);

        std::shared_ptr<VesselBasedDiscreteSource<2> > p_vessel_source_const = VesselBasedDiscreteSource<2>::Create();
        p_vessel_source_const->SetConstantInUValue(2.0* unit::mole_per_metre_cubed_per_second);
        p_vessel_source_const->SetDensityMap(p_density_map);

        // Set up a function map
        FunctionMap<2> solver;
        solver.SetGrid(p_grid);

        // Get the source values at each point on the grid
        std::vector<QRate > point_rates = p_vessel_source_lin->GetLinearInUValues();
        std::vector<QConcentrationFlowRate > point_conc_rates = p_vessel_source_const->GetConstantInUValues();
        std::vector<double> solution;
        for(unsigned idx=0; idx<p_density_map->GetGridCalculator()->GetGrid()->GetNumberOfPoints(); idx++)
        {
            solution.push_back(double(point_rates[idx].value() + point_conc_rates[idx].value()));
        }

        solver.UpdateSolution(solution);
        solver.SetFileHandler(p_output_file_handler);
        solver.Write();

        p_network->Write(p_output_file_handler->GetOutputDirectoryFullPath()+"network.vtp");
    }

    void TestMeshFunction() throw(Exception)
    {
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestVesselBasedDiscreteSource/TestMeshFunction"));

        QLength vessel_length(100.0*unit::microns);
        QLength reference_length(1.0*unit::microns);
        VesselNetworkGenerator<2> generator;
        std::shared_ptr<VesselNetwork<2> > p_network = generator.GenerateSingleVessel(vessel_length, DimensionalChastePoint<2>());
        p_network->GetVessels()[0]->GetFlowProperties()->SetHaematocrit(0.4);

        // Set up the grid
        std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(vessel_length, vessel_length, DimensionalChastePoint<2>());
        DimensionalChastePoint<2> translation_vector(-vessel_length/(2.0*reference_length),
                                                     0.0, 0.0, reference_length);
        p_domain->Translate(translation_vector);

        // Set up the grid
        std::shared_ptr<DiscreteContinuumMeshGenerator<2> > p_mesh_generator = DiscreteContinuumMeshGenerator<2>::Create();
        p_mesh_generator->SetDomain(p_domain);
        p_mesh_generator->SetMaxElementArea(Qpow3(0.02*vessel_length));
        p_mesh_generator->Update();

        // Set up a density map
        std::shared_ptr<DensityMap<2> > p_density_map = DensityMap<2>::Create();
        p_density_map->SetVesselNetwork(p_network);
        p_density_map->SetGrid(p_mesh_generator->GetMesh());

        // Set up the discrete source
        std::shared_ptr<VesselBasedDiscreteSource<2> > p_linear_point_source = VesselBasedDiscreteSource<2>::Create();
        p_linear_point_source->SetLinearInUValue(1.0 * unit::per_second);
        p_linear_point_source->SetDensityMap(p_density_map);

        std::shared_ptr<VesselBasedDiscreteSource<2> > p_const_point_source = VesselBasedDiscreteSource<2>::Create();
        QConcentrationFlowRate consumption_rate(2.0 * unit::mole_per_metre_cubed_per_second);
        p_const_point_source->SetConstantInUValue(consumption_rate);
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
        solver.SetFileHandler(p_output_file_handler);
        solver.Write();

        p_network->Write(p_output_file_handler->GetOutputDirectoryFullPath()+"network.vtp");
    }

    void TestSimpleLinearEllipticFiniteDifferenceSolver() throw(Exception)
    {
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler,
                ("TestVesselBasedDiscreteSource/TestSimpleLinearEllipticFiniteDifferenceSolver"));

        // Set up the vessel network
        QLength vessel_length(100.0*unit::microns);
        QLength reference_length(1.0*unit::microns);
        VesselNetworkGenerator<2> generator;
        std::shared_ptr<VesselNetwork<2> > p_network = generator.GenerateSingleVessel(vessel_length,
                DimensionalChastePoint<2>());

        p_network->GetVessels()[0]->GetFlowProperties()->SetHaematocrit(0.45);

        // Set up the grid
        std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(vessel_length, vessel_length, DimensionalChastePoint<2>());
        DimensionalChastePoint<2> translation_vector(-vessel_length/(2.0*reference_length),
                                                     0.0, 0.0, reference_length);
        p_domain->Translate(translation_vector);
        std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        QLength spacing(10.0*unit::microns);
        p_grid->GenerateFromPart(p_domain, spacing);

        // Choose the PDE
        std::shared_ptr<DiscreteContinuumLinearEllipticPde<2> > p_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
        QDiffusivity diffusivity(0.0033 * unit::metre_squared_per_second);
        QConcentrationFlowRate consumption_rate(-2.e0 * unit::mole_per_metre_cubed_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetContinuumConstantInUTerm(consumption_rate);

        // Set up the discrete source
        std::shared_ptr<VesselBasedDiscreteSource<2> > p_vessel_source = VesselBasedDiscreteSource<2>::Create();
        QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
                GenericParameters::mpGasConcentrationAtStp->GetValue("User");
        QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
                Owen11Parameters::mpReferencePartialPressure->GetValue("User");
        p_vessel_source->SetReferenceConcentration(vessel_oxygen_concentration);
        p_pde->AddDiscreteSource(p_vessel_source);

        // Set up and run the solver
        SimpleLinearEllipticFiniteDifferenceSolver<2> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        solver.SetVesselNetwork(p_network);

        solver.SetFileHandler(p_output_file_handler);
        p_network->Write(p_output_file_handler->GetOutputDirectoryFullPath()+"network.vtp");
        solver.SetWriteSolution(true);
        solver.Solve();
    }

    void TestSimpleLinearEllipticFiniteElementSolver() throw(Exception)
    {
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler,
                ("TestVesselBasedDiscreteSource/TestSimpleLinearEllipticFiniteElementSolver"));


        // Set up the vessel network
        QLength vessel_length(100.0*unit::microns);
        QLength reference_length(1.0*unit::microns);
        VesselNetworkGenerator<2> generator;
        std::shared_ptr<VesselNetwork<2> > p_network =
                generator.GenerateSingleVessel(vessel_length, DimensionalChastePoint<2>());
        p_network->GetVessels()[0]->GetFlowProperties()->SetHaematocrit(0.45);

        // Set up the grid
        std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(vessel_length, vessel_length, DimensionalChastePoint<2>());
        DimensionalChastePoint<2> translation_vector(-vessel_length/(2.0*reference_length),
                                                     0.0, 0.0, reference_length);
        p_domain->Translate(translation_vector);

        std::shared_ptr<DiscreteContinuumMeshGenerator<2> > p_mesh_generator =
                DiscreteContinuumMeshGenerator<2>::Create();
        p_mesh_generator->SetDomain(p_domain);
        QLength spacing(10.0*unit::microns);
        p_mesh_generator->SetMaxElementArea(Qpow3(0.02*vessel_length));
        p_mesh_generator->Update();

        // Choose the PDE
        std::shared_ptr<DiscreteContinuumLinearEllipticPde<2> > p_pde =
                DiscreteContinuumLinearEllipticPde<2>::Create();
        QDiffusivity diffusivity(0.0033 * unit::metre_squared_per_second);
        QConcentrationFlowRate consumption_rate(-2.0 * unit::mole_per_metre_cubed_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetContinuumConstantInUTerm(consumption_rate);

        // Set up the discrete source
        // Set up the discrete source
        std::shared_ptr<VesselBasedDiscreteSource<2> > p_vessel_source = VesselBasedDiscreteSource<2>::Create();
        QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
                GenericParameters::mpGasConcentrationAtStp->GetValue("User");
        QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
                Owen11Parameters::mpReferencePartialPressure->GetValue("User");
        p_vessel_source->SetReferenceConcentration(vessel_oxygen_concentration);
        p_pde->AddDiscreteSource(p_vessel_source);

        // Set up and run the solver
        SimpleLinearEllipticFiniteElementSolver<2> solver;
        solver.SetGrid(p_mesh_generator->GetMesh());
        solver.SetPde(p_pde);
        solver.SetVesselNetwork(p_network);
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.Solve();
    }
};

#endif /*TESTVESSELBASEDDISCRETESOURCE_HPP_*/
