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

#ifndef TEST2DTUMOURSPHEROID_HPP_
#define TEST2DTUMOURSPHEROID_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>

#include "LinearSteadyStateDiffusionReactionPde.hpp"
#include "SmartPointers.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "HoneycombMeshGenerator.hpp"
#include "GeneralisedLinearSpringForce.hpp"
#include "Owen2011OxygenBasedCellCycleModel.hpp"
#include "CellProliferativePhasesCountWriter.hpp"
#include "CellProliferativeTypesWriter.hpp"
#include "CellProliferativePhasesWriter.hpp"
#include "CellMutationStatesWriter.hpp"
#include "CellLabelWriter.hpp"
#include "CellProliferativePhasesCountWriter.hpp"
#include "OffLatticeSimulation.hpp"
#include "VoronoiDataWriter.hpp"
#include "Owen2011TrackingModifier.hpp"
#include "QuiescentCancerCellMutationState.hpp"
#include "CancerCellMutationState.hpp"
#include "ApoptoticCellProperty.hpp"
#include "WildTypeCellMutationState.hpp"
#include "CancerCellMutationState.hpp"
#include "TipCellMutationState.hpp"
#include "StalkCellMutationState.hpp"
#include "StemCellProliferativeType.hpp"
#include "RegularGrid.hpp"
#include "FiniteDifferenceSolver.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "CellStateDependentDiscreteSource.hpp"
#include "DiscreteSource.hpp"
#include "MicrovesselSimulationModifier.hpp"
#include "CaBasedCellPopulation.hpp"
#include "PottsMeshGenerator.hpp"
#include "PottsMesh.hpp"
#include "OnLatticeSimulation.hpp"
#include "LQRadiotherapyCellKiller.hpp"
#include "VesselNetworkGenerator.hpp"

#include "PetscSetupAndFinalize.hpp"

class Test2dTumourSpheroid : public AbstractCellBasedWithTimingsTestSuite
{
    boost::shared_ptr<LinearSteadyStateDiffusionReactionPde<2> > GetOxygenPde()
        {
            boost::shared_ptr<LinearSteadyStateDiffusionReactionPde<2> > p_pde = LinearSteadyStateDiffusionReactionPde<2>::Create();
            units::quantity<unit::diffusivity> oxygen_diffusivity(8700000.0/400.0 * unit::metre_squared_per_second);
            p_pde->SetIsotropicDiffusionConstant(oxygen_diffusivity); // assume cell width is 20 microns

            // Add a cell state specific discrete source for cells consuming oxygen
            boost::shared_ptr<CellStateDependentDiscreteSource<2> > p_cell_sink = CellStateDependentDiscreteSource<2>::Create();
            std::map<unsigned, units::quantity<unit::molar_flow_rate> > state_specific_rates;

            MAKE_PTR(WildTypeCellMutationState, p_normal_cell_state);
            MAKE_PTR(CancerCellMutationState, p_cancer_state);
            MAKE_PTR(QuiescentCancerCellMutationState, p_quiescent_cancer_state);
            MAKE_PTR(ApoptoticCellProperty, p_apoptotic_property);

            state_specific_rates[p_apoptotic_property->GetColour()] = 0.0*unit::mole_per_second;
            state_specific_rates[p_cancer_state->GetColour()] = -780.0*unit::mole_per_second;
            state_specific_rates[p_quiescent_cancer_state->GetColour()] = -780.0*unit::mole_per_second;
            state_specific_rates[p_normal_cell_state->GetColour()] = -500.0*unit::mole_per_second;

            p_cell_sink->SetStateRateMap(state_specific_rates);
            p_pde->AddDiscreteSource(p_cell_sink);

            boost::shared_ptr<CellStateDependentDiscreteSource<2> > p_cell_source = CellStateDependentDiscreteSource<2>::Create();
            std::map<unsigned, units::quantity<unit::molar_flow_rate> > state_specific_rates2;
            p_cell_source->SetStateRateMap(state_specific_rates2);
            p_pde->AddDiscreteSource(p_cell_source);

            return p_pde;
        }

public:

    void TestOnLattice2dTumourSpheroid() throw (Exception)
    {
        // Set up simulation domain
        double domain_x = 40.0;
        double domain_y = 40.0;
        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(domain_x*1.e-6*unit::metres, domain_y*1.e-6*unit::metres, DimensionalChastePoint<2>(0.0, 0.0));

        // Create a lattice for the cell population
        double spacing = 1.0;
        unsigned num_x = unsigned(p_domain->GetBoundingBox()[1] / spacing);
        unsigned num_y = unsigned(p_domain->GetBoundingBox()[3] / spacing);
        PottsMeshGenerator<2> generator(num_x, 0, 0, num_y, 0, 0);
        PottsMesh<2>* p_mesh = generator.GetMesh();
        p_mesh->Scale(spacing, spacing);

        VesselNetworkGenerator<2> network_generator;

        double length = spacing * num_y; // full domain in y direction
        unsigned divisions = num_y - 1; // divide the vessel to coincide with grid
        unsigned alignment_axis = 1; // pointing y direction
        boost::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateSingleVessel(length*1.e-6*unit::metres, DimensionalChastePoint<2>(20.0, 0.0),
                                                                                            divisions, alignment_axis);

        // Get initital tumour cell region
        double radius = 3.0;
        boost::shared_ptr<Part<2> > p_sub_domain = Part<2>::Create();
        boost::shared_ptr<Polygon> circle = p_sub_domain->AddCircle(radius*1.e-6*unit::metres, DimensionalChastePoint<2>(20.0, 20.0));
        std::vector<unsigned> location_indices;
        for (unsigned ind = 0; ind < p_mesh->GetNumNodes(); ind++)
        {
            if (p_sub_domain->IsPointInPart(p_mesh->GetNode(ind)->rGetLocation()))
            {
                location_indices.push_back(ind);
            }
        }

        // Make the cells and set up the state and type
        std::vector<CellPtr> cells;
        MAKE_PTR(CancerCellMutationState, p_state);
        MAKE_PTR(StemCellProliferativeType, p_stem_type);

        double oxygen_concentration = 30.0;
        for (unsigned i = 0; i < location_indices.size(); i++)
        {
            // Assign an oxygen based cell cycle model, which requires a dimension to be set.
            Owen2011OxygenBasedCellCycleModel* const p_model = new Owen2011OxygenBasedCellCycleModel;
            p_model->SetDimension(2);
            CellPtr p_cell(new Cell(p_state, p_model));
            p_cell->SetCellProliferativeType(p_stem_type);
            p_cell->SetApoptosisTime(30);
            cells.push_back(p_cell);
            p_cell->GetCellData()->SetItem("oxygen", oxygen_concentration);
        }

        // Create cell population
        CaBasedCellPopulation<2> cell_population(*p_mesh, cells, location_indices);
        cell_population.SetOutputResultsForChasteVisualizer(false);
        cell_population.AddCellWriter<CellLabelWriter>();
        cell_population.AddCellPopulationCountWriter<CellProliferativePhasesCountWriter>();
        cell_population.AddCellWriter<CellMutationStatesWriter>();
        cell_population.AddCellWriter<CellProliferativeTypesWriter>();
        cell_population.AddCellWriter<CellProliferativePhasesWriter>();

        // Create a grid to solve PDEs on
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        p_grid->SetSpacing(1.0 * 1.e-6 * unit::metres);
        std::vector<unsigned> extents;
        extents.push_back(41); // num_x
        extents.push_back(41); // num_y
        extents.push_back(1); // num_z
        p_grid->SetExtents(extents);

        // Create the oxygen pde solver
        boost::shared_ptr<FiniteDifferenceSolver<2> > p_oxygen_solver = FiniteDifferenceSolver<2>::Create();
        p_oxygen_solver->SetGrid(p_grid);
        p_oxygen_solver->SetPde(GetOxygenPde());
        p_oxygen_solver->SetLabel("oxygen");
        p_grid->SetCellPopulation(cell_population);
        p_grid->SetVesselNetwork(p_network);

        boost::shared_ptr<DiscreteContinuumBoundaryCondition<2> > p_vessel_ox_boundary_condition = DiscreteContinuumBoundaryCondition<2>::Create();
        units::quantity<unit::concentration> boundary_concentration(40.0 * unit::mole_per_metre_cubed);
        p_vessel_ox_boundary_condition->SetValue(boundary_concentration);
        p_vessel_ox_boundary_condition->SetType(BoundaryConditionType::VESSEL_LINE);
        p_vessel_ox_boundary_condition->SetSource(BoundaryConditionSource::PRESCRIBED);

        // Add a dirichlet boundary condition for oxygen on the outer walls of the domain
//        boost::shared_ptr<DiscreteContinuumBoundaryCondition<2> > p_domain_ox_boundary_condition = DiscreteContinuumBoundaryCondition<2>::Create();
//        p_domain_ox_boundary_condition->SetValue(oxygen_concentration);
//        p_oxygen_solver->AddBoundaryCondition(p_domain_ox_boundary_condition);
        p_oxygen_solver->AddBoundaryCondition(p_vessel_ox_boundary_condition);

        // Create the vascular tumour solver, which manages all pde solves
        boost::shared_ptr<MicrovesselSolver<2> > p_vascular_tumour_solver = MicrovesselSolver<2>::Create();
        p_vascular_tumour_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
        p_vascular_tumour_solver->SetOutputFrequency(10);
        p_vascular_tumour_solver->SetVesselNetwork(p_network);

        OnLatticeSimulation<2> simulator(cell_population);

        // Create the vascular tumour modifier which integrates with cell based Chaste
        boost::shared_ptr<MicrovesselSimulationModifier<2> > p_vascular_tumour_modifier = MicrovesselSimulationModifier<2>::Create();
        p_vascular_tumour_modifier->SetMicrovesselSolver(p_vascular_tumour_solver);
        simulator.AddSimulationModifier(p_vascular_tumour_modifier);

        // Create a Cell Concentration tracking modifier and add it to the simulation
        MAKE_PTR(Owen2011TrackingModifier<2>, p_modifier);
        simulator.AddSimulationModifier(p_modifier);

        std::string resultsDirectoryName = "Test2dStaticVasculatureTumourSpheroid/OnLattice";
        simulator.SetOutputDirectory(resultsDirectoryName);
        simulator.SetSamplingTimestepMultiple(10);
        simulator.SetDt(1);
        simulator.SetEndTime(100);
        MAKE_PTR_ARGS(LQRadiotherapyCellKiller<2>, p_killer, (&cell_population));

        std::vector<double> rad_times;
        rad_times.push_back(700.0);
        p_killer->SetTimeOfRadiation(rad_times);
        p_killer->SetDoseInjected(5.0);
        simulator.AddCellKiller(p_killer);

        simulator.Solve();
    }
};

#endif /*TEST2DTUMOURSPHEROID_HPP_*/
