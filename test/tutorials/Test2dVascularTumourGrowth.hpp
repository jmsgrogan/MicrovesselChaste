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

#ifndef TEST2DVASCULARTUMOURGROWTH_HPP_
#define TEST2DVASCULARTUMOURGROWTH_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include "LinearSteadyStateDiffusionReactionPde.hpp"
#include "SmartPointers.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "HoneycombMeshGenerator.hpp"
#include "GeneralisedLinearSpringForce.hpp"
#include "Owen2011OxygenBasedCellCycleModel.hpp"
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
#include "ApoptoticCellKiller.hpp"
#include "VesselNetworkGenerator.hpp"
#include "VesselNetwork.hpp"
#include "CaBasedCellPopulation.hpp"
#include "AngiogenesisSolver.hpp"
#include "FlowSolver.hpp"
#include "UnitCollection.hpp"
#include "VesselNetworkCellPopulationInteractor.hpp"
#include "StalkCellMutationState.hpp"
#include "TipCellMutationState.hpp"
#include "CancerCellMutationState.hpp"
#include "MacrophageMutationState.hpp"

#include "PetscSetupAndFinalize.hpp"

class Test2dVascularTumourGrowth : public AbstractCellBasedWithTimingsTestSuite
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
        MAKE_PTR(TipCellMutationState, p_tip_state);
        MAKE_PTR(StalkCellMutationState, p_stalk_state);
        state_specific_rates[p_apoptotic_property->GetColour()] = 0.0*unit::mole_per_second;
        state_specific_rates[p_cancer_state->GetColour()] = -7800.0*unit::mole_per_second;
        state_specific_rates[p_quiescent_cancer_state->GetColour()] = -7800.0*unit::mole_per_second;
        state_specific_rates[p_tip_state->GetColour()] = -5000.0*unit::mole_per_second;
        state_specific_rates[p_stalk_state->GetColour()] = -5000.0*unit::mole_per_second;
        state_specific_rates[p_normal_cell_state->GetColour()] = -5000.0*unit::mole_per_second;
        p_cell_sink->SetStateRateMap(state_specific_rates);
        p_pde->AddDiscreteSource(p_cell_sink);

        // todo this needs to be updated so that source strength is proportional to haematocrit level
        boost::shared_ptr<CellStateDependentDiscreteSource<2> > p_cell_source = CellStateDependentDiscreteSource<2>::Create();
        std::map<unsigned, units::quantity<unit::molar_flow_rate> > state_specific_rates2;
        double segment_radius = 0.5;
        double segment_length = 1.0;
        double segment_surface_area = 2 * M_PI * segment_radius * segment_length;
        double permeability = 20000.0;
        double inter_vessel_O2_level = 5.0;
        state_specific_rates2[p_stalk_state->GetColour()] = segment_surface_area * permeability * inter_vessel_O2_level / pow(segment_length, 3.0)*unit::mole_per_second;
        state_specific_rates2[p_tip_state->GetColour()] = segment_surface_area * permeability * inter_vessel_O2_level / pow(segment_length, 3.0)*unit::mole_per_second;
        p_cell_source->SetStateRateMap(state_specific_rates2);
        p_pde->AddDiscreteSource(p_cell_source);
        return p_pde;
    }

    // todo need to check parameters in sink/source terms in here
    boost::shared_ptr<LinearSteadyStateDiffusionReactionPde<2> > GetVegfPde()
    {
        boost::shared_ptr<LinearSteadyStateDiffusionReactionPde<2> > p_pde = LinearSteadyStateDiffusionReactionPde<2>::Create();
        units::quantity<unit::diffusivity> vegf_diffusivity(60000.0 / 400.0 * unit::metre_squared_per_second);
        p_pde->SetIsotropicDiffusionConstant(vegf_diffusivity); // assume cell width is 20 microns
        p_pde->SetContinuumLinearInUTerm(-0.8*unit::per_second); //Vegf decay

        // VEGF release for normal cells and quiescent cancer cells:
        // normal cells release only when intracellular vegf reaches a certain value
        // quiescent cancer cells always release vegf
        boost::shared_ptr<CellStateDependentDiscreteSource<2> > p_normal_and_quiescent_cell_source = CellStateDependentDiscreteSource<2>::Create();

        // Set mutation specific source strengths and thresholds
        std::map<unsigned, units::quantity<unit::molar_flow_rate> > normal_and_quiescent_cell_rates;
        std::map<unsigned, units::quantity<unit::concentration> > normal_and_quiescent_cell_rate_thresholds;
        MAKE_PTR(QuiescentCancerCellMutationState, p_quiescent_cancer_state);
        MAKE_PTR(WildTypeCellMutationState, p_normal_cell_state);
        normal_and_quiescent_cell_rates[p_normal_cell_state->GetColour()] = 0.6*unit::mole_per_second;
        normal_and_quiescent_cell_rate_thresholds[p_normal_cell_state->GetColour()] = 0.27*unit::mole_per_metre_cubed;
        normal_and_quiescent_cell_rates[p_quiescent_cancer_state->GetColour()] = 0.6*unit::mole_per_second;
        normal_and_quiescent_cell_rate_thresholds[p_quiescent_cancer_state->GetColour()] = 0.0*unit::mole_per_metre_cubed;

        p_normal_and_quiescent_cell_source->SetStateRateMap(normal_and_quiescent_cell_rates);
        p_normal_and_quiescent_cell_source->SetLabelName("VEGF");
        p_normal_and_quiescent_cell_source->SetStateRateThresholdMap(normal_and_quiescent_cell_rate_thresholds);
        p_pde->AddDiscreteSource(p_normal_and_quiescent_cell_source);

        // VEGF release for cancer cells and tip cells, now there is no threshold so we use a different source
        boost::shared_ptr<CellStateDependentDiscreteSource<2> > p_other_cell_sinks = CellStateDependentDiscreteSource<2>::Create();

        std::map<unsigned, units::quantity<unit::molar_flow_rate> > other_cell_rates;
        double segment_radius = 0.5;
        double segment_length = 1.0;
        double permeability = 15;
        MAKE_PTR(TipCellMutationState, p_tip_state);
        MAKE_PTR(StalkCellMutationState, p_stalk_state);
        other_cell_rates[p_tip_state->GetColour()] = -2 * M_PI * segment_radius * segment_length * permeability / pow(segment_length, 3.0)*unit::mole_per_second; // tip cell mutation state
        other_cell_rates[p_stalk_state->GetColour()] = -2 * M_PI * segment_radius * segment_length * permeability / pow(segment_length, 3.0)*unit::mole_per_second; // stalk cell mutation state
        p_other_cell_sinks->SetStateRateMap(other_cell_rates);
        p_pde->AddDiscreteSource(p_other_cell_sinks);
        return p_pde;
    }

    boost::shared_ptr<VesselNetwork<2> > GetHexagonalNetwork(double domain_x, double domain_y)
    {
        VesselNetworkGenerator<2> network_generator;
        boost::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateHexagonalNetwork(domain_x*1.e-6*unit::metres,
                                                                                                    domain_y*1.e-6*unit::metres,
                                                                                                    7.0*1.e-6*unit::metres);

        std::vector<boost::shared_ptr<VesselNode<2> > > nodes;
        nodes.push_back(VesselNode<2>::Create(0, 0));
        nodes.push_back(VesselNode<2>::Create(1, 0));
        boost::shared_ptr<VesselSegment<2> > p_segment(VesselSegment<2>::Create(nodes[0], nodes[1]));

        double initial_vessel_radius = 10.0e-6;
        p_segment->SetRadius(initial_vessel_radius * unit::metres);
        double haematocrit = 0.45;
        p_segment->GetFlowProperties()->SetHaematocrit(haematocrit);
        double viscosity = 2e-3;
        p_segment->GetFlowProperties()->SetViscosity(viscosity * unit::poiseuille);
        p_network->SetSegmentProperties(p_segment);

        std::pair<DimensionalChastePoint<2>, DimensionalChastePoint<2> > network_extents = p_network->GetExtents();
        double y_middle = (network_extents.first[1] + network_extents.second[1]) / 2.0;
        double x_middle = (network_extents.first[0] + network_extents.second[0]) / 2.0;

        std::vector<boost::shared_ptr<Vessel<2> > >::iterator vessel_iterator;
        std::vector<boost::shared_ptr<Vessel<2> > > vessels = p_network->GetVessels();
        for (vessel_iterator = vessels.begin(); vessel_iterator != vessels.end(); vessel_iterator++)
        {
            if ((*vessel_iterator)->GetStartNode()->GetNumberOfSegments() == 1)
            {
                if ((*vessel_iterator)->GetStartNode()->rGetLocation()[1] > y_middle)
                {
                    if ((*vessel_iterator)->GetStartNode()->rGetLocation()[0] > x_middle)
                    {
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsInputNode(true);
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(3320 * unit::pascals);
                    }
                }
            }
            if ((*vessel_iterator)->GetEndNode()->GetNumberOfSegments() == 1)
            {
                if ((*vessel_iterator)->GetEndNode()->rGetLocation()[1] > y_middle)
                {
                    if ((*vessel_iterator)->GetStartNode()->rGetLocation()[0] > x_middle)
                    {
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsInputNode(true);
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(3320 * unit::pascals);
                    }
                }
            }
            if ((*vessel_iterator)->GetStartNode()->GetNumberOfSegments() == 1)
            {
                if ((*vessel_iterator)->GetStartNode()->rGetLocation()[1] <= y_middle)
                {
                    if ((*vessel_iterator)->GetStartNode()->rGetLocation()[0] < x_middle)
                    {
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsOutputNode(true);
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(2090 * unit::pascals);
                    }
                }
            }
            if ((*vessel_iterator)->GetEndNode()->GetNumberOfSegments() == 1)
            {
                if ((*vessel_iterator)->GetEndNode()->rGetLocation()[1] <= y_middle)
                {
                    if ((*vessel_iterator)->GetStartNode()->rGetLocation()[0] < x_middle)
                    {
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(2090 * unit::pascals);
                    }
                }
            }
        }
        return p_network;
    }

public:

    void TestOnLattice2dVascularTumourGrowth() throw (Exception)
    {

//
//        // set volume fractions occupied by each cell
//        boost::shared_ptr<WildTypeCellMutationState> wild_mutation_state(new WildTypeCellMutationState);
//        boost::shared_ptr<CancerCellMutationState> cancer_mutation_state(new CancerCellMutationState);
//        boost::shared_ptr<QuiescentCancerCellMutationState> quiescent_cancer_mutation_state(new QuiescentCancerCellMutationState);
//        boost::shared_ptr<StalkCellMutationState> stalk_mutation_state(new StalkCellMutationState);
//        boost::shared_ptr<TipCellMutationState> tip_mutation_state(new TipCellMutationState);
//
//
//// Move to ca rule
////        cell_population.SetVolumeFraction(wild_mutation_state, 0.6);
////        cell_population.SetVolumeFraction(cancer_mutation_state, 0.6);
////        cell_population.SetVolumeFraction(quiescent_cancer_mutation_state, 0.6);
////        cell_population.SetVolumeFraction(stalk_mutation_state, 0.4);
////        cell_population.SetVolumeFraction(tip_mutation_state, 0.4);
//
//        // Create a grid to solve PDEs on
//        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
//        p_grid->SetSpacing(1.0);
//        std::vector<unsigned> extents;
//        extents.push_back(domain_x + 1); // num_x
//        extents.push_back(domain_y + 1); // num_y
//        extents.push_back(1); // num_z
//        p_grid->SetExtents(extents);
//
//        // Create the oxygen pde solver
//        boost::shared_ptr<FiniteDifferenceSolver<2> > p_oxygen_solver = FiniteDifferenceSolver<2>::Create();
//        p_oxygen_solver->SetGrid(p_grid);
//        p_oxygen_solver->SetPde(GetOxygenPde());
//        p_oxygen_solver->SetLabel("oxygen");
//
//        // Create the vegf pde solver
//        boost::shared_ptr<FiniteDifferenceSolver<2> > p_vegf_solver = FiniteDifferenceSolver<2>::Create();
//        p_vegf_solver->SetGrid(p_grid);
//        p_vegf_solver->SetPde(GetVegfPde());
//        p_vegf_solver->SetLabel("VEGF");
//        cell_population.AddPdeHandler(p_vegf_solver);
//
//        // add angiogenesis solver to vascular tumour solver
//        boost::shared_ptr<AngiogenesisSolver<2> > p_angiogenesis_solver = AngiogenesisSolver<2>::Create();
//        p_angiogenesis_solver->SetCellPopulation(cell_population);
//        p_angiogenesis_solver->SetVesselNetwork(p_network);
//        p_vascular_tumour_solver->SetAngiogenesisSolver(p_angiogenesis_solver);
//
//        // todo currently there is an issue with the flow calculation - some blunt-ended vessels contain flow (they shouldn't)
//        // this is an angiogenesis issue, sprouts take properties (including flow props) from parent vessels
////        boost::shared_ptr<FlowSolver<2> > flow_solver = FlowSolver<2>::Create();
////        p_vascular_tumour_solver->SetFlowSolver(flow_solver);
//
//        OnLatticeSimulation<2> simulator(*(cell_population));
//
//        // Create the vascular tumour modifier which integrates with cell based Chaste
//        boost::shared_ptr<MicrovesselSimulationModifier<2> > p_vascular_tumour_modifier = MicrovesselSimulationModifier<2>::Create();
//        p_vascular_tumour_modifier->SetMicrovesselSolver(p_vascular_tumour_solver);
//
//        simulator.AddSimulationModifier(p_vascular_tumour_modifier);
//
//        // Create a Cell Concentration tracking modifier and add it to the simulation
//        MAKE_PTR(Owen2011TrackingModifier<2>, p_modifier);
//        simulator.AddSimulationModifier(p_modifier);
//
//        //Create cell killer to remove apoptotic cell from simulation
//        boost::shared_ptr<ApoptoticCellKiller<2> > apoptotic_cell_killer(new ApoptoticCellKiller<2>(cell_population));
//        simulator.AddCellKiller(apoptotic_cell_killer);
//
//        std::string resultsDirectoryName = "Test2dVascularTumourGrowth/OnLatticeLarge";
//        simulator.SetOutputDirectory(resultsDirectoryName);
//        simulator.SetSamplingTimestepMultiple(5);
//        // todo this seems to break simulations if dt is set to 1 - causes a CVode error:
//        //          *CVODE Error -27 in module CVODE function CVode: tout too close to t0 to start integration.
//        //          CVODE failed to solve system: CV_TOO_CLOSE
//        simulator.SetDt(0.5);
//        simulator.SetEndTime(200);
//        simulator.Solve();

    }
};

#endif /*TESTOWEN2011TUMOURSPHEROIDSIMULATIONS_HPP_*/
