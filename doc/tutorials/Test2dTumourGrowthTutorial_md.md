---
layout: page-full-width 
title: Test2d Tumour Growth Tutorial
---
This tutorial is automatically generated from the file test/tutorials//Test2dTumourGrowthTutorial.hpp.
Note that the code is given in full at the bottom of the page.



```cpp
#include <cxxtest/TestSuite.h>
#include <vector>
#include "DiscreteContinuumLinearEllipticPde.hpp"
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
//    boost::shared_ptr<DiscreteContinuumLinearEllipticPde<2> > GetOxygenPde()
//    {
//        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<2> > p_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
//
//        units::quantity<unit::diffusivity> oxygen_diffusivity(8700000.0/400.0 * unit::metre_squared_per_second);
//        p_pde->SetIsotropicDiffusionConstant(oxygen_diffusivity); // assume cell width is 20 microns
//
//        // Add a cell state specific discrete source for cells consuming oxygen
//        boost::shared_ptr<CellStateDependentDiscreteSource<2> > p_cell_sink = CellStateDependentDiscreteSource<2>::Create();
//        std::map<unsigned, units::quantity<unit::molar_flow_rate> > state_specific_rates;
//        MAKE_PTR(WildTypeCellMutationState, p_normal_cell_state);
//        MAKE_PTR(CancerCellMutationState, p_cancer_state);
//        MAKE_PTR(QuiescentCancerCellMutationState, p_quiescent_cancer_state);
//        MAKE_PTR(ApoptoticCellProperty, p_apoptotic_property);
//        MAKE_PTR(TipCellMutationState, p_tip_state);
//        MAKE_PTR(StalkCellMutationState, p_stalk_state);
//        state_specific_rates[p_apoptotic_property->GetColour()] = 0.0*unit::mole_per_second;
//        state_specific_rates[p_cancer_state->GetColour()] = -7800.0*unit::mole_per_second;
//        state_specific_rates[p_quiescent_cancer_state->GetColour()] = -7800.0*unit::mole_per_second;
//        state_specific_rates[p_tip_state->GetColour()] = -5000.0*unit::mole_per_second;
//        state_specific_rates[p_stalk_state->GetColour()] = -5000.0*unit::mole_per_second;
//        state_specific_rates[p_normal_cell_state->GetColour()] = -5000.0*unit::mole_per_second;
//        p_cell_sink->SetStateRateMap(state_specific_rates);
//        p_pde->AddDiscreteSource(p_cell_sink);
//
//        // todo this needs to be updated so that source strength is proportional to haematocrit level
//        boost::shared_ptr<CellStateDependentDiscreteSource<2> > p_cell_source = CellStateDependentDiscreteSource<2>::Create();
//        std::map<unsigned, units::quantity<unit::molar_flow_rate> > state_specific_rates2;
//        double segment_radius = 0.5;
//        double segment_length = 1.0;
//        double segment_surface_area = 2 * M_PI * segment_radius * segment_length;
//        double permeability = 20000.0;
//        double inter_vessel_O2_level = 5.0;
//        state_specific_rates2[p_stalk_state->GetColour()] = segment_surface_area * permeability * inter_vessel_O2_level / pow(segment_length, 3.0)*unit::mole_per_second;
//        state_specific_rates2[p_tip_state->GetColour()] = segment_surface_area * permeability * inter_vessel_O2_level / pow(segment_length, 3.0)*unit::mole_per_second;
//        p_cell_source->SetStateRateMap(state_specific_rates2);
//        p_pde->AddDiscreteSource(p_cell_source);
//        return p_pde;
//    }
//
//    // todo need to check parameters in sink/source terms in here
//    boost::shared_ptr<DiscreteContinuumLinearEllipticPde<2> > GetVegfPde()
//    {
//        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<2> > p_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
//        units::quantity<unit::diffusivity> vegf_diffusivity(60000.0 / 400.0 * unit::metre_squared_per_second);
//        p_pde->SetIsotropicDiffusionConstant(vegf_diffusivity); // assume cell width is 20 microns
//        p_pde->SetContinuumLinearInUTerm(-0.8*unit::per_second); //Vegf decay
//
//        // VEGF release for normal cells and quiescent cancer cells:
//        // normal cells release only when intracellular vegf reaches a certain value
//        // quiescent cancer cells always release vegf
//        boost::shared_ptr<CellStateDependentDiscreteSource<2> > p_normal_and_quiescent_cell_source = CellStateDependentDiscreteSource<2>::Create();
//
//        // Set mutation specific source strengths and thresholds
//        std::map<unsigned, units::quantity<unit::molar_flow_rate> > normal_and_quiescent_cell_rates;
//        std::map<unsigned, units::quantity<unit::concentration> > normal_and_quiescent_cell_rate_thresholds;
//        MAKE_PTR(QuiescentCancerCellMutationState, p_quiescent_cancer_state);
//        MAKE_PTR(WildTypeCellMutationState, p_normal_cell_state);
//        normal_and_quiescent_cell_rates[p_normal_cell_state->GetColour()] = 0.6*unit::mole_per_second;
//        normal_and_quiescent_cell_rate_thresholds[p_normal_cell_state->GetColour()] = 0.27*unit::mole_per_metre_cubed;
//        normal_and_quiescent_cell_rates[p_quiescent_cancer_state->GetColour()] = 0.6*unit::mole_per_second;
//        normal_and_quiescent_cell_rate_thresholds[p_quiescent_cancer_state->GetColour()] = 0.0*unit::mole_per_metre_cubed;
//
//        p_normal_and_quiescent_cell_source->SetStateRateMap(normal_and_quiescent_cell_rates);
//        p_normal_and_quiescent_cell_source->SetLabelName("VEGF");
//        p_normal_and_quiescent_cell_source->SetStateRateThresholdMap(normal_and_quiescent_cell_rate_thresholds);
//        p_pde->AddDiscreteSource(p_normal_and_quiescent_cell_source);
//
//        // VEGF release for cancer cells and tip cells, now there is no threshold so we use a different source
//        boost::shared_ptr<CellStateDependentDiscreteSource<2> > p_other_cell_sinks = CellStateDependentDiscreteSource<2>::Create();
//
//        std::map<unsigned, units::quantity<unit::molar_flow_rate> > other_cell_rates;
//        double segment_radius = 0.5;
//        double segment_length = 1.0;
//        double permeability = 15;
//        MAKE_PTR(TipCellMutationState, p_tip_state);
//        MAKE_PTR(StalkCellMutationState, p_stalk_state);
//        other_cell_rates[p_tip_state->GetColour()] = -2 * M_PI * segment_radius * segment_length * permeability / pow(segment_length, 3.0)*unit::mole_per_second; // tip cell mutation state
//        other_cell_rates[p_stalk_state->GetColour()] = -2 * M_PI * segment_radius * segment_length * permeability / pow(segment_length, 3.0)*unit::mole_per_second; // stalk cell mutation state
//        p_other_cell_sinks->SetStateRateMap(other_cell_rates);
//        p_pde->AddDiscreteSource(p_other_cell_sinks);
//        return p_pde;
//    }

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

////        cell_population.SetVolumeFraction(wild_mutation_state, 0.6);
////        cell_population.SetVolumeFraction(cancer_mutation_state, 0.6);
////        cell_population.SetVolumeFraction(quiescent_cancer_mutation_state, 0.6);
////        cell_population.SetVolumeFraction(stalk_mutation_state, 0.4);
////        cell_population.SetVolumeFraction(tip_mutation_state, 0.4);
//

    }
};

```


# Code 
The full code is given below


## File name `Test2dTumourGrowthTutorial.hpp` 

```cpp
#include <cxxtest/TestSuite.h>
#include <vector>
#include "DiscreteContinuumLinearEllipticPde.hpp"
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
//    boost::shared_ptr<DiscreteContinuumLinearEllipticPde<2> > GetOxygenPde()
//    {
//        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<2> > p_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
//
//        units::quantity<unit::diffusivity> oxygen_diffusivity(8700000.0/400.0 * unit::metre_squared_per_second);
//        p_pde->SetIsotropicDiffusionConstant(oxygen_diffusivity); // assume cell width is 20 microns
//
//        // Add a cell state specific discrete source for cells consuming oxygen
//        boost::shared_ptr<CellStateDependentDiscreteSource<2> > p_cell_sink = CellStateDependentDiscreteSource<2>::Create();
//        std::map<unsigned, units::quantity<unit::molar_flow_rate> > state_specific_rates;
//        MAKE_PTR(WildTypeCellMutationState, p_normal_cell_state);
//        MAKE_PTR(CancerCellMutationState, p_cancer_state);
//        MAKE_PTR(QuiescentCancerCellMutationState, p_quiescent_cancer_state);
//        MAKE_PTR(ApoptoticCellProperty, p_apoptotic_property);
//        MAKE_PTR(TipCellMutationState, p_tip_state);
//        MAKE_PTR(StalkCellMutationState, p_stalk_state);
//        state_specific_rates[p_apoptotic_property->GetColour()] = 0.0*unit::mole_per_second;
//        state_specific_rates[p_cancer_state->GetColour()] = -7800.0*unit::mole_per_second;
//        state_specific_rates[p_quiescent_cancer_state->GetColour()] = -7800.0*unit::mole_per_second;
//        state_specific_rates[p_tip_state->GetColour()] = -5000.0*unit::mole_per_second;
//        state_specific_rates[p_stalk_state->GetColour()] = -5000.0*unit::mole_per_second;
//        state_specific_rates[p_normal_cell_state->GetColour()] = -5000.0*unit::mole_per_second;
//        p_cell_sink->SetStateRateMap(state_specific_rates);
//        p_pde->AddDiscreteSource(p_cell_sink);
//
//        // todo this needs to be updated so that source strength is proportional to haematocrit level
//        boost::shared_ptr<CellStateDependentDiscreteSource<2> > p_cell_source = CellStateDependentDiscreteSource<2>::Create();
//        std::map<unsigned, units::quantity<unit::molar_flow_rate> > state_specific_rates2;
//        double segment_radius = 0.5;
//        double segment_length = 1.0;
//        double segment_surface_area = 2 * M_PI * segment_radius * segment_length;
//        double permeability = 20000.0;
//        double inter_vessel_O2_level = 5.0;
//        state_specific_rates2[p_stalk_state->GetColour()] = segment_surface_area * permeability * inter_vessel_O2_level / pow(segment_length, 3.0)*unit::mole_per_second;
//        state_specific_rates2[p_tip_state->GetColour()] = segment_surface_area * permeability * inter_vessel_O2_level / pow(segment_length, 3.0)*unit::mole_per_second;
//        p_cell_source->SetStateRateMap(state_specific_rates2);
//        p_pde->AddDiscreteSource(p_cell_source);
//        return p_pde;
//    }
//
//    // todo need to check parameters in sink/source terms in here
//    boost::shared_ptr<DiscreteContinuumLinearEllipticPde<2> > GetVegfPde()
//    {
//        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<2> > p_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
//        units::quantity<unit::diffusivity> vegf_diffusivity(60000.0 / 400.0 * unit::metre_squared_per_second);
//        p_pde->SetIsotropicDiffusionConstant(vegf_diffusivity); // assume cell width is 20 microns
//        p_pde->SetContinuumLinearInUTerm(-0.8*unit::per_second); //Vegf decay
//
//        // VEGF release for normal cells and quiescent cancer cells:
//        // normal cells release only when intracellular vegf reaches a certain value
//        // quiescent cancer cells always release vegf
//        boost::shared_ptr<CellStateDependentDiscreteSource<2> > p_normal_and_quiescent_cell_source = CellStateDependentDiscreteSource<2>::Create();
//
//        // Set mutation specific source strengths and thresholds
//        std::map<unsigned, units::quantity<unit::molar_flow_rate> > normal_and_quiescent_cell_rates;
//        std::map<unsigned, units::quantity<unit::concentration> > normal_and_quiescent_cell_rate_thresholds;
//        MAKE_PTR(QuiescentCancerCellMutationState, p_quiescent_cancer_state);
//        MAKE_PTR(WildTypeCellMutationState, p_normal_cell_state);
//        normal_and_quiescent_cell_rates[p_normal_cell_state->GetColour()] = 0.6*unit::mole_per_second;
//        normal_and_quiescent_cell_rate_thresholds[p_normal_cell_state->GetColour()] = 0.27*unit::mole_per_metre_cubed;
//        normal_and_quiescent_cell_rates[p_quiescent_cancer_state->GetColour()] = 0.6*unit::mole_per_second;
//        normal_and_quiescent_cell_rate_thresholds[p_quiescent_cancer_state->GetColour()] = 0.0*unit::mole_per_metre_cubed;
//
//        p_normal_and_quiescent_cell_source->SetStateRateMap(normal_and_quiescent_cell_rates);
//        p_normal_and_quiescent_cell_source->SetLabelName("VEGF");
//        p_normal_and_quiescent_cell_source->SetStateRateThresholdMap(normal_and_quiescent_cell_rate_thresholds);
//        p_pde->AddDiscreteSource(p_normal_and_quiescent_cell_source);
//
//        // VEGF release for cancer cells and tip cells, now there is no threshold so we use a different source
//        boost::shared_ptr<CellStateDependentDiscreteSource<2> > p_other_cell_sinks = CellStateDependentDiscreteSource<2>::Create();
//
//        std::map<unsigned, units::quantity<unit::molar_flow_rate> > other_cell_rates;
//        double segment_radius = 0.5;
//        double segment_length = 1.0;
//        double permeability = 15;
//        MAKE_PTR(TipCellMutationState, p_tip_state);
//        MAKE_PTR(StalkCellMutationState, p_stalk_state);
//        other_cell_rates[p_tip_state->GetColour()] = -2 * M_PI * segment_radius * segment_length * permeability / pow(segment_length, 3.0)*unit::mole_per_second; // tip cell mutation state
//        other_cell_rates[p_stalk_state->GetColour()] = -2 * M_PI * segment_radius * segment_length * permeability / pow(segment_length, 3.0)*unit::mole_per_second; // stalk cell mutation state
//        p_other_cell_sinks->SetStateRateMap(other_cell_rates);
//        p_pde->AddDiscreteSource(p_other_cell_sinks);
//        return p_pde;
//    }

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

////        cell_population.SetVolumeFraction(wild_mutation_state, 0.6);
////        cell_population.SetVolumeFraction(cancer_mutation_state, 0.6);
////        cell_population.SetVolumeFraction(quiescent_cancer_mutation_state, 0.6);
////        cell_population.SetVolumeFraction(stalk_mutation_state, 0.4);
////        cell_population.SetVolumeFraction(tip_mutation_state, 0.4);
//

    }
};

```

