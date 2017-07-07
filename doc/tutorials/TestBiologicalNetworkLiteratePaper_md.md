---
layout: page-full-width 
title: Test Biological Network Literate Paper
---
This tutorial is automatically generated from the file test/tutorials//TestBiologicalNetworkLiteratePaper.hpp.
Note that the code is given in full at the bottom of the page.



# A Biological Network Demo
## The Test
Start by introducing the necessary header files. The first contain functionality for setting up unit tests,
smart pointer tools and output management,

```cpp
#include <vector>
#include "SmartPointers.hpp"
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "RandomNumberGenerator.hpp"
```

dimensional analysis,

```cpp
#include "DimensionalChastePoint.hpp"
#include "UnitCollection.hpp"
#include "Owen11Parameters.hpp"
#include "Secomb04Parameters.hpp"
#include "GenericParameters.hpp"
#include "ParameterCollection.hpp"
#include "BaseUnits.hpp"
```

vessel networks,

```cpp
#include "VesselNode.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkReader.hpp"
```

cells,

```cpp
#include "CancerCellMutationState.hpp"
#include "StalkCellMutationState.hpp"
#include "QuiescentCancerCellMutationState.hpp"
#include "WildTypeCellMutationState.hpp"
#include "Owen11CellPopulationGenerator.hpp"
#include "Owen2011TrackingModifier.hpp"
#include "CaBasedCellPopulation.hpp"
#include "ApoptoticCellKiller.hpp"
```

flow,

```cpp
#include "VesselImpedanceCalculator.hpp"
#include "FlowSolver.hpp"
#include "ConstantHaematocritSolver.hpp"
#include "StructuralAdaptationSolver.hpp"
#include "WallShearStressCalculator.hpp"
#include "MechanicalStimulusCalculator.hpp"
#include "MetabolicStimulusCalculator.hpp"
#include "ShrinkingStimulusCalculator.hpp"
#include "ViscosityCalculator.hpp"
```

grids and PDEs,

```cpp
#include "RegularGrid.hpp"
#include "FiniteDifferenceSolver.hpp"
#include "CellBasedDiscreteSource.hpp"
#include "VesselBasedDiscreteSource.hpp"
#include "CellStateDependentDiscreteSource.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
```

angiogenesis and regression,

```cpp
#include "OffLatticeSproutingRule.hpp"
#include "OffLatticeMigrationRule.hpp"
#include "AngiogenesisSolver.hpp"
#include "WallShearStressBasedRegressionSolver.hpp"
```

classes for managing the simulation,

```cpp
#include "MicrovesselSolver.hpp"
#include "MicrovesselSimulationModifier.hpp"
#include "OnLatticeSimulation.hpp"
```

Visualization

```cpp
#include "MicrovesselVtkScene.hpp"
#include "VtkSceneMicrovesselModifier.hpp"
```

This should appear last.

```cpp
#include "PetscSetupAndFinalize.hpp"
class TestLatticeBasedAngiogenesisLiteratePaper : public AbstractCellBasedWithTimingsTestSuite
{

public:

    void Test3dLatticeBased() throw (Exception)
    {
```

Set up output file management and seed the random number generator.

```cpp
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestBiologicalNetworkLiteratePaper"));
        RandomNumberGenerator::Instance()->Reseed(12345);
```

This component uses explicit dimensions for all quantities, but interfaces with solvers which take
non-dimensional inputs. The `BaseUnits` singleton takes time, length and mass reference scales to
allow non-dimensionalisation when sending quantities to external solvers and re-dimensionalisation of
results. For our purposes microns for length and hours for time are suitable base units.

```cpp
        QLength reference_length(1.0 * unit::microns);
        QTime reference_time(1.0* unit::hours);
        QConcentration reference_concentration(1.e-6* unit::mole_per_metre_cubed);
        BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
        BaseUnits::Instance()->SetReferenceTimeScale(reference_time);
        BaseUnits::Instance()->SetReferenceConcentrationScale(reference_concentration);
```

Read a vessel network derived from biological images from file

```cpp
        boost::shared_ptr<VesselNetworkReader<3> > p_vessel_reader =
                boost::shared_ptr<VesselNetworkReader<3> >(new VesselNetworkReader<3> );

        FileFinder file_finder = FileFinder("/projects/MicrovesselChaste/test/data/bio_original.vtp", RelativeTo::ChasteSourceRoot);
        p_vessel_reader->SetFileName(file_finder.GetAbsolutePath());
        p_vessel_reader->SetMergeCoincidentPoints(true);
        p_vessel_reader->SetTargetSegmentLength(40.0e-6*unit::metres);
        boost::shared_ptr<VesselNetwork<3> >  p_network = p_vessel_reader->Read();
```
The vessel network may contain short vessels due to image processing artifacts,
we remove any vessels that are on the order of a single cell length and are not connected
to other vessels at both ends. Note that units are explicitly specified for all quantities. It is
ok to allow some small disconnected regions to remain for our purposes.

```cpp
        QLength short_vessel_cutoff = 40.0e-6 * unit::metres;
        bool remove_end_vessels_only = true;
        p_network->RemoveShortVessels(short_vessel_cutoff, remove_end_vessels_only);
        p_network->UpdateAll();
        p_network->MergeCoincidentNodes();
        p_network->UpdateAll();
```

Write the modified network to file for inspection

```cpp
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "cleaned_network.vtp");
        boost::shared_ptr<MicrovesselVtkScene<3> > p_scene = boost::shared_ptr<MicrovesselVtkScene<3> >(new MicrovesselVtkScene<3>);
        p_scene->SetIsInteractive(true);
        p_scene->SetVesselNetwork(p_network);
        p_scene->GetVesselNetworkActorGenerator()->SetEdgeSize(20.0);
        p_scene->Start();

```
Simulating tumour growth for the entire network would be prohibitive for this tutorial, so
we sample a small region. We can use some geometry tools to help.

```cpp
        boost::shared_ptr<Part<3> > p_cylinder = Part<3>::Create();
        DimensionalChastePoint<3> centre(2300.0, 2300.0, -5.0, 1_um);
        QLength radius = 600.0e-6*unit::metres;
        QLength depth = 205.e-6*unit::metres;
        p_cylinder->AddCylinder(radius, depth, centre, 24);
        p_cylinder->BooleanWithNetwork(p_network);
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "cleaned_cut_network.vtp");
        p_scene->Start();
```

We are ready to simulate tumour growth and angiogenesis. We will use a regular lattice for
this purpose. We size and position the lattice according to the bounds of the vessel network.

```cpp
        std::vector<DimensionalChastePoint<3> > bbox;
        bbox.push_back(DimensionalChastePoint<3>(1500.0, 1600.0, -10.0, 1_um));
        bbox.push_back(DimensionalChastePoint<3>(3100.0, 3000.0, 300.0, 1_um));
```

Set up the lattice (grid), we will use the same dimensions as [Owen et al. 2011](http://www.ncbi.nlm.nih.gov/pubmed/21363914).
Note that we are using hard-coded parameters from that paper. You can see the values by inspecting `Owen11Parameters.cpp`.
Alternatively each parameter supports the `<<` operator for streaming. When we get the value of the parameter by doing
`Owen11Parameters::mpLatticeSpacing->GetValue("User")` a record is kept that this parameter has been used in the simulation.
A record of all parameters used in a simulation can be dumped to file on completion, as will be shown below.

```cpp
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        QLength grid_spacing = 40.0e-6*unit::metres;
        p_grid->SetSpacing(grid_spacing);
```

We can use the built-in dimensional analysis functionality to get the network extents in terms of grid units

```cpp
        c_vector<double, 3> botom_front_left =  bbox[0].GetLocation(grid_spacing);
        c_vector<double, 3> top_back_right =  bbox[1].GetLocation(grid_spacing);
        c_vector<double, 3> extents = top_back_right - botom_front_left;
        std::vector<unsigned> grid_extents;
        for(unsigned idx=0; idx<3 ; idx++)
        {
            grid_extents.push_back(std::floor(extents[idx])+1);
        }
        p_grid->SetExtents(grid_extents);
        p_network->Translate(DimensionalChastePoint<3>(-1500.0, -1600.0, +10.0, 1_um));
```

We can write the lattice to file for quick visualization with Paraview. Rendering of this and subsequent images is performed
using standard Paraview operations, not detailed here.

```cpp
        p_grid->Write(p_handler);
        p_scene->SetRegularGrid(p_grid);
        p_scene->Start();
```

Next we set the inflow and outflow boundary conditions for blood flow. Because the network connectivity
is relatively low we assign all vessels near the top of the domain (z coord) as inflows and the bottom
as outflows.

```cpp
        for(unsigned idx=0;idx<p_network->GetNodes().size(); idx++)
        {
            if(p_network->GetNodes()[idx]->GetNumberOfSegments() == 1)
            {
                if(std::abs(p_network->GetNodes()[idx]->rGetLocation().GetLocation(1_um)[2] -
                        bbox[1].GetLocation(1_um)[2]) < 80.0)
                {
                    p_network->GetNodes()[idx]->GetFlowProperties()->SetIsInputNode(true);
                    p_network->GetNodes()[idx]->GetFlowProperties()->SetPressure(Owen11Parameters::mpInletPressure->GetValue("User"));
                }
                else if(std::abs(p_network->GetNodes()[idx]->rGetLocation().GetLocation(1_um)[2] -
                        bbox[0].GetLocation(1_um)[2]) < 80.0)
                {
                    p_network->GetNodes()[idx]->GetFlowProperties()->SetIsOutputNode(true);
                    p_network->GetNodes()[idx]->GetFlowProperties()->SetPressure(Owen11Parameters::mpOutletPressure->GetValue("User"));
                }
            }
        }
```

Again, we can write the network to file for visualization

```cpp
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "flow_boundary_labelled_network.vtp");
```

Next, set up the cell populations. We will setup up a population similar to that used in the Owen et al., 2011 paper. That is, a grid
filled with normal cells and a tumour spheroid in the middle. We can use a generator for this purpose. The generator simply sets up
the population using conventional Cell Based Chaste methods.

```cpp
        boost::shared_ptr<Owen11CellPopulationGenerator<3> > p_cell_population_genenerator = Owen11CellPopulationGenerator<3>::Create();
        p_cell_population_genenerator->SetRegularGrid(p_grid);
        p_cell_population_genenerator->SetVesselNetwork(p_network);
        QLength tumour_radius(300.0 * unit::microns);
        p_cell_population_genenerator->SetTumourRadius(tumour_radius);
        boost::shared_ptr<CaBasedCellPopulation<3> > p_cell_population = p_cell_population_genenerator->Update();

        p_scene->SetCellPopulation(p_cell_population);
        p_scene->GetCellPopulationActorGenerator()->GetDiscreteColorTransferFunction()->AddRGBPoint(1.0, 0.0, 0.0, 0.6);
        p_scene->GetCellPopulationActorGenerator()->SetPointSize(20.0);
        p_scene->GetCellPopulationActorGenerator()->SetColorByCellMutationState(true);
        p_scene->ResetRenderer();
        p_scene->Start();
```

Next set up the PDEs for oxygen and VEGF. Cells will act as discrete oxygen sinks and discrete vegf sources.

```cpp
        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<3> > p_oxygen_pde = DiscreteContinuumLinearEllipticPde<3>::Create();
        p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
        boost::shared_ptr<CellBasedDiscreteSource<3> > p_cell_oxygen_sink = CellBasedDiscreteSource<3>::Create();
        p_cell_oxygen_sink->SetLinearInUConsumptionRatePerCell(Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));
        p_oxygen_pde->AddDiscreteSource(p_cell_oxygen_sink);
```

Vessels release oxygen depending on their haematocrit levels

```cpp
        boost::shared_ptr<VesselBasedDiscreteSource<3> > p_vessel_oxygen_source = VesselBasedDiscreteSource<3>::Create();
        QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
                GenericParameters::mpGasConcentrationAtStp->GetValue("User");
        QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
                Owen11Parameters::mpReferencePartialPressure->GetValue("User");
        p_vessel_oxygen_source->SetReferenceConcentration(vessel_oxygen_concentration);
        p_vessel_oxygen_source->SetVesselPermeability(Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
        p_vessel_oxygen_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
        p_oxygen_pde->AddDiscreteSource(p_vessel_oxygen_source);
```

Set up a finite difference solver and pass it the pde and grid.

```cpp
        boost::shared_ptr<FiniteDifferenceSolver<3> > p_oxygen_solver = FiniteDifferenceSolver<3>::Create();
        p_oxygen_solver->SetPde(p_oxygen_pde);
        p_oxygen_solver->SetLabel("oxygen");
        p_oxygen_solver->SetGrid(p_grid);
```

The rate of VEGF release depends on the cell type and intracellular VEGF levels, so we need a more detailed
type of discrete source.

```cpp
        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<3> > p_vegf_pde = DiscreteContinuumLinearEllipticPde<3>::Create();
        p_vegf_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpVegfDiffusivity->GetValue("User"));
        p_vegf_pde->SetContinuumLinearInUTerm(-Owen11Parameters::mpVegfDecayRate->GetValue("User"));
```

Set up a map for different release rates depending on cell type. Also include a threshold intracellular VEGF below which
there is no release.

```cpp
        boost::shared_ptr<CellStateDependentDiscreteSource<3> > p_normal_and_quiescent_cell_source =
        		CellStateDependentDiscreteSource<3>::Create();
        std::map<unsigned, QConcentrationFlowRate > normal_and_quiescent_cell_rates;
        std::map<unsigned, QConcentration > normal_and_quiescent_cell_rate_thresholds;
        MAKE_PTR(QuiescentCancerCellMutationState, p_quiescent_cancer_state);
        MAKE_PTR(WildTypeCellMutationState, p_normal_cell_state);
        normal_and_quiescent_cell_rates[p_normal_cell_state->GetColour()] = Owen11Parameters::mpCellVegfSecretionRate->GetValue("User");
        normal_and_quiescent_cell_rate_thresholds[p_normal_cell_state->GetColour()] = 0.27*unit::mole_per_metre_cubed;
        normal_and_quiescent_cell_rates[p_quiescent_cancer_state->GetColour()] = Owen11Parameters::mpCellVegfSecretionRate->GetValue("User");
        normal_and_quiescent_cell_rate_thresholds[p_quiescent_cancer_state->GetColour()] = 0.0*unit::mole_per_metre_cubed;
        p_normal_and_quiescent_cell_source->SetStateRateMap(normal_and_quiescent_cell_rates);
        p_normal_and_quiescent_cell_source->SetLabelName("VEGF");
        p_normal_and_quiescent_cell_source->SetStateRateThresholdMap(normal_and_quiescent_cell_rate_thresholds);
        p_vegf_pde->AddDiscreteSource(p_normal_and_quiescent_cell_source);
```

Add a vessel related VEGF sink

```cpp
        boost::shared_ptr<VesselBasedDiscreteSource<3> > p_vessel_vegf_sink = VesselBasedDiscreteSource<3>::Create();
        p_vessel_vegf_sink->SetReferenceConcentration(0.0*unit::mole_per_metre_cubed);
        p_vessel_vegf_sink->SetVesselPermeability(Owen11Parameters::mpVesselVegfPermeability->GetValue("User"));
        p_vegf_pde->AddDiscreteSource(p_vessel_vegf_sink);
```

Set up a finite difference solver as before.

```cpp
        boost::shared_ptr<FiniteDifferenceSolver<3> > p_vegf_solver = FiniteDifferenceSolver<3>::Create();
        p_vegf_solver->SetPde(p_vegf_pde);
        p_vegf_solver->SetLabel("VEGF_Extracellular");
        p_vegf_solver->SetGrid(p_grid);
```

Next set up the flow problem. Assign a blood plasma viscosity to the vessels. The actual viscosity will
depend on haematocrit and diameter. This solver manages growth and shrinkage of vessels in response to
flow related stimuli.

```cpp
        QLength large_vessel_radius(25.0 * unit::microns);
        p_network->SetSegmentRadii(large_vessel_radius);
        QDynamicViscosity viscosity = Owen11Parameters::mpPlasmaViscosity->GetValue("User");
        p_network->SetSegmentViscosity(viscosity);
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "flow_boundary_labelled_network_flow.vtp");
```

Set up the pre- and post flow calculators.

```cpp
        boost::shared_ptr<VesselImpedanceCalculator<3> > p_impedance_calculator = VesselImpedanceCalculator<3>::Create();
        boost::shared_ptr<ConstantHaematocritSolver<3> > p_haematocrit_calculator = ConstantHaematocritSolver<3>::Create();
        p_haematocrit_calculator->SetHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
        boost::shared_ptr<WallShearStressCalculator<3> > p_wss_calculator = WallShearStressCalculator<3>::Create();
        boost::shared_ptr<MechanicalStimulusCalculator<3> > p_mech_stimulus_calculator = MechanicalStimulusCalculator<3>::Create();
        boost::shared_ptr<MetabolicStimulusCalculator<3> > p_metabolic_stim_calculator = MetabolicStimulusCalculator<3>::Create();
        boost::shared_ptr<ShrinkingStimulusCalculator<3> > p_shrinking_stimulus_calculator = ShrinkingStimulusCalculator<3>::Create();
        boost::shared_ptr<ViscosityCalculator<3> > p_viscosity_calculator = ViscosityCalculator<3>::Create();
```

Set up and configure the structural adaptation solver.

```cpp
        boost::shared_ptr<StructuralAdaptationSolver<3> > p_structural_adaptation_solver = StructuralAdaptationSolver<3>::Create();
        p_structural_adaptation_solver->SetTolerance(0.0001);
        p_structural_adaptation_solver->SetMaxIterations(100);
        p_structural_adaptation_solver->SetTimeIncrement(Owen11Parameters::mpVesselRadiusUpdateTimestep->GetValue("User"));
        p_structural_adaptation_solver->AddPreFlowSolveCalculator(p_impedance_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_haematocrit_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_wss_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_metabolic_stim_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_mech_stimulus_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_viscosity_calculator);
```

Set up a regression solver.

```cpp
        boost::shared_ptr<WallShearStressBasedRegressionSolver<3> > p_regression_solver =
                WallShearStressBasedRegressionSolver<3>::Create();
```

Set up an angiogenesis solver and add sprouting and migration rules.

```cpp
        boost::shared_ptr<AngiogenesisSolver<3> > p_angiogenesis_solver = AngiogenesisSolver<3>::Create();
        boost::shared_ptr<OffLatticeSproutingRule<3> > p_sprouting_rule = OffLatticeSproutingRule<3>::Create();
        p_sprouting_rule->SetSproutingProbability(1.e-4* unit::per_second);
        boost::shared_ptr<OffLatticeMigrationRule<3> > p_migration_rule = OffLatticeMigrationRule<3>::Create();
        p_migration_rule->SetChemotacticStrength(0.1);
        p_migration_rule->SetAttractionStrength(0.5);

        QVelocity sprout_velocity(40.0*unit::microns/unit::hours);
        p_migration_rule->SetSproutingVelocity(sprout_velocity);
        p_angiogenesis_solver->SetMigrationRule(p_migration_rule);
        p_angiogenesis_solver->SetSproutingRule(p_sprouting_rule);
        p_sprouting_rule->SetDiscreteContinuumSolver(p_vegf_solver);
        p_migration_rule->SetDiscreteContinuumSolver(p_vegf_solver);
        //p_angiogenesis_solver->SetVesselGrid(p_grid);
        p_angiogenesis_solver->SetVesselNetwork(p_network);
```

The microvessel solver will manage all aspects of the vessel solve.

```cpp
        boost::shared_ptr<MicrovesselSolver<3> > p_microvessel_solver = MicrovesselSolver<3>::Create();
        p_microvessel_solver->SetVesselNetwork(p_network);
        p_microvessel_solver->SetOutputFrequency(1);
        p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
        p_microvessel_solver->AddDiscreteContinuumSolver(p_vegf_solver);
        p_microvessel_solver->SetStructuralAdaptationSolver(p_structural_adaptation_solver);
        p_microvessel_solver->SetRegressionSolver(p_regression_solver);
        p_microvessel_solver->SetAngiogenesisSolver(p_angiogenesis_solver);
```

Set up real time plotting.

```cpp
        //p_scene->GetCellPopulationActorGenerator()->SetColorByCellData(true);
        //p_scene->GetCellPopulationActorGenerator()->SetDataLabel("oxygen");
        boost::shared_ptr<VtkSceneMicrovesselModifier<3> > p_scene_modifier =
                boost::shared_ptr<VtkSceneMicrovesselModifier<3> >(new VtkSceneMicrovesselModifier<3>);
        p_scene_modifier->SetVtkScene(p_scene);
        p_scene_modifier->SetUpdateFrequency(3);
        p_microvessel_solver->AddMicrovesselModifier(p_scene_modifier);
```

The microvessel solution modifier will link the vessel and cell solvers. We need to explicitly tell is
which extracellular fields to update based on PDE solutions.

```cpp
        boost::shared_ptr<MicrovesselSimulationModifier<3> > p_microvessel_modifier = MicrovesselSimulationModifier<3>::Create();
        p_microvessel_modifier->SetMicrovesselSolver(p_microvessel_solver);
        std::vector<std::string> update_labels;
        update_labels.push_back("oxygen");
        update_labels.push_back("VEGF_Extracellular");
        p_microvessel_modifier->SetCellDataUpdateLabels(update_labels);
```

The full simulation is run as a typical Cell Based Chaste simulation

```cpp
        OnLatticeSimulation<3> simulator(*p_cell_population);
        simulator.AddSimulationModifier(p_microvessel_modifier);
```

Add a killer to remove apoptotic cells

```cpp
        boost::shared_ptr<ApoptoticCellKiller<3> > p_apoptotic_cell_killer(new ApoptoticCellKiller<3>(p_cell_population.get()));
        simulator.AddCellKiller(p_apoptotic_cell_killer);
```

Add another modifier for updating cell cycle quantities.

```cpp
        boost::shared_ptr<Owen2011TrackingModifier<3> > p_owen11_tracking_modifier(new Owen2011TrackingModifier<3>);
        simulator.AddSimulationModifier(p_owen11_tracking_modifier);
```

Set up the remainder of the simulation

```cpp
        simulator.SetOutputDirectory("TestBiologicalNetworkLiteratePaper");
        simulator.SetSamplingTimestepMultiple(1);
        simulator.SetDt(0.5);
```

This end time corresponds to roughly 10 minutes run-time on a desktop PC. Increase it or decrease as
preferred. The end time used in Owen et al. 2011 is 4800 hours.

```cpp
        simulator.SetEndTime(200.0);
```

Do the solve.

```cpp
        simulator.Solve();
```

Dump the parameters to file for inspection.

```cpp
        ParameterCollection::Instance()->DumpToFile(p_handler->GetOutputDirectoryFullPath()+"parameter_collection.xml");
    }
};

```


# Code 
The full code is given below


## File name `TestBiologicalNetworkLiteratePaper.hpp` 

```cpp
#include <vector>
#include "SmartPointers.hpp"
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "RandomNumberGenerator.hpp"
#include "DimensionalChastePoint.hpp"
#include "UnitCollection.hpp"
#include "Owen11Parameters.hpp"
#include "Secomb04Parameters.hpp"
#include "GenericParameters.hpp"
#include "ParameterCollection.hpp"
#include "BaseUnits.hpp"
#include "VesselNode.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkReader.hpp"
#include "CancerCellMutationState.hpp"
#include "StalkCellMutationState.hpp"
#include "QuiescentCancerCellMutationState.hpp"
#include "WildTypeCellMutationState.hpp"
#include "Owen11CellPopulationGenerator.hpp"
#include "Owen2011TrackingModifier.hpp"
#include "CaBasedCellPopulation.hpp"
#include "ApoptoticCellKiller.hpp"
#include "VesselImpedanceCalculator.hpp"
#include "FlowSolver.hpp"
#include "ConstantHaematocritSolver.hpp"
#include "StructuralAdaptationSolver.hpp"
#include "WallShearStressCalculator.hpp"
#include "MechanicalStimulusCalculator.hpp"
#include "MetabolicStimulusCalculator.hpp"
#include "ShrinkingStimulusCalculator.hpp"
#include "ViscosityCalculator.hpp"
#include "RegularGrid.hpp"
#include "FiniteDifferenceSolver.hpp"
#include "CellBasedDiscreteSource.hpp"
#include "VesselBasedDiscreteSource.hpp"
#include "CellStateDependentDiscreteSource.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "OffLatticeSproutingRule.hpp"
#include "OffLatticeMigrationRule.hpp"
#include "AngiogenesisSolver.hpp"
#include "WallShearStressBasedRegressionSolver.hpp"
#include "MicrovesselSolver.hpp"
#include "MicrovesselSimulationModifier.hpp"
#include "OnLatticeSimulation.hpp"
#include "MicrovesselVtkScene.hpp"
#include "VtkSceneMicrovesselModifier.hpp"
#include "PetscSetupAndFinalize.hpp"
class TestLatticeBasedAngiogenesisLiteratePaper : public AbstractCellBasedWithTimingsTestSuite
{

public:

    void Test3dLatticeBased() throw (Exception)
    {
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestBiologicalNetworkLiteratePaper"));
        RandomNumberGenerator::Instance()->Reseed(12345);
        QLength reference_length(1.0 * unit::microns);
        QTime reference_time(1.0* unit::hours);
        QConcentration reference_concentration(1.e-6* unit::mole_per_metre_cubed);
        BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
        BaseUnits::Instance()->SetReferenceTimeScale(reference_time);
        BaseUnits::Instance()->SetReferenceConcentrationScale(reference_concentration);
        boost::shared_ptr<VesselNetworkReader<3> > p_vessel_reader =
                boost::shared_ptr<VesselNetworkReader<3> >(new VesselNetworkReader<3> );

        FileFinder file_finder = FileFinder("/projects/MicrovesselChaste/test/data/bio_original.vtp", RelativeTo::ChasteSourceRoot);
        p_vessel_reader->SetFileName(file_finder.GetAbsolutePath());
        p_vessel_reader->SetMergeCoincidentPoints(true);
        p_vessel_reader->SetTargetSegmentLength(40.0e-6*unit::metres);
        boost::shared_ptr<VesselNetwork<3> >  p_network = p_vessel_reader->Read();
        QLength short_vessel_cutoff = 40.0e-6 * unit::metres;
        bool remove_end_vessels_only = true;
        p_network->RemoveShortVessels(short_vessel_cutoff, remove_end_vessels_only);
        p_network->UpdateAll();
        p_network->MergeCoincidentNodes();
        p_network->UpdateAll();
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "cleaned_network.vtp");
        boost::shared_ptr<MicrovesselVtkScene<3> > p_scene = boost::shared_ptr<MicrovesselVtkScene<3> >(new MicrovesselVtkScene<3>);
        p_scene->SetIsInteractive(true);
        p_scene->SetVesselNetwork(p_network);
        p_scene->GetVesselNetworkActorGenerator()->SetEdgeSize(20.0);
        p_scene->Start();

        boost::shared_ptr<Part<3> > p_cylinder = Part<3>::Create();
        DimensionalChastePoint<3> centre(2300.0, 2300.0, -5.0, 1_um);
        QLength radius = 600.0e-6*unit::metres;
        QLength depth = 205.e-6*unit::metres;
        p_cylinder->AddCylinder(radius, depth, centre, 24);
        p_cylinder->BooleanWithNetwork(p_network);
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "cleaned_cut_network.vtp");
        p_scene->Start();
        std::vector<DimensionalChastePoint<3> > bbox;
        bbox.push_back(DimensionalChastePoint<3>(1500.0, 1600.0, -10.0, 1_um));
        bbox.push_back(DimensionalChastePoint<3>(3100.0, 3000.0, 300.0, 1_um));
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        QLength grid_spacing = 40.0e-6*unit::metres;
        p_grid->SetSpacing(grid_spacing);
        c_vector<double, 3> botom_front_left =  bbox[0].GetLocation(grid_spacing);
        c_vector<double, 3> top_back_right =  bbox[1].GetLocation(grid_spacing);
        c_vector<double, 3> extents = top_back_right - botom_front_left;
        std::vector<unsigned> grid_extents;
        for(unsigned idx=0; idx<3 ; idx++)
        {
            grid_extents.push_back(std::floor(extents[idx])+1);
        }
        p_grid->SetExtents(grid_extents);
        p_network->Translate(DimensionalChastePoint<3>(-1500.0, -1600.0, +10.0, 1_um));
        p_grid->Write(p_handler);
        p_scene->SetRegularGrid(p_grid);
        p_scene->Start();
        for(unsigned idx=0;idx<p_network->GetNodes().size(); idx++)
        {
            if(p_network->GetNodes()[idx]->GetNumberOfSegments() == 1)
            {
                if(std::abs(p_network->GetNodes()[idx]->rGetLocation().GetLocation(1_um)[2] -
                        bbox[1].GetLocation(1_um)[2]) < 80.0)
                {
                    p_network->GetNodes()[idx]->GetFlowProperties()->SetIsInputNode(true);
                    p_network->GetNodes()[idx]->GetFlowProperties()->SetPressure(Owen11Parameters::mpInletPressure->GetValue("User"));
                }
                else if(std::abs(p_network->GetNodes()[idx]->rGetLocation().GetLocation(1_um)[2] -
                        bbox[0].GetLocation(1_um)[2]) < 80.0)
                {
                    p_network->GetNodes()[idx]->GetFlowProperties()->SetIsOutputNode(true);
                    p_network->GetNodes()[idx]->GetFlowProperties()->SetPressure(Owen11Parameters::mpOutletPressure->GetValue("User"));
                }
            }
        }
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "flow_boundary_labelled_network.vtp");
        boost::shared_ptr<Owen11CellPopulationGenerator<3> > p_cell_population_genenerator = Owen11CellPopulationGenerator<3>::Create();
        p_cell_population_genenerator->SetRegularGrid(p_grid);
        p_cell_population_genenerator->SetVesselNetwork(p_network);
        QLength tumour_radius(300.0 * unit::microns);
        p_cell_population_genenerator->SetTumourRadius(tumour_radius);
        boost::shared_ptr<CaBasedCellPopulation<3> > p_cell_population = p_cell_population_genenerator->Update();

        p_scene->SetCellPopulation(p_cell_population);
        p_scene->GetCellPopulationActorGenerator()->GetDiscreteColorTransferFunction()->AddRGBPoint(1.0, 0.0, 0.0, 0.6);
        p_scene->GetCellPopulationActorGenerator()->SetPointSize(20.0);
        p_scene->GetCellPopulationActorGenerator()->SetColorByCellMutationState(true);
        p_scene->ResetRenderer();
        p_scene->Start();
        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<3> > p_oxygen_pde = DiscreteContinuumLinearEllipticPde<3>::Create();
        p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
        boost::shared_ptr<CellBasedDiscreteSource<3> > p_cell_oxygen_sink = CellBasedDiscreteSource<3>::Create();
        p_cell_oxygen_sink->SetLinearInUConsumptionRatePerCell(Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));
        p_oxygen_pde->AddDiscreteSource(p_cell_oxygen_sink);
        boost::shared_ptr<VesselBasedDiscreteSource<3> > p_vessel_oxygen_source = VesselBasedDiscreteSource<3>::Create();
        QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
                GenericParameters::mpGasConcentrationAtStp->GetValue("User");
        QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
                Owen11Parameters::mpReferencePartialPressure->GetValue("User");
        p_vessel_oxygen_source->SetReferenceConcentration(vessel_oxygen_concentration);
        p_vessel_oxygen_source->SetVesselPermeability(Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
        p_vessel_oxygen_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
        p_oxygen_pde->AddDiscreteSource(p_vessel_oxygen_source);
        boost::shared_ptr<FiniteDifferenceSolver<3> > p_oxygen_solver = FiniteDifferenceSolver<3>::Create();
        p_oxygen_solver->SetPde(p_oxygen_pde);
        p_oxygen_solver->SetLabel("oxygen");
        p_oxygen_solver->SetGrid(p_grid);
        boost::shared_ptr<DiscreteContinuumLinearEllipticPde<3> > p_vegf_pde = DiscreteContinuumLinearEllipticPde<3>::Create();
        p_vegf_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpVegfDiffusivity->GetValue("User"));
        p_vegf_pde->SetContinuumLinearInUTerm(-Owen11Parameters::mpVegfDecayRate->GetValue("User"));
        boost::shared_ptr<CellStateDependentDiscreteSource<3> > p_normal_and_quiescent_cell_source =
        		CellStateDependentDiscreteSource<3>::Create();
        std::map<unsigned, QConcentrationFlowRate > normal_and_quiescent_cell_rates;
        std::map<unsigned, QConcentration > normal_and_quiescent_cell_rate_thresholds;
        MAKE_PTR(QuiescentCancerCellMutationState, p_quiescent_cancer_state);
        MAKE_PTR(WildTypeCellMutationState, p_normal_cell_state);
        normal_and_quiescent_cell_rates[p_normal_cell_state->GetColour()] = Owen11Parameters::mpCellVegfSecretionRate->GetValue("User");
        normal_and_quiescent_cell_rate_thresholds[p_normal_cell_state->GetColour()] = 0.27*unit::mole_per_metre_cubed;
        normal_and_quiescent_cell_rates[p_quiescent_cancer_state->GetColour()] = Owen11Parameters::mpCellVegfSecretionRate->GetValue("User");
        normal_and_quiescent_cell_rate_thresholds[p_quiescent_cancer_state->GetColour()] = 0.0*unit::mole_per_metre_cubed;
        p_normal_and_quiescent_cell_source->SetStateRateMap(normal_and_quiescent_cell_rates);
        p_normal_and_quiescent_cell_source->SetLabelName("VEGF");
        p_normal_and_quiescent_cell_source->SetStateRateThresholdMap(normal_and_quiescent_cell_rate_thresholds);
        p_vegf_pde->AddDiscreteSource(p_normal_and_quiescent_cell_source);
        boost::shared_ptr<VesselBasedDiscreteSource<3> > p_vessel_vegf_sink = VesselBasedDiscreteSource<3>::Create();
        p_vessel_vegf_sink->SetReferenceConcentration(0.0*unit::mole_per_metre_cubed);
        p_vessel_vegf_sink->SetVesselPermeability(Owen11Parameters::mpVesselVegfPermeability->GetValue("User"));
        p_vegf_pde->AddDiscreteSource(p_vessel_vegf_sink);
        boost::shared_ptr<FiniteDifferenceSolver<3> > p_vegf_solver = FiniteDifferenceSolver<3>::Create();
        p_vegf_solver->SetPde(p_vegf_pde);
        p_vegf_solver->SetLabel("VEGF_Extracellular");
        p_vegf_solver->SetGrid(p_grid);
        QLength large_vessel_radius(25.0 * unit::microns);
        p_network->SetSegmentRadii(large_vessel_radius);
        QDynamicViscosity viscosity = Owen11Parameters::mpPlasmaViscosity->GetValue("User");
        p_network->SetSegmentViscosity(viscosity);
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "flow_boundary_labelled_network_flow.vtp");
        boost::shared_ptr<VesselImpedanceCalculator<3> > p_impedance_calculator = VesselImpedanceCalculator<3>::Create();
        boost::shared_ptr<ConstantHaematocritSolver<3> > p_haematocrit_calculator = ConstantHaematocritSolver<3>::Create();
        p_haematocrit_calculator->SetHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
        boost::shared_ptr<WallShearStressCalculator<3> > p_wss_calculator = WallShearStressCalculator<3>::Create();
        boost::shared_ptr<MechanicalStimulusCalculator<3> > p_mech_stimulus_calculator = MechanicalStimulusCalculator<3>::Create();
        boost::shared_ptr<MetabolicStimulusCalculator<3> > p_metabolic_stim_calculator = MetabolicStimulusCalculator<3>::Create();
        boost::shared_ptr<ShrinkingStimulusCalculator<3> > p_shrinking_stimulus_calculator = ShrinkingStimulusCalculator<3>::Create();
        boost::shared_ptr<ViscosityCalculator<3> > p_viscosity_calculator = ViscosityCalculator<3>::Create();
        boost::shared_ptr<StructuralAdaptationSolver<3> > p_structural_adaptation_solver = StructuralAdaptationSolver<3>::Create();
        p_structural_adaptation_solver->SetTolerance(0.0001);
        p_structural_adaptation_solver->SetMaxIterations(100);
        p_structural_adaptation_solver->SetTimeIncrement(Owen11Parameters::mpVesselRadiusUpdateTimestep->GetValue("User"));
        p_structural_adaptation_solver->AddPreFlowSolveCalculator(p_impedance_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_haematocrit_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_wss_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_metabolic_stim_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_mech_stimulus_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_viscosity_calculator);
        boost::shared_ptr<WallShearStressBasedRegressionSolver<3> > p_regression_solver =
                WallShearStressBasedRegressionSolver<3>::Create();
        boost::shared_ptr<AngiogenesisSolver<3> > p_angiogenesis_solver = AngiogenesisSolver<3>::Create();
        boost::shared_ptr<OffLatticeSproutingRule<3> > p_sprouting_rule = OffLatticeSproutingRule<3>::Create();
        p_sprouting_rule->SetSproutingProbability(1.e-4* unit::per_second);
        boost::shared_ptr<OffLatticeMigrationRule<3> > p_migration_rule = OffLatticeMigrationRule<3>::Create();
        p_migration_rule->SetChemotacticStrength(0.1);
        p_migration_rule->SetAttractionStrength(0.5);

        QVelocity sprout_velocity(40.0*unit::microns/unit::hours);
        p_migration_rule->SetSproutingVelocity(sprout_velocity);
        p_angiogenesis_solver->SetMigrationRule(p_migration_rule);
        p_angiogenesis_solver->SetSproutingRule(p_sprouting_rule);
        p_sprouting_rule->SetDiscreteContinuumSolver(p_vegf_solver);
        p_migration_rule->SetDiscreteContinuumSolver(p_vegf_solver);
        //p_angiogenesis_solver->SetVesselGrid(p_grid);
        p_angiogenesis_solver->SetVesselNetwork(p_network);
        boost::shared_ptr<MicrovesselSolver<3> > p_microvessel_solver = MicrovesselSolver<3>::Create();
        p_microvessel_solver->SetVesselNetwork(p_network);
        p_microvessel_solver->SetOutputFrequency(1);
        p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
        p_microvessel_solver->AddDiscreteContinuumSolver(p_vegf_solver);
        p_microvessel_solver->SetStructuralAdaptationSolver(p_structural_adaptation_solver);
        p_microvessel_solver->SetRegressionSolver(p_regression_solver);
        p_microvessel_solver->SetAngiogenesisSolver(p_angiogenesis_solver);
        //p_scene->GetCellPopulationActorGenerator()->SetColorByCellData(true);
        //p_scene->GetCellPopulationActorGenerator()->SetDataLabel("oxygen");
        boost::shared_ptr<VtkSceneMicrovesselModifier<3> > p_scene_modifier =
                boost::shared_ptr<VtkSceneMicrovesselModifier<3> >(new VtkSceneMicrovesselModifier<3>);
        p_scene_modifier->SetVtkScene(p_scene);
        p_scene_modifier->SetUpdateFrequency(3);
        p_microvessel_solver->AddMicrovesselModifier(p_scene_modifier);
        boost::shared_ptr<MicrovesselSimulationModifier<3> > p_microvessel_modifier = MicrovesselSimulationModifier<3>::Create();
        p_microvessel_modifier->SetMicrovesselSolver(p_microvessel_solver);
        std::vector<std::string> update_labels;
        update_labels.push_back("oxygen");
        update_labels.push_back("VEGF_Extracellular");
        p_microvessel_modifier->SetCellDataUpdateLabels(update_labels);
        OnLatticeSimulation<3> simulator(*p_cell_population);
        simulator.AddSimulationModifier(p_microvessel_modifier);
        boost::shared_ptr<ApoptoticCellKiller<3> > p_apoptotic_cell_killer(new ApoptoticCellKiller<3>(p_cell_population.get()));
        simulator.AddCellKiller(p_apoptotic_cell_killer);
        boost::shared_ptr<Owen2011TrackingModifier<3> > p_owen11_tracking_modifier(new Owen2011TrackingModifier<3>);
        simulator.AddSimulationModifier(p_owen11_tracking_modifier);
        simulator.SetOutputDirectory("TestBiologicalNetworkLiteratePaper");
        simulator.SetSamplingTimestepMultiple(1);
        simulator.SetDt(0.5);
        simulator.SetEndTime(200.0);
        simulator.Solve();
        ParameterCollection::Instance()->DumpToFile(p_handler->GetOutputDirectoryFullPath()+"parameter_collection.xml");
    }
};

```

