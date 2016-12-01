This tutorial is automatically generated from the file test/python/tutorials//TestPythonBiologicalNetworkLiteratePaper.py.
Note that the code is given in full at the bottom of the page.



# A Lattice Based Angiogenesis Tutorial
This tutorial is designed to introduce a lattice based angiogenesis problem based on a simplified version of the
vascular tumour application described in
[Owen et al. 2011](http://www.ncbi.nlm.nih.gov/pubmed/21363914).

It is a 3D simulation using cellular automaton
for cells, a regular grid for vessel movement and the same grid for the solution of partial differential equations
for oxygen and VEGF transport using the finite difference method.

![Lattice Based Angiogenesis Image](https://github.com/jmsgrogan/MicrovesselChaste/raw/master/test/tutorials/images/landing.png)

## The Test

```python
import unittest
import chaste.core
import chaste.cell_based
chaste.init()
import microvessel_chaste
import microvessel_chaste.geometry
import microvessel_chaste.mesh
import microvessel_chaste.population.vessel
import microvessel_chaste.pde
import microvessel_chaste.simulation
import microvessel_chaste.visualization
from microvessel_chaste.utility import * # bring in all units for convenience

class TestBiologicalNetwork(chaste.cell_based.AbstractCellBasedTestSuite):

    def test_fixed_outer_boundary(self):

```
Set up output file management and seed the random number generator.

```python
        file_handler = chaste.core.OutputFileHandler("Python/TestBiologicalNetworkLiteratePaper")
        chaste.core.RandomNumberGenerator.Instance().Reseed(12345)

```
This component uses explicit dimensions for all quantities, but interfaces with solvers which take
non-dimensional inputs. The `BaseUnits` singleton takes time, length and mass reference scales to
allow non-dimensionalisation when sending quantities to external solvers and re-dimensionalisation of
results. For our purposes microns for length and hours for time are suitable base units.

```python
        reference_length = 1.e-6*metre()
        reference_time = 3600.0*second()
        reference_concentration = 1.e-6*mole_per_metre_cubed()
        BaseUnits.Instance().SetReferenceLengthScale(reference_length)
        BaseUnits.Instance().SetReferenceTimeScale(reference_time)
        BaseUnits.Instance().SetReferenceConcentrationScale(reference_concentration)

```
Read a vessel network derived from biological images from file

```python
        vessel_reader = microvessel_chaste.population.vessel.VesselNetworkReader3()
        file_finder = chaste.core.FileFinder("/projects/MicrovesselChaste/test/data/bio_original.vtp",
                                             chaste.core.RelativeTo.ChasteSourceRoot)
        vessel_reader.SetFileName(file_finder.GetAbsolutePath())
        #vessel_reader.SetMergeCoincidentPoints(True)
        #vessel_reader.SetTargetSegmentLength(40.0e-6*metre())
        network = vessel_reader.Read()

```
The vessel network may contain short vessels due to image processing artifacts,
we remove any vessels that are on the order of a single cell length and are not connected
to other vessels at both ends. Note that units are explicitly specified for all quantities. It is
ok to allow some small disconnected regions to remain for our purposes.

```python
        short_vessel_cutoff = 40.0e-6 * metre()
        remove_end_vessels_only = True
        network.RemoveShortVessels(short_vessel_cutoff, remove_end_vessels_only)
        network.UpdateAll()
        network.MergeCoincidentNodes()
        network.UpdateAll()

```
Write the modified network to file for inspection

```python
        network.Write(file_handler.GetOutputDirectoryFullPath() + "cleaned_network.vtp")

        scene = microvessel_chaste.visualization.MicrovesselVtkScene3()
        scene.SetIsInteractive(True)
        scene.SetOutputFilePath(file_handler.GetOutputDirectoryFullPath()+"render")
        scene.SetVesselNetwork(network)
        scene.GetVesselNetworkActorGenerator().SetEdgeSize(20.0)

```
Simulating tumour growth for the entire network would be prohibitive for this tutorial, so
we sample a small region. We can use some geometry tools to help.
cylinder = microvessel_chaste.geometry.Part3()
```python
        centre = microvessel_chaste.mesh.DimensionalChastePoint3(2300.0, 2300.0, -5.0, 1.e-6*metre())
        radius = 600.0e-6*metre()
        depth = 205.e-6*metre()
        cylinder.AddCylinder(radius, depth, centre, 24)
        cylinder.BooleanWithNetwork(network)

        network.Write(file_handler.GetOutputDirectoryFullPath() + "cleaned_cut_network.vtp")

```
We are ready to simulate tumour growth and angiogenesis. We will use a regular lattice for
this purpose. We size and position the lattice according to the bounds of the vessel network.

```python
        #network_bounding_box = network.GetExtents()
        network_bounding_box = [microvessel_chaste.mesh.DimensionalChastePoint3(1500.0, 1600.0, -10.0, 1.e-6*metre()),
                                microvessel_chaste.mesh.DimensionalChastePoint3(3100.0, 3000.0, 300.0, 1.e-6*metre())]
        grid = microvessel_chaste.mesh.RegularGrid3()
        grid_spacing = 40.0e-6* metre()
        grid.SetSpacing(grid_spacing)

```
We can use the built-in dimensional analysis functionality to get the network extents in terms of grid units
botom_front_left =  network_bounding_box[0].GetLocation(grid_spacing)
```python
        top_back_right =  network_bounding_box[1].GetLocation(grid_spacing)
        extents = top_back_right - botom_front_left
        extents = [int(x)+1 for x in extents] # snap to the nearest unit, overestimate size if needed

        grid.SetExtents(extents)
        network.Translate(microvessel_chaste.mesh.DimensionalChastePoint3(-1500.0, -1600.0, +10.0, 1.e-6*metre()))

```
We can write the lattice to file for quick visualization.

```python
        grid.Write(file_handler)
        scene.SetRegularGrid(grid)

```
Next we set the inflow and outflow boundary conditions for blood flow. Because the network connectivity
is relatively low we assign all vessels near the top of the domain (z coord) as inflows and the bottom
as outflows.

```python
        for eachNode in network.GetNodes():
            if eachNode.GetNumberOfSegments() == 1:
                if abs(eachNode.rGetLocation().GetLocation(1.e-6*metre())[2] - network_bounding_box[1].GetLocation(1.e-6*metre())[2]) < 80.0:
                    eachNode.GetFlowProperties().SetIsInputNode(True)
                    eachNode.GetFlowProperties().SetPressure(Owen11Parameters.mpInletPressure.GetValue("User"))
                elif abs(eachNode.rGetLocation().GetLocation(1.e-6*metre())[2] - network_bounding_box[0].GetLocation(1.e-6*metre())[2]) < 80.0:
                    eachNode.GetFlowProperties().SetIsOutputNode(True);
                    eachNode.GetFlowProperties().SetPressure(Owen11Parameters.mpOutletPressure.GetValue("User"))

```
Again, we can write the network to file for visualization

```python
        network.Write(file_handler.GetOutputDirectoryFullPath() + "flow_boundary_labelled_network.vtp")

```
Next, set up the cell populations. We will setup up a population similar to that used in the Owen et al., 2011 paper. That is, a grid
filled with normal cells and a tumour spheroid in the middle. We can use a generator for this purpose. The generator simply sets up
the population using conventional Cell Based Chaste methods.

```python
        cell_population_genenerator = microvessel_chaste.population.cell.Owen11CellPopulationGenerator3()
        cell_population_genenerator.SetRegularGrid(grid)
        cell_population_genenerator.SetVesselNetwork(network)
        tumour_radius = 300.0 * 1.e-6 * metre()
        cell_population_genenerator.SetTumourRadius(tumour_radius)
        cell_population = cell_population_genenerator.Update()

        #scene.Start()
        #scene.StartInteractiveEventHandler()
```

At this point the simulation domain will look as follows:

![Lattice Based Angiogenesis Image](https://github.com/jmsgrogan/MicrovesselChaste/raw/master/test/tutorials/images/Lattice_Based_Tutorial_Cell_Setup.png)

Next set up the PDEs for oxygen and VEGF. Cells will act as discrete oxygen sinks and discrete vegf sources. A sample PDE solution for
oxygen is shown below:

![Lattice Based Angiogenesis Image](https://github.com/jmsgrogan/MicrovesselChaste/raw/master/test/tutorials/images/LatticeTutorialSampleOxygen.png)

```python
        oxygen_pde = microvessel_chaste.pde.LinearSteadyStateDiffusionReactionPde3_3()
        oxygen_pde.SetIsotropicDiffusionConstant(Owen11Parameters.mpOxygenDiffusivity.GetValue("User"))
        cell_oxygen_sink = microvessel_chaste.pde.CellBasedDiscreteSource3()
        cell_oxygen_sink.SetLinearInUConsumptionRatePerCell(Owen11Parameters.mpCellOxygenConsumptionRate.GetValue("User"))
        oxygen_pde.AddDiscreteSource(cell_oxygen_sink)

```
Vessels release oxygen depending on their haematocrit levels

```python
        vessel_oxygen_source = microvessel_chaste.pde.VesselBasedDiscreteSource3()
        #oxygen_solubility_at_stp = Secomb04Parameters.mpOxygenVolumetricSolubility.GetValue("User") * GenericParameters.mpGasConcentrationAtStp.GetValue("User")
        #vessel_oxygen_concentration = oxygen_solubility_at_stp * Owen11Parameters.mpReferencePartialPressure.GetValue("User")
        vessel_oxygen_concentration = 0.03 * mole_per_metre_cubed()
        vessel_oxygen_source.SetReferenceConcentration(vessel_oxygen_concentration)
        vessel_oxygen_source.SetVesselPermeability(Owen11Parameters.mpVesselOxygenPermeability.GetValue("User"))
        vessel_oxygen_source.SetReferenceHaematocrit(Owen11Parameters.mpInflowHaematocrit.GetValue("User"))
        oxygen_pde.AddDiscreteSource(vessel_oxygen_source);

```
Set up a finite difference solver and pass it the pde and grid.

```python
        oxygen_solver = microvessel_chaste.pde.FiniteDifferenceSolver3()
        oxygen_solver.SetPde(oxygen_pde)
        oxygen_solver.SetLabel("oxygen")
        oxygen_solver.SetGrid(grid)

```
The rate of VEGF release depends on the cell type and intracellular VEGF levels, so we need a more detailed
type of discrete source. A sample PDE solution for VEGF is shown below.

![Lattice Based Angiogenesis Image](https://github.com/jmsgrogan/MicrovesselChaste/raw/master/test/tutorials/images/LatticeTutorialSampleVegf.png)

```python
        vegf_pde = microvessel_chaste.pde.LinearSteadyStateDiffusionReactionPde3_3()
        vegf_pde.SetIsotropicDiffusionConstant(Owen11Parameters.mpVegfDiffusivity.GetValue("User"))
        vegf_pde.SetContinuumLinearInUTerm(-1.0 * Owen11Parameters.mpVegfDecayRate.GetValue("User"))

```
Set up a map for different release rates depending on cell type. Also include a threshold intracellular VEGF below which
there is no release.

```python
        normal_and_quiescent_cell_source = microvessel_chaste.pde.CellStateDependentDiscreteSource3()
        normal_and_quiescent_cell_rates = microvessel_chaste.pde.MapUnsigned_ConcentrationFlowRate()
        normal_and_quiescent_cell_rate_thresholds = microvessel_chaste.pde.MapUnsigned_Concentration()
        quiescent_cancer_state = microvessel_chaste.population.cell.QuiescentCancerCellMutationState()
        normal_cell_state = chaste.cell_based.WildTypeCellMutationState()

        normal_and_quiescent_cell_rates[normal_cell_state.GetColour()] = Owen11Parameters.mpCellVegfSecretionRate.GetValue("User")
        normal_and_quiescent_cell_rate_thresholds[normal_cell_state.GetColour()] = 0.27*mole_per_metre_cubed()
        normal_and_quiescent_cell_rates[quiescent_cancer_state.GetColour()] = Owen11Parameters.mpCellVegfSecretionRate.GetValue("User")
        normal_and_quiescent_cell_rate_thresholds[quiescent_cancer_state.GetColour()] = 0.0*mole_per_metre_cubed()
        normal_and_quiescent_cell_source.SetStateRateMap(normal_and_quiescent_cell_rates)
        normal_and_quiescent_cell_source.SetLabelName("VEGF")
        normal_and_quiescent_cell_source.SetStateRateThresholdMap(normal_and_quiescent_cell_rate_thresholds)
        vegf_pde.AddDiscreteSource(normal_and_quiescent_cell_source)

```
Add a vessel related VEGF sink

```python
        vessel_vegf_sink = microvessel_chaste.pde.VesselBasedDiscreteSource3()
        vessel_vegf_sink.SetReferenceConcentration(0.0*mole_per_metre_cubed())
        vessel_vegf_sink.SetVesselPermeability(Owen11Parameters.mpVesselVegfPermeability.GetValue("User"))
        vegf_pde.AddDiscreteSource(vessel_vegf_sink)

```
Set up a finite difference solver as before.

vegf_solver = microvessel_chaste.pde.FiniteDifferenceSolver3()
```python
        vegf_solver.SetPde(vegf_pde)
        vegf_solver.SetLabel("VEGF_Extracellular")
        vegf_solver.SetGrid(grid)

```
Next set up the flow problem. Assign a blood plasma viscosity to the vessels. The actual viscosity will
depend on haematocrit and diameter. This solver manages growth and shrinkage of vessels in response to
flow related stimuli. A sample plot of the stimulus distrbution during a simulation is shown below:

![Lattice Based Angiogenesis Image](https://github.com/jmsgrogan/MicrovesselChaste/raw/master/test/tutorials/images/LatticeTutorialSampleGrowth.png)

```python
        large_vessel_radius = 25.0e-6 * metre()
        network.SetSegmentRadii(large_vessel_radius)
        viscosity = Owen11Parameters.mpPlasmaViscosity.GetValue("User")
        network.SetSegmentViscosity(viscosity);

```
Set up the pre- and post flow calculators.

```python
        impedance_calculator = microvessel_chaste.simulation.VesselImpedanceCalculator3()
        haematocrit_calculator = microvessel_chaste.simulation.ConstantHaematocritSolver3()
        haematocrit_calculator.SetHaematocrit(Owen11Parameters.mpInflowHaematocrit.GetValue("User"))
        wss_calculator = microvessel_chaste.simulation.WallShearStressCalculator3()
        mech_stimulus_calculator = microvessel_chaste.simulation.MechanicalStimulusCalculator3()
        metabolic_stim_calculator = microvessel_chaste.simulation.MetabolicStimulusCalculator3()
        shrinking_stimulus_calculator = microvessel_chaste.simulation.ShrinkingStimulusCalculator3()
        viscosity_calculator = microvessel_chaste.simulation.ViscosityCalculator3()

```
Set up and configure the structural adaptation solver.

```python
        structural_adaptation_solver = microvessel_chaste.simulation.StructuralAdaptationSolver3()
        structural_adaptation_solver.SetTolerance(0.0001)
        structural_adaptation_solver.SetMaxIterations(100)
        structural_adaptation_solver.SetTimeIncrement(Owen11Parameters.mpVesselRadiusUpdateTimestep.GetValue("User"));
        structural_adaptation_solver.AddPreFlowSolveCalculator(impedance_calculator)
        structural_adaptation_solver.AddPostFlowSolveCalculator(haematocrit_calculator)
        structural_adaptation_solver.AddPostFlowSolveCalculator(wss_calculator)
        structural_adaptation_solver.AddPostFlowSolveCalculator(metabolic_stim_calculator)
        structural_adaptation_solver.AddPostFlowSolveCalculator(mech_stimulus_calculator)
        structural_adaptation_solver.AddPostFlowSolveCalculator(viscosity_calculator)

```
Set up a regression solver.

```python
        regression_solver = microvessel_chaste.simulation.WallShearStressBasedRegressionSolver3()

```
Set up an angiogenesis solver and add sprouting and migration rules.

```python
        angiogenesis_solver = microvessel_chaste.simulation.AngiogenesisSolver3()
        sprouting_rule = microvessel_chaste.simulation.Owen2011SproutingRule3()
        sprouting_rule.SetSproutingProbability(1.e-4*per_second())
        migration_rule = microvessel_chaste.simulation.Owen2011MigrationRule3()
        angiogenesis_solver.SetMigrationRule(migration_rule)
        angiogenesis_solver.SetSproutingRule(sprouting_rule)
        sprouting_rule.SetDiscreteContinuumSolver(vegf_solver)
        migration_rule.SetDiscreteContinuumSolver(vegf_solver)
        angiogenesis_solver.SetVesselGrid(grid)
        angiogenesis_solver.SetVesselNetwork(network)

```
The microvessel solver will manage all aspects of the vessel solve.

```python
        microvessel_solver = microvessel_chaste.simulation.MicrovesselSolver3()
        microvessel_solver.SetVesselNetwork(network)
        microvessel_solver.SetOutputFrequency(5)
        microvessel_solver.AddDiscreteContinuumSolver(oxygen_solver)
        microvessel_solver.AddDiscreteContinuumSolver(vegf_solver)
        microvessel_solver.SetStructuralAdaptationSolver(structural_adaptation_solver)
        microvessel_solver.SetRegressionSolver(regression_solver)
        microvessel_solver.SetAngiogenesisSolver(angiogenesis_solver)

```
The microvessel solution modifier will link the vessel and cell solvers. We need to explicitly tell is
which extracellular fields to update based on PDE solutions.

```python
        microvessel_modifier = microvessel_chaste.simulation.MicrovesselSimulationModifier3()
        microvessel_modifier.SetMicrovesselSolver(microvessel_solver)
        update_labels = microvessel_chaste.simulation.VecString()
        update_labels.append("oxygen")
        update_labels.append("VEGF_Extracellular")
        microvessel_modifier.SetCellDataUpdateLabels(update_labels)

```
The full simulation is run as a typical Cell Based Chaste simulation

```python
        simulator = chaste.cell_based.OnLatticeSimulation3(cell_population)
        simulator.AddSimulationModifier(microvessel_modifier)

```
Add a killer to remove apoptotic cells

```python
        apoptotic_cell_killer = chaste.cell_based.ApoptoticCellKiller3(cell_population)
        simulator.AddCellKiller(apoptotic_cell_killer)

```
Add another modifier for updating cell cycle quantities.

```python
        owen11_tracking_modifier = microvessel_chaste.simulation.Owen2011TrackingModifier3()
        simulator.AddSimulationModifier(owen11_tracking_modifier)

```
Set up the remainder of the simulation

```python
        simulator.SetOutputDirectory("Python/TestBiologicalNetworkLiteratePaper")
        simulator.SetSamplingTimestepMultiple(5)
        simulator.SetDt(0.5)

```
This end time corresponds to roughly 10 minutes run-time on a desktop PC. Increase it or decrease as
preferred. The end time used in Owen et al. 2011 is 4800 hours.

```python
        simulator.SetEndTime(100.0)

```
Do the solve. A sample solution is shown at the top of this test.

try:
```python
            simulator.Solve()
        except chaste.ChasteException as e:
            print e.GetMessage

```
Dump the parameters to file for inspection.

```python
        ParameterCollection.Instance().DumpToFile(file_handler.GetOutputDirectoryFullPath()+"parameter_collection.xml")

if __name__ == '__main__':
    unittest.main(verbosity=2)

```


# Code 
The full code is given below


## File name `TestPythonBiologicalNetworkLiteratePaper.py` 

```python
import unittest
import chaste.core
import chaste.cell_based
chaste.init()
import microvessel_chaste
import microvessel_chaste.geometry
import microvessel_chaste.mesh
import microvessel_chaste.population.vessel
import microvessel_chaste.pde
import microvessel_chaste.simulation
import microvessel_chaste.visualization
from microvessel_chaste.utility import * # bring in all units for convenience

class TestBiologicalNetwork(chaste.cell_based.AbstractCellBasedTestSuite):

    def test_fixed_outer_boundary(self):

        file_handler = chaste.core.OutputFileHandler("Python/TestBiologicalNetworkLiteratePaper")
        chaste.core.RandomNumberGenerator.Instance().Reseed(12345)

        reference_length = 1.e-6*metre()
        reference_time = 3600.0*second()
        reference_concentration = 1.e-6*mole_per_metre_cubed()
        BaseUnits.Instance().SetReferenceLengthScale(reference_length)
        BaseUnits.Instance().SetReferenceTimeScale(reference_time)
        BaseUnits.Instance().SetReferenceConcentrationScale(reference_concentration)

        vessel_reader = microvessel_chaste.population.vessel.VesselNetworkReader3()
        file_finder = chaste.core.FileFinder("/projects/MicrovesselChaste/test/data/bio_original.vtp",
                                             chaste.core.RelativeTo.ChasteSourceRoot)
        vessel_reader.SetFileName(file_finder.GetAbsolutePath())
        #vessel_reader.SetMergeCoincidentPoints(True)
        #vessel_reader.SetTargetSegmentLength(40.0e-6*metre())
        network = vessel_reader.Read()

        short_vessel_cutoff = 40.0e-6 * metre()
        remove_end_vessels_only = True
        network.RemoveShortVessels(short_vessel_cutoff, remove_end_vessels_only)
        network.UpdateAll()
        network.MergeCoincidentNodes()
        network.UpdateAll()

        network.Write(file_handler.GetOutputDirectoryFullPath() + "cleaned_network.vtp")

        scene = microvessel_chaste.visualization.MicrovesselVtkScene3()
        scene.SetIsInteractive(True)
        scene.SetOutputFilePath(file_handler.GetOutputDirectoryFullPath()+"render")
        scene.SetVesselNetwork(network)
        scene.GetVesselNetworkActorGenerator().SetEdgeSize(20.0)

        centre = microvessel_chaste.mesh.DimensionalChastePoint3(2300.0, 2300.0, -5.0, 1.e-6*metre())
        radius = 600.0e-6*metre()
        depth = 205.e-6*metre()
        cylinder.AddCylinder(radius, depth, centre, 24)
        cylinder.BooleanWithNetwork(network)

        network.Write(file_handler.GetOutputDirectoryFullPath() + "cleaned_cut_network.vtp")

        #network_bounding_box = network.GetExtents()
        network_bounding_box = [microvessel_chaste.mesh.DimensionalChastePoint3(1500.0, 1600.0, -10.0, 1.e-6*metre()),
                                microvessel_chaste.mesh.DimensionalChastePoint3(3100.0, 3000.0, 300.0, 1.e-6*metre())]
        grid = microvessel_chaste.mesh.RegularGrid3()
        grid_spacing = 40.0e-6* metre()
        grid.SetSpacing(grid_spacing)

        top_back_right =  network_bounding_box[1].GetLocation(grid_spacing)
        extents = top_back_right - botom_front_left
        extents = [int(x)+1 for x in extents] # snap to the nearest unit, overestimate size if needed

        grid.SetExtents(extents)
        network.Translate(microvessel_chaste.mesh.DimensionalChastePoint3(-1500.0, -1600.0, +10.0, 1.e-6*metre()))

        grid.Write(file_handler)
        scene.SetRegularGrid(grid)

        for eachNode in network.GetNodes():
            if eachNode.GetNumberOfSegments() == 1:
                if abs(eachNode.rGetLocation().GetLocation(1.e-6*metre())[2] - network_bounding_box[1].GetLocation(1.e-6*metre())[2]) < 80.0:
                    eachNode.GetFlowProperties().SetIsInputNode(True)
                    eachNode.GetFlowProperties().SetPressure(Owen11Parameters.mpInletPressure.GetValue("User"))
                elif abs(eachNode.rGetLocation().GetLocation(1.e-6*metre())[2] - network_bounding_box[0].GetLocation(1.e-6*metre())[2]) < 80.0:
                    eachNode.GetFlowProperties().SetIsOutputNode(True);
                    eachNode.GetFlowProperties().SetPressure(Owen11Parameters.mpOutletPressure.GetValue("User"))

        network.Write(file_handler.GetOutputDirectoryFullPath() + "flow_boundary_labelled_network.vtp")

        cell_population_genenerator = microvessel_chaste.population.cell.Owen11CellPopulationGenerator3()
        cell_population_genenerator.SetRegularGrid(grid)
        cell_population_genenerator.SetVesselNetwork(network)
        tumour_radius = 300.0 * 1.e-6 * metre()
        cell_population_genenerator.SetTumourRadius(tumour_radius)
        cell_population = cell_population_genenerator.Update()

        #scene.Start()
        #scene.StartInteractiveEventHandler()
        oxygen_pde = microvessel_chaste.pde.LinearSteadyStateDiffusionReactionPde3_3()
        oxygen_pde.SetIsotropicDiffusionConstant(Owen11Parameters.mpOxygenDiffusivity.GetValue("User"))
        cell_oxygen_sink = microvessel_chaste.pde.CellBasedDiscreteSource3()
        cell_oxygen_sink.SetLinearInUConsumptionRatePerCell(Owen11Parameters.mpCellOxygenConsumptionRate.GetValue("User"))
        oxygen_pde.AddDiscreteSource(cell_oxygen_sink)

        vessel_oxygen_source = microvessel_chaste.pde.VesselBasedDiscreteSource3()
        #oxygen_solubility_at_stp = Secomb04Parameters.mpOxygenVolumetricSolubility.GetValue("User") * GenericParameters.mpGasConcentrationAtStp.GetValue("User")
        #vessel_oxygen_concentration = oxygen_solubility_at_stp * Owen11Parameters.mpReferencePartialPressure.GetValue("User")
        vessel_oxygen_concentration = 0.03 * mole_per_metre_cubed()
        vessel_oxygen_source.SetReferenceConcentration(vessel_oxygen_concentration)
        vessel_oxygen_source.SetVesselPermeability(Owen11Parameters.mpVesselOxygenPermeability.GetValue("User"))
        vessel_oxygen_source.SetReferenceHaematocrit(Owen11Parameters.mpInflowHaematocrit.GetValue("User"))
        oxygen_pde.AddDiscreteSource(vessel_oxygen_source);

        oxygen_solver = microvessel_chaste.pde.FiniteDifferenceSolver3()
        oxygen_solver.SetPde(oxygen_pde)
        oxygen_solver.SetLabel("oxygen")
        oxygen_solver.SetGrid(grid)

        vegf_pde = microvessel_chaste.pde.LinearSteadyStateDiffusionReactionPde3_3()
        vegf_pde.SetIsotropicDiffusionConstant(Owen11Parameters.mpVegfDiffusivity.GetValue("User"))
        vegf_pde.SetContinuumLinearInUTerm(-1.0 * Owen11Parameters.mpVegfDecayRate.GetValue("User"))

        normal_and_quiescent_cell_source = microvessel_chaste.pde.CellStateDependentDiscreteSource3()
        normal_and_quiescent_cell_rates = microvessel_chaste.pde.MapUnsigned_ConcentrationFlowRate()
        normal_and_quiescent_cell_rate_thresholds = microvessel_chaste.pde.MapUnsigned_Concentration()
        quiescent_cancer_state = microvessel_chaste.population.cell.QuiescentCancerCellMutationState()
        normal_cell_state = chaste.cell_based.WildTypeCellMutationState()

        normal_and_quiescent_cell_rates[normal_cell_state.GetColour()] = Owen11Parameters.mpCellVegfSecretionRate.GetValue("User")
        normal_and_quiescent_cell_rate_thresholds[normal_cell_state.GetColour()] = 0.27*mole_per_metre_cubed()
        normal_and_quiescent_cell_rates[quiescent_cancer_state.GetColour()] = Owen11Parameters.mpCellVegfSecretionRate.GetValue("User")
        normal_and_quiescent_cell_rate_thresholds[quiescent_cancer_state.GetColour()] = 0.0*mole_per_metre_cubed()
        normal_and_quiescent_cell_source.SetStateRateMap(normal_and_quiescent_cell_rates)
        normal_and_quiescent_cell_source.SetLabelName("VEGF")
        normal_and_quiescent_cell_source.SetStateRateThresholdMap(normal_and_quiescent_cell_rate_thresholds)
        vegf_pde.AddDiscreteSource(normal_and_quiescent_cell_source)

        vessel_vegf_sink = microvessel_chaste.pde.VesselBasedDiscreteSource3()
        vessel_vegf_sink.SetReferenceConcentration(0.0*mole_per_metre_cubed())
        vessel_vegf_sink.SetVesselPermeability(Owen11Parameters.mpVesselVegfPermeability.GetValue("User"))
        vegf_pde.AddDiscreteSource(vessel_vegf_sink)

        vegf_solver.SetPde(vegf_pde)
        vegf_solver.SetLabel("VEGF_Extracellular")
        vegf_solver.SetGrid(grid)

        large_vessel_radius = 25.0e-6 * metre()
        network.SetSegmentRadii(large_vessel_radius)
        viscosity = Owen11Parameters.mpPlasmaViscosity.GetValue("User")
        network.SetSegmentViscosity(viscosity);

        impedance_calculator = microvessel_chaste.simulation.VesselImpedanceCalculator3()
        haematocrit_calculator = microvessel_chaste.simulation.ConstantHaematocritSolver3()
        haematocrit_calculator.SetHaematocrit(Owen11Parameters.mpInflowHaematocrit.GetValue("User"))
        wss_calculator = microvessel_chaste.simulation.WallShearStressCalculator3()
        mech_stimulus_calculator = microvessel_chaste.simulation.MechanicalStimulusCalculator3()
        metabolic_stim_calculator = microvessel_chaste.simulation.MetabolicStimulusCalculator3()
        shrinking_stimulus_calculator = microvessel_chaste.simulation.ShrinkingStimulusCalculator3()
        viscosity_calculator = microvessel_chaste.simulation.ViscosityCalculator3()

        structural_adaptation_solver = microvessel_chaste.simulation.StructuralAdaptationSolver3()
        structural_adaptation_solver.SetTolerance(0.0001)
        structural_adaptation_solver.SetMaxIterations(100)
        structural_adaptation_solver.SetTimeIncrement(Owen11Parameters.mpVesselRadiusUpdateTimestep.GetValue("User"));
        structural_adaptation_solver.AddPreFlowSolveCalculator(impedance_calculator)
        structural_adaptation_solver.AddPostFlowSolveCalculator(haematocrit_calculator)
        structural_adaptation_solver.AddPostFlowSolveCalculator(wss_calculator)
        structural_adaptation_solver.AddPostFlowSolveCalculator(metabolic_stim_calculator)
        structural_adaptation_solver.AddPostFlowSolveCalculator(mech_stimulus_calculator)
        structural_adaptation_solver.AddPostFlowSolveCalculator(viscosity_calculator)

        regression_solver = microvessel_chaste.simulation.WallShearStressBasedRegressionSolver3()

        angiogenesis_solver = microvessel_chaste.simulation.AngiogenesisSolver3()
        sprouting_rule = microvessel_chaste.simulation.Owen2011SproutingRule3()
        sprouting_rule.SetSproutingProbability(1.e-4*per_second())
        migration_rule = microvessel_chaste.simulation.Owen2011MigrationRule3()
        angiogenesis_solver.SetMigrationRule(migration_rule)
        angiogenesis_solver.SetSproutingRule(sprouting_rule)
        sprouting_rule.SetDiscreteContinuumSolver(vegf_solver)
        migration_rule.SetDiscreteContinuumSolver(vegf_solver)
        angiogenesis_solver.SetVesselGrid(grid)
        angiogenesis_solver.SetVesselNetwork(network)

        microvessel_solver = microvessel_chaste.simulation.MicrovesselSolver3()
        microvessel_solver.SetVesselNetwork(network)
        microvessel_solver.SetOutputFrequency(5)
        microvessel_solver.AddDiscreteContinuumSolver(oxygen_solver)
        microvessel_solver.AddDiscreteContinuumSolver(vegf_solver)
        microvessel_solver.SetStructuralAdaptationSolver(structural_adaptation_solver)
        microvessel_solver.SetRegressionSolver(regression_solver)
        microvessel_solver.SetAngiogenesisSolver(angiogenesis_solver)

        microvessel_modifier = microvessel_chaste.simulation.MicrovesselSimulationModifier3()
        microvessel_modifier.SetMicrovesselSolver(microvessel_solver)
        update_labels = microvessel_chaste.simulation.VecString()
        update_labels.append("oxygen")
        update_labels.append("VEGF_Extracellular")
        microvessel_modifier.SetCellDataUpdateLabels(update_labels)

        simulator = chaste.cell_based.OnLatticeSimulation3(cell_population)
        simulator.AddSimulationModifier(microvessel_modifier)

        apoptotic_cell_killer = chaste.cell_based.ApoptoticCellKiller3(cell_population)
        simulator.AddCellKiller(apoptotic_cell_killer)

        owen11_tracking_modifier = microvessel_chaste.simulation.Owen2011TrackingModifier3()
        simulator.AddSimulationModifier(owen11_tracking_modifier)

        simulator.SetOutputDirectory("Python/TestBiologicalNetworkLiteratePaper")
        simulator.SetSamplingTimestepMultiple(5)
        simulator.SetDt(0.5)

        simulator.SetEndTime(100.0)

            simulator.Solve()
        except chaste.ChasteException as e:
            print e.GetMessage

        ParameterCollection.Instance().DumpToFile(file_handler.GetOutputDirectoryFullPath()+"parameter_collection.xml")

if __name__ == '__main__':
    unittest.main(verbosity=2)

```

