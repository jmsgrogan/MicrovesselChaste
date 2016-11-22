This tutorial is automatically generated from the file test/python/tutorials//TestPythonLatticeBasedAngiogenesisTutorial.py.
Note that the code is given in full at the bottom of the page.



# A Lattice Based Angiogenesis Tutorial
This tutorial is designed to introduce a lattice based angiogenesis problem based on a simplified version of the
vascular tumour application described in
[Owen et al. 2011](http://www.ncbi.nlm.nih.gov/pubmed/21363914).

It is a 2D simulation using cellular automaton
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
from microvessel_chaste.utility import * # bring in all units for convenience

class TestLatticeBasedAngiogenesis(chaste.cell_based.AbstractCellBasedTestSuite):

    def test_fixed_outer_boundary(self):

```
Set up output file management and seed the random number generator.

```python
        file_handler = chaste.core.OutputFileHandler("Python/TestLatticeBasedAngiogenesisTutorial")
        chaste.core.RandomNumberGenerator.Instance().Reseed(12345)

```
This component uses explicit dimensions for all quantities, but interfaces with solvers which take
non-dimensional inputs. The `BaseUnits` singleton takes time, length and mass reference scales to
allow non-dimensionalisation when sending quantities to external solvers and re-dimensionalisation of
results. For our purposes microns for length and hours for time are suitable base units.

```python
        reference_length = 1.e-6 * metre()
        reference_time = 3600.0 * second()
        BaseUnits.Instance().SetReferenceLengthScale(reference_length)
        BaseUnits.Instance().SetReferenceTimeScale(reference_time)

```
Set up the lattice (grid), we will use the same dimensions as [Owen et al. 2011](http://www.ncbi.nlm.nih.gov/pubmed/21363914).
Note that we are using hard-coded parameters from that paper. You can see the values by inspecting `Owen11Parameters.cpp`.
Alternatively each parameter supports the `<<` operator for streaming. When we get the value of the parameter by doing
`Owen11Parameters::mpLatticeSpacing->GetValue("User")` a record is kept that this parameter has been used in the simulation.
A record of all parameters used in a simulation can be dumped to file on completion, as will be shown below.

```python
        grid = microvessel_chaste.mesh.RegularGrid2()
        grid_spacing = Owen11Parameters.mpLatticeSpacing.GetValue("User")
        grid.SetSpacing(grid_spacing)
        grid.SetExtents([51, 51, 1])

```
We can write the lattice to file for quick visualization with Paraview. Rendering of this and subsequent images is performed
using standard Paraview operations, not detailed here.

![Lattice Based Angiogenesis Image](https://github.com/jmsgrogan/MicrovesselChaste/raw/master/test/tutorials/images/Lattice_Tutorial_Initial_Grid.png)

```python
        grid.Write(file_handler)

```
Next, set up the vessel network, this will initially consist of two, large counter-flowing vessels. Also set the inlet
and outlet pressures and flags.

```python
        node1 = microvessel_chaste.population.vessel.VesselNode2.Create(0.0, 400.0, 0.0, reference_length)
        node2 = microvessel_chaste.population.vessel.VesselNode2.Create(2000.0, 400.0, 0.0, reference_length)
        node1.GetFlowProperties().SetIsInputNode(True)
        node1.GetFlowProperties().SetPressure(Owen11Parameters.mpInletPressure.GetValue("User"))
        node2.GetFlowProperties().SetIsOutputNode(True);
        node2.GetFlowProperties().SetPressure(Owen11Parameters.mpOutletPressure.GetValue("User"))
        node3 = microvessel_chaste.population.vessel.VesselNode2.Create(2000.0, 1600.0, 0.0, reference_length)
        node4 = microvessel_chaste.population.vessel.VesselNode2.Create(0.0, 1600.0, 0.0, reference_length)
        node3.GetFlowProperties().SetIsInputNode(True)
        node3.GetFlowProperties().SetPressure(Owen11Parameters.mpInletPressure.GetValue("User"))
        node4.GetFlowProperties().SetIsOutputNode(True)
        node4.GetFlowProperties().SetPressure(Owen11Parameters.mpOutletPressure.GetValue("User"))
        vessel1 = microvessel_chaste.population.vessel.Vessel2.Create(node1, node2)
        vessel2 = microvessel_chaste.population.vessel.Vessel2.Create(node3, node4)
        network = microvessel_chaste.population.vessel.VesselNetwork2.Create()
        network.AddVessel(vessel1)
        network.AddVessel(vessel2)

```
Again, we can write the network to file for quick visualization with Paraview.

![Lattice Based Angiogenesis Image](https://github.com/jmsgrogan/MicrovesselChaste/raw/master/test/tutorials/images/Lattice_Angiogenesis_Tutorial_Grid_Vessels.png)

```python
        network.Write(file_handler.GetOutputDirectoryFullPath() + "initial_network.vtp")

```
Next, set up the cell populations. We will setup up a population similar to that used in the Owen et al., 2011 paper. That is, a grid
filled with normal cells and a tumour spheroid in the middle. We can use a generator for this purpose. The generator simply sets up
the population using conventional Cell Based Chaste methods.

```python
        cell_population_genenerator = microvessel_chaste.population.cell.Owen11CellPopulationGenerator2()
        cell_population_genenerator.SetRegularGrid(grid)
        cell_population_genenerator.SetVesselNetwork(network)
        tumour_radius = 300.0 * 1.e-6 * metre()
        cell_population_genenerator.SetTumourRadius(tumour_radius)
        cell_population = cell_population_genenerator.Update()

```
At this point the simulation domain will look as follows:

![Lattice Based Angiogenesis Image](https://github.com/jmsgrogan/MicrovesselChaste/raw/master/test/tutorials/images/Lattice_Based_Tutorial_Cell_Setup.png)

Next set up the PDEs for oxygen and VEGF. Cells will act as discrete oxygen sinks and discrete vegf sources. A sample PDE solution for
oxygen is shown below:

![Lattice Based Angiogenesis Image](https://github.com/jmsgrogan/MicrovesselChaste/raw/master/test/tutorials/images/LatticeTutorialSampleOxygen.png)

```python
        oxygen_pde = microvessel_chaste.pde.LinearSteadyStateDiffusionReactionPde2_2()
        oxygen_pde.SetIsotropicDiffusionConstant(Owen11Parameters.mpOxygenDiffusivity.GetValue("User"))
        cell_oxygen_sink = microvessel_chaste.pde.CellBasedDiscreteSource2()
        cell_oxygen_sink.SetLinearInUConsumptionRatePerCell(Owen11Parameters.mpCellOxygenConsumptionRate.GetValue("User"))
        oxygen_pde.AddDiscreteSource(cell_oxygen_sink)

```
Vessels release oxygen depending on their haematocrit levels

```python
        vessel_oxygen_source = microvessel_chaste.pde.VesselBasedDiscreteSource2()
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
        oxygen_solver = microvessel_chaste.pde.FiniteDifferenceSolver2()
        oxygen_solver.SetPde(oxygen_pde)
        oxygen_solver.SetLabel("oxygen")
        oxygen_solver.SetGrid(grid)

```
The rate of VEGF release depends on the cell type and intracellular VEGF levels, so we need a more detailed
type of discrete source. A sample PDE solution for VEGF is shown below.

![Lattice Based Angiogenesis Image](https://github.com/jmsgrogan/MicrovesselChaste/raw/master/test/tutorials/images/LatticeTutorialSampleVegf.png)

```python
        vegf_pde = microvessel_chaste.pde.LinearSteadyStateDiffusionReactionPde2_2()
        vegf_pde.SetIsotropicDiffusionConstant(Owen11Parameters.mpVegfDiffusivity.GetValue("User"))
        vegf_pde.SetContinuumLinearInUTerm(-1.0 * Owen11Parameters.mpVegfDecayRate.GetValue("User"))

```
Set up a map for different release rates depending on cell type. Also include a threshold intracellular VEGF below which
there is no release.

```python
        normal_and_quiescent_cell_source = microvessel_chaste.pde.CellStateDependentDiscreteSource2()
        normal_and_quiescent_cell_rates = microvessel_chaste.pde.MapUnsigned_ConcentrationFlowRate()
        normal_and_quiescent_cell_rate_thresholds = microvessel_chaste.pde.MapUnsigned_Concentration()
        quiescent_cancer_state = microvessel_chaste.population.cell.QuiescentCancerCellMutationState()
        normal_cell_state = chaste.cell_based.WildTypeCellMutationState()

#        normal_and_quiescent_cell_rates[normal_cell_state.GetColour()] = Owen11Parameters.mpCellVegfSecretionRate.GetValue("User")
        normal_and_quiescent_cell_rate_thresholds[normal_cell_state.GetColour()] = 0.27*mole_per_metre_cubed()
#        normal_and_quiescent_cell_rates[quiescent_cancer_state.GetColour()] = Owen11Parameters.mpCellVegfSecretionRate.GetValue("User")
        normal_and_quiescent_cell_rate_thresholds[quiescent_cancer_state.GetColour()] = 0.0*mole_per_metre_cubed()
#        normal_and_quiescent_cell_source.SetStateRateMap(normal_and_quiescent_cell_rates)
        normal_and_quiescent_cell_source.SetLabelName("VEGF")
        normal_and_quiescent_cell_source.SetStateRateThresholdMap(normal_and_quiescent_cell_rate_thresholds)
        vegf_pde.AddDiscreteSource(normal_and_quiescent_cell_source)

```
Add a vessel related VEGF sink

```python
        vessel_vegf_sink = microvessel_chaste.pde.VesselBasedDiscreteSource2()
        vessel_vegf_sink.SetReferenceConcentration(0.0*mole_per_metre_cubed())
        vessel_vegf_sink.SetVesselPermeability(Owen11Parameters.mpVesselVegfPermeability.GetValue("User"))
        vegf_pde.AddDiscreteSource(vessel_vegf_sink)

```
Set up a finite difference solver as before.

vegf_solver = microvessel_chaste.pde.FiniteDifferenceSolver2()
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
        impedance_calculator = microvessel_chaste.simulation.VesselImpedanceCalculator2()
        haematocrit_calculator = microvessel_chaste.simulation.ConstantHaematocritSolver2()
        haematocrit_calculator.SetHaematocrit(Owen11Parameters.mpInflowHaematocrit.GetValue("User"))
        wss_calculator = microvessel_chaste.simulation.WallShearStressCalculator2()
        mech_stimulus_calculator = microvessel_chaste.simulation.MechanicalStimulusCalculator2()
        metabolic_stim_calculator = microvessel_chaste.simulation.MetabolicStimulusCalculator2()
        shrinking_stimulus_calculator = microvessel_chaste.simulation.ShrinkingStimulusCalculator2()
        viscosity_calculator = microvessel_chaste.simulation.ViscosityCalculator2()

```
Set up and configure the structural adaptation solver.

```python
        structural_adaptation_solver = microvessel_chaste.simulation.StructuralAdaptationSolver2()
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
        regression_solver = microvessel_chaste.simulation.WallShearStressBasedRegressionSolver2()

```
Set up an angiogenesis solver and add sprouting and migration rules.

```python
        angiogenesis_solver = microvessel_chaste.simulation.AngiogenesisSolver2()
        sprouting_rule = microvessel_chaste.simulation.Owen2011SproutingRule2()
        migration_rule = microvessel_chaste.simulation.Owen2011MigrationRule2()
        angiogenesis_solver.SetMigrationRule(migration_rule)
        angiogenesis_solver.SetSproutingRule(sprouting_rule)
        sprouting_rule.SetDiscreteContinuumSolver(vegf_solver)
        migration_rule.SetDiscreteContinuumSolver(vegf_solver)
        angiogenesis_solver.SetVesselGrid(grid)
        angiogenesis_solver.SetVesselNetwork(network)

```
The microvessel solver will manage all aspects of the vessel solve.

```python
        microvessel_solver = microvessel_chaste.simulation.MicrovesselSolver2()
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
        microvessel_modifier = microvessel_chaste.simulation.MicrovesselSimulationModifier2()
        microvessel_modifier.SetMicrovesselSolver(microvessel_solver)
        update_labels = microvessel_chaste.simulation.VecString()
        update_labels.append("oxygen")
        update_labels.append("VEGF_Extracellular")
        microvessel_modifier.SetCellDataUpdateLabels(update_labels)

```
The full simulation is run as a typical Cell Based Chaste simulation

```python
        simulator = chaste.cell_based.OnLatticeSimulation2(cell_population)
        simulator.AddSimulationModifier(microvessel_modifier)

```
Add a killer to remove apoptotic cells

```python
        apoptotic_cell_killer = chaste.cell_based.ApoptoticCellKiller2(cell_population)
        simulator.AddCellKiller(apoptotic_cell_killer)

```
Add another modifier for updating cell cycle quantities.

```python
        owen11_tracking_modifier = microvessel_chaste.simulation.Owen2011TrackingModifier2()
        simulator.AddSimulationModifier(owen11_tracking_modifier)

```
Set up the remainder of the simulation

```python
        simulator.SetOutputDirectory("Python/TestLatticeBasedAngiogenesisLiteratePaper")
        simulator.SetSamplingTimestepMultiple(5)
        simulator.SetDt(0.5)

```
This end time corresponds to roughly 10 minutes run-time on a desktop PC. Increase it or decrease as
preferred. The end time used in Owen et al. 2011 is 4800 hours.

```python
        simulator.SetEndTime(20.0)

```
Do the solve. A sample solution is shown at the top of this test.

```python
        simulator.Solve()

```
Dump the parameters to file for inspection.

```python
        ParameterCollection.Instance().DumpToFile(file_handler.GetOutputDirectoryFullPath()+"parameter_collection.xml")

if __name__ == '__main__':
    unittest.main(verbosity=2)

```


# Code 
The full code is given below


## File name `TestPythonLatticeBasedAngiogenesisTutorial.py` 

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
from microvessel_chaste.utility import * # bring in all units for convenience

class TestLatticeBasedAngiogenesis(chaste.cell_based.AbstractCellBasedTestSuite):

    def test_fixed_outer_boundary(self):

        file_handler = chaste.core.OutputFileHandler("Python/TestLatticeBasedAngiogenesisTutorial")
        chaste.core.RandomNumberGenerator.Instance().Reseed(12345)

        reference_length = 1.e-6 * metre()
        reference_time = 3600.0 * second()
        BaseUnits.Instance().SetReferenceLengthScale(reference_length)
        BaseUnits.Instance().SetReferenceTimeScale(reference_time)

        grid = microvessel_chaste.mesh.RegularGrid2()
        grid_spacing = Owen11Parameters.mpLatticeSpacing.GetValue("User")
        grid.SetSpacing(grid_spacing)
        grid.SetExtents([51, 51, 1])

        grid.Write(file_handler)

        node1 = microvessel_chaste.population.vessel.VesselNode2.Create(0.0, 400.0, 0.0, reference_length)
        node2 = microvessel_chaste.population.vessel.VesselNode2.Create(2000.0, 400.0, 0.0, reference_length)
        node1.GetFlowProperties().SetIsInputNode(True)
        node1.GetFlowProperties().SetPressure(Owen11Parameters.mpInletPressure.GetValue("User"))
        node2.GetFlowProperties().SetIsOutputNode(True);
        node2.GetFlowProperties().SetPressure(Owen11Parameters.mpOutletPressure.GetValue("User"))
        node3 = microvessel_chaste.population.vessel.VesselNode2.Create(2000.0, 1600.0, 0.0, reference_length)
        node4 = microvessel_chaste.population.vessel.VesselNode2.Create(0.0, 1600.0, 0.0, reference_length)
        node3.GetFlowProperties().SetIsInputNode(True)
        node3.GetFlowProperties().SetPressure(Owen11Parameters.mpInletPressure.GetValue("User"))
        node4.GetFlowProperties().SetIsOutputNode(True)
        node4.GetFlowProperties().SetPressure(Owen11Parameters.mpOutletPressure.GetValue("User"))
        vessel1 = microvessel_chaste.population.vessel.Vessel2.Create(node1, node2)
        vessel2 = microvessel_chaste.population.vessel.Vessel2.Create(node3, node4)
        network = microvessel_chaste.population.vessel.VesselNetwork2.Create()
        network.AddVessel(vessel1)
        network.AddVessel(vessel2)

        network.Write(file_handler.GetOutputDirectoryFullPath() + "initial_network.vtp")

        cell_population_genenerator = microvessel_chaste.population.cell.Owen11CellPopulationGenerator2()
        cell_population_genenerator.SetRegularGrid(grid)
        cell_population_genenerator.SetVesselNetwork(network)
        tumour_radius = 300.0 * 1.e-6 * metre()
        cell_population_genenerator.SetTumourRadius(tumour_radius)
        cell_population = cell_population_genenerator.Update()

        oxygen_pde = microvessel_chaste.pde.LinearSteadyStateDiffusionReactionPde2_2()
        oxygen_pde.SetIsotropicDiffusionConstant(Owen11Parameters.mpOxygenDiffusivity.GetValue("User"))
        cell_oxygen_sink = microvessel_chaste.pde.CellBasedDiscreteSource2()
        cell_oxygen_sink.SetLinearInUConsumptionRatePerCell(Owen11Parameters.mpCellOxygenConsumptionRate.GetValue("User"))
        oxygen_pde.AddDiscreteSource(cell_oxygen_sink)

        vessel_oxygen_source = microvessel_chaste.pde.VesselBasedDiscreteSource2()
        #oxygen_solubility_at_stp = Secomb04Parameters.mpOxygenVolumetricSolubility.GetValue("User") * GenericParameters.mpGasConcentrationAtStp.GetValue("User")
        #vessel_oxygen_concentration = oxygen_solubility_at_stp * Owen11Parameters.mpReferencePartialPressure.GetValue("User")
        vessel_oxygen_concentration = 0.03 * mole_per_metre_cubed()
        vessel_oxygen_source.SetReferenceConcentration(vessel_oxygen_concentration)
        vessel_oxygen_source.SetVesselPermeability(Owen11Parameters.mpVesselOxygenPermeability.GetValue("User"))
        vessel_oxygen_source.SetReferenceHaematocrit(Owen11Parameters.mpInflowHaematocrit.GetValue("User"))
        oxygen_pde.AddDiscreteSource(vessel_oxygen_source);

        oxygen_solver = microvessel_chaste.pde.FiniteDifferenceSolver2()
        oxygen_solver.SetPde(oxygen_pde)
        oxygen_solver.SetLabel("oxygen")
        oxygen_solver.SetGrid(grid)

        vegf_pde = microvessel_chaste.pde.LinearSteadyStateDiffusionReactionPde2_2()
        vegf_pde.SetIsotropicDiffusionConstant(Owen11Parameters.mpVegfDiffusivity.GetValue("User"))
        vegf_pde.SetContinuumLinearInUTerm(-1.0 * Owen11Parameters.mpVegfDecayRate.GetValue("User"))

        normal_and_quiescent_cell_source = microvessel_chaste.pde.CellStateDependentDiscreteSource2()
        normal_and_quiescent_cell_rates = microvessel_chaste.pde.MapUnsigned_ConcentrationFlowRate()
        normal_and_quiescent_cell_rate_thresholds = microvessel_chaste.pde.MapUnsigned_Concentration()
        quiescent_cancer_state = microvessel_chaste.population.cell.QuiescentCancerCellMutationState()
        normal_cell_state = chaste.cell_based.WildTypeCellMutationState()

#        normal_and_quiescent_cell_rates[normal_cell_state.GetColour()] = Owen11Parameters.mpCellVegfSecretionRate.GetValue("User")
        normal_and_quiescent_cell_rate_thresholds[normal_cell_state.GetColour()] = 0.27*mole_per_metre_cubed()
#        normal_and_quiescent_cell_rates[quiescent_cancer_state.GetColour()] = Owen11Parameters.mpCellVegfSecretionRate.GetValue("User")
        normal_and_quiescent_cell_rate_thresholds[quiescent_cancer_state.GetColour()] = 0.0*mole_per_metre_cubed()
#        normal_and_quiescent_cell_source.SetStateRateMap(normal_and_quiescent_cell_rates)
        normal_and_quiescent_cell_source.SetLabelName("VEGF")
        normal_and_quiescent_cell_source.SetStateRateThresholdMap(normal_and_quiescent_cell_rate_thresholds)
        vegf_pde.AddDiscreteSource(normal_and_quiescent_cell_source)

        vessel_vegf_sink = microvessel_chaste.pde.VesselBasedDiscreteSource2()
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

        impedance_calculator = microvessel_chaste.simulation.VesselImpedanceCalculator2()
        haematocrit_calculator = microvessel_chaste.simulation.ConstantHaematocritSolver2()
        haematocrit_calculator.SetHaematocrit(Owen11Parameters.mpInflowHaematocrit.GetValue("User"))
        wss_calculator = microvessel_chaste.simulation.WallShearStressCalculator2()
        mech_stimulus_calculator = microvessel_chaste.simulation.MechanicalStimulusCalculator2()
        metabolic_stim_calculator = microvessel_chaste.simulation.MetabolicStimulusCalculator2()
        shrinking_stimulus_calculator = microvessel_chaste.simulation.ShrinkingStimulusCalculator2()
        viscosity_calculator = microvessel_chaste.simulation.ViscosityCalculator2()

        structural_adaptation_solver = microvessel_chaste.simulation.StructuralAdaptationSolver2()
        structural_adaptation_solver.SetTolerance(0.0001)
        structural_adaptation_solver.SetMaxIterations(100)
        structural_adaptation_solver.SetTimeIncrement(Owen11Parameters.mpVesselRadiusUpdateTimestep.GetValue("User"));
        structural_adaptation_solver.AddPreFlowSolveCalculator(impedance_calculator)
        structural_adaptation_solver.AddPostFlowSolveCalculator(haematocrit_calculator)
        structural_adaptation_solver.AddPostFlowSolveCalculator(wss_calculator)
        structural_adaptation_solver.AddPostFlowSolveCalculator(metabolic_stim_calculator)
        structural_adaptation_solver.AddPostFlowSolveCalculator(mech_stimulus_calculator)
        structural_adaptation_solver.AddPostFlowSolveCalculator(viscosity_calculator)

        regression_solver = microvessel_chaste.simulation.WallShearStressBasedRegressionSolver2()

        angiogenesis_solver = microvessel_chaste.simulation.AngiogenesisSolver2()
        sprouting_rule = microvessel_chaste.simulation.Owen2011SproutingRule2()
        migration_rule = microvessel_chaste.simulation.Owen2011MigrationRule2()
        angiogenesis_solver.SetMigrationRule(migration_rule)
        angiogenesis_solver.SetSproutingRule(sprouting_rule)
        sprouting_rule.SetDiscreteContinuumSolver(vegf_solver)
        migration_rule.SetDiscreteContinuumSolver(vegf_solver)
        angiogenesis_solver.SetVesselGrid(grid)
        angiogenesis_solver.SetVesselNetwork(network)

        microvessel_solver = microvessel_chaste.simulation.MicrovesselSolver2()
        microvessel_solver.SetVesselNetwork(network)
        microvessel_solver.SetOutputFrequency(5)
        microvessel_solver.AddDiscreteContinuumSolver(oxygen_solver)
        microvessel_solver.AddDiscreteContinuumSolver(vegf_solver)
        microvessel_solver.SetStructuralAdaptationSolver(structural_adaptation_solver)
        microvessel_solver.SetRegressionSolver(regression_solver)
        microvessel_solver.SetAngiogenesisSolver(angiogenesis_solver)

        microvessel_modifier = microvessel_chaste.simulation.MicrovesselSimulationModifier2()
        microvessel_modifier.SetMicrovesselSolver(microvessel_solver)
        update_labels = microvessel_chaste.simulation.VecString()
        update_labels.append("oxygen")
        update_labels.append("VEGF_Extracellular")
        microvessel_modifier.SetCellDataUpdateLabels(update_labels)

        simulator = chaste.cell_based.OnLatticeSimulation2(cell_population)
        simulator.AddSimulationModifier(microvessel_modifier)

        apoptotic_cell_killer = chaste.cell_based.ApoptoticCellKiller2(cell_population)
        simulator.AddCellKiller(apoptotic_cell_killer)

        owen11_tracking_modifier = microvessel_chaste.simulation.Owen2011TrackingModifier2()
        simulator.AddSimulationModifier(owen11_tracking_modifier)

        simulator.SetOutputDirectory("Python/TestLatticeBasedAngiogenesisLiteratePaper")
        simulator.SetSamplingTimestepMultiple(5)
        simulator.SetDt(0.5)

        simulator.SetEndTime(20.0)

        simulator.Solve()

        ParameterCollection.Instance().DumpToFile(file_handler.GetOutputDirectoryFullPath()+"parameter_collection.xml")

if __name__ == '__main__':
    unittest.main(verbosity=2)

```

