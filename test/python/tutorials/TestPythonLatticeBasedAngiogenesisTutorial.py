
"""Copyright (c) 2005-2016, University of Oxford.
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
"""
#ifndef
#define TRIGGER_WIKI

## # A Lattice Based Angiogenesis Tutorial
## This tutorial introduces a lattice based angiogenesis problem based on a simplified version of the
## vascular tumour application described in
##  [Owen et al. 2011](http://www.ncbi.nlm.nih.gov/pubmed/21363914).
##
## It is a 2D simulation using cellular automaton
## for cells, a regular grid for vessel movement and the same grid for the solution of 
## partial differential equations for oxygen and VEGF transport using the finite difference method.
##
## ## The Test

import unittest # Testing framework
import chaste # Core Chaste functionality
import chaste.cell_based # Chaste Cell Populations
chaste.init() # Initialize MPI and PETSc
import microvessel_chaste # Core Microvessel Chaste functionality
import microvessel_chaste.geometry # Geometry tools
import microvessel_chaste.mesh # Meshing
import microvessel_chaste.population.vessel # Vessel tools
import microvessel_chaste.pde # PDE and solvers
import microvessel_chaste.simulation # Flow and angiogenesis solvers
import microvessel_chaste.visualization # Visualization
from microvessel_chaste.utility import * # Dimensional analysis: bring in all units for convenience

class TestLatticeBasedAngiogenesis(chaste.cell_based.AbstractCellBasedTestSuite):
          
    def test_fixed_outer_boundary(self):
        
        # JUPYTER_SETUP 
        
        ## Set up output file management and seed the random number generator.
        
        file_handler = chaste.core.OutputFileHandler("Python/TestLatticeBasedAngiogenesisTutorial")
        chaste.core.RandomNumberGenerator.Instance().Reseed(12345)
        
        ## This component uses explicit dimensions for all quantities, but interfaces with solvers which take
        ## non-dimensional inputs. The `BaseUnits` singleton takes time, length and mass reference scales to
        ## allow non-dimensionalisation when sending quantities to external solvers and re-dimensionalisation of
        ## results. For our purposes microns for length and hours for time are suitable base units.
        
        reference_length = 1.e-6 * metre()
        reference_time = 3600.0 * second()
        BaseUnits.Instance().SetReferenceLengthScale(reference_length)
        BaseUnits.Instance().SetReferenceTimeScale(reference_time)
        
        ## Set up the lattice (grid), we will use the same dimensions as [Owen et al. 2011](http://www.ncbi.nlm.nih.gov/pubmed/21363914).
        ## Note that we are using hard-coded parameters from that paper. You can see the values by printing.
        ## When we get the value of the parameter by doing
        ## `Owen11Parameters.mpLatticeSpacing.GetValue("User")` a record is kept that this parameter has been used 
        ## in the simulation. A record of all parameters used in a simulation can be dumped to file on completion, 
        ## as will be shown below.
        
        grid = microvessel_chaste.mesh.RegularGrid2()
        grid_spacing = Owen11Parameters.mpLatticeSpacing.GetValue("User")
        grid.SetSpacing(grid_spacing)
        grid.SetExtents([51, 51, 1])
        
        ## We can write and visualize the grid.
        
        grid.Write(file_handler) 
        scene = microvessel_chaste.visualization.MicrovesselVtkScene2()
        scene.SetIsInteractive(True)
        scene.SetRegularGrid(grid)
        scene.GetRegularGridActorGenerator().SetVolumeOpacity(0.1)
        # JUPYTER_SHOW_FIRST
        scene.Start()  # JUPYTER_SHOW
                
        ## Next, set up the vessel network, this will initially consist of two, large counter-flowing vessels. Also set the inlet
        ## and outlet pressures and flags.
        
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
        
        ## Again, we can visualize and write the network
        
        scene.SetVesselNetwork(network)
        scene.GetVesselNetworkActorGenerator().SetEdgeSize(20.0)
        network.Write(file_handler.GetOutputDirectoryFullPath() + "initial_network.vtp")
        scene.Start()  # JUPYTER_SHOW
        
        ## Next, set up the cell populations. We will setup up a population similar to that used in 
        ## the Owen et al., 2011 paper. That is, a grid filled with normal cells and a tumour spheroid 
        ## in the middle. We can use a generator for this purpose. The generator simply sets up
        ## the population using conventional Cell Based Chaste methods.
        
        cell_population_genenerator = microvessel_chaste.population.cell.Owen11CellPopulationGenerator2()
        cell_population_genenerator.SetRegularGrid(grid)
        cell_population_genenerator.SetVesselNetwork(network)
        tumour_radius = 300.0 * 1.e-6 * metre()
        cell_population_genenerator.SetTumourRadius(tumour_radius)
        cell_population = cell_population_genenerator.Update()
        
        ## Again, we visualize. First turn off the grid edges so it is easier to see the cells.
        
        scene.GetRegularGridActorGenerator().SetShowEdges(False)
        scene.GetRegularGridActorGenerator().SetVolumeOpacity(0.0)
        scene.SetCellPopulation(cell_population)
        scene.GetCellPopulationActorGenerator().GetDiscreteColorTransferFunction().AddRGBPoint(1.0, 0.0, 0.0, 0.6)
        scene.GetCellPopulationActorGenerator().SetPointSize(20)
        scene.GetCellPopulationActorGenerator().SetColorByCellMutationState(True)
        scene.ResetRenderer()
        scene.Start()  # JUPYTER_SHOW
        
        ## Next set up the PDEs for oxygen and VEGF. Cells will act as discrete oxygen sinks and discrete vegf sources. 

        oxygen_pde = microvessel_chaste.pde.LinearSteadyStateDiffusionReactionPde2_2()
        oxygen_pde.SetIsotropicDiffusionConstant(Owen11Parameters.mpOxygenDiffusivity.GetValue("User"))
        cell_oxygen_sink = microvessel_chaste.pde.CellBasedDiscreteSource2()
        cell_oxygen_sink.SetLinearInUConsumptionRatePerCell(Owen11Parameters.mpCellOxygenConsumptionRate.GetValue("User"))
        oxygen_pde.AddDiscreteSource(cell_oxygen_sink)
    
        ## Vessels release oxygen depending on their haematocrit levels
        
        vessel_oxygen_source = microvessel_chaste.pde.VesselBasedDiscreteSource2()
        #oxygen_solubility_at_stp = Secomb04Parameters.mpOxygenVolumetricSolubility.GetValue("User") * GenericParameters.mpGasConcentrationAtStp.GetValue("User")
        #vessel_oxygen_concentration = oxygen_solubility_at_stp * Owen11Parameters.mpReferencePartialPressure.GetValue("User")
        vessel_oxygen_concentration = 0.03 * mole_per_metre_cubed()
        vessel_oxygen_source.SetReferenceConcentration(vessel_oxygen_concentration)
        vessel_oxygen_source.SetVesselPermeability(Owen11Parameters.mpVesselOxygenPermeability.GetValue("User"))
        vessel_oxygen_source.SetReferenceHaematocrit(Owen11Parameters.mpInflowHaematocrit.GetValue("User"))
        oxygen_pde.AddDiscreteSource(vessel_oxygen_source);
        
        ## Set up a finite difference solver and pass it the pde and grid.
        
        oxygen_solver = microvessel_chaste.pde.FiniteDifferenceSolver2()
        oxygen_solver.SetPde(oxygen_pde)
        oxygen_solver.SetLabel("oxygen")
        oxygen_solver.SetGrid(grid)        
        
        ## The rate of VEGF release depends on the cell type and intracellular VEGF levels, so we need a more detailed
        ## type of discrete source. 
        
        vegf_pde = microvessel_chaste.pde.LinearSteadyStateDiffusionReactionPde2_2()
        vegf_pde.SetIsotropicDiffusionConstant(Owen11Parameters.mpVegfDiffusivity.GetValue("User"))
        vegf_pde.SetContinuumLinearInUTerm(-1.0 * Owen11Parameters.mpVegfDecayRate.GetValue("User"))
        
        ## Set up a map for different release rates depending on cell type. Also include a threshold intracellular 
        ## VEGF below which there is no release.
        
        normal_and_quiescent_cell_source = microvessel_chaste.pde.CellStateDependentDiscreteSource2()
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
        
        ## Add a vessel related VEGF sink
         
        vessel_vegf_sink = microvessel_chaste.pde.VesselBasedDiscreteSource2()
        vessel_vegf_sink.SetReferenceConcentration(0.0*mole_per_metre_cubed())
        vessel_vegf_sink.SetVesselPermeability(Owen11Parameters.mpVesselVegfPermeability.GetValue("User"))
        vegf_pde.AddDiscreteSource(vessel_vegf_sink)
        
        ## Set up a finite difference solver as before.

        vegf_solver = microvessel_chaste.pde.FiniteDifferenceSolver2()
        vegf_solver.SetPde(vegf_pde)
        vegf_solver.SetLabel("VEGF_Extracellular")
        vegf_solver.SetGrid(grid)
        
        ## Next set up the flow problem. Assign a blood plasma viscosity to the vessels. The actual viscosity will
        ## depend on haematocrit and diameter. This solver manages growth and shrinkage of vessels in response to
        ## flow related stimuli.
         
        large_vessel_radius = 25.0e-6 * metre()
        network.SetSegmentRadii(large_vessel_radius)
        viscosity = Owen11Parameters.mpPlasmaViscosity.GetValue("User")
        network.SetSegmentViscosity(viscosity);
        
        ## Set up the pre- and post flow calculators.
        
        impedance_calculator = microvessel_chaste.simulation.VesselImpedanceCalculator2()
        haematocrit_calculator = microvessel_chaste.simulation.ConstantHaematocritSolver2()
        haematocrit_calculator.SetHaematocrit(Owen11Parameters.mpInflowHaematocrit.GetValue("User"))
        wss_calculator = microvessel_chaste.simulation.WallShearStressCalculator2()
        mech_stimulus_calculator = microvessel_chaste.simulation.MechanicalStimulusCalculator2()
        metabolic_stim_calculator = microvessel_chaste.simulation.MetabolicStimulusCalculator2()
        shrinking_stimulus_calculator = microvessel_chaste.simulation.ShrinkingStimulusCalculator2()
        viscosity_calculator = microvessel_chaste.simulation.ViscosityCalculator2()
        
        ## Set up and configure the structural adaptation solver.
        
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
        
        ## Set up a regression solver.
        
        regression_solver = microvessel_chaste.simulation.WallShearStressBasedRegressionSolver2()
        
        ## Set up an angiogenesis solver and add sprouting and migration rules.
        
        angiogenesis_solver = microvessel_chaste.simulation.AngiogenesisSolver2()
        sprouting_rule = microvessel_chaste.simulation.Owen2011SproutingRule2()
        migration_rule = microvessel_chaste.simulation.Owen2011MigrationRule2()
        angiogenesis_solver.SetMigrationRule(migration_rule)
        angiogenesis_solver.SetSproutingRule(sprouting_rule)
        sprouting_rule.SetDiscreteContinuumSolver(vegf_solver)
        migration_rule.SetDiscreteContinuumSolver(vegf_solver)
        angiogenesis_solver.SetVesselGrid(grid)
        angiogenesis_solver.SetVesselNetwork(network)
        
        ## The microvessel solver will manage all aspects of the vessel solve.
        
        microvessel_solver = microvessel_chaste.simulation.MicrovesselSolver2()
        microvessel_solver.SetVesselNetwork(network)
        microvessel_solver.SetOutputFrequency(5)
        microvessel_solver.AddDiscreteContinuumSolver(oxygen_solver)
        microvessel_solver.AddDiscreteContinuumSolver(vegf_solver)
        microvessel_solver.SetStructuralAdaptationSolver(structural_adaptation_solver)
        microvessel_solver.SetRegressionSolver(regression_solver)
        microvessel_solver.SetAngiogenesisSolver(angiogenesis_solver)
        
        ## Set up plotting
        
        #scene.GetCellPopulationActorGenerator().SetColorByCellData(True)
        #scene.GetCellPopulationActorGenerator().SetDataLabel("oxygen")
        scene_modifier = microvessel_chaste.visualization.VtkSceneMicrovesselModifier2()

        scene_modifier.SetVtkScene(scene)
        scene_modifier.SetUpdateFrequency(2)
        microvessel_solver.AddMicrovesselModifier(scene_modifier)
        
        ## The microvessel solution modifier will link the vessel and cell solvers. We need to explicitly tell it
        ## which extracellular fields to update based on PDE solutions.
        
        microvessel_modifier = microvessel_chaste.simulation.MicrovesselSimulationModifier2()
        microvessel_modifier.SetMicrovesselSolver(microvessel_solver)
        update_labels = microvessel_chaste.simulation.VecString()
        update_labels.append("oxygen")
        update_labels.append("VEGF_Extracellular")
        microvessel_modifier.SetCellDataUpdateLabels(update_labels)
        
        ## The full simulation is run as a typical Cell Based Chaste simulation
        
        simulator = chaste.cell_based.OnLatticeSimulation2(cell_population)
        simulator.AddSimulationModifier(microvessel_modifier)
        
        ## Add a killer to remove apoptotic cells
        
        apoptotic_cell_killer = chaste.cell_based.ApoptoticCellKiller2(cell_population)
        simulator.AddCellKiller(apoptotic_cell_killer)
        
        ## Add another modifier for updating cell cycle quantities.
        
        owen11_tracking_modifier = microvessel_chaste.simulation.Owen2011TrackingModifier2()
        simulator.AddSimulationModifier(owen11_tracking_modifier)
        
        ## Set up the remainder of the simulation
        
        simulator.SetOutputDirectory("Python/TestLatticeBasedAngiogenesisLiteratePaper")
        simulator.SetSamplingTimestepMultiple(5)
        simulator.SetDt(0.5)
        
        ## This end time corresponds to roughly 10 minutes run-time on a desktop PC. Increase it or decrease as
        ## preferred. The end time used in Owen et al. 2011 is 4800 hours.
        
        simulator.SetEndTime(20.0)
        
        ## Do the solve. A sample solution is shown at the top of this test.
        
        simulator.Solve()
        
        ## Dump the parameters to file for inspection.
        
        ParameterCollection.Instance().DumpToFile(file_handler.GetOutputDirectoryFullPath()+"parameter_collection.xml")

        # JUPYTER_PARAMETER_DUMP

        # JUPYTER_TEARDOWN 
        
if __name__ == '__main__':
    unittest.main(verbosity=2)
    
#endif END_WIKI
