This tutorial is automatically generated from the file test/python/tutorials//TestPythonOffLatticeAngiogenesisLiteratePaper.py.
Note that the code is given in full at the bottom of the page.



# An Off Lattice Angiogenesis Tutorial
This tutorial demonstrates functionality for modelling 3D off-lattice angiogenesis in a corneal micro
pocket application, similar to that described in [Connor et al. 2015](http://rsif.royalsocietypublishing.org/content/12/110/20150546.abstract).

It is a 3D simulation modelling VEGF diffusion and decay from an implanted pellet using finite element methods and lattice-free angiogenesis
from a large limbal vessel towards the pellet.

![Off Lattice Angiogenesis Image](https://github.com/jmsgrogan/MicrovesselChaste/raw/master/test/tutorials/images/OffLatticeMidPoint.png)

# The Test

```python
import unittest
import numpy as np
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

class TestOffLatticeAngiogenesis(chaste.cell_based.AbstractCellBasedTestSuite):

    def test_fixed_outer_boundary(self):

```
Set up output file management.

```python
        file_handler = chaste.core.OutputFileHandler("Python/TestOffLatticeAngiogenesisLiteratePaper")
        chaste.core.RandomNumberGenerator.Instance().Reseed(12345)

```
This component uses explicit dimensions for all quantities, but interfaces with solvers which take
non-dimensional inputs. The `BaseUnits` singleton takes time, length and mass reference scales to
allow non-dimensionalisation when sending quantities to external solvers and re-dimensionalisation of
results. For our purposes microns for length and hours for time are suitable base units.

```python
        reference_length = 1.e-6 * metre()
        reference_time = 3600.0 * second()
        reference_concentration = 1.e-9*mole_per_metre_cubed()
        BaseUnits.Instance().SetReferenceLengthScale(reference_length)
        BaseUnits.Instance().SetReferenceTimeScale(reference_time)
        BaseUnits.Instance().SetReferenceConcentrationScale(reference_concentration)

```
Set up the domain representing the cornea. This is a thin hemispherical shell. We assume some symmetry to
reduce computational expense.

```python
        hemisphere_generator = microvessel_chaste.geometry.MappableGridGenerator()
        radius = 1400.0e-6*metre()
        thickness = 100.0e-6*metre()
        num_divisions_x = 10
        num_divisions_y = 10
        azimuth_angle = 1.0 * np.pi
        polar_angle = 0.5 * np.pi
        domain = hemisphere_generator.GenerateHemisphere(radius/reference_length,
                                                         thickness/reference_length,
                                                         num_divisions_x,
                                                         num_divisions_y,
                                                         azimuth_angle,
                                                         polar_angle)

```
We can visualize the part

```python
        scene = microvessel_chaste.visualization.MicrovesselVtkScene3()
        #scene.SetPart(domain)
        #scene.GetPartActorGenerator().SetVolumeOpacity(0.7)
        #scene.GetPartActorGenerator().SetVolumeColor((255.0, 255.0, 255.0))
        scene.SetIsInteractive(True)
        scene.SetOutputFilePath(file_handler.GetOutputDirectoryFullPath()+"render")

```
Set up a vessel network, with divisions roughly every 'cell length'. Initially it is straight. We will map it onto the hemisphere.

```python
        network_generator = microvessel_chaste.population.vessel.VesselNetworkGenerator3()
        vessel_length = np.pi * radius
        cell_length = 40.0e-6 * metre()
        origin = microvessel_chaste.mesh.DimensionalChastePoint3(0.0, 4000.0, 0.0)
        network  = network_generator.GenerateSingleVessel(vessel_length, origin, int(float(vessel_length/cell_length)) + 1, 0)

        network.GetNode(0).GetFlowProperties().SetIsInputNode(True);
        network.GetNode(0).GetFlowProperties().SetPressure(Owen11Parameters.mpInletPressure.GetValue("User"))
        network.GetNode(network.GetNumberOfNodes()-1).GetFlowProperties().SetIsOutputNode(True)
        network.GetNode(network.GetNumberOfNodes()-1).GetFlowProperties().SetPressure(Owen11Parameters.mpOutletPressure.GetValue("User"))
        nodes = network.GetNodes();
        for eachNode in nodes:

            node_azimuth_angle = float(azimuth_angle * eachNode.rGetLocation().GetLocation(reference_length)[0]*reference_length/vessel_length)
            node_polar_angle = float(polar_angle*eachNode.rGetLocation().GetLocation(reference_length)[1]*reference_length/vessel_length)
            dimless_radius = (float(radius/reference_length)+(-0.5*float(thickness/reference_length)))
            new_position = microvessel_chaste.mesh.DimensionalChastePoint3(dimless_radius * np.cos(node_azimuth_angle) * np.sin(node_polar_angle),
                                                                           dimless_radius * np.cos(node_polar_angle),
                                                                           dimless_radius * np.sin(node_azimuth_angle) * np.sin(node_polar_angle),
                                                                           reference_length)
            eachNode.SetLocation(new_position)

        scene.SetVesselNetwork(network)
        scene.GetVesselNetworkActorGenerator().SetEdgeSize(20.0)

```
The initial domain and vessel network now look as follows:

![Off Lattice Angiogenesis Image](https://github.com/jmsgrogan/MicrovesselChaste/raw/master/test/tutorials/images/OffLatticeTurorialHemisphere.png)

In the experimental assay a pellet containing VEGF is implanted near the top of the cornea. We model this
as a fixed concentration of VEGF in a cuboidal region. First set up the vegf sub domain.

```python
        vegf_domain = microvessel_chaste.geometry.Part3()
        pellet_side_length = 300.0e-6 * metre()
        origin = microvessel_chaste.mesh.DimensionalChastePoint3(-150.0,900.0,0.0)
        vegf_domain.AddCuboid(pellet_side_length, pellet_side_length, 5.0*pellet_side_length, origin)

        vegf_domain.Write(file_handler.GetOutputDirectoryFullPath()+"initial_vegf_domain.vtp", microvessel_chaste.geometry.GeometryFormat.VTP)

```
Now make a finite element mesh on the cornea.

```python
        mesh_generator = microvessel_chaste.mesh.DiscreteContinuumMeshGenerator3_3()
        mesh_generator.SetDomain(domain)
        mesh_generator.SetMaxElementArea(1e-6 * metre_cubed())
        mesh_generator.Update()
        mesh = mesh_generator.GetMesh()

        scene.SetMesh(mesh)

        scene.Start()
        scene.StartInteractiveEventHandler()

```
Set up the vegf pde

```python
        vegf_pde = microvessel_chaste.pde.LinearSteadyStateDiffusionReactionPde3_3()
        vegf_pde.SetIsotropicDiffusionConstant(Owen11Parameters.mpVegfDiffusivity.GetValue("User"))
        vegf_pde.SetContinuumLinearInUTerm(-1.0*Owen11Parameters.mpVegfDecayRate.GetValue("User"))
        vegf_pde.SetMesh(mesh)
        vegf_pde.SetUseRegularGrid(False)
        vegf_pde.SetReferenceConcentration(1.e-9*mole_per_metre_cubed())

```
Add a boundary condition to fix the VEGF concentration in the vegf subdomain.

```python
        vegf_boundary = microvessel_chaste.pde.DiscreteContinuumBoundaryCondition3()
        vegf_boundary.SetType(microvessel_chaste.pde.BoundaryConditionType.IN_PART)
        vegf_boundary.SetSource(microvessel_chaste.pde.BoundaryConditionSource.PRESCRIBED)
        vegf_boundary.SetValue(3.e-9*mole_per_metre_cubed())
        vegf_boundary.SetDomain(vegf_domain)

```
Set up the PDE solvers for the vegf problem. Note the scaling of the concentration to nM to avoid numerical
precision problems.

```python
        vegf_solver = microvessel_chaste.pde.FiniteElementSolver3()
        vegf_solver.SetPde(vegf_pde)
        vegf_solver.SetLabel("vegf")
        vegf_solver.SetMesh(mesh)
        vegf_solver.AddBoundaryCondition(vegf_boundary)

```
An example of the VEGF solution is shown here:

![Off Lattice Angiogenesis Image](https://github.com/jmsgrogan/MicrovesselChaste/raw/master/test/tutorials/images/OffLatticeTutorialVegf.png)

Set up an angiogenesis solver and add sprouting and migration rules.

angiogenesis_solver = microvessel_chaste.simulation.AngiogenesisSolver3()
```python
        sprouting_rule = microvessel_chaste.simulation.OffLatticeSproutingRule3()
        sprouting_rule.SetSproutingProbability(1.e6* per_second())
        migration_rule = microvessel_chaste.simulation.OffLatticeMigrationRule3()
        migration_rule.SetChemotacticStrength(0.1)
        migration_rule.SetAttractionStrength(0.5)

        sprout_velocity = (50.0e-6/(24.0*3600.0))*metre_per_second() #Secomb13
        migration_rule.SetSproutingVelocity(sprout_velocity)

        angiogenesis_solver.SetMigrationRule(migration_rule)
        angiogenesis_solver.SetSproutingRule(sprouting_rule)
        sprouting_rule.SetDiscreteContinuumSolver(vegf_solver)
        migration_rule.SetDiscreteContinuumSolver(vegf_solver)
        angiogenesis_solver.SetVesselNetwork(network)
        angiogenesis_solver.SetBoundingDomain(domain)

```
Set up the `MicrovesselSolver` which coordinates all solves. Note that for sequentially
coupled PDE solves, the solution propagates in the order that the PDE solvers are added to the `MicrovesselSolver`.

```python
        microvessel_solver = microvessel_chaste.simulation.MicrovesselSolver3()
        microvessel_solver.SetVesselNetwork(network)
        microvessel_solver.AddDiscreteContinuumSolver(vegf_solver)
        microvessel_solver.SetOutputFileHandler(file_handler)
        microvessel_solver.SetOutputFrequency(5)
        microvessel_solver.SetAngiogenesisSolver(angiogenesis_solver)
        microvessel_solver.SetUpdatePdeEachSolve(False)

```
Set the simulation time and run the solver. The result is shown at the top of the tutorial.

```python
        chaste.cell_based.SimulationTime.Instance().SetEndTimeAndNumberOfTimeSteps(100.0, 10)
        microvessel_solver.Run()

if __name__ == '__main__':
    unittest.main()

```


# Code 
The full code is given below


## File name `TestPythonOffLatticeAngiogenesisLiteratePaper.py` 

```python
import unittest
import numpy as np
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

class TestOffLatticeAngiogenesis(chaste.cell_based.AbstractCellBasedTestSuite):

    def test_fixed_outer_boundary(self):

        file_handler = chaste.core.OutputFileHandler("Python/TestOffLatticeAngiogenesisLiteratePaper")
        chaste.core.RandomNumberGenerator.Instance().Reseed(12345)

        reference_length = 1.e-6 * metre()
        reference_time = 3600.0 * second()
        reference_concentration = 1.e-9*mole_per_metre_cubed()
        BaseUnits.Instance().SetReferenceLengthScale(reference_length)
        BaseUnits.Instance().SetReferenceTimeScale(reference_time)
        BaseUnits.Instance().SetReferenceConcentrationScale(reference_concentration)

        hemisphere_generator = microvessel_chaste.geometry.MappableGridGenerator()
        radius = 1400.0e-6*metre()
        thickness = 100.0e-6*metre()
        num_divisions_x = 10
        num_divisions_y = 10
        azimuth_angle = 1.0 * np.pi
        polar_angle = 0.5 * np.pi
        domain = hemisphere_generator.GenerateHemisphere(radius/reference_length,
                                                         thickness/reference_length,
                                                         num_divisions_x,
                                                         num_divisions_y,
                                                         azimuth_angle,
                                                         polar_angle)

        scene = microvessel_chaste.visualization.MicrovesselVtkScene3()
        #scene.SetPart(domain)
        #scene.GetPartActorGenerator().SetVolumeOpacity(0.7)
        #scene.GetPartActorGenerator().SetVolumeColor((255.0, 255.0, 255.0))
        scene.SetIsInteractive(True)
        scene.SetOutputFilePath(file_handler.GetOutputDirectoryFullPath()+"render")

        network_generator = microvessel_chaste.population.vessel.VesselNetworkGenerator3()
        vessel_length = np.pi * radius
        cell_length = 40.0e-6 * metre()
        origin = microvessel_chaste.mesh.DimensionalChastePoint3(0.0, 4000.0, 0.0)
        network  = network_generator.GenerateSingleVessel(vessel_length, origin, int(float(vessel_length/cell_length)) + 1, 0)

        network.GetNode(0).GetFlowProperties().SetIsInputNode(True);
        network.GetNode(0).GetFlowProperties().SetPressure(Owen11Parameters.mpInletPressure.GetValue("User"))
        network.GetNode(network.GetNumberOfNodes()-1).GetFlowProperties().SetIsOutputNode(True)
        network.GetNode(network.GetNumberOfNodes()-1).GetFlowProperties().SetPressure(Owen11Parameters.mpOutletPressure.GetValue("User"))
        nodes = network.GetNodes();
        for eachNode in nodes:

            node_azimuth_angle = float(azimuth_angle * eachNode.rGetLocation().GetLocation(reference_length)[0]*reference_length/vessel_length)
            node_polar_angle = float(polar_angle*eachNode.rGetLocation().GetLocation(reference_length)[1]*reference_length/vessel_length)
            dimless_radius = (float(radius/reference_length)+(-0.5*float(thickness/reference_length)))
            new_position = microvessel_chaste.mesh.DimensionalChastePoint3(dimless_radius * np.cos(node_azimuth_angle) * np.sin(node_polar_angle),
                                                                           dimless_radius * np.cos(node_polar_angle),
                                                                           dimless_radius * np.sin(node_azimuth_angle) * np.sin(node_polar_angle),
                                                                           reference_length)
            eachNode.SetLocation(new_position)

        scene.SetVesselNetwork(network)
        scene.GetVesselNetworkActorGenerator().SetEdgeSize(20.0)

        vegf_domain = microvessel_chaste.geometry.Part3()
        pellet_side_length = 300.0e-6 * metre()
        origin = microvessel_chaste.mesh.DimensionalChastePoint3(-150.0,900.0,0.0)
        vegf_domain.AddCuboid(pellet_side_length, pellet_side_length, 5.0*pellet_side_length, origin)

        vegf_domain.Write(file_handler.GetOutputDirectoryFullPath()+"initial_vegf_domain.vtp", microvessel_chaste.geometry.GeometryFormat.VTP)

        mesh_generator = microvessel_chaste.mesh.DiscreteContinuumMeshGenerator3_3()
        mesh_generator.SetDomain(domain)
        mesh_generator.SetMaxElementArea(1e-6 * metre_cubed())
        mesh_generator.Update()
        mesh = mesh_generator.GetMesh()

        scene.SetMesh(mesh)

        scene.Start()
        scene.StartInteractiveEventHandler()

        vegf_pde = microvessel_chaste.pde.LinearSteadyStateDiffusionReactionPde3_3()
        vegf_pde.SetIsotropicDiffusionConstant(Owen11Parameters.mpVegfDiffusivity.GetValue("User"))
        vegf_pde.SetContinuumLinearInUTerm(-1.0*Owen11Parameters.mpVegfDecayRate.GetValue("User"))
        vegf_pde.SetMesh(mesh)
        vegf_pde.SetUseRegularGrid(False)
        vegf_pde.SetReferenceConcentration(1.e-9*mole_per_metre_cubed())

        vegf_boundary = microvessel_chaste.pde.DiscreteContinuumBoundaryCondition3()
        vegf_boundary.SetType(microvessel_chaste.pde.BoundaryConditionType.IN_PART)
        vegf_boundary.SetSource(microvessel_chaste.pde.BoundaryConditionSource.PRESCRIBED)
        vegf_boundary.SetValue(3.e-9*mole_per_metre_cubed())
        vegf_boundary.SetDomain(vegf_domain)

        vegf_solver = microvessel_chaste.pde.FiniteElementSolver3()
        vegf_solver.SetPde(vegf_pde)
        vegf_solver.SetLabel("vegf")
        vegf_solver.SetMesh(mesh)
        vegf_solver.AddBoundaryCondition(vegf_boundary)

        sprouting_rule = microvessel_chaste.simulation.OffLatticeSproutingRule3()
        sprouting_rule.SetSproutingProbability(1.e6* per_second())
        migration_rule = microvessel_chaste.simulation.OffLatticeMigrationRule3()
        migration_rule.SetChemotacticStrength(0.1)
        migration_rule.SetAttractionStrength(0.5)

        sprout_velocity = (50.0e-6/(24.0*3600.0))*metre_per_second() #Secomb13
        migration_rule.SetSproutingVelocity(sprout_velocity)

        angiogenesis_solver.SetMigrationRule(migration_rule)
        angiogenesis_solver.SetSproutingRule(sprouting_rule)
        sprouting_rule.SetDiscreteContinuumSolver(vegf_solver)
        migration_rule.SetDiscreteContinuumSolver(vegf_solver)
        angiogenesis_solver.SetVesselNetwork(network)
        angiogenesis_solver.SetBoundingDomain(domain)

        microvessel_solver = microvessel_chaste.simulation.MicrovesselSolver3()
        microvessel_solver.SetVesselNetwork(network)
        microvessel_solver.AddDiscreteContinuumSolver(vegf_solver)
        microvessel_solver.SetOutputFileHandler(file_handler)
        microvessel_solver.SetOutputFrequency(5)
        microvessel_solver.SetAngiogenesisSolver(angiogenesis_solver)
        microvessel_solver.SetUpdatePdeEachSolve(False)

        chaste.cell_based.SimulationTime.Instance().SetEndTimeAndNumberOfTimeSteps(100.0, 10)
        microvessel_solver.Run()

if __name__ == '__main__':
    unittest.main()

```

