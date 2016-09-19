"""
Sample Snail Trail Angiogenesis Model
"""

import unittest
import chaste
import chaste.simulation
import chaste.pde
import chaste.population.vessel

class TestSnailTrail(unittest.TestCase):
          
    def test_fixed_outer_boundary(self):

        file_handler = chaste.simulation.setup.setup("/home/grogan/test/tutorial/Snail-Trail")
        
        # Set up a grid
        grid = chaste.mesh.RegularGrid()
        grid.SetSpacing(40.0)
        grid.SetExtents((25, 25, 1))
        
        # Set up a vegf field
        field = chaste.pde.FunctionMap()
        field.SetGrid(grid)
        
        vegf_field = []
        for idx in range(grid.GetExtents()[0]*grid.GetExtents()[1]):
            vegf_field.append(0.2*grid.GetLocationOf1dIndex(idx)[0]/(grid.GetSpacing()*grid.GetExtents()[0]))
            
        field.SetFileName("Function.vti")
        field.SetFileHandler(file_handler)
        field.Setup()
        field.UpdateSolution(vegf_field)
        field.Write()
        
        # Set up the initial vessel
        length = grid.GetSpacing() * (grid.GetExtents()[1] - 1) # full domain in y direction
        divisions = grid.GetExtents()[1] - 2 # divide the vessel to coincide with grid
        
            
        network = chaste.population.vessel.VesselNetworkGenerator().GenerateSingleVessel(length, (2.0 * grid.GetSpacing(), 0.0, 0.0), divisions, 1)
        network.write(file_handler.GetOutputDirectoryFullPath() + "/network.vtp")
        
        # Set up the sprouting and migration rules for angiogenesis
        # migration_rule = chaste.simulation.Owen2011MigrationRule()
        # migration_rule.SetGrid(grid)
        # migration_rule.SetDiscreteContinuumSolver(field)
        # migration_rule.SetCellMotilityParameter(100.0)
        # migration_rule.SetCellChemotacticParameter(80000.0)
        # migration_rule.SetNetwork(network)
        # # 
        # sprouting_rule = chaste.simulation.Owen2011SproutingRule()
        # sprouting_rule.SetDiscreteContinuumSolver(field)
        # sprouting_rule.SetGrid(grid)
        # sprouting_rule.SetVesselNetwork(network)
        # sprouting_rule.SetSproutingProbability(0.5);
        
        migration_rule = chaste.simulation.Owen2011MigrationRule()
        migration_rule.SetGrid(grid)
        migration_rule.SetDiscreteContinuumSolver(field)
        migration_rule.SetCellMotilityParameter(100.0)
        migration_rule.SetCellChemotacticParameter(80000.0)
        migration_rule.SetNetwork(network)
        # 
        sprouting_rule = chaste.simulation.Owen2011SproutingRule()
        sprouting_rule.SetDiscreteContinuumSolver(field)
        sprouting_rule.SetGrid(grid)
        sprouting_rule.SetVesselNetwork(network)
        sprouting_rule.SetSproutingProbability(0.5);
        # 
        # # Set up the angiogenesis solver
        solver = chaste.simulation.AngiogenesisSolver()
        solver.SetVesselNetwork(network)
        solver.SetVesselGrid(grid)
        solver.SetOutputFileHandler(file_handler)
        solver.SetSproutingRule(sprouting_rule)
        solver.SetMigrationRule(migration_rule)
        
        manager = chaste.simulation.SimulationManager()
        manager.Setup()
        manager.SetEndTimeAndNumberOfTimeSteps(10.0, 10.0)
        
        try:
            solver.Run(True)
        except chaste.core.CPPException as e:
            print e.GetMessage
        
        manager.TearDown()
