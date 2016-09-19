"""
Sample Snail Trail Angiogenesis Model
"""

import unittest
import numpy as np
import chaste.simulation
import chaste.mesh
import chaste.population.vessel
import chaste.pde

class TestSnailTrailOffLattice(unittest.TestCase):
          
    def test_fixed_outer_boundary(self):
        # Set up i/o management
        file_handler = chaste.simulation.setup.setup("/home/grogan/test/tutorial/Snail-Trail/OffLatticeOval")
        
        # Set up a regularly spaced grid to specify a vegf field on
        grid = chaste.mesh.RegularGrid()
        grid.SetSpacing(40.0)
        grid.SetExtents((30, 30, 10))
        grid.SetOrigin((-100.0, -550.0, 200.0))
        
        # Set up a vegf field on the grid, fall of from max value with square of distance to source location
        field = chaste.pde.FunctionMap()
        field.SetGrid(grid)
        
        source_location = np.array((500.0, 0.0, 400.0))
        d_max = 1000.0
        max_vegf = 0.2
        
        points_indices = range(grid.GetNumberOfPoints())
        grid_locs = [grid.GetLocationOf1dIndex(x) for x in points_indices]
        grid_dist = np.array([np.linalg.norm(grid_locs[x]-source_location) for x in points_indices])
        grid_facs = (np.multiply(grid_dist, grid_dist))/(d_max*d_max)
        vegf_field = np.where(grid_facs<=1.0, max_vegf*(1.0 - grid_facs), np.ones(grid.GetNumberOfPoints()))
        
        field.SetFileName("Function.vti")
        field.SetFileHandler(file_handler)
        field.Setup()
        field.UpdateSolution(vegf_field)
        field.Write()
        
        # Create and oval shaped planar vessel network
        network = chaste.population.vessel.VesselNetworkGenerator().GenerateOvalNetwork(400, 40, 0.5, 1.0)
        
        # Set up an off lattice tip cell migration rule
        migration_rule = chaste.simulation.OffLatticeMigrationRule()
        migration_rule.SetDiscreteContinuumSolver(field)
        migration_rule.SetNetwork(network)
        migration_rule.SetSproutingVelocity(10.0) #um/hr
        migration_rule.SetChemotacticStrength(0.3)
        migration_rule.SetAttractionStrength(0.5)
        
        #Set up a random sprouting rule
        sprouting_rule = chaste.simulation.OffLatticeSproutingRule()
        sprouting_rule.SetVesselNetwork(network)
        sprouting_rule.SetDiscreteContinuumSolver(field)
        sprouting_rule.SetSproutingProbability(0.005) # prob nex sprout per hour
        
        # Set up an angiogenesis solver
        solver = chaste.simulation.AngiogenesisSolver()
        solver.SetVesselNetwork(network)
        solver.SetOutputFileHandler(file_handler)
        solver.SetSproutingRule(sprouting_rule)
        solver.SetMigrationRule(migration_rule)
        
        # Set up a simulaiton manager
        manager = chaste.simulation.SimulationManager()
        manager.Setup()
        manager.SetEndTimeAndNumberOfTimeSteps(20.0, 20) # 100 hour simulation, 1 hour dt
        
        # Run, printing any C++ exceptions to console
        try:
            solver.Run(True)
        except chaste.core.CPPException as e:
            print e.GetMessage