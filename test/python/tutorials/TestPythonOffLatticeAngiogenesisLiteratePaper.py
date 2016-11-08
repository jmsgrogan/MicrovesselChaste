
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

import unittest
import numpy as np
import chaste.core
import chaste.core
chaste.init()
import microvessel_chaste
import microvessel_chaste.geometry
import microvessel_chaste.mesh 
import microvessel_chaste.population.vessel
import microvessel_chaste.pde
import microvessel_chaste.simulation
from microvessel_chaste.utility import * # bring in all units for convenience

class TestOffLatticeAngiogenesis(unittest.TestCase):
          
    def test_fixed_outer_boundary(self):
        # Set up i/o management
        file_handler = chaste.core.OutputFileHandler("Python/TestPythonOffLatticeAngiogenesisLiteratePaper")
        
        # Set up a regularly spaced grid to specify a vegf field on
        length_scale = 1.e-6*metre()
        grid = microvessel_chaste.mesh.RegularGrid3()
        grid.SetSpacing(40.e-6*metre())
        grid.SetExtents((30, 30, 10))
        
        origin = microvessel_chaste.mesh.DimensionalChastePoint3(-100.0, -550.0, 200.0, length_scale)
        grid.SetOrigin(origin)
        
        # Set up a vegf field on the grid, fall of from max value with square of distance to source location
        field = microvessel_chaste.pde.FunctionMap3()
        field.SetGrid(grid)
        
        source_location = np.array((500.0, 0.0, 400.0))
        d_max = 1000.0
        max_vegf = 0.2
        
        points_indices = range(grid.GetNumberOfPoints())
        grid_locs = [grid.GetLocationOf1dIndex(x) for x in points_indices]
        grid_dist = np.array([np.linalg.norm(grid_locs[x].GetLocation(length_scale)-source_location) for x in points_indices])
        grid_facs = (np.multiply(grid_dist, grid_dist))/(d_max*d_max)
        vegf_field = np.where(grid_facs<=1.0, max_vegf*(1.0 - grid_facs), np.ones(grid.GetNumberOfPoints()))
        
        field.SetFileName("Function")
        field.SetFileHandler(file_handler)
        field.Setup()
        field.UpdateSolution(vegf_field)
        field.Write()
        
        # Create and oval shaped planar vessel network
        network = microvessel_chaste.population.vessel.VesselNetworkGenerator3().GenerateOvalNetwork(400*length_scale, 40, 0.5, 1.0)
        
        # Set up an off lattice tip cell migration rule
        migration_rule = microvessel_chaste.simulation.OffLatticeMigrationRule3()
        migration_rule.SetDiscreteContinuumSolver(field)
        migration_rule.SetNetwork(network)
        migration_rule.SetSproutingVelocity(10.0*metre_per_second()) #um/hr
        migration_rule.SetChemotacticStrength(0.3)
        migration_rule.SetAttractionStrength(0.5)
        
        #Set up a random sprouting rule
        sprouting_rule = microvessel_chaste.simulation.OffLatticeSproutingRule3()
        sprouting_rule.SetVesselNetwork(network)
        sprouting_rule.SetDiscreteContinuumSolver(field)
        sprouting_rule.SetSproutingProbability(0.005 * per_second())
        
        # Set up an angiogenesis solver
        solver = microvessel_chaste.simulation.AngiogenesisSolver3()
        solver.SetVesselNetwork(network)
        solver.SetOutputFileHandler(file_handler)
        solver.SetSproutingRule(sprouting_rule)
        solver.SetMigrationRule(migration_rule)
        
        # Set up a simulaiton manager
        manager = microvessel_chaste.simulation.SimulationManager()
        manager.Setup()
        manager.SetEndTimeAndNumberOfTimeSteps(20.0, 20) # 100 hour simulation, 1 hour dt
        
        # Run, printing any C++ exceptions to console
#         try:
#             #solver.Run(True)
#         except chaste.core.CPPException as e:
#             print e.GetMessage
            
if __name__ == '__main__':
    unittest.main()