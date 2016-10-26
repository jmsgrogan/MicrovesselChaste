
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
import chaste.core
chaste.init()
import chaste.projects.microvessel as microvessel
import chaste.projects.microvessel.geometry
import chaste.projects.microvessel.mesh 
import chaste.projects.microvessel.utility as utility
import chaste.projects.microvessel.population.vessel as vessel
import chaste.projects.microvessel.simulation as simulation
import chaste.projects.microvessel.pde

class TestLatticeBasedAngiogenesis(unittest.TestCase):
          
    def test_fixed_outer_boundary(self):

        file_handler = chaste.core.OutputFileHandler("Python/TestPythonLatticeBasedAngiogenesisLiteratePaper")
        
        # Set up a grid
        length_scale = 1.e-6*utility.metre()
        grid = chaste.projects.microvessel.mesh.RegularGrid3()
        grid.SetSpacing(40.e-6*utility.metre())
        grid.SetExtents((25, 25, 1))
        
        # Set up a vegf field
        field = chaste.projects.microvessel.pde.FunctionMap3()
        field.SetGrid(grid)
        
        vegf_field = []
        for idx in range(grid.GetExtents()[0]*grid.GetExtents()[1]):
            vegf_field.append(0.2*grid.GetLocationOf1dIndex(idx).GetLocation(length_scale)[0]/(40.0*grid.GetExtents()[0]))
            
        field.SetFileName("Function")
        field.SetFileHandler(file_handler)
        field.Setup()
        field.UpdateSolution(vegf_field)
        field.Write()
        
        # Set up the initial vessel
        length =  float(grid.GetExtents()[1] - 1) * grid.GetSpacing()
        divisions = grid.GetExtents()[1] - 2 # divide the vessel to coincide with grid
        
        start_point = microvessel.mesh.DimensionalChastePoint3(2.0 * 40.0, 0.0, 0.0, length_scale)
        network = vessel.VesselNetworkGenerator3().GenerateSingleVessel(length, start_point, divisions, 1)
        #network.write(file_handler.GetOutputDirectoryFullPath() + "/network.vtp")
        
        migration_rule = simulation.Owen2011MigrationRule3()
        migration_rule.SetGrid(grid)
        migration_rule.SetDiscreteContinuumSolver(field)
        migration_rule.SetCellMotilityParameter(100.0 * utility.metre_squared_per_second())
        migration_rule.SetCellChemotacticParameter(80000.0 * utility.metre_pow5_per_second_per_mole())
        migration_rule.SetNetwork(network)
        # 
        sprouting_rule = simulation.Owen2011SproutingRule3()
        sprouting_rule.SetDiscreteContinuumSolver(field)
        sprouting_rule.SetGrid(grid)
        sprouting_rule.SetVesselNetwork(network)
        sprouting_rule.SetSproutingProbability(0.5 * utility.per_second());
        # 
        # # Set up the angiogenesis solver
        solver = simulation.AngiogenesisSolver3()
        solver.SetVesselNetwork(network)
        solver.SetVesselGrid(grid)
        solver.SetOutputFileHandler(file_handler)
#         solver.SetSproutingRule(sprouting_rule)
        solver.SetMigrationRule(migration_rule)
        
        manager = simulation.SimulationManager()
        manager.Setup()
        manager.SetEndTimeAndNumberOfTimeSteps(10.0, 10.0)
        
        try:
            solver.Run(True)
        except chaste.core.CPPException as e:
            print e.GetMessage
        
        manager.TearDown()
