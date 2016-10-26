
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

import math
import unittest
import chaste.core
chaste.init()
import chaste.projects.microvessel as microvessel
import chaste.projects.microvessel.geometry
import chaste.projects.microvessel.mesh 
import chaste.projects.microvessel.utility as utility
import chaste.projects.microvessel.population.vessel as vessel
import chaste.projects.microvessel.simulation as simulation

class TestBetteridgeCalculator(unittest.TestCase):
    
    file_handler = chaste.core.OutputFileHandler("Python/TestHaematocritTransport/")
    
    # Set up a vessel network
    length = 100.0
    network = vessel.VesselNetwork3()
    n1 = vessel.VesselNode3(0.0, 0.0, 0.0)
    n2 = vessel.VesselNode3(length, 0.0, 0.0)
    n3 = vessel.VesselNode3(length + math.cos(math.pi/6.0)*length, math.sin(math.pi/6.0)*length, 0.0)
    n4 = vessel.VesselNode3(length + math.cos(math.pi/6.0)*length, -math.sin(math.pi/6.0)*length, 0.0)
    n5 = vessel.VesselNode3(length + 2.0*math.cos(math.pi/6.0)*length, 0.0, 0.0)
    n6 = vessel.VesselNode3(2.0 * length + 2.0*math.cos(math.pi/6.0)*length, 0.0, 0.0)
    n7 = vessel.VesselNode3(3.0 * length + 2.0*math.cos(math.pi/6.0)*length, 0.0, 0.0)
    
    n1.GetFlowProperties().SetIsInputNode(True)#
    n1.GetFlowProperties().SetPressure(5000.0 * utility.pascal())
    n7.GetFlowProperties().SetIsOutputNode(True)
    n7.GetFlowProperties().SetPressure(3000.0 * utility.pascal())
    
    v1 = vessel.Vessel3([n1, n2])
    network.AddVessel(v1)
    v2 = vessel.Vessel3([n2, n3])
    network.AddVessel(v2)
    v3 = vessel.Vessel3([n2, n4])
    network.AddVessel(v3)
    v4 = vessel.Vessel3([n3, n5])
    network.AddVessel(v4)
    v5 = vessel.Vessel3([n4, n5])
    network.AddVessel(v5)
    v6 = vessel.Vessel3([n5, n6])
    network.AddVessel(v6)
    v7 = vessel.Vessel3([n6, n7])
    network.AddVessel(v7)
    
    network.SetSegmentRadii(10.e-6 * utility.metre())
    viscosity = 1.e-3 *utility.poiseuille()
    initial_haematocrit = 0.1
    for eachVessel in network.GetVessels():
        for eachSegment in eachVessel.GetSegments():
            eachSegment.GetFlowProperties().SetViscosity(viscosity)
            #eachSegment.GetFlowProperties().SetHaematocrit(initial_haematocrit)
            
    v2.GetSegments()[0].SetRadius(5.e-6 * utility.metre())
    v4.GetSegments()[0].SetRadius(5.e-6 * utility.metre())
    
    impedance_calculator = simulation.VesselImpedanceCalculator3()
    impedance_calculator.SetVesselNetwork(network)
    impedance_calculator.Calculate()
    
    #network.Write(file_handler.GetOutputDirectoryFullPath() + "/original_network.vtp")
    
    flow_solver = simulation.FlowSolver3()
    flow_solver.SetVesselNetwork(network)
    flow_solver.Solve()
    
    haematocrit_calculator = simulation.BetteridgeHaematocritSolver3()
    #haematocrit_calculator.SetHaematocrit(initial_haematocrit)
    haematocrit_calculator.SetVesselNetwork(network)
    haematocrit_calculator.Calculate()
    
    #network.Write(file_handler.GetOutputDirectoryFullPath() + "/flow_network.vtp")

if __name__ == '__main__':
    unittest.main()
