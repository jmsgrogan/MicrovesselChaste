
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
import chaste
chaste.init()
import microvessel_chaste
import microvessel_chaste.geometry
from microvessel_chaste.utility import * # bring in all units for convenience
import microvessel_chaste.population.vessel as vessel
import microvessel_chaste.simulation as simulation

class TestBetteridgeCalculator(unittest.TestCase):
    
    file_handler = chaste.core.OutputFileHandler("Python/TestHaematocritTransport/")
    
    # Set up a vessel network
    length = 100.0e-6*metres
    network = microvessel_chaste.population.vessel.VesselNetwork3.Create()
    n1 = microvessel_chaste.population.vessel.VesselNode3.Create(0.0*length)
    n2 = microvessel_chaste.population.vessel.VesselNode3.Create(length)
    n3 = microvessel_chaste.population.vessel.VesselNode3.Create(length + math.cos(math.pi/6.0)*length, math.sin(math.pi/6.0)*length)
    n4 = microvessel_chaste.population.vessel.VesselNode3.Create(length + math.cos(math.pi/6.0)*length, -math.sin(math.pi/6.0)*length)
    n5 = microvessel_chaste.population.vessel.VesselNode3.Create(length + 2.0*math.cos(math.pi/6.0)*length)
    n6 = microvessel_chaste.population.vessel.VesselNode3.Create(2.0 * length + 2.0*math.cos(math.pi/6.0)*length)
    n7 = microvessel_chaste.population.vessel.VesselNode3.Create(3.0 * length + 2.0*math.cos(math.pi/6.0)*length)
    
    n1.GetFlowProperties().SetIsInputNode(True)#
    n1.GetFlowProperties().SetPressure(5000.0 * pascals)
    n7.GetFlowProperties().SetIsOutputNode(True)
    n7.GetFlowProperties().SetPressure(3000.0 * pascals)
    
    v1 = microvessel_chaste.population.vessel.Vessel3.Create([n1, n2])
    network.AddVessel(v1)
    v2 = microvessel_chaste.population.vessel.Vessel3.Create([n2, n3])
    network.AddVessel(v2)
    v3 = microvessel_chaste.population.vessel.Vessel3.Create([n2, n4])
    network.AddVessel(v3)
    v4 = microvessel_chaste.population.vessel.Vessel3.Create([n3, n5])
    network.AddVessel(v4)
    v5 = microvessel_chaste.population.vessel.Vessel3.Create([n4, n5])
    network.AddVessel(v5)
    v6 = microvessel_chaste.population.vessel.Vessel3.Create([n5, n6])
    network.AddVessel(v6)
    v7 = microvessel_chaste.population.vessel.Vessel3.Create([n6, n7])
    network.AddVessel(v7)
    
    microvessel_chaste.population.vessel.VesselNetworkPropertyManager3.SetSegmentRadii(network, 1.e-6*metres)
    viscosity = 1.e-3 * poiseuille
    initial_haematocrit = 0.1 * dimensionless
    for eachVessel in network.GetVessels():
        for eachSegment in eachVessel.GetSegments():
            eachSegment.GetFlowProperties().SetViscosity(viscosity)
            eachSegment.GetFlowProperties().SetHaematocrit(initial_haematocrit)
            
    v2.GetSegments()[0].SetRadius(5.e-6 * metres)
    v4.GetSegments()[0].SetRadius(5.e-6 * metres)
    
    impedance_calculator = microvessel_chaste.simulation.VesselImpedanceCalculator3()
    impedance_calculator.SetVesselNetwork(network)
    impedance_calculator.Calculate()
    
    network.Write(file_handler.GetOutputDirectoryFullPath() + "/original_network.vtp")
    
    flow_solver = microvessel_chaste.simulation.FlowSolver3()
    flow_solver.SetVesselNetwork(network)
    flow_solver.Solve()
    
    haematocrit_calculator = microvessel_chaste.simulation.BetteridgeHaematocritSolver3()
    haematocrit_calculator.SetHaematocrit(initial_haematocrit)
    haematocrit_calculator.SetVesselNetwork(network)
    haematocrit_calculator.Calculate()
    
    network.Write(file_handler.GetOutputDirectoryFullPath() + "/flow_network.vtp")

if __name__ == '__main__':
    unittest.main()
