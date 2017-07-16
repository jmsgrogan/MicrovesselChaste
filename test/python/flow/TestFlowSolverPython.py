
"""Copyright (c) 2005-2017, University of Oxford.
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
import microvessel_chaste
import microvessel_chaste.geometry
import microvessel_chaste.mesh 
from microvessel_chaste.utility import * # bring in all units for convenience
import microvessel_chaste.population.vessel
import microvessel_chaste.simulation

class TestStraightVesselFlow(unittest.TestCase):
    
    def setup_network(self):
        length = 100.0e-6*metres
        radius = 10.e-6*metres
        viscosity = 4.e-3*poiseuille
        
        n1 = microvessel_chaste.population.vessel.VesselNode3.Create(0.0*length)
        n2 = microvessel_chaste.population.vessel.VesselNode3.Create(length)
        
        n1.GetFlowProperties().SetIsInputNode(True)
        n2.GetFlowProperties().SetIsOutputNode(True)
        
        v1 = microvessel_chaste.population.vessel.Vessel3.Create([n1 ,n2])
        network = microvessel_chaste.population.vessel.VesselNetwork3.Create()
        network.AddVessel(v1)
        
        for eachVessel in network.GetVessels():
            for eachSegment in eachVessel.GetSegments():
                eachSegment.GetFlowProperties().SetViscosity(viscosity)
        microvessel_chaste.population.vessel.VesselNetworkPropertyManager3.SetSegmentRadii(network, radius)
        
        return network
    
    def test_one_d(self):
        
        network = self.setup_network()
        file_handler = chaste.core.OutputFileHandler("Python/TestStraightVesselFlow/")
        
        impedance_calculator = microvessel_chaste.simulation.VesselImpedanceCalculator3()
        impedance_calculator.SetVesselNetwork(network)
        impedance_calculator.Calculate()
        
        flow_solver = microvessel_chaste.simulation.FlowSolver3()
        flow_solver.SetVesselNetwork(network)
        flow_solver.Solve() 
        
if __name__ == '__main__':
    unittest.main()
