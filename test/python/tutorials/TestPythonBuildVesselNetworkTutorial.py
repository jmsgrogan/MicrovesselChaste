
"""Copyright (c) 2005-2015, University of Oxford.
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

#define TRIGGER_WIKI

## = Introduction =
## This tutorial is designed to introduce the Python interface for modelling vessel networks. An equivalent C++ tutorial
## is [wiki:PaperTutorials/Angiogenesis/BuildVesselNetwork here]. 
##
## This tutorial covers:
## * Building a network from a collection of nodes, segments and vessels
## * Writing networks to file and visualizing with Paraview
## * Building a network using a network generator
## * Reading a network from file
##
## Further functionality is gradually introduced over the course of subsequent tutorials.
##
## = The Test =

import unittest
import chaste.core
import microvessel_chaste
import microvessel_chaste.geometry
import microvessel_chaste.mesh 
import microvessel_chaste.population.vessel
from microvessel_chaste.utility import * # bring in all units for convenience

class TestPythonBuildVesselNetworkLiteratePaper(unittest.TestCase):
    ## = Test 1 - Building a vessel network manually, writing it to file and visualizing it=
    ## In the first test we will build a vessel network from its constituent components; nodes, segments and vessels. We will do some
    ## simple tests to make sure the network has been formed as expected. Then we write the network to file and visualize it in Paraview.
    
    def test_BuildNetworkManually(self):
        
        ## First we make some nodes, which are point features from which vessels can be constructed. They are initialized with a location.
        ## All vessel network components are created using special factory methods which return shared pointers, rather than being created
        ## directly through their constructors. Vessel network components are templated over spatial dimension, and can be 2D or 3D. We will
        ## create a Y shaped network. Later we will learn how to build up networks in a more efficient manner.
        
        length_scale = 1.e-6*metre()
        length = 100.0
        n1 = microvessel_chaste.population.vessel.VesselNode3(0.0, 0.0 ,0.0, length_scale)
        n2 = microvessel_chaste.population.vessel.VesselNode3(length, 0.0, 0.0, length_scale)
        n3 = microvessel_chaste.population.vessel.VesselNode3(2.0 * length, length, 0.0, length_scale)
        n4 = microvessel_chaste.population.vessel.VesselNode3(2.0 * length, -length, 0.0, length_scale)
        
        ## Next we make vessel segments and vessels. Vessel segments are straight-line features which contain a vascular node at each end. Vessels
        ## can be constructed from multiple vessel segments, but in this case each vessel just has a single segment.
        
        v1 = microvessel_chaste.population.vessel.Vessel3([n1 ,n2])
        v2 = microvessel_chaste.population.vessel.Vessel3([n2, n3])
        v3 = microvessel_chaste.population.vessel.Vessel3([n2, n4])
        
        ## Now we can add our vessels to a vessel network.
        
        network = microvessel_chaste.population.vessel.VesselNetwork3()
        network.AddVessel(v1)
        network.AddVessel(v2)
        network.AddVessel(v3)
        
        ## We use our test framework to make sure that the network has been created correctly by checking the number of vessels and nodes
        
        self.assertEqual(network.GetNumberOfNodes(), 4)
        self.assertEqual(network.GetNumberOfVessels(), 3)
        
        ## Next we write out network to file. We use the Chaste `OutputFileHandler` functionality to management the output location
        ## Networks are written using VTKs PolyData format, which should have a .vtp extension.
        
        file_handler = chaste.core.OutputFileHandler("Python/TestPythonBuildVesselNetworkLiteratePaper", True)
        writer = microvessel_chaste.population.vessel.VesselNetworkWriter3()
        writer.SetVesselNetwork(network)
        writer.SetFileName(file_handler.GetOutputDirectoryFullPath() + "bifurcating_network.vtp")
        writer.Write()

        ## Now we can visualize then network in Paraview. See the tutorial [wiki:UserTutorials/VisualizingWithParaview here], to get started. To view the network import the file
        ## `TestPythonBuildVesselNetworkLiteratePaper\bifurcating_network.vtp` into Paraview. For a nicer rendering you can do `Filters->Alphabetical->Tube`.
        
if __name__ == '__main__':
    unittest.main()