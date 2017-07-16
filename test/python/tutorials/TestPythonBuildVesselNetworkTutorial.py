
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

## ## Introduction
## This tutorial introduces modelling vessel networks. It will cover the following techniques:
##
## * Building a network from a collection of nodes, segments and vessels
## * Writing networks to file and visualizing it
## * Building a network using a network generator
##
## ## The Test

import unittest # Testing framework
import chaste # Core Chaste functionality
import microvessel_chaste.geometry # Core Microvessel Chaste functionality
import microvessel_chaste.population.vessel # Vessel tools
import microvessel_chaste.visualization # Visualization
from microvessel_chaste.utility import * # Dimensional analysis: bring in all units for convenience

class TestPythonBuildVesselNetworkLiteratePaper(unittest.TestCase):
    
    ## ## Test 1 - Building a vessel network manually, writing it to file and visualizing it
    ## In the first test we will build a vessel network from its constituent components; nodes, 
    ## segments and vessels. We will do some simple tests to make sure the network has been formed 
    ## as expected. Then we write the network to file and visualize it.
    
    def test_build_network_manually(self):
        
        ## First we make some nodes, which are point features from which vessels can be constructed. 
        ## They are initialized with a location. Vessel network components are specialized (templated) 
        ## over spatial dimension, and can be 2D or 3D. We will create a Y shaped network. 
        ## Later we will learn how to build up networks in a more efficient manner. Note that we
        ## are being explicit regarding units, setting a length scale of 1 micron.
        
        file_handler = chaste.core.OutputFileHandler("Python/TestPythonBuildVesselNetworkLiteratePaper", True)
        length = 100.0e-6*metres
        n1 = microvessel_chaste.population.vessel.VesselNode3.Create(0.0*length)
        n2 = microvessel_chaste.population.vessel.VesselNode3.Create(length)
        n3 = microvessel_chaste.population.vessel.VesselNode3.Create(2.0 * length, length)
        n4 = microvessel_chaste.population.vessel.VesselNode3.Create(2.0 * length, -1.0*length)
        
        ## Next we make vessel segments and vessels. Vessel segments are straight-line features 
        ## which contain a vascular node at each end. Vessels can be constructed from multiple vessel segments, 
        ## but in this case each vessel just has a single segment.
        
        v1 = microvessel_chaste.population.vessel.Vessel3.Create([n1 ,n2])
        v2 = microvessel_chaste.population.vessel.Vessel3.Create([n2, n3])
        v3 = microvessel_chaste.population.vessel.Vessel3.Create([n2, n4])
        
        ## Now we can add our vessels to a vessel network.
        
        network = microvessel_chaste.population.vessel.VesselNetwork3.Create()
        network.AddVessel(v1)
        network.AddVessel(v2)
        network.AddVessel(v3)
        
        property_manager = microvessel_chaste.population.vessel.VesselNetworkPropertyManager3()
        property_manager.SetSegmentRadii(network, 10.0e-6*metres)
        property_manager.SetNodeRadii(network, 10.0e-6*metres)
        
        ## We can visualize the network
        
        scene = microvessel_chaste.visualization.MicrovesselVtkScene3()
        scene.SetVesselNetwork(network)
        # JUPYTER_SHOW_FIRST
        scene.Start()  # JUPYTER_SHOW
        
        ## Next we write out network to file. We use the Chaste `OutputFileHandler` functionality to manage 
        ## the output location. Networks are written using VTKs PolyData format, which should have a .vtp extension.
        
        writer = microvessel_chaste.population.vessel.VesselNetworkWriter3()
        writer.SetVesselNetwork(network)
        writer.SetFileName(file_handler.GetOutputDirectoryFullPath() + "bifurcating_network.vtp")
        writer.Write()

        ## We can visualize then network in Paraview. 
        
    ## ## Test 2 - Building a vessel network with a generator
    ## In the first test we manually built a network from its components. This is tedious. We can use a generator
    ## instead.
    
    def test_build_network_with_generator(self):
        
        ## Create a hexagonal network in 3D space using a generator. Specify the target network 
        ## width and height and the desired vessel length. The use of dimensional analysis is demonstrated 
        ## by now using a fictitious 'cell width' reference length unit instead of microns.
        
        file_handler = chaste.core.OutputFileHandler("Python/TestPythonBuildVesselNetworkLiteratePaperGenerator", True)
        cell_width = 25.e-6 * metres
        BaseUnits.Instance().SetReferenceLengthScale(cell_width)
    
        target_width = 60.0 * cell_width
        target_height = 30.0 * cell_width
        vessel_length = 4.0 * cell_width
        network_generator = microvessel_chaste.population.vessel.VesselNetworkGenerator3()
        network = network_generator.GenerateHexagonalNetwork(target_width, target_height, vessel_length)
        
        ## We can visualize the network
        
        scene = microvessel_chaste.visualization.MicrovesselVtkScene3()
        scene.SetVesselNetwork(network)
        scene.Start()  # JUPYTER_SHOW
        
        ## We write the network to file as before. We want to over-ride the
        ## reference length scale so that the output is written in micron.

        writer = microvessel_chaste.population.vessel.VesselNetworkWriter3()
        writer.SetFileName(file_handler.GetOutputDirectoryFullPath() + "hexagonal_network.vtp")
        writer.SetVesselNetwork(network)
        micron_length_scale = 1.e-6 * metres
        writer.SetReferenceLengthScale(micron_length_scale)
        writer.Write()
        

        ## Use a reader to read the network back in from the VTK file. Our network was written in units of micron, so
        ## we need to tell the reader this so that locations are suitably stored.

        network_reader = microvessel_chaste.population.vessel.VesselNetworkReader3()
        network_reader.SetReferenceLengthScale(micron_length_scale)
        network_reader.SetFileName(file_handler.GetOutputDirectoryFullPath() + "hexagonal_network.vtp")
        network_from_file = network_reader.Read()
        
        ## Again, we can visualize the network
        
        scene = microvessel_chaste.visualization.MicrovesselVtkScene3()
        scene.SetVesselNetwork(network)
        scene.Start()  # JUPYTER_SHOW
        
if __name__ == '__main__':
    unittest.main()
    
#endif END_WIKI