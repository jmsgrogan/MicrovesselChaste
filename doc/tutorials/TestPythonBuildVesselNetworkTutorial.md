This tutorial is automatically generated from the file test/python/tutorials//TestPythonBuildVesselNetworkTutorial.py.
Note that the code is given in full at the bottom of the page.



= Introduction =
This tutorial is designed to introduce the Python interface for modelling vessel networks. An equivalent C++ tutorial
is [wiki:PaperTutorials/Angiogenesis/BuildVesselNetwork here].

This tutorial covers:
 * Building a network from a collection of nodes, segments and vessels
 * Writing networks to file and visualizing with Paraview
 * Building a network using a network generator
 * Reading a network from file
 
Further functionality is gradually introduced over the course of subsequent tutorials.

= The Test =

```python
import unittest
import chaste.core
import microvessel_chaste
import microvessel_chaste.geometry
import microvessel_chaste.mesh
import microvessel_chaste.population.vessel
from microvessel_chaste.utility import * # bring in all units for convenience

class TestPythonBuildVesselNetworkLiteratePaper(unittest.TestCase):
```
= Test 1 - Building a vessel network manually, writing it to file and visualizing it=
In the first test we will build a vessel network from its constituent components; nodes, segments and vessels. We will do some
simple tests to make sure the network has been formed as expected. Then we write the network to file and visualize it in Paraview.

```python
    def test_BuildNetworkManually(self):

```
First we make some nodes, which are point features from which vessels can be constructed. They are initialized with a location.
All vessel network components are created using special factory methods which return shared pointers, rather than being created
directly through their constructors. Vessel network components are templated over spatial dimension, and can be 2D or 3D. We will
create a Y shaped network. Later we will learn how to build up networks in a more efficient manner.

```python
        length_scale = 1.e-6*metre()
        length = 100.0
        n1 = microvessel_chaste.population.vessel.VesselNode3(0.0, 0.0 ,0.0, length_scale)
        n2 = microvessel_chaste.population.vessel.VesselNode3(length, 0.0, 0.0, length_scale)
        n3 = microvessel_chaste.population.vessel.VesselNode3(2.0 * length, length, 0.0, length_scale)
        n4 = microvessel_chaste.population.vessel.VesselNode3(2.0 * length, -length, 0.0, length_scale)

```
Next we make vessel segments and vessels. Vessel segments are straight-line features which contain a vascular node at each end. Vessels
can be constructed from multiple vessel segments, but in this case each vessel just has a single segment.

```python
        v1 = microvessel_chaste.population.vessel.Vessel3([n1 ,n2])
        v2 = microvessel_chaste.population.vessel.Vessel3([n2, n3])
        v3 = microvessel_chaste.population.vessel.Vessel3([n2, n4])

```
Now we can add our vessels to a vessel network.

```python
        network = microvessel_chaste.population.vessel.VesselNetwork3()
        network.AddVessel(v1)
        network.AddVessel(v2)
        network.AddVessel(v3)

```
We use our test framework to make sure that the network has been created correctly by checking the number of vessels and nodes

```python
        self.assertEqual(network.GetNumberOfNodes(), 4)
        self.assertEqual(network.GetNumberOfVessels(), 3)

```
Next we write out network to file. We use the Chaste `OutputFileHandler` functionality to management the output location
Networks are written using VTKs PolyData format, which should have a .vtp extension.

```python
        file_handler = chaste.core.OutputFileHandler("Python/TestPythonBuildVesselNetworkLiteratePaper", True)
        writer = microvessel_chaste.population.vessel.VesselNetworkWriter3()
        writer.SetVesselNetwork(network)
        writer.SetFileName(file_handler.GetOutputDirectoryFullPath() + "bifurcating_network.vtp")
        writer.Write()

```
Now we can visualize then network in Paraview. See the tutorial [wiki:UserTutorials/VisualizingWithParaview here], to get started. To view the network import the file
`TestPythonBuildVesselNetworkLiteratePaper\bifurcating_network.vtp` into Paraview. For a nicer rendering you can do `Filters->Alphabetical->Tube`.

```python
if __name__ == '__main__':
    unittest.main()


# Code 
The full code is given below


## File name `TestPythonBuildVesselNetworkTutorial.py` 

```python
import unittest
import chaste.core
import microvessel_chaste
import microvessel_chaste.geometry
import microvessel_chaste.mesh
import microvessel_chaste.population.vessel
from microvessel_chaste.utility import * # bring in all units for convenience

class TestPythonBuildVesselNetworkLiteratePaper(unittest.TestCase):
    def test_BuildNetworkManually(self):

        length_scale = 1.e-6*metre()
        length = 100.0
        n1 = microvessel_chaste.population.vessel.VesselNode3(0.0, 0.0 ,0.0, length_scale)
        n2 = microvessel_chaste.population.vessel.VesselNode3(length, 0.0, 0.0, length_scale)
        n3 = microvessel_chaste.population.vessel.VesselNode3(2.0 * length, length, 0.0, length_scale)
        n4 = microvessel_chaste.population.vessel.VesselNode3(2.0 * length, -length, 0.0, length_scale)

        v1 = microvessel_chaste.population.vessel.Vessel3([n1 ,n2])
        v2 = microvessel_chaste.population.vessel.Vessel3([n2, n3])
        v3 = microvessel_chaste.population.vessel.Vessel3([n2, n4])

        network = microvessel_chaste.population.vessel.VesselNetwork3()
        network.AddVessel(v1)
        network.AddVessel(v2)
        network.AddVessel(v3)

        self.assertEqual(network.GetNumberOfNodes(), 4)
        self.assertEqual(network.GetNumberOfVessels(), 3)

        file_handler = chaste.core.OutputFileHandler("Python/TestPythonBuildVesselNetworkLiteratePaper", True)
        writer = microvessel_chaste.population.vessel.VesselNetworkWriter3()
        writer.SetVesselNetwork(network)
        writer.SetFileName(file_handler.GetOutputDirectoryFullPath() + "bifurcating_network.vtp")
        writer.Write()

if __name__ == '__main__':
    unittest.main()
```

