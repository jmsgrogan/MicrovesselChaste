import unittest
import chaste.geometry
import chaste.population.vessel
import chaste.visualization.vtk_scene

class TestVtkScene(unittest.TestCase):
    
    def test_part(self):
        # Create a circle with radius 10
        part = chaste.geometry.Part()
        circle = part.AddCircle(100)
        part.Extrude(circle, 100)
        
        # Visualize it using VTK
        scene = chaste.visualization.vtk_scene.Scene()
        scene.add_part(part)
        scene.filename = "/home/grogan/test/visualization/TestVtkScene/part_capture.png"
        scene.show(interactive=False)
        
    def test_vessel_network(self):
        generator = chaste.population.vessel.VesselNetworkGenerator()
        network = generator.GenerateHexagonalUnit(100.0)
        
        # Visualize it using VTK
        scene = chaste.visualization.vtk_scene.Scene()
        scene.add_network(network)
        scene.filename = "/home/grogan/test/visualization/TestVtkScene/network_capture.png"
        scene.show(interactive=False)   
        
    def test_mesh(self):
        
        pass     

if __name__ == '__main__':
    unittest.main()