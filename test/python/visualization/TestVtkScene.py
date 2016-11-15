import unittest
import chaste
import microvessel_chaste
import microvessel_chaste.population.vessel
from microvessel_chaste.utility import *
import microvessel_chaste.visualization
import microvessel_chaste.mesh

class TestVtkScene(unittest.TestCase):
    
    def test_grid(self):
        
        file_handler = chaste.core.OutputFileHandler("Python/TestMicrovesselVtkScene/TestGrid2d");

        grid = microvessel_chaste.mesh.RegularGrid2()
        grid.SetExtents([20, 20, 1]);
        grid.SetSpacing(20.e-6* metre());

        grid_values = []
        for idx in range(grid.GetNumberOfPoints()):
            grid_values.append(float(idx)*10.0 + 100.0)
        grid.SetPointValues(grid_values)

        scene = microvessel_chaste.visualization.MicrovesselVtkScene2()
        scene.SetRegularGrid(grid)
        scene.GetRegularGridActorGenerator().SetShowPoints(False)
        scene.GetRegularGridActorGenerator().SetShowEdges(False)
        scene.GetRegularGridActorGenerator().SetShowVolume(True)
        scene.GetRegularGridActorGenerator().SetDataLabel("Point Values")
        scene.GetRegularGridActorGenerator().SetEdgeSize(1.0)
        scene.GetRegularGridActorGenerator().SetVolumeOpacity(0.8)

        scene.SetIsInteractive(False)
        scene.SetSaveAsAnimation(False)
        scene.SetSaveAsImages(True)
        scene.SetOutputFilePath(file_handler.GetOutputDirectoryFullPath()+"render")
        scene.Start()
        
    def test_grid_3d(self):
        
        file_handler = chaste.core.OutputFileHandler("Python/TestMicrovesselVtkScene/TestGrid3d");

        grid = microvessel_chaste.mesh.RegularGrid3()
        grid.SetExtents([20, 20, 5]);
        grid.SetSpacing(20.e-6* metre());

        grid_values = []
        for idx in range(grid.GetNumberOfPoints()):
            grid_values.append(float(idx)*10.0 + 100.0)
        grid.SetPointValues(grid_values)

        scene = microvessel_chaste.visualization.MicrovesselVtkScene3()
        scene.SetRegularGrid(grid)
        scene.GetRegularGridActorGenerator().SetShowPoints(False)
        scene.GetRegularGridActorGenerator().SetShowEdges(False)
        scene.GetRegularGridActorGenerator().SetShowVolume(True)
        scene.GetRegularGridActorGenerator().SetDataLabel("Point Values")
        scene.GetRegularGridActorGenerator().SetEdgeSize(1.0)
        scene.GetRegularGridActorGenerator().SetVolumeOpacity(0.8)

        scene.SetIsInteractive(False)
        scene.SetSaveAsAnimation(False)
        scene.SetSaveAsImages(True)
        scene.SetOutputFilePath(file_handler.GetOutputDirectoryFullPath()+"render")
        scene.Start()

if __name__ == '__main__':
    unittest.main()