''' Tests for the geometry module.
'''

import unittest
import chaste.utility
import chaste.geometry
import chaste.mesh
import chaste.simulation.setup
        
class TestRWC(unittest.TestCase):
    
    def test_vtk_read_write(self):
        
        file_handler = chaste.simulation.setup.setup("/home/grogan/test/utility/RWC")
        
        part = chaste.geometry.Part()
        part.AddRectangle(100.0, 200.0)
        vtk_part = part.GetVtk(True)
        chaste.utility.readwrite.write(vtk_part, file_handler.GetOutputDirectoryFullPath() + "part.vtp")
        input_part = chaste.utility.readwrite.read(file_handler.GetOutputDirectoryFullPath() + "part.vtp")
        
        grid = chaste.mesh.RegularGrid()
        grid.SetExtents((100, 100, 3))
        grid.SetUpVtkGrid()
        grid.Write(file_handler)
        input_grid = chaste.utility.readwrite.read(file_handler.GetOutputDirectoryFullPath() + "grid.vti")
        chaste.utility.readwrite.write(input_grid, file_handler.GetOutputDirectoryFullPath() + "out_grid.vti")
        
if __name__ == '__main__':
    unittest.main()