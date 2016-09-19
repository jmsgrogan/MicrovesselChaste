''' Tests for the geometry module.
'''

import math
import numpy as np
import unittest
import chaste
import chaste.projects.angiogenesis as angiogenesis
import chaste.projects.angiogenesis.geometry 

class TestPart(unittest.TestCase):
    
    ''' Test Part Functionality'''
          
    def test_all_methods(self):
        
        file_handler = chaste.core.OutputFileHandler("geometry/TestPart", True)
        
        # Make a composite Part, a circle in a square
        part = angiogenesis.geometry.Part3()
        part.AddRectangle(1.0, 1.0, (0.0, 0.0, 0.0))
        centre = (0.5, 0.5 ,0.0)
        part.AddCircle(0.33, centre, 24)
        
        # Get the VTK Representation
        part.Write(file_handler.GetOutputDirectoryFullPath() + "original_part.vtp")
        
        vtk_part = part.GetVtk(True)
        
if __name__ == '__main__':
    unittest.main()
        
        
        