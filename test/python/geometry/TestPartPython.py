''' Tests for the geometry module.
'''

import math
import numpy as np
import unittest
import chaste.core
import microvessel_chaste
import microvessel_chaste.geometry
import microvessel_chaste.mesh 
from microvessel_chaste.utility import * # bring in all units for convenience

class TestPart(unittest.TestCase):
    
    ''' Test Part Functionality'''
          
    def test_all_methods(self):
        
        file_handler = chaste.core.OutputFileHandler("Python/TestPart", True)
        
        # Make a composite Part, a circle in a square
        length_scale = 1.e-6*metre()
        part = microvessel_chaste.geometry.Part3()
        centre = microvessel_chaste.mesh.DimensionalChastePoint3((0.0, 0.0, 0.0), length_scale)
        part.AddRectangle(1.0 * length_scale, 1.0 * length_scale, centre)
        part.AddCircle(0.33 * length_scale, centre, 24)
        
        # Get the VTK Representation
        part.Write(file_handler.GetOutputDirectoryFullPath() + "original_part.vtp", 
                   microvessel_chaste.geometry.GeometryFormat.VTP, True)
        
if __name__ == '__main__':
    unittest.main()
        
        
        