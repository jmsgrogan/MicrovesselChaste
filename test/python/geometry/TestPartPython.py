''' Tests for the geometry module.
'''

import math
import numpy as np
import unittest
import chaste
import chaste.projects.microvessel as microvessel
import chaste.projects.microvessel.geometry
import chaste.projects.microvessel.mesh 
import chaste.projects.microvessel.utility as utility

class TestPart(unittest.TestCase):
    
    ''' Test Part Functionality'''
          
    def test_all_methods(self):
        
        file_handler = chaste.core.OutputFileHandler("Python/TestPart", True)
        
        # Make a composite Part, a circle in a square
        length_scale = 1.e-6*utility.metre()
        part = microvessel.geometry.Part3()
        centre = microvessel.mesh.DimensionalChastePoint3((0.0, 0.0, 0.0), length_scale)
        part.AddRectangle(1.0 * length_scale, 1.0 * length_scale, centre)
        part.AddCircle(0.33 * length_scale, centre, 24)
        
        # Get the VTK Representation
        part.Write(file_handler.GetOutputDirectoryFullPath() + "original_part.vtp", microvessel.geometry.GeometryFormat.VTP)
        
if __name__ == '__main__':
    unittest.main()
        
        
        