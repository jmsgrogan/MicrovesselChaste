''' Tests for the geometry module.
'''

import unittest
import math
import numpy as np
import chaste.projects.microvessel as microvessel
import chaste.projects.microvessel.mesh

class TestVertex(unittest.TestCase):
    
    ''' Test Vertex Functionality'''
          
    def test_all_methods(self):
        
        # Make a point at the specified location
        input_location = np.array((0.0, 1.0, 2.0))
        point = microvessel.mesh.DimensionalChastePoint(input_location)
        
        # Assert that the location is returned correctly
        
        # Set the Id and check it
        point.SetId(10)
        self.assertEqual(point.GetId(), 10)
        
        # Move the vertex and check the new location
        translation_vector = np.array((1.0, 2.0, 3.0))
        point.Translate(point(translation_vector))
            
        # Rotate the vertex and check the location
        rotation_axis = (0.0, 0.0, 1.0)
        point.RotateAboutAxis(rotation_axis, math.pi)
        
if __name__ == '__main__':
    unittest.main()