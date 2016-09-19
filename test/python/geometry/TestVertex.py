''' Tests for the geometry module.
'''

import unittest
import math
import numpy as np
import chaste.projects.angiogenesis as angiogenesis
import chaste.projects.angiogenesis.geometry

class TestVertex(unittest.TestCase):
    
    ''' Test Vertex Functionality'''
          
    def test_all_methods(self):
        
        # Make a vertex at the specified location
        input_location = np.array((0.0, 1.0, 2.0))
        vertex = angiogenesis.geometry.Vertex(input_location)
        
        # Assert that the location is returned correctly
        
        # Set the Id and check it
        vertex.id = 10
        self.assertEqual(vertex.id, 10)
        
        # Move the vertex and check the new location
        translation_vector = np.array((1.0, 2.0, 3.0))
        vertex.Translate(translation_vector)
            
        # Rotate the vertex and check the location
        rotation_axis = (0.0, 0.0, 1.0)
        vertex.RotateAboutAxis(rotation_axis, math.pi)
        
if __name__ == '__main__':
    unittest.main()