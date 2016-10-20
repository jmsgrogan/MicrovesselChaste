''' Tests for the geometry module.
'''

import unittest
import math
import numpy as np
import os
import chaste
import chaste.projects.microvessel as microvessel
import chaste.projects.microvessel.mesh
import chaste.projects.microvessel.utility as utility

class TestDimensionalChastePoint(unittest.TestCase):
    
    ''' Test Vertex Functionality'''
          
    def test_all_methods(self):
        
        # Make a point at the specified location
        input_location = np.array((0.0, 1.0, 2.0))
        point = microvessel.mesh.DimensionalChastePoint3(input_location, 1.e-6*utility.length())
        point2 = microvessel.mesh.DimensionalChastePoint3(2.0*input_location, 1.e-6*utility.length())
        print point.GetLocation(1.e-6*utility.length())
        # Assert that the location is returned correctly
         
        # Set the Id and check it
        point.SetIndex(10)
        self.assertEqual(point.GetIndex(), 10)
         
        # Move the vertex and check the new location
        translation_vector = microvessel.mesh.DimensionalChastePoint3(1.0, 2.0, 3.0, 1.e-6*utility.length())
        point.Translate(translation_vector)
             
        # Rotate the vertex and check the location
        rotation_axis = (0.0, 0.0, 1.0)
        point.RotateAboutAxis(rotation_axis, math.pi)
         
        print point.GetLocation(1.e-6*utility.length())
        
if __name__ == '__main__':
    unittest.main()