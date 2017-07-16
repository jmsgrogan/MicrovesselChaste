''' Tests for the geometry module.
'''

import unittest
import math
import numpy as np
import microvessel_chaste
import microvessel_chaste.geometry
import microvessel_chaste.mesh 
from microvessel_chaste.utility import * # bring in all units for convenience
        
class TestFacet(unittest.TestCase):
    
    ''' Test Facet Functionality'''
          
    def test_all_methods(self):
        
        # Make some vertices
        length_scale = 1.e-6*metres
        vertex1 = microvessel_chaste.geometry.Vertex3(0.0*length_scale, 0.0*length_scale)
        vertex2 = microvessel_chaste.geometry.Vertex3(1.0*length_scale, 0.0*length_scale)
        vertex3 = microvessel_chaste.geometry.Vertex3(1.0*length_scale, 1.0*length_scale)
        vertex4 = microvessel_chaste.geometry.Vertex3(0.0*length_scale, 1.0*length_scale)
        
        # Make a polygon with one vertex
        polygon1 = microvessel_chaste.geometry.Polygon3(vertex1)
        self.assertEqual(len(polygon1.rGetVertices()), 1)
        polygon1.AddVertex(vertex2)
        polygon1.AddVertex(vertex3)
        polygon1.AddVertex(vertex4)

        # Make a facet
        facet = microvessel_chaste.geometry.Facet3(polygon1)
        self.assertEqual(len(facet.rGetVertices()), 4)
        self.assertEqual(len(facet.GetPolygons()), 1)
            
        # Check translating and rotating
        translation_vector = microvessel_chaste.geometry.Vertex3(1.0*length_scale, 
                                                                 2.0*length_scale, 
                                                                 3.0*length_scale)
        facet.Translate(translation_vector)
        rotation_axis = (0.0, 0.0, 1.0)
        facet.RotateAboutAxis(rotation_axis, math.pi)
        
if __name__ == '__main__':
    unittest.main()