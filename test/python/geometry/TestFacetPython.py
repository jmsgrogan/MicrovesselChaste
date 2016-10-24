''' Tests for the geometry module.
'''

import unittest
import math
import numpy as np
import chaste.projects.microvessel as microvessel
import chaste.projects.microvessel.geometry
import chaste.projects.microvessel.mesh 
import chaste.projects.microvessel.utility as utility
        
class TestFacet(unittest.TestCase):
    
    ''' Test Facet Functionality'''
          
    def test_all_methods(self):
        
        # Make some vertices
        length_scale = 1.e-6*utility.metre()
        vertex1 = microvessel.mesh.DimensionalChastePoint3((0.0, 0.0, 0.0), length_scale)
        vertex2 = microvessel.mesh.DimensionalChastePoint3((1.0, 0.0, 0.0), length_scale)
        vertex3 = microvessel.mesh.DimensionalChastePoint3((1.0, 1.0, 0.0), length_scale)
        vertex4 = microvessel.mesh.DimensionalChastePoint3((0.0, 1.0, 0.0), length_scale)
        
        # Make a polygon with one vertex
        polygon1 = microvessel.geometry.Polygon3(vertex1)
        self.assertEqual(len(polygon1.GetVertices()), 1)
        polygon1.AddVertex(vertex2)
        polygon1.AddVertex(vertex3)
        polygon1.AddVertex(vertex4)

        # Make a facet
        facet = microvessel.geometry.Facet3(polygon1)
        self.assertEqual(len(facet.GetVertices()), 4)
        self.assertEqual(len(facet.GetPolygons()), 1)

        # Check the geometric features
        centroid = (0.5, 0.5, 0.0)
        normal = (0.0, 0.0, 1.0)
            
        # Check translating and rotating
        translation_vector = microvessel.mesh.DimensionalChastePoint3(1.0, 2.0, 3.0, length_scale)
        facet.Translate(translation_vector)
        rotation_axis = (0.0, 0.0, 1.0)
        facet.RotateAboutAxis(rotation_axis, math.pi)
        
if __name__ == '__main__':
    unittest.main()