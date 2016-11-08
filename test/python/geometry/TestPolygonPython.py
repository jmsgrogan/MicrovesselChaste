''' Tests for the geometry module.
'''

import unittest
import math
import numpy as np
import microvessel_chaste
import microvessel_chaste.geometry
import microvessel_chaste.mesh 
from microvessel_chaste.utility import * # bring in all units for convenience
        
class TestPolygon(unittest.TestCase):
    
    ''' Test Polygon Functionality'''
          
    def test_all_methods(self):
        
        # Make some vertices
        length_scale = 1.e-6*metre()
        vertex1 = microvessel_chaste.mesh.DimensionalChastePoint3((0.0, 0.0, 0.0), length_scale)
        vertex2 = microvessel_chaste.mesh.DimensionalChastePoint3((1.0, 0.0, 0.0), length_scale)
        vertex3 = microvessel_chaste.mesh.DimensionalChastePoint3((1.0, 1.0, 0.0), length_scale)
        vertex4 = microvessel_chaste.mesh.DimensionalChastePoint3((0.0, 1.0, 0.0), length_scale)
        
        # Make a polygon with one vertex
        polygon1 = microvessel_chaste.geometry.Polygon3(vertex1)
        self.assertEqual(len(polygon1.GetVertices()), 1)
        verts = [vertex2, vertex3, vertex4]
        polygon1.AddVertices(verts)
        
        # Make a polygon with several vertices
        polygon2 = microvessel_chaste.geometry.Polygon3(vertex2)
        polygon2.AddVertex(vertex3)
        polygon2.AddVertex(vertex4)
        self.assertEqual(len(polygon2.GetVertices()), 3)
        polygon2.AddVertex(vertex1)
        
        # Check the geometric features
        centroid = (0.5, 0.5, 0.0)
        normal = (0.0, 0.0, 1.0)
            
        bounding_box = polygon2.GetBoundingBox()
        target_bbox = [0.0, 1.0, 0.0, 1.0, 0.0, 0.0]
        
        # Check translating and rotating
        translation_vector = microvessel_chaste.mesh.DimensionalChastePoint3(1.0, 2.0, 3.0, length_scale)
        polygon2.Translate(translation_vector)
        rotation_axis = (0.0, 0.0, 1.0)
        polygon2.RotateAboutAxis(rotation_axis, math.pi)
        
if __name__ == '__main__':
    unittest.main()
        