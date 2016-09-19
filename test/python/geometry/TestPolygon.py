''' Tests for the geometry module.
'''

import unittest
import math
import numpy as np
import chaste.projects.angiogenesis as angiogenesis
import chaste.projects.angiogenesis.geometry 
        
class TestPolygon(unittest.TestCase):
    
    ''' Test Polygon Functionality'''
          
    def test_all_methods(self):
        
        # Make some vertices
        vertex1 = angiogenesis.geometry.Vertex((0.0, 0.0, 0.0))
        vertex2 = angiogenesis.geometry.Vertex((1.0, 0.0, 0.0))
        vertex3 = angiogenesis.geometry.Vertex((1.0, 1.0, 0.0))
        vertex4 = angiogenesis.geometry.Vertex((0.0, 1.0, 0.0))
        
        # Make a polygon with one vertex
        polygon1 = angiogenesis.geometry.Polygon(vertex1)
        self.assertEqual(len(polygon1.GetVertices()), 1)
        polygon1.AddVertices([vertex2, vertex3, vertex4])
        
        # Make a polygon with several vertices
        polygon2 = angiogenesis.geometry.Polygon([vertex2, vertex3, vertex4])
        self.assertEqual(len(polygon2.GetVertices()), 3)
        polygon2.AddVertex(vertex1)
        
        # Check the geometric features
        centroid = (0.5, 0.5, 0.0)
        normal = (0.0, 0.0, 1.0)
            
        bounding_box = polygon2.GetBoundingBox()
        target_bbox = [0.0, 1.0, 0.0, 1.0, 0.0, 0.0]
        
        # Check translating and rotating
        translation_vector = (1.0, 2.0, 3.0)
        polygon2.Translate(translation_vector)
        rotation_axis = (0.0, 0.0, 1.0)
        polygon2.RotateAboutAxis(rotation_axis, math.pi)
        