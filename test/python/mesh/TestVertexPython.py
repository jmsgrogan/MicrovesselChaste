
"""Copyright (c) 2005-2016, University of Oxford.
 All rights reserved.

 University of Oxford means the Chancellor, Masters and Scholars of the
 University of Oxford, having an administrative office at Wellington
 Square, Oxford OX1 2JD, UK.

 This file is part of Chaste.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.
 * Neither the name of the University of Oxford nor the names of its
 contributors may be used to endorse or promote products derived from this
 software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import unittest
import math
import numpy as np
import os
import chaste
import microvessel_chaste
import microvessel_chaste.geometry
import microvessel_chaste.mesh 
from microvessel_chaste.utility import * # bring in all units for convenience

class TestVertex(unittest.TestCase):
    
    ''' Test Vertex Functionality'''
          
    def test_all_methods(self):
        
        # Make a point at the specified location
        length_scale = 1.e-6*metres
        input_location = np.array((0.0, 1.0, 2.0))
        point = microvessel_chaste.mesh.Vertex3(input_location, length_scale)
        self.assertAlmostEqual(point.GetLocation(length_scale)[0], 0.0, 2)
        self.assertAlmostEqual(point.GetLocation(length_scale)[1], 1.0, 2)
        self.assertAlmostEqual(point.GetLocation(length_scale)[2], 2.0, 2)
  
        # Set the Id and check it
        point.SetIndex(10)
        self.assertEqual(point.GetIndex(), 10)
         
        # Move the vertex and check the new location
        translation_vector = microvessel_chaste.mesh.Vertex3(1.0, 2.0, 3.0, length_scale)
        point.Translate(translation_vector)
        self.assertAlmostEqual(point.GetLocation(length_scale)[0], 1.0, 2)
        self.assertAlmostEqual(point.GetLocation(length_scale)[1], 3.0, 2)
        self.assertAlmostEqual(point.GetLocation(length_scale)[2], 5.0, 2)
  
        # Rotate the vertex and check the location
        rotation_axis = (0.0, 0.0, 1.0)
        point.RotateAboutAxis(rotation_axis, math.pi)
        self.assertAlmostEqual(point.GetLocation(length_scale)[0], -1.0, 2)
        self.assertAlmostEqual(point.GetLocation(length_scale)[1], -3.0, 2)
        self.assertAlmostEqual(point.GetLocation(length_scale)[2], 5.0, 2)
        
        input_location = np.array((0.0, 1.0, 2.0))
        point2d = microvessel_chaste.mesh.Vertex2(input_location, length_scale)
        
if __name__ == '__main__':
    unittest.main()