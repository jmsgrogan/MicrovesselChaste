#!/usr/bin/env python
"""Helper classes for running tests
"""

__copyright__ = """Copyright (c) 2005-2017, University of Oxford.
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
import chaste.cell_based
import microvessel_chaste.utility

def MvcSetupNotebookTest():
    simulation_time = chaste.cell_based.SimulationTime.Instance()
    simulation_time.SetStartTime(0.0)
    chaste.core.RandomNumberGenerator.Instance().Reseed(0)
    chaste.cell_based.CellId.ResetMaxCellId()
    
def MvcTearDownNotebookTest():
    simulation_time = chaste.cell_based.SimulationTime.Instance()
    simulation_time.Destroy()
    
    chaste.core.RandomNumberGenerator.Instance().Destroy()
    #chaste.cell_based.CellPropertyRegistry.Instance().Clear()    

class AbstractMicrovesselTestSuite(unittest.TestCase):
    
    def setUp(self):
        simulation_time = chaste.cell_based.SimulationTime.Instance()
        simulation_time.SetStartTime(0.0)
        chaste.core.RandomNumberGenerator.Instance().Reseed(0)
        chaste.cell_based.CellId.ResetMaxCellId()
        vtk_controller = microvessel_chaste.utility.VtkSetupAndFinalize()
        vtk_controller.setUpVtk()
        
    def tearDown(self):
        #vtk_controller = microvessel_chaste.utility.VtkSetupAndFinalize()
        #vtk_controller.tearDownVtk()
        simulation_time = chaste.cell_based.SimulationTime.Instance()
        simulation_time.Destroy()
        
        chaste.core.RandomNumberGenerator.Instance().Destroy()
        #chaste.cell_based.CellPropertyRegistry.Instance().Clear()
        
class AbstractMicrovesselWithTimingsTestSuite(AbstractMicrovesselTestSuite):
    
    def setUp(self):
        chaste.core.Timer().Reset()
        super(AbstractCellBasedWithTimingsTestSuite, self).setUp()
        microvessel_chaste.utility.InitializeVtk()
        
    def tearDown(self):
        microvessel_chaste.utility.TeardownVtk()
        super(AbstractCellBasedWithTimingsTestSuite, self).tearDown()
        chaste.core.Timer().Print("Test elapsed")
        