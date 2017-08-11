#!/usr/bin/env python
"""Utility Module
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

from _chaste_project_MicrovesselChaste_utility import *

_symbol_dict = {QLength: "m",
                QTime: "s",
                QRate: "Hz",
                QMolarFlowRate: "kat",
                QVelocity: "ms^-1",
                QVolume: "m^3",
                QConcentration: "m^-3mol",
                QDiffusivity: "m^2s^-1"}

_unit_dict = {"m": "metres",
              "s": "seconds",
              "Hz": "per_second",
              "kat": "mole_per_second",
              "ms^-1": "metre_per_second",
              "m^3": "metres_cubed",
              "m^-3mol": "mole_per_metre_cubed",
              "m^2s^-1": "metre_squared_per_second"}

def get_symbol(quantity_type):
    return _symbol_dict(quantity_type)

def get_unit(symbol):
    return getattr(microvessel_chaste.utility, _unit_dict[symbol]) 

# Some units
_m = 1.0*metres
_um = 1.e-6*metres
_s = 1.0*seconds
_h = 1.0*hours
_M = 1.e3*mole_per_metre_cubed
_uM = 1.e-3*mole_per_metre_cubed
_nM = 1.e-6*mole_per_metre_cubed
