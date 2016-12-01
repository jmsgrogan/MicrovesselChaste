#!/usr/bin/env python

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

import sys
from pyplusplus import module_builder
from pyplusplus.module_builder import call_policies
from pygccxml import parser
import generate_bindings

def update_builder(builder):
    
    include_classes = ["FlowSolver<2>",
                   "FlowSolver<3>",
                   "WallShearStressCalculator<2>",
                   "WallShearStressCalculator<3>",
                   "VesselImpedanceCalculator<3>",
                   "BetteridgeHaematocritSolver<3>",
                   "AbstractVesselNetworkCalculator<3>",
                   "MechanicalStimulusCalculator<3>",
                   "MetabolicStimulusCalculator<3>",
                   "RadiusCalculator<3>",
                   "ShrinkingStimulusCalculator<3>",
                   "ViscosityCalculator<3>",
                   "AbstractStructuralAdaptationSolver<3>",
                   "StructuralAdaptationSolver<3>",
                   "AbstractHaematocritSolver<3>",
                   "ConstantHaematocritSolver<3>",
                   "VesselImpedanceCalculator<2>",
                   "BetteridgeHaematocritSolver<2>",
                   "AbstractVesselNetworkCalculator<2>",
                   "MechanicalStimulusCalculator<2>",
                   "RadiusCalculator<2>",
                   "ShrinkingStimulusCalculator<2>",
                   "ViscosityCalculator<2>",
                   "AbstractStructuralAdaptationSolver<2>",
                   "StructuralAdaptationSolver<2>",
                   "AbstractHaematocritSolver<2>",
                   "MetabolicStimulusCalculator<2>",
                   "AlarconHaematocritSolver<2>",
                   "ConstantHaematocritSolver<2>",]

    class_collection = []

    for eachClass in include_classes:
        builder.class_(eachClass).include()  
        new_name = generate_bindings.template_replace(eachClass)
        class_collection.append(new_name)
        if(new_name != eachClass):
            builder.class_(eachClass).rename(new_name) 
    
    return builder, class_collection
