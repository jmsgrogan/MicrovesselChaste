#!/usr/bin/env python

"""
This scipt automatically generates Python bindings using a rule based approach
"""
import sys
from pyplusplus import module_builder
from pyplusplus.module_builder import call_policies
from pygccxml import parser
import generate_bindings

def update_builder(builder):
    
    include_classes = ["DiscreteSource<3>", 
                       "DiscreteContinuumBoundaryCondition<3>", 
                       "AbstractDiscreteContinuumNonLinearEllipticPde<3,3>", 
                       "LinearSteadyStateDiffusionReactionPde<3,3>",
                       "AbstractRegularGridDiscreteContinuumSolver<3>",
                       "FiniteDifferenceSolver<3>",
                       "FiniteElementSolver<3>",
                       "FunctionMap<3>",
                       "GreensFunctionSolver<3>",
                       "CellStateDependentDiscreteSource<3>",
                       "BoundaryConditionType",
                       "BoundaryConditionSource",
                       "SourceType",
                       "SourceStrength"]

    for eachClass in include_classes:
        builder.class_(eachClass).include()  
        new_name = generate_bindings.template_replace(eachClass)
        if(new_name != eachClass):
            builder.class_(eachClass).rename(new_name) 

    return builder