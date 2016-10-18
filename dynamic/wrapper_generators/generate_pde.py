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
    
    include_classes = ["CellStateDependentDiscreteSource<3>", 
                       "CellBasedDiscreteSource<3>", 
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
                       "SourceStrength"
                       "DiscreteSource<3>",
                       "SolutionDependentDiscreteSource<3>",
                       "VesselBasedDiscreteSource<3>",
                       "DiscreteContinuumBoundaryCondition<3>",
                       "AbstractDiscreteContinuumLinearEllipticPde<3, 3>",
                       "AbstractDiscreteContinuumNonLinearEllipticPde<3, 3>",
                       "LinearSteadyStateDiffusionReactionPde<3, 3>",
                       "MichaelisMentenSteadyStateDiffusionReactionPde<3, 3>",
                       "AbstractDiscreteContinuumSolver<3>",
                       "AbstractRegularGridDiscreteContinuumSolver<3>",
                       "AbstractUnstructuredGridDiscreteContinuumSolver<3>",
                       "FiniteDifferenceSolver<3>",
                       "FiniteElementSolver<3>",
                       "FunctionMap<3>",
                       "GreensFunctionSolver<3>",
                       "DiscreteSource<2>",
                       "CellStateDependentDiscreteSource<2>",
                       "CellBasedDiscreteSource<2>",
                       "SolutionDependentDiscreteSource<2>",
                       "VesselBasedDiscreteSource<2>",
                       "DiscreteContinuumBoundaryCondition<2>",
                       "AbstractDiscreteContinuumLinearEllipticPde<2, 2>",
                       "AbstractDiscreteContinuumNonLinearEllipticPde<2, 2>",
                       "LinearSteadyStateDiffusionReactionPde<2, 2>",
                       "MichaelisMentenSteadyStateDiffusionReactionPde<2, 2>",
                       "AbstractDiscreteContinuumSolver<2>",
                       "AbstractRegularGridDiscreteContinuumSolver<2>",
                       "AbstractUnstructuredGridDiscreteContinuumSolver<2>",
                       "FiniteDifferenceSolver<2>",
                       "FiniteElementSolver<2>",
                       "FunctionMap<2>",
                       "GreensFunctionSolver<2>",
                       ]

    for eachClass in include_classes:
        builder.class_(eachClass).include()  
        new_name = generate_bindings.template_replace(eachClass)
        if(new_name != eachClass):
            builder.class_(eachClass).rename(new_name) 

    return builder