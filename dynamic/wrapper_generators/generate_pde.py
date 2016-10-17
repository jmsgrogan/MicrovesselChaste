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
                       "SourceStrength"]
    
    sizeof(DiscreteSource<3>) +
                    sizeof(CellStateDependentDiscreteSource<3>) +
                    sizeof(CellBasedDiscreteSource<3>) +
                    sizeof(SolutionDependentDiscreteSource<3>) +
                    sizeof(VesselBasedDiscreteSource<3>) +
                    sizeof(DiscreteContinuumBoundaryCondition<3>) +
                    sizeof(AbstractDiscreteContinuumLinearEllipticPde<3, 3>) +
                    sizeof(AbstractDiscreteContinuumNonLinearEllipticPde<3, 3>) +
                    sizeof(LinearSteadyStateDiffusionReactionPde<3, 3>) +
                    sizeof(MichaelisMentenSteadyStateDiffusionReactionPde<3, 3>) +
                    sizeof(LinearSteadyStateDiffusionReactionPde<3, 3>) +
                    sizeof(AbstractDiscreteContinuumSolver<3>) +
                    sizeof(AbstractRegularGridDiscreteContinuumSolver<3>) +
                    sizeof(AbstractUnstructuredGridDiscreteContinuumSolver<3>) +
                    sizeof(FiniteDifferenceSolver<3>) +
                    sizeof(FiniteElementSolver<3>) +
                    sizeof(FunctionMap<3>) +
                    sizeof(GreensFunctionSolver<3>) +
                    sizeof(DiscreteSource<2>) +
                    sizeof(CellStateDependentDiscreteSource<2>) +
                    sizeof(CellBasedDiscreteSource<2>) +
                    sizeof(SolutionDependentDiscreteSource<2>) +
                    sizeof(VesselBasedDiscreteSource<2>) +
                    sizeof(DiscreteContinuumBoundaryCondition<2>) +
                    sizeof(AbstractDiscreteContinuumLinearEllipticPde<2, 2>) +
                    sizeof(AbstractDiscreteContinuumNonLinearEllipticPde<2, 2>) +
                    sizeof(LinearSteadyStateDiffusionReactionPde<2, 2>) +
                    sizeof(MichaelisMentenSteadyStateDiffusionReactionPde<2, 2>) +
                    sizeof(LinearSteadyStateDiffusionReactionPde<2, 2>) +
                    sizeof(AbstractDiscreteContinuumSolver<2>) +
                    sizeof(AbstractRegularGridDiscreteContinuumSolver<2>) +
                    sizeof(AbstractUnstructuredGridDiscreteContinuumSolver<2>) +
                    sizeof(FiniteDifferenceSolver<2>) +
                    sizeof(FiniteElementSolver<2>) +
                    sizeof(FunctionMap<2>) +
                    sizeof(GreensFunctionSolver<2>);

    for eachClass in include_classes:
        builder.class_(eachClass).include()  
        new_name = generate_bindings.template_replace(eachClass)
        if(new_name != eachClass):
            builder.class_(eachClass).rename(new_name) 

    return builder