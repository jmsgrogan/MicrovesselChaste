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
    
    include_classes = ["AngiogenesisSolver<3>",
                       "Owen2011MigrationRule<3>",
                       "Owen2011SproutingRule<3>",
                       "OffLatticeMigrationRule<3>",
                       "OffLatticeMigrationRule<3>",
                       "OffLatticeSproutingRule<3>",
                       "AbstractMigrationRule<3>",
                       "AbstractSproutingRule<3>",
                       "AngiogenesisSolver<2>",
                       "Owen2011MigrationRule<2>",
                       "Owen2011SproutingRule<2>",
                       "OffLatticeMigrationRule<2>",
                       "OffLatticeMigrationRule<2>",
                       "OffLatticeSproutingRule<2>",
                       "AbstractMigrationRule<2>",
                       "AbstractSproutingRule<2>",
                       "LatticeBasedMigrationRule<2>",
                       "CellPopulationMigrationRule<2>",
                       "RegressionSolver<2>",
                       "WallShearStressBasedRegressionSolver<2>",
                       "LatticeBasedMigrationRule<3>",
                       "CellPopulationMigrationRule<3>",
                       "RegressionSolver<3>",
                       "WallShearStressBasedRegressionSolver<3>",
                       ]
    
    for eachClass in include_classes:
        builder.class_(eachClass).include()  
        new_name = generate_bindings.template_replace(eachClass)
        if(new_name != eachClass):
            builder.class_(eachClass).rename(new_name) 
    
    return builder
