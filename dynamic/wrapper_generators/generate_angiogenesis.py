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
                       "AbstractSproutingRule<3>"]
    
    for eachClass in include_classes:
        builder.class_(eachClass).include()  
        new_name = generate_bindings.template_replace(eachClass)
        if(new_name != eachClass):
            builder.class_(eachClass).rename(new_name) 
    
    return builder
