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

    include_classes = ["Vertex"] 
#                       "Facet", 
#                       "Polygon", 
#                       "Part<3>"]

    for eachClass in include_classes:
        builder.class_(eachClass).include()  
        new_name = generate_bindings.template_replace(eachClass)
        if(new_name != eachClass):
            builder.class_(eachClass).rename(new_name) 

    # There is a problem with templated default arguements, so turn them off for now.  
#    builder.class_('Part<3>').calldefs().use_default_arguments=False    
  
    return builder