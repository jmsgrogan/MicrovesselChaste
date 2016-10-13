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

    include_classes = ["Facet<3>", 
                       "Polygon<3>", 
                       "Part<3>",
                       "Facet<2>", 
                       "Polygon<2>", 
                       "Part<2>"
                       "GeometryFormat",]
#                        "NetworkToSurface<3>",
#                        "NetworkToSurface<3>",
#                        "VesselSurfaceGenerator<3>",
#                        "VesselSurfaceGenerator<2>",
#                        "VoronoiGenerator<2>",
#                        "VoronoiGenerator<2>",
#                        "BoundaryExtractor",
#                        "SurfaceCleaner",
#                        "GeometryWriter"]

    for eachClass in include_classes:
        builder.class_(eachClass).include()  
        new_name = generate_bindings.template_replace(eachClass)
        if(new_name != eachClass):
            builder.class_(eachClass).rename(new_name) 

    # There is a problem with templated default arguements, so turn them off for now.  
    builder.class_('Part<3>').calldefs().use_default_arguments=False   
    builder.class_('Facet<3>').calldefs().use_default_arguments=False    
    builder.class_('Polygon<3>').calldefs().use_default_arguments=False 
    builder.class_('Part<2>').calldefs().use_default_arguments=False   
    builder.class_('Facet<2>').calldefs().use_default_arguments=False    
    builder.class_('Polygon<2>').calldefs().use_default_arguments=False      
  
    return builder