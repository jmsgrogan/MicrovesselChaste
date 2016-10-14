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

    include_classes = ["RegularGrid<3>", 
                       "RegularGrid<2>", 
                       "DiscreteContinuumMesh<3,3>", 
                       "DiscreteContinuumMesh<2,2>",
                       "RegularGridWriter",
                       "DiscreteContinuumMeshGenerator<3,3>",
                       "DiscreteContinuumMeshGenerator<2,2>",
                       "MultiFormatMeshWriter<3>",
                       "MultiFormatMeshWriter<2>",
                       "DimensionalChastePoint<3>",
                       "DimensionalChastePoint<2>" ,
                       "MeshFormat"]
    
    for eachClass in include_classes:
        builder.class_(eachClass).include()  
        new_name = generate_bindings.template_replace(eachClass)
        if(new_name != eachClass):
            builder.class_(eachClass).rename(new_name) 
   
    return builder