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
import doxygen_extractor
from pygccxml import parser

def template_replace(class_name):

    # How to replace template names in Python output
    new_name = class_name
    if "<3>" in class_name[-3:]:
        new_name = class_name[:-3] + "3"            
    if "<2>" in class_name[-3:]:
        new_name = class_name[:-3] + "2"       
    if "<3,3>" in class_name[-5:]:
        new_name = class_name[:-5] + "3_3"        
    if "<2,2>" in class_name[-5:]:
        new_name = class_name[:-5] + "2_2"  
    return new_name

def boost_units_namespace_fix(module_file):
    
    # There is a bug (maybe in boost units) where sometimes static_rational does not have
    # the full boost::units namespace. Manually put it in.
    lines = []
    replacements = {", static_rational": ", boost::units::static_rational"}
    with open(module_file) as infile:
        for line in infile:
            for src, target in replacements.iteritems():
                line = line.replace(src, target)
            lines.append(line)
    with open(module_file, 'w') as outfile:
        for line in lines:
            outfile.write(line)    
            
def do_module(module_name, builder):
    
    # Set up the builder with module specifc classes
    this_module = __import__("generate_" + module_name)
    return this_module.update_builder(builder)
       
def generate_wrappers(args):
    module_name = args[1]
    work_dir = args[2]
    header_collection = args[3]
    castxml_binary = args[4]
    includes = args[5:]

    xml_generator_config = parser.xml_generator_configuration_t(xml_generator_path=castxml_binary, 
                                                                xml_generator="castxml",
                                                                compiler = "gnu",
                                                                compiler_path="/usr/bin/c++",
                                                                include_paths=includes)

    builder = module_builder.module_builder_t([header_collection],
                                                xml_generator_path = castxml_binary,
                                                xml_generator_config = xml_generator_config,
                                                start_with_declarations = ['chaste'],
                                                include_paths = includes,
                                                indexing_suite_version=2)
    
    # Don't wrap std library
    builder.global_ns.namespace('std').exclude()
    builder.global_ns.exclude()
    
    # Set up the builder for each module
    builder = do_module(module_name, builder)
    
    # Make the wrapper code
#     builder.build_code_creator(module_name="_chaste_project_MicrovesselChaste_" + module_name, 
#                                doc_extractor=doxygen_extractor.doxygen_doc_extractor())
    builder.build_code_creator(module_name="_chaste_project_MicrovesselChaste_" + module_name)
    builder.code_creator.user_defined_directories.append(work_dir + "/dynamic/wrapper_headers/")
    builder.write_module(work_dir + "/dynamic/" + module_name + ".cpp")
    
    # Fix a bug with boost units
    boost_units_namespace_fix(work_dir + "/dynamic/" + module_name + ".cpp")
    
if __name__=="__main__":
    generate_wrappers(sys.argv)
