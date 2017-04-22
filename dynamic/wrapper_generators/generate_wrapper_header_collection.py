#!/usr/bin/env python

"""Copyright (c) 2005-2017, University of Oxford.
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

"""
Generate the file classes_to_be_wrapped.hpp, which contains includes, instantiation and
naming typedefs for all classes that are to be automatically wrapped.
"""

import os
import sys
import fnmatch
try:
   import cPickle as pickle
except:
   import pickle
import classes_to_be_wrapped
from wrapper_utilities.class_data import CppClass

package_name = "MICROVESSELCHASTE"

def generate_class_data(args):
    
    # Store the classes to be wrapped in a dict
    class_dict = {}
    for eachClass in classes_to_be_wrapped.classes:
        class_dict[eachClass.name] = eachClass
    
    # Traverse the source tree and get the filepath for each class to be wrapped
    source_root = args[1]
    component_keys = ["angiogenesis", "flow", "cell", "geometry", "pde", "mesh",
                      "simulation", "utility", "tutorial", "vessel", "visualization"]
    
    for root, dirnames, filenames in os.walk(source_root, followlinks=True):
        for filename in fnmatch.filter(filenames, '*.hpp'):
            filename_no_ext = os.path.splitext(filename)[0]
            if filename_no_ext in class_dict.keys():
                class_dict[filename_no_ext].full_path = os.path.join(root, filename)

    # Set the component name
    for eachClass in classes_to_be_wrapped.classes:
        if eachClass.component is None and eachClass.full_path is not None:
            for eachKey in component_keys:
                if "/" + eachKey + "/" in eachClass.full_path:
                    eachClass.component = eachKey
                    
    # Set the template args by trying to guess what they are from the HPP files.
    # DOn't guess if they have been manually specified.
    same_dims_1 = [[2], [3]] # i.e. <DIM>
    same_dims_2 = [[2, 2], [3, 3]] # i.e. <ELEMENT_DIM, SPACE_DIM>
    
    template_strings_2d = ["<unsignedELEMENT_DIM,unsignedSPACE_DIM>", 
                           "<unsignedDIM,unsignedDIM>",
                           "unsignedELEMENT_DIM,unsignedSPACE_DIM=ELEMENT_DIM"]
    template_strings_1d = ["<unsignedSPACE_DIM>", 
                           "<unsignedDIM>"]    
    
    for eachClass in classes_to_be_wrapped.classes:
        if eachClass.template_args is None and eachClass.full_path is not None:
            if os.path.exists(eachClass.full_path):
                f = open(eachClass.full_path)
                lines = (line.rstrip() for line in f) # Remove blank lines
                lines = list(line for line in lines if line)
                for idx, eachLine in enumerate(lines):
                    stripped_line = eachLine.replace(" ", "")
                    if idx+1 < len(lines):
                        stripped_next_line = lines[idx+1].replace(" ", "")
                    else:
                        continue
                    
                    for eachTemplateString in template_strings_2d:
                        if eachTemplateString in stripped_line:
                            if "class" + eachClass.name +":" in stripped_next_line or "class" + eachClass.name ==stripped_next_line:
                                eachClass.template_args = same_dims_2
                                break
                        
                    for eachTemplateString in template_strings_1d:
                        if eachTemplateString in stripped_line:
                            if "class" + eachClass.name +":" in stripped_next_line or "class" + eachClass.name ==stripped_next_line:
                                eachClass.template_args = same_dims_1
                                break          
                f.close()
                
    with open(args[2]+"class_data.p", 'wb') as fp:
        pickle.dump(classes_to_be_wrapped.classes, fp)
        
    return classes_to_be_wrapped.classes

def generate_hpp_file(args):
    
    classes = generate_class_data(args)
    
    file_path = args[2] + "/wrapper_header_collection.hpp"
    hpp_file = open(file_path, 'w')
    
    # Add the includes
    hpp_file.write("#ifndef " + package_name + "HEADERS_HPP_ \n")
    hpp_file.write("#define " + package_name + "HEADERS_HPP_ \n")
    hpp_file.write("\n// Includes \n")
    
    # Start with STL components
    hpp_file.write("#include <vector>\n")
    hpp_file.write("#include <set>\n")
    hpp_file.write("#include <map>\n")
    
    # Now Chaste includes
    for eachClass in classes:
        if eachClass.needs_include_file:
            hpp_file.write("#include " + '"' + eachClass.name + '.hpp"' + "\n")
        
    # Add the instantiations
    hpp_file.write("\n// Instantiate Template Classes \n")
    for eachClass in classes:
        if not eachClass.needs_header_file_instantiation():
            continue
        for eachClassTemplateName in eachClass.get_full_names():
            hpp_file.write("template class " + eachClassTemplateName + ";\n")   
            
    # Add typdefs for nice naming
    hpp_file.write("\n// Typedef for nicer naming in Py++ \n")
    hpp_file.write("namespace pyplusplus{ \n")
    hpp_file.write("namespace aliases{ \n")
    for eachClass in classes:
        if not eachClass.needs_header_file_typdef():
            continue
        
        short_names = eachClass.get_short_names()
        for idx, eachClassTemplateName in enumerate(eachClass.get_full_names()):
            hpp_file.write("typedef " + eachClassTemplateName + " " + 
                           short_names[idx] + ";\n")    
            
    # Add extra typdefs for STL and UBLAS components so name conflicts do not arise
    # in wrapper classes.
    extra_typdefs = [CppClass("std::map", "std", [["std::string", "std::string"]], True),
                     CppClass("std::set", "std", [["unsigned"]], True),
                     CppClass("std::vector", "std", [["double"], 
                                                    ["unsigned"], 
                                                    ["bool"],
                                                    ["std::string"],
                                                    ["c_vector<double,3>"],
                                                    ["c_vector<double,2>"],
                                                    ["c_vector<unsigned,5>"],
                                                    ["std::set<unsigned int>"],
                                                    ["std::vector<unsigned int>"],
                                                    ["std::pair<unsigned int, unsigned int>"]], True),
                     CppClass("c_vector", "blas",[["double", 2], ["double", 3], ["unsigned", 5]], True)]
    for eachClass in extra_typdefs:
        if not eachClass.needs_header_file_typdef():
            continue
        
        short_names = eachClass.get_short_names()
        for idx, eachClassTemplateName in enumerate(eachClass.get_full_names()):
            hpp_file.write("typedef " + eachClassTemplateName + " " + 
                           short_names[idx] + ";\n")    
      
    hpp_file.write("    }\n")
    hpp_file.write("}\n")
    
    ## Add some special includes for Boost and PETSc
    hpp_file.write("\n// Need to specifically instantiate PETSc Vec and Mat \n")
    hpp_file.write("typedef boost::filesystem::path boost_filesystem_path;\n")
    hpp_file.write("\n inline int Instantiation()\n{\nreturn sizeof(Mat) + sizeof(Vec);\n}\n")
    hpp_file.write("\n inline Mat GetPetscMatForWrapper()\n{\nMat A;\nPetscTools::SetupMat(A, 3, 3, 3);\nreturn A;\n}\n")
    
    hpp_file.write("#endif // " + package_name + "HEADERS_HPP_\n")

if __name__=="__main__":
    
    #fake_path = ["","/home/grogan/Chaste", "/home/grogan/"]
    generate_hpp_file(sys.argv)
    #generate_hpp_file(fake_path)
