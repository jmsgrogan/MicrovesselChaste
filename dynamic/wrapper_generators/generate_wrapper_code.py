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

import sys
sys.setrecursionlimit(5000) # Avoid: RuntimeError: maximum recursion depth exceeded in cmp
import os
try:
   import cPickle as pickle
except:
   import pickle
import doxygen_extractor
from pyplusplus import module_builder
from pyplusplus.module_builder import call_policies, file_cache_t
from pyplusplus import messages
from pygccxml import parser, declarations
from pprint import pprint
import wrapper_utilities.vessel_additions
import wrapper_utilities.utility_additions

chaste_license = """
/*

Copyright (c) 2005-2017, University of Oxford.
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

*/
"""



def add_autowrap_classes_to_builder(builder, component_name, classes):
    
    # Convience dict for call policies
    call_policies_collection = {"reference_existing_object": call_policies.return_value_policy(call_policies.reference_existing_object),
                                "return_opaque_pointer": call_policies.return_value_policy(call_policies.return_opaque_pointer) ,
                                "return_internal_reference" : call_policies.return_internal_reference()}
    
    # Classes which have methods taking or returning PETSc vec or mat need to add
    # this custom wrapper code to allow wrapping of PETSc opaque pointers. Any 
    # methods returning them also needs to set up opaque pointer management by
    # choosing the Py++ 'return_opaque_pointer' option.
    petsc_mat_custom_code = "BOOST_PYTHON_OPAQUE_SPECIALIZED_TYPE_ID( _p_Mat )"
    petsc_vec_custom_code = "BOOST_PYTHON_OPAQUE_SPECIALIZED_TYPE_ID( _p_Vec )"
    
    # Remove any classes not in this module. Also use the class to collect cell and population writers.
    classes_not_in_module = []
    for eachClass in classes:

        if not eachClass.needs_auto_wrapper_generation():
            continue
        if eachClass.component != component_name:
            full_class_names = eachClass.get_full_names()
            for eachTemplatedClassName in full_class_names:
                classes_not_in_module.append(eachTemplatedClassName.replace(' ', ''))
                
    builder.classes(lambda decl: decl.name.replace(' ', '') in classes_not_in_module).exclude()
    
    # Exclude all iterators
    builder.classes( lambda x: x.name in ("Iterator",)).exclude()
    
    # Set up the class in the Py++ builder
    for eachClass in classes:
        if not eachClass.needs_auto_wrapper_generation():
            continue
        
        if eachClass.component == component_name:
            short_class_names = eachClass.get_short_names()
            full_class_names = eachClass.get_full_names()
            for idx, eachTemplatedClassName in enumerate(full_class_names):
                
                # Add the class to the builder
                print "Processing: ", eachTemplatedClassName, " aka ", short_class_names[idx]
                this_class = builder.class_(eachTemplatedClassName)
                this_class.include() 
                
                # Rename the class with its short name, avoids having complicated
                # class names, which Py++ would have to deal with, in the C++ wrapper code.
                if(short_class_names[idx] != eachTemplatedClassName):
                    this_class.rename(short_class_names[idx]) 
                    
                # Set up member function excludes and pointer management
                has_members = False
                try:
                    this_class.member_functions()
                    has_members = True
                except RuntimeError:
                    pass
                
                has_constructors = False
                try:
                    this_class.constructors()
                    has_constructors = True
                except RuntimeError:
                    pass
                
                add_petsc_vec_code = False
                add_petsc_mat_code = False
                petsc_vec_code_will_auto = False
                petsc_mat_code_will_auto = False
                
                if has_constructors:
                    for eachConstructor in this_class.constructors():
                        for eachArgType in eachConstructor.arguments:
                            
                            if "Vec" in eachArgType.type.decl_string and not "CellVecData" in eachArgType.type.decl_string:
                                add_petsc_vec_code = True
                            if "Mat" in eachArgType.type.decl_string:
                                add_petsc_mat_code = True
                            
                            # Workaround for Bug with default arguments and templates.
                            # Assume the template value in the argument is the same as
                            # in the default.
                            if eachArgType.default_value is not None:
                                if "DIM" in eachArgType.default_value:
                                    print "INFO: Found method default arguement with incomplete type. Guessing the type."
                                    if "3" in eachArgType.type.decl_string:
                                        eachArgType.default_value = eachArgType.default_value.replace("DIM", str(3))
                                    if "2" in eachArgType.type.decl_string:
                                        eachArgType.default_value = eachArgType.default_value.replace("DIM", str(2))                                
                
                if has_members:
                    for eachMemberFunction in this_class.member_functions():
                        
                        # Exclude any specified member functions
                        if eachClass.excluded_methods is not None and eachMemberFunction.name in eachClass.excluded_methods:
                            eachMemberFunction.exclude()
                            continue
                        
                        # PETSc Vec and Mat args need special care
                        for eachArgType in eachMemberFunction.arguments:
                            #pprint (vars(eachArgType))
                            if "Vec" in eachArgType.type.decl_string and not "CellVecData" in eachArgType.type.decl_string:
                                add_petsc_vec_code = True
                            if "Mat" in eachArgType.type.decl_string:
                                add_petsc_mat_code = True
                                
                            # Bug with default arguments and templates
                            if eachArgType.default_value is not None:
                                if "DIM" in eachArgType.default_value:
                                    print "INFO: Found method default arguement with incomplete type. Guessing the type."
                                    if "3" in eachArgType.type.decl_string:
                                        eachArgType.default_value = eachArgType.default_value.replace("DIM", str(3))
                                    if "2" in eachArgType.type.decl_string:
                                        eachArgType.default_value = eachArgType.default_value.replace("DIM", str(2)) 
                                
                        # If there are explicit call policies add them
                        break_out = False
                        if eachClass.pointer_return_methods is not None:
                            for eachDefinedPointerPolicy in eachClass.pointer_return_methods:
                                if eachMemberFunction.name == eachDefinedPointerPolicy[0]:
                                    eachMemberFunction.call_policies = call_policies_collection[eachDefinedPointerPolicy[1]]
                                    break_out = True
                                    break
                        if break_out:
                            continue
                        
                        # PETSc Vec and Mat need special care
                        if "Vec" in str(eachMemberFunction.return_type) and not "CellVecData" in str(eachMemberFunction.return_type):
                            eachMemberFunction.call_policies = call_policies_collection["return_opaque_pointer"]
                            petsc_vec_code_will_auto = True
                            continue
                        
                        if "Mat" in str(eachMemberFunction.return_type):
                            eachMemberFunction.call_policies = call_policies_collection["return_opaque_pointer"]
                            petsc_mat_code_will_auto = True
                            continue
                        
                        if declarations.is_pointer(eachMemberFunction.return_type):
                            eachMemberFunction.call_policies = call_policies_collection["reference_existing_object"]
                            continue
                        
                        if declarations.is_reference(eachMemberFunction.return_type):
                            eachMemberFunction.call_policies = call_policies_collection["return_internal_reference"]
                            continue                        
                        
                # Explicitly remove abstract class constructors
#                 if "Abstract" in eachClass.name:
#                     this_class.constructors().exclude()
                        
                # Set up variable excludes
                if eachClass.excluded_variables is not None:
                    for eachVariable in eachClass.excluded_variables:
                        this_class.variables(eachVariable).exclude()                
                        
                # Add declaration code
                if add_petsc_vec_code and not petsc_vec_code_will_auto:
                    print "Petsc Vec found: Adding custom declaration code."
                    this_class.add_declaration_code(petsc_vec_custom_code)
                     
                if add_petsc_mat_code and not petsc_mat_code_will_auto:
                    print "Petsc Mat found: Adding custom declaration code."
                    this_class.add_declaration_code(petsc_mat_custom_code)
                
                if eachClass.declaration_code is not None:
                    for eachLine in eachClass.declaration_code:
                        this_class.add_declaration_code(eachLine)
    
    return builder, classes

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
            
def pypp_template_name_fix(module_file):
    
    # Pyplusplus does not deal with negative template values in names
    lines = []
    replacements = {"__-1_": "__neg1_",
                    "__-2_": "__neg2_",
                    "__-3_": "__neg3_",}
    
    with open(module_file) as infile:
        for line in infile:
            for src, target in replacements.iteritems():
                line = line.replace(src, target)
            lines.append(line)
    with open(module_file, 'w') as outfile:
        for line in lines:
            outfile.write(line)     
            
def do_module(module_name, builder, work_dir, classes):
    
    # Set up the builder with module specifc classes
    builder, classes = add_autowrap_classes_to_builder(builder, module_name, classes)
    
    # If there is a module with some extra wrapper code execute it
    if module_name == "vessel":
        builder, classes = wrapper_utilities.vessel_additions.update_builder(builder, classes)  
        
    if module_name == "utility":
        builder, classes = wrapper_utilities.utility_additions.update_builder(builder, classes)      
        
    # Write the class names to file for building Python docs later on
    f = open(work_dir + '/class_names_for_doc.txt','w')
    for eachClass in classes:
        for eachName in eachClass.get_short_names():
            f.write('.. autoclass:: microvessel_chaste.' + module_name + '.' + eachName + '\n\t:members:\n\n')
    f.close()
    return builder
       
def generate_wrappers(args):
    
    work_dir = args[1]
    header_collection = args[2]
    castxml_binary = args[3]
    includes = args[4:]
    
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
                                                indexing_suite_version=2,)
                                                #cache=file_cache_t(work_dir + "/dynamic/wrappers/castxml_cache.xml"))
    
    messages.disable(messages.W1040) # unexposed declaration
    messages.disable(messages.W1031) # user to expose non public member function
    messages.disable(messages.W1023) # user to define some functions
    messages.disable(messages.W1014) # operator not supported
    messages.disable(messages.W1036) # can't expose immutable member variables
    
    # Don't wrap std library
    builder.global_ns.namespace('std').exclude()
    
    # Strip out Instantiation 'tricks' in the header file
    # todo - the first line is presumably no longer necessary
    builder.free_function("GetPetscMatForWrapper").call_policies = call_policies.return_value_policy(
        call_policies.return_opaque_pointer)
    builder.free_function("GetPetscMatForWrapper").exclude()
    builder.free_function("Instantiation").exclude()
    
    # Load the classes to be wrapped
    with open(work_dir + "/dynamic/wrappers/class_data.p", 'rb') as fp:
        classes = pickle.load(fp)
        
    module_names = ["mesh", "geometry", "cell", "angiogenesis", "pde", "simulation", "vessel",
                    "visualization", "utility"]
    
    # Just for debugging
    ignore_modules = ["geometry", "cell", "angiogenesis", "pde", "simulation", "vessel",
                    "visualization", "utility"]
    #ignore_modules = []
    
    for idx, module_name in enumerate(module_names):
        
        if module_name in ignore_modules:
            continue
        
        print 'Generating Wrapper Code for: ' + module_name + ' Module.'
        
        if "mesh" not in module_name:
            builder.register_module_dependency(work_dir + "/dynamic/wrappers/"+module_names[idx-1])
        
        # Set up the builder for each module
        
        print 'Starting Module: ' + module_name + ' Module.'
        builder = do_module(module_name, builder, work_dir + "/dynamic/wrappers/" + module_name + "/", classes)

        # Make the wrapper code
    #     builder.build_code_creator(module_name="_chaste_project_PyChaste_" + module_name, 
    #                                doc_extractor=doxygen_extractor.doxygen_doc_extractor())
        builder.build_code_creator(module_name="_chaste_project_MicrovesselChaste_" + module_name)
        builder.code_creator.user_defined_directories.append(work_dir)
        builder.code_creator.user_defined_directories.append(work_dir + "/dynamic/wrappers/")
        builder.code_creator.user_defined_directories.append(work_dir + "/dynamic/wrappers/" + module_name + "/")
        builder.code_creator.license = chaste_license
        
        builder.split_module(work_dir+"/dynamic/wrappers/"+module_name)
        
        # Manually strip any undefined call policies we have missed. Strictly there should not be any/many.
#         for file in os.listdir(work_dir + "/dynamic/wrappers/" + module_name + "/"):
#             if file.endswith(".cpp"):
#                 strip_undefined_call_policies(work_dir + "/dynamic/wrappers/" + module_name + "/" + file)
                
        # Manually remove some value traits in std headers (https://mail.python.org/pipermail/cplusplus-sig/2008-April/013105.html)
#         for file in os.listdir(work_dir + "/dynamic/wrappers/" + module_name + "/"):
#             if file.endswith(".cpp"):
#                 strip_value_traits(work_dir + "/dynamic/wrappers/" + module_name + "/" + file)   
        
        # Fix a bug with boost units
        for file in os.listdir(work_dir + "/dynamic/wrappers/" + module_name + "/"):
            if file.endswith(".cpp"):
                boost_units_namespace_fix(work_dir + "/dynamic/wrappers/" + module_name + "/" + file)   
                pypp_template_name_fix(work_dir + "/dynamic/wrappers/" + module_name + "/" + file)
    
if __name__=="__main__":
    generate_wrappers(sys.argv)
