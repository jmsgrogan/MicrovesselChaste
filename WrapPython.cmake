# Copyright (c) 2005-2016, University of Oxford.
# All rights reserved.
# 
# University of Oxford means the Chancellor, Masters and Scholars of the
# University of Oxford, having an administrative office at Wellington
# Square, Oxford OX1 2JD, UK.
# 
# This file is part of Chaste.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the University of Oxford nor the names of its
#    contributors may be used to endorse or promote products derived from this
#    software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
add_compile_options(-Wno-unused-local-typedefs)
# Build the Python bindings
add_definitions(-DCHASTE_MicrovesselChaste_PYTHON)
find_package(Boost COMPONENTS python REQUIRED)
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# Find the Chaste and third party dependency header files.
include_directories(${Chaste_INCLUDE_DIRS} ${Chaste_THIRD_PARTY_INCLUDE_DIRS})

# Any non-wrapper code (code in the src folder) in this project needs to be put in its own shared library. 
set(PROJECT_MicrovesselChaste_LIB ${CMAKE_CURRENT_BINARY_DIR}/libchaste_project_MicrovesselChaste.so)

# These packages are needed for binding generation
#find_python_module(pyplusplus 1.6.0)
#find_python_module(pygccxml 1.7.2)
#find_package(castxml)
set(CASTXML_EXE_LOC "/usr/bin/castxml" CACHE FILEPATH "Path to the castxml executable.")

# Collect the header directories for this project
include(${CMAKE_CURRENT_SOURCE_DIR}/ProjectIncludes.cmake)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/dynamic/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/dynamic/wrapper_headers)
include_directories(${CMAKE_SOURCE_DIR}/projects/PyChaste/src)

######### Build the Python modules ###################### 
set (MicrovesselChaste_AUTO_MODULES "")
set (MicrovesselChaste_PYTHON_MODULES "")
set (MicrovesselChaste_PYTHON_MODULE_LOCATIONS "")

# Auto wrapper
#list (APPEND MicrovesselChaste_AUTO_MODULES utility)
#list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel/utility)
#list (APPEND MicrovesselChaste_AUTO_MODULES geometry)
#list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel/geometry)
#list (APPEND MicrovesselChaste_AUTO_MODULES mesh)
#list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel/mesh)
#list (APPEND MicrovesselChaste_AUTO_MODULES vessel)
#list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel/population/vessel/)
#list (APPEND MicrovesselChaste_AUTO_MODULES pde)
#list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel/pde/)
#list (APPEND MicrovesselChaste_AUTO_MODULES flow)
#list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel/simulation/)
#list (APPEND MicrovesselChaste_AUTO_MODULES simulation)
#list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel/simulation/)
#list (APPEND MicrovesselChaste_AUTO_MODULES angiogenesis)
#list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel/simulation/)
list (APPEND MicrovesselChaste_AUTO_MODULES cell)
list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel/population/cell/)

list (APPEND MicrovesselChaste_PYTHON_MODULES ${MicrovesselChaste_AUTO_MODULES})
list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel/)
list (APPEND MicrovesselChaste_PYTHON_MODULES preload)
list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel/)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/src/python/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/python/ PATTERN "*.so" EXCLUDE)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test/python/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/python/test/)

# Loop through each module that uses auto wrapper code generation and make the wrapper code
add_custom_target(project_MicrovesselChaste_Python_Bindings)
list(LENGTH MicrovesselChaste_AUTO_MODULES len1_auto)
math(EXPR len2_auto "${len1_auto} - 1")
foreach(val RANGE ${len2_auto})
    list(GET MicrovesselChaste_AUTO_MODULES ${val} python_module)
    SET(arguments ${python_module})
    LIST(APPEND arguments ${CMAKE_CURRENT_SOURCE_DIR})
    LIST(APPEND arguments ${CMAKE_CURRENT_SOURCE_DIR}/dynamic/wrapper_headers/${python_module}_headers.hpp)
    LIST(APPEND arguments ${CASTXML_EXE_LOC})
    LIST(APPEND arguments ${MicrovesselChaste_INCLUDE_DIRS})
    LIST(APPEND arguments ${Chaste_INCLUDE_DIRS})
    LIST(APPEND arguments ${Chaste_THIRD_PARTY_INCLUDE_DIRS})
    add_custom_command(TARGET project_MicrovesselChaste_Python_Bindings COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/dynamic/wrapper_generators/generate_bindings.py ${arguments})
endforeach()

# Build the modules
list(LENGTH MicrovesselChaste_PYTHON_MODULES len1)
math(EXPR len2 "${len1} - 1")
foreach(val RANGE ${len2})
    list(GET MicrovesselChaste_PYTHON_MODULES ${val} python_module)
    list(GET MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${val} python_module_location)
    add_library(_chaste_project_MicrovesselChaste_${python_module} SHARED ${CMAKE_CURRENT_SOURCE_DIR}/dynamic/${python_module}.cpp)
    set_target_properties(_chaste_project_MicrovesselChaste_${python_module} PROPERTIES PREFIX "" LIBRARY_OUTPUT_DIRECTORY ${python_module_location})
    target_link_libraries(_chaste_project_MicrovesselChaste_${python_module} boost_python ${PYTHON_LIBRARIES} ${Chaste_THIRD_PARTY_LIBRARIES} ${Chaste_LIBRARIES} ${PROJECT_MicrovesselChaste_LIB})
endforeach()

# Add a target so all the libraries are built with a single command
add_custom_target(project_MicrovesselChaste_Python)
foreach(val RANGE ${len2})
    list(GET MicrovesselChaste_PYTHON_MODULES ${val} python_module)
    add_dependencies(project_MicrovesselChaste_Python _chaste_project_MicrovesselChaste_${python_module})
endforeach()
