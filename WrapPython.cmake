# Copyright (c) 2005-2017, University of Oxford.
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
# Add any cmake modules defined in this project
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
include_directories(${Chaste_INCLUDE_DIRS} ${Chaste_THIRD_PARTY_INCLUDE_DIRS})

# Any non-wrapper code (code in the src folder) in this project needs to be put in its own shared library. 
if(APPLE)
    set(PROJECT_MicrovesselChaste_LIB ${CMAKE_CURRENT_BINARY_DIR}/libchaste_project_MicrovesselChaste.dylib)
elseif(WIN32)
    set(PROJECT_MicrovesselChaste_LIB ${CMAKE_CURRENT_BINARY_DIR}/libchaste_project_MicrovesselChaste.dll)  
else()
    set(PROJECT_MicrovesselChaste_LIB ${CMAKE_CURRENT_BINARY_DIR}/libchaste_project_MicrovesselChaste.so)  
endif()

# These packages are needed for binding generation
#find_python_module(pyplusplus 1.6.0)
#find_python_module(pygccxml 1.7.2)
#find_package(castxml)
# Numpy is needed for wrapping
find_package(NumPy)
set(CASTXML_EXE_LOC "/usr/bin/castxml" CACHE FILEPATH "Path to the castxml executable.")

# Collect the header directories for this project
include(${CMAKE_CURRENT_SOURCE_DIR}/ProjectIncludes.cmake)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/dynamic/  ${PYTHON_NUMPY_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/dynamic/wrappers)
include_directories(${CMAKE_SOURCE_DIR}/projects/PyChaste/src)

set (CMAKE_CXX_STANDARD 11)
if(CMAKE_COMPILER_IS_GNUCXX)
        # https://svn.boost.org/trac/boost/ticket/9240
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fext-numeric-literals")
endif()
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/dynamic/pybind11/include)

set(PYBIND11_PYTHON_VERSION 2.7)
set(PYBIND11_CPP_STANDARD -std=c++11)
#add_subdirectory(dynamic/pybind11)
include_directories(${PYTHON_INCLUDE_DIRS})

######### Build the Python modules ###################### 
set (MicrovesselChaste_AUTO_MODULES "")
set (MicrovesselChaste_PYTHON_MODULES "")
set (MicrovesselChaste_PYTHON_MODULE_LOCATIONS "")

# Auto wrapper
list (APPEND MicrovesselChaste_AUTO_MODULES utility)
list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel_chaste/utility)
#list (APPEND MicrovesselChaste_AUTO_MODULES geometry)
#list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel_chaste/geometry)
#list (APPEND MicrovesselChaste_AUTO_MODULES mesh)
#list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel_chaste/mesh)
#list (APPEND MicrovesselChaste_AUTO_MODULES vessel)
#list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel_chaste/population/vessel/)
#list (APPEND MicrovesselChaste_AUTO_MODULES pde)
#list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel_chaste/pde/)
#list (APPEND MicrovesselChaste_AUTO_MODULES flow)
#list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel_chaste/simulation/)
#list (APPEND MicrovesselChaste_AUTO_MODULES simulation)
#list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel_chaste/simulation/)
#list (APPEND MicrovesselChaste_AUTO_MODULES angiogenesis)
#list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel_chaste/simulation/)
#list (APPEND MicrovesselChaste_AUTO_MODULES cell)
#list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel_chaste/population/cell/)
#list (APPEND MicrovesselChaste_AUTO_MODULES visualization)
#list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel_chaste/visualization)
#list (APPEND MicrovesselChaste_AUTO_MODULES image)
#list (APPEND MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${CMAKE_CURRENT_BINARY_DIR}/python/microvessel_chaste/image)

list (APPEND MicrovesselChaste_PYTHON_MODULES ${MicrovesselChaste_AUTO_MODULES})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/src/python/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/python/ PATTERN "*.so" EXCLUDE)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test/python/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/python/test/)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/doc/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/python/doc/)

# Loop through each module that uses auto wrapper code generation and make the wrapper code
add_custom_target(project_MicrovesselChaste_Python_Bindings)

list(LENGTH MicrovesselChaste_AUTO_MODULES len1_auto)
math(EXPR len2_auto "${len1_auto} - 1")
foreach(val RANGE ${len2_auto})
    list(GET MicrovesselChaste_AUTO_MODULES ${val} python_module)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/dynamic/wrappers/${python_module})
endforeach()

SET(arguments ${CMAKE_SOURCE_DIR}/)
LIST(APPEND arguments ${CMAKE_CURRENT_SOURCE_DIR}/dynamic/wrappers/)
LIST(APPEND arguments ${CMAKE_CURRENT_SOURCE_DIR}/dynamic/wrapper_generators/package_info.yaml)
LIST(APPEND arguments ${CASTXML_EXE_LOC})
LIST(APPEND arguments ${MicrovesselChaste_INCLUDE_DIRS})
LIST(APPEND arguments ${Chaste_INCLUDE_DIRS})
LIST(APPEND arguments ${Chaste_THIRD_PARTY_INCLUDE_DIRS})
add_custom_command(TARGET project_MicrovesselChaste_Python_Bindings COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/dynamic/wrapper_generators/generate.py ${arguments})

# Build the modules
list(LENGTH MicrovesselChaste_PYTHON_MODULES len1)
math(EXPR len2 "${len1} - 1")
foreach(val RANGE ${len2})
    list(GET MicrovesselChaste_PYTHON_MODULES ${val} python_module)
    list(GET MicrovesselChaste_PYTHON_MODULE_LOCATIONS ${val} python_module_location)
    
    file(GLOB MODULE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/dynamic/wrappers/${python_module}/*.cpp)
    
    # each module is in the 'dynamic' directory. The library name must be the same as that defined in the cpp file. It is customary
    # to start the name with an underscore. The usual 'lib' prefix is disabled.
    add_library(_chaste_project_MicrovesselChaste_${python_module} SHARED ${MODULE_SOURCES})
    set_target_properties(_chaste_project_MicrovesselChaste_${python_module} PROPERTIES PREFIX ""  SUFFIX ".so"
    LIBRARY_OUTPUT_DIRECTORY ${python_module_location})
    target_compile_features(_chaste_project_MicrovesselChaste_${python_module} PRIVATE cxx_range_for)

    # order is important, boost python and python come first
    target_link_libraries(_chaste_project_MicrovesselChaste_${python_module} pybind11::module ${PYTHON_LIBRARIES} ${Chaste_THIRD_PARTY_LIBRARIES} ${Chaste_LIBRARIES} ${PROJECT_MicrovesselChaste_LIB})
    add_dependencies(_chaste_project_MicrovesselChaste_${python_module} chaste_project_MicrovesselChaste)
endforeach()
#
# Add a target so all the libraries are built with a single command
add_custom_target(project_MicrovesselChaste_Python)
foreach(val RANGE ${len2})
    list(GET MicrovesselChaste_PYTHON_MODULES ${val} python_module)
    add_dependencies(project_MicrovesselChaste_Python _chaste_project_MicrovesselChaste_${python_module})
endforeach()
