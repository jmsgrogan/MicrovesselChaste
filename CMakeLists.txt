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

find_package(Chaste COMPONENTS cell_based)

# Microvessel Chaste needs some additional VTK libraries, take them all for now.
find_package(VTK COMPONENTS vtkParallelCore vtkParallelMPI vtkIOMovie vtkRenderingAnnotation vtkRenderingFreeType vtkRenderingFreeTypeOpenGL vtkRenderingCore vtkRenderingOpenGL vtkInteractionStyle REQUIRED)
list(APPEND Chaste_THIRD_PARTY_LIBRARIES ${VTK_LIBRARIES})

chaste_do_project(MicrovesselChaste)
    
set(BUILD_MicrovesselChaste_PYTHON ON CACHE BOOL "Build Python Bindings for MicrovesselChaste component")
if(${BUILD_MicrovesselChaste_PYTHON})
    include(${CMAKE_CURRENT_SOURCE_DIR}/WrapPython.cmake)
endif(${BUILD_MicrovesselChaste_PYTHON})

# Add some custom test targets so we can just run on MicrovesselChaste code
if (Chaste_PROFILE_GPROF OR Chaste_PROFILE_GPERFTOOLS)
    if (Chaste_PROFILE_GPERFTOOLS)
        set(extension gif)
    else()
        set(extension gmon)
    endif()
    
    set(CTEST_COMMAND ctest)
    add_custom_target(profile_project_MicrovesselChaste
        COMMAND ${CTEST_COMMAND} "-L" "project_MicrovesselChaste" "--output-on-failure"
        COMMAND ${PYTHON_EXECUTABLE} "${Chaste_SOURCE_DIR}/cmake/process_profile.py" "${Chaste_PROFILE_OUTPUT_DIR}" ${extension}
        DEPENDS Profile 
        WORKING_DIRECTORY ${Chaste_BINARY_DIR}
        VERBATIM)
endif()
