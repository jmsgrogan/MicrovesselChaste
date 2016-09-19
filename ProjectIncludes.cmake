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

######### Collect the include directories for the project. ###################### 
set(ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/population/vessel/properties/)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/utility)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/geometry)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/writers)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/generators)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/population/vessel)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/population/vessel/properties)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/population/vessel/calculators)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/population/vessel/generators)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/population/vessel/readers)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/population/vessel/writers)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/population/cell)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/population/cell/killers)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/population/cell/properties/mutations)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/population/cell/writers)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/population/cell/cycle)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/wrappers)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/angiogenesis)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/angiogenesis/migration_rules)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/angiogenesis/migration_rules/lattice)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/angiogenesis/migration_rules/off_lattice)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/angiogenesis/sprouting_rules)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/angiogenesis/regression)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/flow)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/flow/calculators)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/flow/calculators/haematocrit)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/flow/structural_adaptation)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/ode/)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/pde/solvers)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/pde/problem)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/pde/discrete_sources)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/pde/boundary_conditions)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/mesh/utilities)
list (APPEND ANGIOGENESIS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/mesh)