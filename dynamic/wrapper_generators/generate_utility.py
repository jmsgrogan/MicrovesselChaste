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
from pygccxml import parser

def update_builder(builder):
    

#     include_classes = [ 
#                        "ParameterCollection",] 
#                        #"BaseParameterInstance", 
#                        #"BaseUnits"]
#       
#     for eachClass in include_classes:
#         builder.class_(eachClass).include()

#     chaste_ns = builder.global_ns.namespace('chaste')
#     helpers = chaste_ns.classes()
#     helpers = builder.classes(lambda decl: decl.name.startswith('ParameterInstance'))
#     for eachClass in helpers:
#         eachClass.include()
#         eachClass.rename("SomethingElse")
#         print eachClass.alias

#    builder.variable("kg").include()
#     helpers = builder.classes(lambda decl: decl.name.startswith('ParameterInstance'))
#     for var_gen_typedef in helpers:
#         print "in"
# #        var_gen_cls = var_gen_typedef.type.declaration
#         var_gen_cls = var_gen_typedef
# #        var_gen_cls.rename(var_gen_typedef.name)
#         var_gen_cls.include()
    
#     pypluplus_alias_ns = builder.global_ns.namespace('pyplusplus')
#     helpers = pypluplus_alias_ns.typedefs(lambda decl: decl.name.startswith('ParameterInstance'))
#     for var_gen_typedef in helpers:
#         print var_gen_typedef.name
# #         print var_gen_typedef.type
#         eachclass = builder.class_(str(var_gen_typedef.type))
#         eachclass.include()
#         eachclass.rename(str(var_gen_typedef.name))
#        var_gen_cls = var_gen_typedef.type.declaration
#         var_gen_cls = var_gen_typedef
#        var_gen_cls.rename(var_gen_typedef.name)
#         var_gen_cls.include()
    # #        var_gen_cls.member_operators( symbol='()' ).create_with_signature = True

    # #        var_gen_cls.constructors().exclude()

    #builder.class_("kg_instance_t<true>").include()
    #builder.class_("metre_cubed_per_second_instance_t< true >").include()
#    builder.class_(dimensionless_unit).include()
#    builder.class_("DimensionlessUnit").include()
    
    # can we find all the boost units classes 
    # they live in the unit namespace
    unit_ns = builder.global_ns.namespace('unit', recursive=False)
    pypluplus_alias_ns = builder.global_ns.namespace('pyplusplus')
   
    unit_names = ["membrane_permeability",
                  "volumetric_solubility",
                  "solubility",
                  "diffusivity_per_concentration",
                  "diffusivity",
                  "flow_impedance",
                 "flow_rate",
                 "dynamic_viscosity",
                "pressure",
                "force",
                "velocity",
                "number_density",
                "molar_mass",
                "rate_per_concentration",
                "concentration_gradient",
                "concentration_flux",
                "concentration",
                "molar_flux",
                "molar_flow_rate",
                "mass", 
                "per_area",
                "per_length",
                "volume",
                "area",
                "length",
                "rate",
                "time",
                "dimensionless"]
#                "plane_angle"]
    
    units = {"membrane_permeability": "metre_per_second", 
             "volumetric_solubility": "per_pascal",
             "solubility": "mole_per_metre_cubed_per_second",
             "diffusivity_per_concentration": "metre_pow5_per_second_per_mole",
             "diffusivity": "metre_squared_per_second",
             "flow_impedance": "pascal_second_per_metre_cubed",
             "flow_rate": "metre_cubed_per_second",
             "dynamic_viscosity": "poiseuille",
             "pressure": "pascal",
             "force": "newton",
             "velocity": "metre_per_second",
             "number_density": "per_metre_cubed",
             "molar_mass": "mole_per_kg",
             "rate_per_concentration": "metre_cubed_per_mole_per_second",
             "concentration_gradient": "mole_per_metre_pow4",
             "concentration_flux": "mole_per_metre_pow5_per_second",
             "concentration": "mole_per_metre_cubed",
             "molar_flux": "mole_per_metre_squared_per_second",
             "molar_flow_rate": "mole_per_second",
             "mass": "kg",
             "per_area": "per_metre_squared",
             "per_length": "per_metre",
             "volume": "metre_cubed",
             "area": "metre_squared",
             "length": "metre",
             "rate":"per_second",
             "time":"second", 
             "dimensionless" : "dimensionless",       
             }

    for eachUnit in unit_names:
        helpers = unit_ns.typedefs(eachUnit)
        for var_gen_typedef in helpers:
            var_gen_cls = var_gen_typedef.type.declaration
            var_gen_cls.rename(units[var_gen_typedef.name])
            var_gen_cls.include()
            var_gen_cls.constructors().exclude()
            var_gen_cls.add_registration_code('def(double() * bp::self)')
            var_gen_cls.add_registration_code('def(bp::self_ns::str(bp::self))')

    quantity_names = ["MembranePermeabilityQuantity",
                      "VolumetricSolubilityQuantity",
                      "SolubilityQuantity",
                      "DiffusivityPerConcentrationQuantity",
                      "DiffusivityQuantity",
                      "FlowRateQuantity",
                      "ViscosityQuantity",
                      "PressureQuantity",
                      "ForceQuantity",
                      "VelocityQuantity",
                      "NumberDensityQuantity",
                      "MolarMassQuantity",
                      "RatePerConcentrationQuantity",
                      "ConcentrationGradientQuantity",
                      "ConcentrationFluxQuantity",
                      "ConcentrationQuantity",
                      "MolarFluxQuantity",
                      "MolarFlowRateQuantity",
                      "AmountQuantity",
                      "MassFluxQuantity",
                      "MassFlowRateQuantity",
                      "MassQuantity",
                      "PerAreaQuantity",
                      "PerLengthQuantity",
                      "VolumeQuantity",
                      "AreaQuantity",
                      "LengthQuantity",
                      "RateQuantity",
                      "TimeQuantity",
                      "AngleQuantity",
                      "DimensionlessQuantity"
                      ]
    
    for eachQuantity in quantity_names:
        helpers = pypluplus_alias_ns.typedefs(eachQuantity)
        for var_gen_typedef in helpers:
            var_gen_cls = var_gen_typedef.type.declaration
            var_gen_cls.rename(var_gen_typedef.name)
            var_gen_cls.include()
            var_gen_cls.constructors().exclude()
            var_gen_cls.add_registration_code('def(double() * bp::self)')
            var_gen_cls.add_registration_code('def(bp::self * double())')
            var_gen_cls.add_registration_code('def(bp::self / bp::self)')
            var_gen_cls.add_registration_code('def(bp::self / double())')
            var_gen_cls.add_registration_code('def(double() / bp::self)')
            var_gen_cls.add_registration_code('def(bp::self_ns::str(bp::self))')

    return builder