#!/usr/bin/env python

"""
This scipt automatically generates Python bindings using a rule based approach
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
    
    class_name = "kg"
    helpers = unit_ns.classes(lambda decl: decl.name.startswith(class_name+'_instance_t<true>'))
    for eachClass in helpers:
        eachClass.include()
        eachClass.rename(class_name)
   
    unit_names = ["membrane_permeability",
                  "volumetric_solubility",
                  "solubility",
                  "diffusivity_per_concentration",
                  "diffusivity",
#                 "flow_impedance",
#                 "flow_rate",
#                 "dynamic_viscosity",
#                 "pressure",]
#                 "force",
#                 "velocity",
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
#                "volume",
#                "area",
#                "length",
                "rate",]
#                "time",
#                "plane_angle"]

    for eachUnit in unit_names:
        helpers = unit_ns.typedefs(eachUnit)
        for var_gen_typedef in helpers:
            var_gen_cls = var_gen_typedef.decl_type.declaration
            var_gen_cls.rename(var_gen_typedef.name)
    #        var_gen_cls.member_operators( symbol='()' ).create_with_signature = True
            var_gen_cls.include()
            try:
                var_gen_cls.constructors().exclude()
            except:
                print var_gen_typedef.name
            var_gen_cls.add_registration_code('def(double() * bp::self)')
            var_gen_cls.add_registration_code('def(bp::self_ns::str(bp::self))')
            var_gen_cls.constructors().exclude()  

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
                      ]
    for eachQuantity in quantity_names:
        helpers = pypluplus_alias_ns.typedefs(eachQuantity)
        for var_gen_typedef in helpers:
            var_gen_cls = var_gen_typedef.decl_type.declaration
            var_gen_cls.rename(var_gen_typedef.name)
            var_gen_cls.include()
            var_gen_cls.constructors().exclude()
            var_gen_cls.add_registration_code('def(double() * bp::self)')
            var_gen_cls.add_registration_code('def(bp::self_ns::str(bp::self))')

    return builder