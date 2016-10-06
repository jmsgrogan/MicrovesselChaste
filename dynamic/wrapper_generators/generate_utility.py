#!/usr/bin/env python

"""
This scipt automatically generates Python bindings using a rule based approach
"""
import sys
from pyplusplus import module_builder
from pyplusplus.module_builder import call_policies
from pygccxml import parser

def update_builder(builder):
    

    include_classes = [ 
                       "ParameterCollection", 
                       "BaseParameterInstance", 
                       "BaseUnits"]
      
    for eachClass in include_classes:
        builder.class_(eachClass).include()

#     helpers = builder.classes(lambda decl: decl.name.startswith('UnitTester'))
#     for eachClass in helpers:
#         eachClass.include()
#        eachClass.rename(class_name)

#     chaste_ns = builder.global_ns.namespace('chaste')
#     helpers = chaste_ns.classes()
#     print module_builder.__file__
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
#    unit_ns = builder.global_ns.namespace( 'unit', recursive=False )
#    pypluplus_alias_ns = builder.global_ns.namespace('pyplusplus')
#    pypluplus_alias_ns = pypluplus_ns.namespace( 'alias', recursive=False )
    
#     helpers = unit_ns.classes(lambda decl: decl.name.startswith('kg'))
#     helpers.include()
    
#     helpers = unit_ns.classes('kg')
#     helpers.include()
    
#     class_name = "kg"
#     helpers = unit_ns.classes(lambda decl: decl.name.startswith(class_name+'_instance_t<true>'))
#     for eachClass in helpers:
#         eachClass.include()
#         eachClass.rename(class_name)
# #    builder.variable("kg").include()
#     
# # #    helpers = unit_ns.typedefs("mass")
# #     for var_gen_typedef in helpers:
# #         var_gen_cls = var_gen_typedef.type.declaration
# # #        var_gen_cls.member_operators( symbol='()' ).create_with_signature = True
# #         var_gen_cls.include()
# # #        var_gen_cls.constructors().exclude()
#     
# #     
#     helpers = unit_ns.typedefs(lambda decl: decl.name.startswith('MassQuantity'))
# #    helpers = unit_ns.typedefs("mass")
#     for var_gen_typedef in helpers:
#         var_gen_cls = var_gen_typedef.type.declaration
#         var_gen_cls.include()
#         var_gen_cls.rename(var_gen_typedef.name)
#         var_gen_cls.constructors().exclude()
#         var_gen_cls.add_registration_code('def(double() * bp::self)')
#         var_gen_cls.add_registration_code('def(bp::self_ns::str(bp::self))')
#         
# #    helpers = unit_ns.typedefs(lambda decl: decl.name.startswith('mass'))
#     helpers = unit_ns.typedefs('mass')
#     for var_gen_typedef in helpers:
# #         print 'found'
#         print var_gen_typedef.name
#         var_gen_cls = var_gen_typedef.type.declaration
# #        var_gen_cls.member_operators( symbol='()' ).create_with_signature = True
#         var_gen_cls.rename(var_gen_typedef.name)
#         var_gen_cls.include()
#         var_gen_cls.add_registration_code('def(double() * bp::self)')
#         var_gen_cls.add_registration_code('def(bp::self_ns::str(bp::self))')
#         var_gen_cls.constructors().exclude()
    
#     helpers = builder.classes(lambda decl: decl.name.startswith('quantity'))
#     helpers.include()
    
#     PressureUnit@::pyplusplus::aliases::PressureUnit
#     builder.print_declarations()
    
#     builder.class_("::boost::units::unit< boost::units::list< boost::units::dim< boost::units::mass_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >").include()
#     builder.variable("kg").include()

#    builder.class_('::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::mass_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double >').add_registration_code('def(float() * bp::self)')
#    builder.class_('::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::mass_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double >').add_registration_code('def(bp::self * float())')
    
    return builder