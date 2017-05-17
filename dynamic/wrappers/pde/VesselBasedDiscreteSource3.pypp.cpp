// This file has been generated by Py++.


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

#include "boost/python.hpp"
#include "wrapper_header_collection.hpp"
#include "VesselBasedDiscreteSource3.pypp.hpp"

namespace bp = boost::python;

struct VesselBasedDiscreteSource_less__3__greater__wrapper : VesselBasedDiscreteSource< 3 >, bp::wrapper< VesselBasedDiscreteSource< 3 > > {

    VesselBasedDiscreteSource_less__3__greater__wrapper(VesselBasedDiscreteSource<3> const & arg )
    : VesselBasedDiscreteSource<3>( arg )
      , bp::wrapper< VesselBasedDiscreteSource< 3 > >(){
        // copy constructor
        
    }

    VesselBasedDiscreteSource_less__3__greater__wrapper( )
    : VesselBasedDiscreteSource<3>( )
      , bp::wrapper< VesselBasedDiscreteSource< 3 > >(){
        // null constructor
    
    }

    virtual ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > GetConstantInUValues(  ) {
        if( bp::override func_GetConstantInUValues = this->get_override( "GetConstantInUValues" ) )
            return func_GetConstantInUValues(  );
        else{
            return this->VesselBasedDiscreteSource< 3 >::GetConstantInUValues(  );
        }
    }
    
    ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > default_GetConstantInUValues(  ) {
        return VesselBasedDiscreteSource< 3 >::GetConstantInUValues( );
    }

    virtual ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > GetLinearInUValues(  ) {
        if( bp::override func_GetLinearInUValues = this->get_override( "GetLinearInUValues" ) )
            return func_GetLinearInUValues(  );
        else{
            return this->VesselBasedDiscreteSource< 3 >::GetLinearInUValues(  );
        }
    }
    
    ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > default_GetLinearInUValues(  ) {
        return VesselBasedDiscreteSource< 3 >::GetLinearInUValues( );
    }

    virtual ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > GetNonlinearTermValues(  ) {
        if( bp::override func_GetNonlinearTermValues = this->get_override( "GetNonlinearTermValues" ) )
            return func_GetNonlinearTermValues(  );
        else{
            return this->VesselBasedDiscreteSource< 3 >::GetNonlinearTermValues(  );
        }
    }
    
    ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > default_GetNonlinearTermValues(  ) {
        return VesselBasedDiscreteSource< 3 >::GetNonlinearTermValues( );
    }

    virtual void UpdateDensityMap(  ) {
        if( bp::override func_UpdateDensityMap = this->get_override( "UpdateDensityMap" ) )
            func_UpdateDensityMap(  );
        else{
            this->DiscreteSource< 3 >::UpdateDensityMap(  );
        }
    }
    
    void default_UpdateDensityMap(  ) {
        DiscreteSource< 3 >::UpdateDensityMap( );
    }

};

void register_VesselBasedDiscreteSource3_class(){

    { //::VesselBasedDiscreteSource< 3 >
        typedef bp::class_< VesselBasedDiscreteSource_less__3__greater__wrapper, bp::bases< DiscreteSource< 3 > > > VesselBasedDiscreteSource3_exposer_t;
        VesselBasedDiscreteSource3_exposer_t VesselBasedDiscreteSource3_exposer = VesselBasedDiscreteSource3_exposer_t( "VesselBasedDiscreteSource3", bp::init< >() );
        bp::scope VesselBasedDiscreteSource3_scope( VesselBasedDiscreteSource3_exposer );
        { //::VesselBasedDiscreteSource< 3 >::Create
        
            typedef VesselBasedDiscreteSource< 3 > exported_class_t;
            typedef ::boost::shared_ptr< VesselBasedDiscreteSource< 3 > > ( *Create_function_type )(  );
            
            VesselBasedDiscreteSource3_exposer.def( 
                "Create"
                , Create_function_type( &::VesselBasedDiscreteSource< 3 >::Create ) );
        
        }
        { //::VesselBasedDiscreteSource< 3 >::GetConstantInUValues
        
            typedef VesselBasedDiscreteSource< 3 > exported_class_t;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( exported_class_t::*GetConstantInUValues_function_type)(  ) ;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( VesselBasedDiscreteSource_less__3__greater__wrapper::*default_GetConstantInUValues_function_type)(  ) ;
            
            VesselBasedDiscreteSource3_exposer.def( 
                "GetConstantInUValues"
                , GetConstantInUValues_function_type(&::VesselBasedDiscreteSource< 3 >::GetConstantInUValues)
                , default_GetConstantInUValues_function_type(&VesselBasedDiscreteSource_less__3__greater__wrapper::default_GetConstantInUValues) );
        
        }
        { //::VesselBasedDiscreteSource< 3 >::GetLinearInUValues
        
            typedef VesselBasedDiscreteSource< 3 > exported_class_t;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( exported_class_t::*GetLinearInUValues_function_type)(  ) ;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( VesselBasedDiscreteSource_less__3__greater__wrapper::*default_GetLinearInUValues_function_type)(  ) ;
            
            VesselBasedDiscreteSource3_exposer.def( 
                "GetLinearInUValues"
                , GetLinearInUValues_function_type(&::VesselBasedDiscreteSource< 3 >::GetLinearInUValues)
                , default_GetLinearInUValues_function_type(&VesselBasedDiscreteSource_less__3__greater__wrapper::default_GetLinearInUValues) );
        
        }
        { //::VesselBasedDiscreteSource< 3 >::GetNonlinearTermValues
        
            typedef VesselBasedDiscreteSource< 3 > exported_class_t;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( exported_class_t::*GetNonlinearTermValues_function_type)(  ) ;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( VesselBasedDiscreteSource_less__3__greater__wrapper::*default_GetNonlinearTermValues_function_type)(  ) ;
            
            VesselBasedDiscreteSource3_exposer.def( 
                "GetNonlinearTermValues"
                , GetNonlinearTermValues_function_type(&::VesselBasedDiscreteSource< 3 >::GetNonlinearTermValues)
                , default_GetNonlinearTermValues_function_type(&VesselBasedDiscreteSource_less__3__greater__wrapper::default_GetNonlinearTermValues) );
        
        }
        { //::VesselBasedDiscreteSource< 3 >::SetHalfMaxUptakeConcentration
        
            typedef VesselBasedDiscreteSource< 3 > exported_class_t;
            typedef void ( exported_class_t::*SetHalfMaxUptakeConcentration_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            VesselBasedDiscreteSource3_exposer.def( 
                "SetHalfMaxUptakeConcentration"
                , SetHalfMaxUptakeConcentration_function_type( &::VesselBasedDiscreteSource< 3 >::SetHalfMaxUptakeConcentration )
                , ( bp::arg("value") ) );
        
        }
        { //::VesselBasedDiscreteSource< 3 >::SetNumberOfCellsPerLength
        
            typedef VesselBasedDiscreteSource< 3 > exported_class_t;
            typedef void ( exported_class_t::*SetNumberOfCellsPerLength_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            VesselBasedDiscreteSource3_exposer.def( 
                "SetNumberOfCellsPerLength"
                , SetNumberOfCellsPerLength_function_type( &::VesselBasedDiscreteSource< 3 >::SetNumberOfCellsPerLength )
                , ( bp::arg("cellsPerLength") ) );
        
        }
        { //::VesselBasedDiscreteSource< 3 >::SetReferenceConcentration
        
            typedef VesselBasedDiscreteSource< 3 > exported_class_t;
            typedef void ( exported_class_t::*SetReferenceConcentration_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            VesselBasedDiscreteSource3_exposer.def( 
                "SetReferenceConcentration"
                , SetReferenceConcentration_function_type( &::VesselBasedDiscreteSource< 3 >::SetReferenceConcentration )
                , ( bp::arg("value") ) );
        
        }
        { //::VesselBasedDiscreteSource< 3 >::SetReferenceHaematocrit
        
            typedef VesselBasedDiscreteSource< 3 > exported_class_t;
            typedef void ( exported_class_t::*SetReferenceHaematocrit_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::dimensionless_type, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            VesselBasedDiscreteSource3_exposer.def( 
                "SetReferenceHaematocrit"
                , SetReferenceHaematocrit_function_type( &::VesselBasedDiscreteSource< 3 >::SetReferenceHaematocrit )
                , ( bp::arg("value") ) );
        
        }
        { //::VesselBasedDiscreteSource< 3 >::SetUptakeRatePerCell
        
            typedef VesselBasedDiscreteSource< 3 > exported_class_t;
            typedef void ( exported_class_t::*SetUptakeRatePerCell_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            VesselBasedDiscreteSource3_exposer.def( 
                "SetUptakeRatePerCell"
                , SetUptakeRatePerCell_function_type( &::VesselBasedDiscreteSource< 3 >::SetUptakeRatePerCell )
                , ( bp::arg("ratePerCell") ) );
        
        }
        { //::VesselBasedDiscreteSource< 3 >::SetVesselPermeability
        
            typedef VesselBasedDiscreteSource< 3 > exported_class_t;
            typedef void ( exported_class_t::*SetVesselPermeability_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            VesselBasedDiscreteSource3_exposer.def( 
                "SetVesselPermeability"
                , SetVesselPermeability_function_type( &::VesselBasedDiscreteSource< 3 >::SetVesselPermeability )
                , ( bp::arg("value") ) );
        
        }
        { //::DiscreteSource< 3 >::UpdateDensityMap
        
            typedef VesselBasedDiscreteSource< 3 > exported_class_t;
            typedef void ( exported_class_t::*UpdateDensityMap_function_type)(  ) ;
            typedef void ( VesselBasedDiscreteSource_less__3__greater__wrapper::*default_UpdateDensityMap_function_type)(  ) ;
            
            VesselBasedDiscreteSource3_exposer.def( 
                "UpdateDensityMap"
                , UpdateDensityMap_function_type(&::DiscreteSource< 3 >::UpdateDensityMap)
                , default_UpdateDensityMap_function_type(&VesselBasedDiscreteSource_less__3__greater__wrapper::default_UpdateDensityMap) );
        
        }
        VesselBasedDiscreteSource3_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< VesselBasedDiscreteSource<3> > >();
        bp::implicitly_convertible< boost::shared_ptr< VesselBasedDiscreteSource< 3 > >, boost::shared_ptr< DiscreteSource< 3 > > >();
    }

}
