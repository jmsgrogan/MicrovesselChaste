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
#include "CellStateDependentDiscreteSource2.pypp.hpp"

namespace bp = boost::python;

struct CellStateDependentDiscreteSource_less__2__greater__wrapper : CellStateDependentDiscreteSource< 2 >, bp::wrapper< CellStateDependentDiscreteSource< 2 > > {

    CellStateDependentDiscreteSource_less__2__greater__wrapper(CellStateDependentDiscreteSource<2> const & arg )
    : CellStateDependentDiscreteSource<2>( arg )
      , bp::wrapper< CellStateDependentDiscreteSource< 2 > >(){
        // copy constructor
        
    }

    CellStateDependentDiscreteSource_less__2__greater__wrapper( )
    : CellStateDependentDiscreteSource<2>( )
      , bp::wrapper< CellStateDependentDiscreteSource< 2 > >(){
        // null constructor
    
    }

    virtual ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > GetConstantInUValues(  ) {
        if( bp::override func_GetConstantInUValues = this->get_override( "GetConstantInUValues" ) )
            return func_GetConstantInUValues(  );
        else{
            return this->CellStateDependentDiscreteSource< 2 >::GetConstantInUValues(  );
        }
    }
    
    ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > default_GetConstantInUValues(  ) {
        return CellStateDependentDiscreteSource< 2 >::GetConstantInUValues( );
    }

    virtual ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > GetLinearInUValues(  ) {
        if( bp::override func_GetLinearInUValues = this->get_override( "GetLinearInUValues" ) )
            return func_GetLinearInUValues(  );
        else{
            return this->CellStateDependentDiscreteSource< 2 >::GetLinearInUValues(  );
        }
    }
    
    ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > default_GetLinearInUValues(  ) {
        return CellStateDependentDiscreteSource< 2 >::GetLinearInUValues( );
    }

    virtual ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > GetNonlinearTermValues(  ) {
        if( bp::override func_GetNonlinearTermValues = this->get_override( "GetNonlinearTermValues" ) )
            return func_GetNonlinearTermValues(  );
        else{
            return this->DiscreteSource< 2 >::GetNonlinearTermValues(  );
        }
    }
    
    ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > default_GetNonlinearTermValues(  ) {
        return DiscreteSource< 2 >::GetNonlinearTermValues( );
    }

    virtual void UpdateDensityMap(  ) {
        if( bp::override func_UpdateDensityMap = this->get_override( "UpdateDensityMap" ) )
            func_UpdateDensityMap(  );
        else{
            this->DiscreteSource< 2 >::UpdateDensityMap(  );
        }
    }
    
    void default_UpdateDensityMap(  ) {
        DiscreteSource< 2 >::UpdateDensityMap( );
    }

};

void register_CellStateDependentDiscreteSource2_class(){

    { //::CellStateDependentDiscreteSource< 2 >
        typedef bp::class_< CellStateDependentDiscreteSource_less__2__greater__wrapper, bp::bases< DiscreteSource< 2 > > > CellStateDependentDiscreteSource2_exposer_t;
        CellStateDependentDiscreteSource2_exposer_t CellStateDependentDiscreteSource2_exposer = CellStateDependentDiscreteSource2_exposer_t( "CellStateDependentDiscreteSource2", bp::init< >() );
        bp::scope CellStateDependentDiscreteSource2_scope( CellStateDependentDiscreteSource2_exposer );
        { //::CellStateDependentDiscreteSource< 2 >::Create
        
            typedef CellStateDependentDiscreteSource< 2 > exported_class_t;
            typedef ::boost::shared_ptr< CellStateDependentDiscreteSource< 2 > > ( *Create_function_type )(  );
            
            CellStateDependentDiscreteSource2_exposer.def( 
                "Create"
                , Create_function_type( &::CellStateDependentDiscreteSource< 2 >::Create ) );
        
        }
        { //::CellStateDependentDiscreteSource< 2 >::GetConstantInUValues
        
            typedef CellStateDependentDiscreteSource< 2 > exported_class_t;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( exported_class_t::*GetConstantInUValues_function_type)(  ) ;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( CellStateDependentDiscreteSource_less__2__greater__wrapper::*default_GetConstantInUValues_function_type)(  ) ;
            
            CellStateDependentDiscreteSource2_exposer.def( 
                "GetConstantInUValues"
                , GetConstantInUValues_function_type(&::CellStateDependentDiscreteSource< 2 >::GetConstantInUValues)
                , default_GetConstantInUValues_function_type(&CellStateDependentDiscreteSource_less__2__greater__wrapper::default_GetConstantInUValues) );
        
        }
        { //::CellStateDependentDiscreteSource< 2 >::GetLinearInUValues
        
            typedef CellStateDependentDiscreteSource< 2 > exported_class_t;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( exported_class_t::*GetLinearInUValues_function_type)(  ) ;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( CellStateDependentDiscreteSource_less__2__greater__wrapper::*default_GetLinearInUValues_function_type)(  ) ;
            
            CellStateDependentDiscreteSource2_exposer.def( 
                "GetLinearInUValues"
                , GetLinearInUValues_function_type(&::CellStateDependentDiscreteSource< 2 >::GetLinearInUValues)
                , default_GetLinearInUValues_function_type(&CellStateDependentDiscreteSource_less__2__greater__wrapper::default_GetLinearInUValues) );
        
        }
        { //::CellStateDependentDiscreteSource< 2 >::SetStateRateMap
        
            typedef CellStateDependentDiscreteSource< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetStateRateMap_function_type)( ::std::map< unsigned int, boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ) ;
            
            CellStateDependentDiscreteSource2_exposer.def( 
                "SetStateRateMap"
                , SetStateRateMap_function_type( &::CellStateDependentDiscreteSource< 2 >::SetStateRateMap )
                , ( bp::arg("stateRateMap") ) );
        
        }
        { //::CellStateDependentDiscreteSource< 2 >::SetStateRateThresholdMap
        
            typedef CellStateDependentDiscreteSource< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetStateRateThresholdMap_function_type)( ::std::map< unsigned int, boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ) ;
            
            CellStateDependentDiscreteSource2_exposer.def( 
                "SetStateRateThresholdMap"
                , SetStateRateThresholdMap_function_type( &::CellStateDependentDiscreteSource< 2 >::SetStateRateThresholdMap )
                , ( bp::arg("stateThresholdMap") ) );
        
        }
        { //::DiscreteSource< 2 >::GetNonlinearTermValues
        
            typedef CellStateDependentDiscreteSource< 2 > exported_class_t;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( exported_class_t::*GetNonlinearTermValues_function_type)(  ) ;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( CellStateDependentDiscreteSource_less__2__greater__wrapper::*default_GetNonlinearTermValues_function_type)(  ) ;
            
            CellStateDependentDiscreteSource2_exposer.def( 
                "GetNonlinearTermValues"
                , GetNonlinearTermValues_function_type(&::DiscreteSource< 2 >::GetNonlinearTermValues)
                , default_GetNonlinearTermValues_function_type(&CellStateDependentDiscreteSource_less__2__greater__wrapper::default_GetNonlinearTermValues) );
        
        }
        { //::DiscreteSource< 2 >::UpdateDensityMap
        
            typedef CellStateDependentDiscreteSource< 2 > exported_class_t;
            typedef void ( exported_class_t::*UpdateDensityMap_function_type)(  ) ;
            typedef void ( CellStateDependentDiscreteSource_less__2__greater__wrapper::*default_UpdateDensityMap_function_type)(  ) ;
            
            CellStateDependentDiscreteSource2_exposer.def( 
                "UpdateDensityMap"
                , UpdateDensityMap_function_type(&::DiscreteSource< 2 >::UpdateDensityMap)
                , default_UpdateDensityMap_function_type(&CellStateDependentDiscreteSource_less__2__greater__wrapper::default_UpdateDensityMap) );
        
        }
        CellStateDependentDiscreteSource2_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< CellStateDependentDiscreteSource<2> > >();
        bp::implicitly_convertible< boost::shared_ptr< CellStateDependentDiscreteSource< 2 > >, boost::shared_ptr< DiscreteSource< 2 > > >();
    }

}