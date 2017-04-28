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
#include "SolutionDependentDiscreteSource3.pypp.hpp"

namespace bp = boost::python;

struct SolutionDependentDiscreteSource_less__3__greater__wrapper : SolutionDependentDiscreteSource< 3 >, bp::wrapper< SolutionDependentDiscreteSource< 3 > > {

    SolutionDependentDiscreteSource_less__3__greater__wrapper(SolutionDependentDiscreteSource<3> const & arg )
    : SolutionDependentDiscreteSource<3>( arg )
      , bp::wrapper< SolutionDependentDiscreteSource< 3 > >(){
        // copy constructor
        
    }

    SolutionDependentDiscreteSource_less__3__greater__wrapper( )
    : SolutionDependentDiscreteSource<3>( )
      , bp::wrapper< SolutionDependentDiscreteSource< 3 > >(){
        // null constructor
    
    }

    virtual ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > GetConstantInUValues(  ) {
        if( bp::override func_GetConstantInUValues = this->get_override( "GetConstantInUValues" ) )
            return func_GetConstantInUValues(  );
        else{
            return this->SolutionDependentDiscreteSource< 3 >::GetConstantInUValues(  );
        }
    }
    
    ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > default_GetConstantInUValues(  ) {
        return SolutionDependentDiscreteSource< 3 >::GetConstantInUValues( );
    }

    virtual ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > GetLinearInUValues(  ) {
        if( bp::override func_GetLinearInUValues = this->get_override( "GetLinearInUValues" ) )
            return func_GetLinearInUValues(  );
        else{
            return this->SolutionDependentDiscreteSource< 3 >::GetLinearInUValues(  );
        }
    }
    
    ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > default_GetLinearInUValues(  ) {
        return SolutionDependentDiscreteSource< 3 >::GetLinearInUValues( );
    }

    virtual ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > GetNonlinearTermValues(  ) {
        if( bp::override func_GetNonlinearTermValues = this->get_override( "GetNonlinearTermValues" ) )
            return func_GetNonlinearTermValues(  );
        else{
            return this->DiscreteSource< 3 >::GetNonlinearTermValues(  );
        }
    }
    
    ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > default_GetNonlinearTermValues(  ) {
        return DiscreteSource< 3 >::GetNonlinearTermValues( );
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

void register_SolutionDependentDiscreteSource3_class(){

    { //::SolutionDependentDiscreteSource< 3 >
        typedef bp::class_< SolutionDependentDiscreteSource_less__3__greater__wrapper, bp::bases< DiscreteSource< 3 > > > SolutionDependentDiscreteSource3_exposer_t;
        SolutionDependentDiscreteSource3_exposer_t SolutionDependentDiscreteSource3_exposer = SolutionDependentDiscreteSource3_exposer_t( "SolutionDependentDiscreteSource3", bp::init< >() );
        bp::scope SolutionDependentDiscreteSource3_scope( SolutionDependentDiscreteSource3_exposer );
        { //::SolutionDependentDiscreteSource< 3 >::Create
        
            typedef SolutionDependentDiscreteSource< 3 > exported_class_t;
            typedef ::boost::shared_ptr< SolutionDependentDiscreteSource< 3 > > ( *Create_function_type )(  );
            
            SolutionDependentDiscreteSource3_exposer.def( 
                "Create"
                , Create_function_type( &::SolutionDependentDiscreteSource< 3 >::Create ) );
        
        }
        { //::SolutionDependentDiscreteSource< 3 >::GetConstantInUValues
        
            typedef SolutionDependentDiscreteSource< 3 > exported_class_t;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( exported_class_t::*GetConstantInUValues_function_type)(  ) ;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( SolutionDependentDiscreteSource_less__3__greater__wrapper::*default_GetConstantInUValues_function_type)(  ) ;
            
            SolutionDependentDiscreteSource3_exposer.def( 
                "GetConstantInUValues"
                , GetConstantInUValues_function_type(&::SolutionDependentDiscreteSource< 3 >::GetConstantInUValues)
                , default_GetConstantInUValues_function_type(&SolutionDependentDiscreteSource_less__3__greater__wrapper::default_GetConstantInUValues) );
        
        }
        { //::SolutionDependentDiscreteSource< 3 >::GetLinearInUValues
        
            typedef SolutionDependentDiscreteSource< 3 > exported_class_t;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( exported_class_t::*GetLinearInUValues_function_type)(  ) ;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( SolutionDependentDiscreteSource_less__3__greater__wrapper::*default_GetLinearInUValues_function_type)(  ) ;
            
            SolutionDependentDiscreteSource3_exposer.def( 
                "GetLinearInUValues"
                , GetLinearInUValues_function_type(&::SolutionDependentDiscreteSource< 3 >::GetLinearInUValues)
                , default_GetLinearInUValues_function_type(&SolutionDependentDiscreteSource_less__3__greater__wrapper::default_GetLinearInUValues) );
        
        }
        { //::SolutionDependentDiscreteSource< 3 >::SetConstantInUSinkRatePerSolutionQuantity
        
            typedef SolutionDependentDiscreteSource< 3 > exported_class_t;
            typedef void ( exported_class_t::*SetConstantInUSinkRatePerSolutionQuantity_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            SolutionDependentDiscreteSource3_exposer.def( 
                "SetConstantInUSinkRatePerSolutionQuantity"
                , SetConstantInUSinkRatePerSolutionQuantity_function_type( &::SolutionDependentDiscreteSource< 3 >::SetConstantInUSinkRatePerSolutionQuantity )
                , ( bp::arg("value") ) );
        
        }
        { //::SolutionDependentDiscreteSource< 3 >::SetLinearInUSinkRatePerSolutionQuantity
        
            typedef SolutionDependentDiscreteSource< 3 > exported_class_t;
            typedef void ( exported_class_t::*SetLinearInUSinkRatePerSolutionQuantity_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 3, 1 > >, boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::dimensionless_type > > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            SolutionDependentDiscreteSource3_exposer.def( 
                "SetLinearInUSinkRatePerSolutionQuantity"
                , SetLinearInUSinkRatePerSolutionQuantity_function_type( &::SolutionDependentDiscreteSource< 3 >::SetLinearInUSinkRatePerSolutionQuantity )
                , ( bp::arg("value") ) );
        
        }
        { //::SolutionDependentDiscreteSource< 3 >::SetSolution
        
            typedef SolutionDependentDiscreteSource< 3 > exported_class_t;
            typedef void ( exported_class_t::*SetSolution_function_type)( ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ) ;
            
            SolutionDependentDiscreteSource3_exposer.def( 
                "SetSolution"
                , SetSolution_function_type( &::SolutionDependentDiscreteSource< 3 >::SetSolution )
                , ( bp::arg("solution") ) );
        
        }
        { //::DiscreteSource< 3 >::GetNonlinearTermValues
        
            typedef SolutionDependentDiscreteSource< 3 > exported_class_t;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( exported_class_t::*GetNonlinearTermValues_function_type)(  ) ;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( SolutionDependentDiscreteSource_less__3__greater__wrapper::*default_GetNonlinearTermValues_function_type)(  ) ;
            
            SolutionDependentDiscreteSource3_exposer.def( 
                "GetNonlinearTermValues"
                , GetNonlinearTermValues_function_type(&::DiscreteSource< 3 >::GetNonlinearTermValues)
                , default_GetNonlinearTermValues_function_type(&SolutionDependentDiscreteSource_less__3__greater__wrapper::default_GetNonlinearTermValues) );
        
        }
        { //::DiscreteSource< 3 >::UpdateDensityMap
        
            typedef SolutionDependentDiscreteSource< 3 > exported_class_t;
            typedef void ( exported_class_t::*UpdateDensityMap_function_type)(  ) ;
            typedef void ( SolutionDependentDiscreteSource_less__3__greater__wrapper::*default_UpdateDensityMap_function_type)(  ) ;
            
            SolutionDependentDiscreteSource3_exposer.def( 
                "UpdateDensityMap"
                , UpdateDensityMap_function_type(&::DiscreteSource< 3 >::UpdateDensityMap)
                , default_UpdateDensityMap_function_type(&SolutionDependentDiscreteSource_less__3__greater__wrapper::default_UpdateDensityMap) );
        
        }
        SolutionDependentDiscreteSource3_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< SolutionDependentDiscreteSource<3> > >();
        bp::implicitly_convertible< boost::shared_ptr< SolutionDependentDiscreteSource< 3 > >, boost::shared_ptr< DiscreteSource< 3 > > >();
    }

}
