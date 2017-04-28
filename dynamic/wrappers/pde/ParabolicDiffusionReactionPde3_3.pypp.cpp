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
#include "ParabolicDiffusionReactionPde3_3.pypp.hpp"

namespace bp = boost::python;

struct ParabolicDiffusionReactionPde_less__3_comma__3__greater__wrapper : ParabolicDiffusionReactionPde< 3, 3 >, bp::wrapper< ParabolicDiffusionReactionPde< 3, 3 > > {

    ParabolicDiffusionReactionPde_less__3_comma__3__greater__wrapper(ParabolicDiffusionReactionPde<3, 3> const & arg )
    : ParabolicDiffusionReactionPde<3, 3>( arg )
      , bp::wrapper< ParabolicDiffusionReactionPde< 3, 3 > >(){
        // copy constructor
        
    }

    ParabolicDiffusionReactionPde_less__3_comma__3__greater__wrapper( )
    : ParabolicDiffusionReactionPde<3, 3>( )
      , bp::wrapper< ParabolicDiffusionReactionPde< 3, 3 > >(){
        // null constructor
    
    }

    virtual double ComputeSourceTerm( ::ChastePoint< 3 > const & rX, double u, ::Element< 3, 3 > * pElement=__null ) {
        if( bp::override func_ComputeSourceTerm = this->get_override( "ComputeSourceTerm" ) )
            return func_ComputeSourceTerm( boost::ref(rX), u, boost::python::ptr(pElement) );
        else{
            return this->ParabolicDiffusionReactionPde< 3, 3 >::ComputeSourceTerm( boost::ref(rX), u, boost::python::ptr(pElement) );
        }
    }
    
    double default_ComputeSourceTerm( ::ChastePoint< 3 > const & rX, double u, ::Element< 3, 3 > * pElement=__null ) {
        return ParabolicDiffusionReactionPde< 3, 3 >::ComputeSourceTerm( boost::ref(rX), u, boost::python::ptr(pElement) );
    }

    virtual ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ComputeSourceTerm( unsigned int gridIndex, ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > u ) {
        if( bp::override func_ComputeSourceTerm = this->get_override( "ComputeSourceTerm" ) )
            return func_ComputeSourceTerm( gridIndex, u );
        else{
            return this->ParabolicDiffusionReactionPde< 3, 3 >::ComputeSourceTerm( gridIndex, u );
        }
    }
    
    ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > default_ComputeSourceTerm( unsigned int gridIndex, ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > u ) {
        return ParabolicDiffusionReactionPde< 3, 3 >::ComputeSourceTerm( gridIndex, u );
    }

    virtual ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ComputeSourceTermPrime( unsigned int gridIndex, ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > u ) {
        if( bp::override func_ComputeSourceTermPrime = this->get_override( "ComputeSourceTermPrime" ) )
            return func_ComputeSourceTermPrime( gridIndex, u );
        else{
            return this->ParabolicDiffusionReactionPde< 3, 3 >::ComputeSourceTermPrime( gridIndex, u );
        }
    }
    
    ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > default_ComputeSourceTermPrime( unsigned int gridIndex, ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > u ) {
        return ParabolicDiffusionReactionPde< 3, 3 >::ComputeSourceTermPrime( gridIndex, u );
    }

    virtual void UpdateDiscreteSourceStrengths(  ) {
        if( bp::override func_UpdateDiscreteSourceStrengths = this->get_override( "UpdateDiscreteSourceStrengths" ) )
            func_UpdateDiscreteSourceStrengths(  );
        else{
            this->ParabolicDiffusionReactionPde< 3, 3 >::UpdateDiscreteSourceStrengths(  );
        }
    }
    
    void default_UpdateDiscreteSourceStrengths(  ) {
        ParabolicDiffusionReactionPde< 3, 3 >::UpdateDiscreteSourceStrengths( );
    }

    virtual ::boost::numeric::ublas::c_matrix< double, 3, 3 > ComputeDiffusionTerm( ::ChastePoint< 3 > const & rX, ::Element< 3, 3 > * pElement=__null ) {
        if( bp::override func_ComputeDiffusionTerm = this->get_override( "ComputeDiffusionTerm" ) )
            return func_ComputeDiffusionTerm( boost::ref(rX), boost::python::ptr(pElement) );
        else{
            return this->AbstractDiscreteContinuumParabolicPde< 3, 3 >::ComputeDiffusionTerm( boost::ref(rX), boost::python::ptr(pElement) );
        }
    }
    
    ::boost::numeric::ublas::c_matrix< double, 3, 3 > default_ComputeDiffusionTerm( ::ChastePoint< 3 > const & rX, ::Element< 3, 3 > * pElement=__null ) {
        return AbstractDiscreteContinuumParabolicPde< 3, 3 >::ComputeDiffusionTerm( boost::ref(rX), boost::python::ptr(pElement) );
    }

    virtual double ComputeDuDtCoefficientFunction( ::ChastePoint< 3 > const & arg0 ) {
        if( bp::override func_ComputeDuDtCoefficientFunction = this->get_override( "ComputeDuDtCoefficientFunction" ) )
            return func_ComputeDuDtCoefficientFunction( boost::ref(arg0) );
        else{
            return this->AbstractDiscreteContinuumParabolicPde< 3, 3 >::ComputeDuDtCoefficientFunction( boost::ref(arg0) );
        }
    }
    
    double default_ComputeDuDtCoefficientFunction( ::ChastePoint< 3 > const & arg0 ) {
        return AbstractDiscreteContinuumParabolicPde< 3, 3 >::ComputeDuDtCoefficientFunction( boost::ref(arg0) );
    }

};

void register_ParabolicDiffusionReactionPde3_3_class(){

    { //::ParabolicDiffusionReactionPde< 3, 3 >
        typedef bp::class_< ParabolicDiffusionReactionPde_less__3_comma__3__greater__wrapper, bp::bases< AbstractDiscreteContinuumParabolicPde< 3, 3 > > > ParabolicDiffusionReactionPde3_3_exposer_t;
        ParabolicDiffusionReactionPde3_3_exposer_t ParabolicDiffusionReactionPde3_3_exposer = ParabolicDiffusionReactionPde3_3_exposer_t( "ParabolicDiffusionReactionPde3_3", bp::init< >() );
        bp::scope ParabolicDiffusionReactionPde3_3_scope( ParabolicDiffusionReactionPde3_3_exposer );
        { //::ParabolicDiffusionReactionPde< 3, 3 >::ComputeSourceTerm
        
            typedef ParabolicDiffusionReactionPde< 3, 3 > exported_class_t;
            typedef double ( exported_class_t::*ComputeSourceTerm_function_type)( ::ChastePoint< 3 > const &,double,::Element< 3, 3 > * ) ;
            typedef double ( ParabolicDiffusionReactionPde_less__3_comma__3__greater__wrapper::*default_ComputeSourceTerm_function_type)( ::ChastePoint< 3 > const &,double,::Element< 3, 3 > * ) ;
            
            ParabolicDiffusionReactionPde3_3_exposer.def( 
                "ComputeSourceTerm"
                , ComputeSourceTerm_function_type(&::ParabolicDiffusionReactionPde< 3, 3 >::ComputeSourceTerm)
                , default_ComputeSourceTerm_function_type(&ParabolicDiffusionReactionPde_less__3_comma__3__greater__wrapper::default_ComputeSourceTerm)
                , ( bp::arg("rX"), bp::arg("u"), bp::arg("pElement")=__null ) );
        
        }
        { //::ParabolicDiffusionReactionPde< 3, 3 >::ComputeSourceTerm
        
            typedef ParabolicDiffusionReactionPde< 3, 3 > exported_class_t;
            typedef ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ( exported_class_t::*ComputeSourceTerm_function_type)( unsigned int,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            typedef ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ( ParabolicDiffusionReactionPde_less__3_comma__3__greater__wrapper::*default_ComputeSourceTerm_function_type)( unsigned int,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            ParabolicDiffusionReactionPde3_3_exposer.def( 
                "ComputeSourceTerm"
                , ComputeSourceTerm_function_type(&::ParabolicDiffusionReactionPde< 3, 3 >::ComputeSourceTerm)
                , default_ComputeSourceTerm_function_type(&ParabolicDiffusionReactionPde_less__3_comma__3__greater__wrapper::default_ComputeSourceTerm)
                , ( bp::arg("gridIndex"), bp::arg("u") ) );
        
        }
        { //::ParabolicDiffusionReactionPde< 3, 3 >::ComputeSourceTermPrime
        
            typedef ParabolicDiffusionReactionPde< 3, 3 > exported_class_t;
            typedef ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ( exported_class_t::*ComputeSourceTermPrime_function_type)( unsigned int,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            typedef ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ( ParabolicDiffusionReactionPde_less__3_comma__3__greater__wrapper::*default_ComputeSourceTermPrime_function_type)( unsigned int,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            ParabolicDiffusionReactionPde3_3_exposer.def( 
                "ComputeSourceTermPrime"
                , ComputeSourceTermPrime_function_type(&::ParabolicDiffusionReactionPde< 3, 3 >::ComputeSourceTermPrime)
                , default_ComputeSourceTermPrime_function_type(&ParabolicDiffusionReactionPde_less__3_comma__3__greater__wrapper::default_ComputeSourceTermPrime)
                , ( bp::arg("gridIndex"), bp::arg("u") ) );
        
        }
        { //::ParabolicDiffusionReactionPde< 3, 3 >::Create
        
            typedef ParabolicDiffusionReactionPde< 3, 3 > exported_class_t;
            typedef ::boost::shared_ptr< ParabolicDiffusionReactionPde< 3, 3 > > ( *Create_function_type )(  );
            
            ParabolicDiffusionReactionPde3_3_exposer.def( 
                "Create"
                , Create_function_type( &::ParabolicDiffusionReactionPde< 3, 3 >::Create ) );
        
        }
        { //::ParabolicDiffusionReactionPde< 3, 3 >::SetContinuumLinearInUTerm
        
            typedef ParabolicDiffusionReactionPde< 3, 3 > exported_class_t;
            typedef void ( exported_class_t::*SetContinuumLinearInUTerm_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            ParabolicDiffusionReactionPde3_3_exposer.def( 
                "SetContinuumLinearInUTerm"
                , SetContinuumLinearInUTerm_function_type( &::ParabolicDiffusionReactionPde< 3, 3 >::SetContinuumLinearInUTerm )
                , ( bp::arg("linearInUTerm") ) );
        
        }
        { //::ParabolicDiffusionReactionPde< 3, 3 >::UpdateDiscreteSourceStrengths
        
            typedef ParabolicDiffusionReactionPde< 3, 3 > exported_class_t;
            typedef void ( exported_class_t::*UpdateDiscreteSourceStrengths_function_type)(  ) ;
            typedef void ( ParabolicDiffusionReactionPde_less__3_comma__3__greater__wrapper::*default_UpdateDiscreteSourceStrengths_function_type)(  ) ;
            
            ParabolicDiffusionReactionPde3_3_exposer.def( 
                "UpdateDiscreteSourceStrengths"
                , UpdateDiscreteSourceStrengths_function_type(&::ParabolicDiffusionReactionPde< 3, 3 >::UpdateDiscreteSourceStrengths)
                , default_UpdateDiscreteSourceStrengths_function_type(&ParabolicDiffusionReactionPde_less__3_comma__3__greater__wrapper::default_UpdateDiscreteSourceStrengths) );
        
        }
        { //::AbstractDiscreteContinuumParabolicPde< 3, 3 >::ComputeDiffusionTerm
        
            typedef ParabolicDiffusionReactionPde< 3, 3 > exported_class_t;
            typedef ::boost::numeric::ublas::c_matrix< double, 3, 3 > ( exported_class_t::*ComputeDiffusionTerm_function_type)( ::ChastePoint< 3 > const &,::Element< 3, 3 > * ) ;
            typedef ::boost::numeric::ublas::c_matrix< double, 3, 3 > ( ParabolicDiffusionReactionPde_less__3_comma__3__greater__wrapper::*default_ComputeDiffusionTerm_function_type)( ::ChastePoint< 3 > const &,::Element< 3, 3 > * ) ;
            
            ParabolicDiffusionReactionPde3_3_exposer.def( 
                "ComputeDiffusionTerm"
                , ComputeDiffusionTerm_function_type(&::AbstractDiscreteContinuumParabolicPde< 3, 3 >::ComputeDiffusionTerm)
                , default_ComputeDiffusionTerm_function_type(&ParabolicDiffusionReactionPde_less__3_comma__3__greater__wrapper::default_ComputeDiffusionTerm)
                , ( bp::arg("rX"), bp::arg("pElement")=__null ) );
        
        }
        { //::AbstractDiscreteContinuumParabolicPde< 3, 3 >::ComputeDuDtCoefficientFunction
        
            typedef ParabolicDiffusionReactionPde< 3, 3 > exported_class_t;
            typedef double ( exported_class_t::*ComputeDuDtCoefficientFunction_function_type)( ::ChastePoint< 3 > const & ) ;
            typedef double ( ParabolicDiffusionReactionPde_less__3_comma__3__greater__wrapper::*default_ComputeDuDtCoefficientFunction_function_type)( ::ChastePoint< 3 > const & ) ;
            
            ParabolicDiffusionReactionPde3_3_exposer.def( 
                "ComputeDuDtCoefficientFunction"
                , ComputeDuDtCoefficientFunction_function_type(&::AbstractDiscreteContinuumParabolicPde< 3, 3 >::ComputeDuDtCoefficientFunction)
                , default_ComputeDuDtCoefficientFunction_function_type(&ParabolicDiffusionReactionPde_less__3_comma__3__greater__wrapper::default_ComputeDuDtCoefficientFunction)
                , ( bp::arg("arg0") ) );
        
        }
        ParabolicDiffusionReactionPde3_3_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< ParabolicDiffusionReactionPde<3, 3> > >();
        bp::implicitly_convertible< boost::shared_ptr< ParabolicDiffusionReactionPde< 3, 3 > >, boost::shared_ptr< AbstractDiscreteContinuumParabolicPde< 3, 3 > > >();
        bp::implicitly_convertible< boost::shared_ptr< ParabolicDiffusionReactionPde< 3, 3 > >, boost::shared_ptr< AbstractLinearParabolicPde< 3, 3 > > >();
        bp::implicitly_convertible< boost::shared_ptr< ParabolicDiffusionReactionPde< 3, 3 > >, boost::shared_ptr< AbstractLinearPde< 3, 3 > > >();
        bp::implicitly_convertible< boost::shared_ptr< ParabolicDiffusionReactionPde< 3, 3 > >, boost::shared_ptr< AbstractDiscreteContinuumPde< 3, 3 > > >();
    }

}
