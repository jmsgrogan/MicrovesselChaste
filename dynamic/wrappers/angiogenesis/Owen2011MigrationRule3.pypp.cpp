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
#include "Owen2011MigrationRule3.pypp.hpp"

namespace bp = boost::python;

struct Owen2011MigrationRule_less__3__greater__wrapper : Owen2011MigrationRule< 3 >, bp::wrapper< Owen2011MigrationRule< 3 > > {

    Owen2011MigrationRule_less__3__greater__wrapper(Owen2011MigrationRule<3> const & arg )
    : Owen2011MigrationRule<3>( arg )
      , bp::wrapper< Owen2011MigrationRule< 3 > >(){
        // copy constructor
        
    }

    Owen2011MigrationRule_less__3__greater__wrapper( )
    : Owen2011MigrationRule<3>( )
      , bp::wrapper< Owen2011MigrationRule< 3 > >(){
        // null constructor
    
    }

    virtual ::std::vector< int > GetIndices( ::std::vector< boost::shared_ptr<VesselNode<3> > > const & rNodes ) {
        if( bp::override func_GetIndices = this->get_override( "GetIndices" ) )
            return func_GetIndices( boost::ref(rNodes) );
        else{
            return this->Owen2011MigrationRule< 3 >::GetIndices( boost::ref(rNodes) );
        }
    }
    
    ::std::vector< int > default_GetIndices( ::std::vector< boost::shared_ptr<VesselNode<3> > > const & rNodes ) {
        return Owen2011MigrationRule< 3 >::GetIndices( boost::ref(rNodes) );
    }

    virtual ::std::vector< DimensionalChastePoint<3> > GetDirections( ::std::vector< boost::shared_ptr<VesselNode<3> > > const & rNodes ) {
        if( bp::override func_GetDirections = this->get_override( "GetDirections" ) )
            return func_GetDirections( boost::ref(rNodes) );
        else{
            return this->AbstractMigrationRule< 3 >::GetDirections( boost::ref(rNodes) );
        }
    }
    
    ::std::vector< DimensionalChastePoint<3> > default_GetDirections( ::std::vector< boost::shared_ptr<VesselNode<3> > > const & rNodes ) {
        return AbstractMigrationRule< 3 >::GetDirections( boost::ref(rNodes) );
    }

    virtual int GetNeighbourMovementIndex( ::std::vector< double > movementProbabilities, ::std::vector< unsigned int > neighbourIndices ){
        if( bp::override func_GetNeighbourMovementIndex = this->get_override( "GetNeighbourMovementIndex" ) )
            return func_GetNeighbourMovementIndex( movementProbabilities, neighbourIndices );
        else{
            return this->LatticeBasedMigrationRule< 3 >::GetNeighbourMovementIndex( movementProbabilities, neighbourIndices );
        }
    }
    
    virtual int default_GetNeighbourMovementIndex( ::std::vector< double > movementProbabilities, ::std::vector< unsigned int > neighbourIndices ){
        return LatticeBasedMigrationRule< 3 >::GetNeighbourMovementIndex( movementProbabilities, neighbourIndices );
    }

};

void register_Owen2011MigrationRule3_class(){

    { //::Owen2011MigrationRule< 3 >
        typedef bp::class_< Owen2011MigrationRule_less__3__greater__wrapper, bp::bases< LatticeBasedMigrationRule< 3 > > > Owen2011MigrationRule3_exposer_t;
        Owen2011MigrationRule3_exposer_t Owen2011MigrationRule3_exposer = Owen2011MigrationRule3_exposer_t( "Owen2011MigrationRule3", bp::init< >() );
        bp::scope Owen2011MigrationRule3_scope( Owen2011MigrationRule3_exposer );
        { //::Owen2011MigrationRule< 3 >::Create
        
            typedef Owen2011MigrationRule< 3 > exported_class_t;
            typedef ::boost::shared_ptr< Owen2011MigrationRule< 3 > > ( *Create_function_type )(  );
            
            Owen2011MigrationRule3_exposer.def( 
                "Create"
                , Create_function_type( &::Owen2011MigrationRule< 3 >::Create ) );
        
        }
        { //::Owen2011MigrationRule< 3 >::GetIndices
        
            typedef Owen2011MigrationRule< 3 > exported_class_t;
            typedef ::std::vector< int > ( exported_class_t::*GetIndices_function_type)( ::std::vector< boost::shared_ptr<VesselNode<3> > > const & ) ;
            typedef ::std::vector< int > ( Owen2011MigrationRule_less__3__greater__wrapper::*default_GetIndices_function_type)( ::std::vector< boost::shared_ptr<VesselNode<3> > > const & ) ;
            
            Owen2011MigrationRule3_exposer.def( 
                "GetIndices"
                , GetIndices_function_type(&::Owen2011MigrationRule< 3 >::GetIndices)
                , default_GetIndices_function_type(&Owen2011MigrationRule_less__3__greater__wrapper::default_GetIndices)
                , ( bp::arg("rNodes") ) );
        
        }
        { //::Owen2011MigrationRule< 3 >::SetCellChemotacticParameter
        
            typedef Owen2011MigrationRule< 3 > exported_class_t;
            typedef void ( exported_class_t::*SetCellChemotacticParameter_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 5, 1 > >, boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::dimensionless_type > > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            Owen2011MigrationRule3_exposer.def( 
                "SetCellChemotacticParameter"
                , SetCellChemotacticParameter_function_type( &::Owen2011MigrationRule< 3 >::SetCellChemotacticParameter )
                , ( bp::arg("cellChemotacticParameter") ) );
        
        }
        { //::Owen2011MigrationRule< 3 >::SetCellMotilityParameter
        
            typedef Owen2011MigrationRule< 3 > exported_class_t;
            typedef void ( exported_class_t::*SetCellMotilityParameter_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 2, 1 > >, boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            Owen2011MigrationRule3_exposer.def( 
                "SetCellMotilityParameter"
                , SetCellMotilityParameter_function_type( &::Owen2011MigrationRule< 3 >::SetCellMotilityParameter )
                , ( bp::arg("cellMotility") ) );
        
        }
        { //::AbstractMigrationRule< 3 >::GetDirections
        
            typedef Owen2011MigrationRule< 3 > exported_class_t;
            typedef ::std::vector< DimensionalChastePoint<3> > ( exported_class_t::*GetDirections_function_type)( ::std::vector< boost::shared_ptr<VesselNode<3> > > const & ) ;
            typedef ::std::vector< DimensionalChastePoint<3> > ( Owen2011MigrationRule_less__3__greater__wrapper::*default_GetDirections_function_type)( ::std::vector< boost::shared_ptr<VesselNode<3> > > const & ) ;
            
            Owen2011MigrationRule3_exposer.def( 
                "GetDirections"
                , GetDirections_function_type(&::AbstractMigrationRule< 3 >::GetDirections)
                , default_GetDirections_function_type(&Owen2011MigrationRule_less__3__greater__wrapper::default_GetDirections)
                , ( bp::arg("rNodes") ) );
        
        }
        { //::LatticeBasedMigrationRule< 3 >::GetNeighbourMovementIndex
        
            typedef Owen2011MigrationRule< 3 > exported_class_t;
            typedef int ( Owen2011MigrationRule_less__3__greater__wrapper::*GetNeighbourMovementIndex_function_type)( ::std::vector< double >,::std::vector< unsigned int > ) ;
            
            Owen2011MigrationRule3_exposer.def( 
                "GetNeighbourMovementIndex"
                , GetNeighbourMovementIndex_function_type( &Owen2011MigrationRule_less__3__greater__wrapper::default_GetNeighbourMovementIndex )
                , ( bp::arg("movementProbabilities"), bp::arg("neighbourIndices") ) );
        
        }
        Owen2011MigrationRule3_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< Owen2011MigrationRule<3> > >();
        bp::implicitly_convertible< boost::shared_ptr< Owen2011MigrationRule< 3 > >, boost::shared_ptr< AbstractMigrationRule< 3 > > >();
    }

}