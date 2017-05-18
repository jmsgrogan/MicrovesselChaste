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
#include "CellPopulationMigrationRule2.pypp.hpp"

namespace bp = boost::python;

struct CellPopulationMigrationRule_less__2__greater__wrapper : CellPopulationMigrationRule< 2 >, bp::wrapper< CellPopulationMigrationRule< 2 > > {

    CellPopulationMigrationRule_less__2__greater__wrapper(CellPopulationMigrationRule<2> const & arg )
    : CellPopulationMigrationRule<2>( arg )
      , bp::wrapper< CellPopulationMigrationRule< 2 > >(){
        // copy constructor
        
    }

    CellPopulationMigrationRule_less__2__greater__wrapper( )
    : CellPopulationMigrationRule<2>( )
      , bp::wrapper< CellPopulationMigrationRule< 2 > >(){
        // null constructor
    
    }

    virtual ::std::vector< int > GetIndices( ::std::vector< boost::shared_ptr<VesselNode<2> > > const & rNodes ) {
        if( bp::override func_GetIndices = this->get_override( "GetIndices" ) )
            return func_GetIndices( boost::ref(rNodes) );
        else{
            return this->CellPopulationMigrationRule< 2 >::GetIndices( boost::ref(rNodes) );
        }
    }
    
    ::std::vector< int > default_GetIndices( ::std::vector< boost::shared_ptr<VesselNode<2> > > const & rNodes ) {
        return CellPopulationMigrationRule< 2 >::GetIndices( boost::ref(rNodes) );
    }

    virtual ::std::vector< double > GetNeighbourMovementProbabilities( ::boost::shared_ptr< VesselNode< 2 > > pNode, ::std::vector< unsigned int > neighbourIndices, unsigned int gridIndex ){
        if( bp::override func_GetNeighbourMovementProbabilities = this->get_override( "GetNeighbourMovementProbabilities" ) )
            return func_GetNeighbourMovementProbabilities( pNode, neighbourIndices, gridIndex );
        else{
            return this->CellPopulationMigrationRule< 2 >::GetNeighbourMovementProbabilities( pNode, neighbourIndices, gridIndex );
        }
    }
    
    virtual ::std::vector< double > default_GetNeighbourMovementProbabilities( ::boost::shared_ptr< VesselNode< 2 > > pNode, ::std::vector< unsigned int > neighbourIndices, unsigned int gridIndex ){
        return CellPopulationMigrationRule< 2 >::GetNeighbourMovementProbabilities( pNode, neighbourIndices, gridIndex );
    }

    virtual ::std::vector< DimensionalChastePoint<2> > GetDirections( ::std::vector< boost::shared_ptr<VesselNode<2> > > const & rNodes ) {
        if( bp::override func_GetDirections = this->get_override( "GetDirections" ) )
            return func_GetDirections( boost::ref(rNodes) );
        else{
            return this->AbstractMigrationRule< 2 >::GetDirections( boost::ref(rNodes) );
        }
    }
    
    ::std::vector< DimensionalChastePoint<2> > default_GetDirections( ::std::vector< boost::shared_ptr<VesselNode<2> > > const & rNodes ) {
        return AbstractMigrationRule< 2 >::GetDirections( boost::ref(rNodes) );
    }

    virtual int GetNeighbourMovementIndex( ::std::vector< double > movementProbabilities, ::std::vector< unsigned int > neighbourIndices ){
        if( bp::override func_GetNeighbourMovementIndex = this->get_override( "GetNeighbourMovementIndex" ) )
            return func_GetNeighbourMovementIndex( movementProbabilities, neighbourIndices );
        else{
            return this->LatticeBasedMigrationRule< 2 >::GetNeighbourMovementIndex( movementProbabilities, neighbourIndices );
        }
    }
    
    virtual int default_GetNeighbourMovementIndex( ::std::vector< double > movementProbabilities, ::std::vector< unsigned int > neighbourIndices ){
        return LatticeBasedMigrationRule< 2 >::GetNeighbourMovementIndex( movementProbabilities, neighbourIndices );
    }

};

void register_CellPopulationMigrationRule2_class(){

    { //::CellPopulationMigrationRule< 2 >
        typedef bp::class_< CellPopulationMigrationRule_less__2__greater__wrapper, bp::bases< LatticeBasedMigrationRule< 2 > > > CellPopulationMigrationRule2_exposer_t;
        CellPopulationMigrationRule2_exposer_t CellPopulationMigrationRule2_exposer = CellPopulationMigrationRule2_exposer_t( "CellPopulationMigrationRule2", bp::init< >() );
        bp::scope CellPopulationMigrationRule2_scope( CellPopulationMigrationRule2_exposer );
        { //::CellPopulationMigrationRule< 2 >::Create
        
            typedef CellPopulationMigrationRule< 2 > exported_class_t;
            typedef ::boost::shared_ptr< CellPopulationMigrationRule< 2 > > ( *Create_function_type )(  );
            
            CellPopulationMigrationRule2_exposer.def( 
                "Create"
                , Create_function_type( &::CellPopulationMigrationRule< 2 >::Create ) );
        
        }
        { //::CellPopulationMigrationRule< 2 >::GetIndices
        
            typedef CellPopulationMigrationRule< 2 > exported_class_t;
            typedef ::std::vector< int > ( exported_class_t::*GetIndices_function_type)( ::std::vector< boost::shared_ptr<VesselNode<2> > > const & ) ;
            typedef ::std::vector< int > ( CellPopulationMigrationRule_less__2__greater__wrapper::*default_GetIndices_function_type)( ::std::vector< boost::shared_ptr<VesselNode<2> > > const & ) ;
            
            CellPopulationMigrationRule2_exposer.def( 
                "GetIndices"
                , GetIndices_function_type(&::CellPopulationMigrationRule< 2 >::GetIndices)
                , default_GetIndices_function_type(&CellPopulationMigrationRule_less__2__greater__wrapper::default_GetIndices)
                , ( bp::arg("rNodes") ) );
        
        }
        { //::CellPopulationMigrationRule< 2 >::GetNeighbourMovementProbabilities
        
            typedef CellPopulationMigrationRule< 2 > exported_class_t;
            typedef ::std::vector< double > ( CellPopulationMigrationRule_less__2__greater__wrapper::*GetNeighbourMovementProbabilities_function_type)( ::boost::shared_ptr< VesselNode< 2 > >,::std::vector< unsigned int >,unsigned int ) ;
            
            CellPopulationMigrationRule2_exposer.def( 
                "GetNeighbourMovementProbabilities"
                , GetNeighbourMovementProbabilities_function_type( &CellPopulationMigrationRule_less__2__greater__wrapper::default_GetNeighbourMovementProbabilities )
                , ( bp::arg("pNode"), bp::arg("neighbourIndices"), bp::arg("gridIndex") ) );
        
        }
        { //::CellPopulationMigrationRule< 2 >::GetOccupyingVolumeFraction
        
            typedef CellPopulationMigrationRule< 2 > exported_class_t;
            typedef double ( exported_class_t::*GetOccupyingVolumeFraction_function_type)( ::boost::shared_ptr< AbstractCellMutationState > ) ;
            
            CellPopulationMigrationRule2_exposer.def( 
                "GetOccupyingVolumeFraction"
                , GetOccupyingVolumeFraction_function_type( &::CellPopulationMigrationRule< 2 >::GetOccupyingVolumeFraction )
                , ( bp::arg("mutation_state") ) );
        
        }
        { //::CellPopulationMigrationRule< 2 >::SetVolumeFraction
        
            typedef CellPopulationMigrationRule< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetVolumeFraction_function_type)( ::boost::shared_ptr< AbstractCellMutationState >,double ) ;
            
            CellPopulationMigrationRule2_exposer.def( 
                "SetVolumeFraction"
                , SetVolumeFraction_function_type( &::CellPopulationMigrationRule< 2 >::SetVolumeFraction )
                , ( bp::arg("mutation_state"), bp::arg("volume_fraction") ) );
        
        }
        { //::AbstractMigrationRule< 2 >::GetDirections
        
            typedef CellPopulationMigrationRule< 2 > exported_class_t;
            typedef ::std::vector< DimensionalChastePoint<2> > ( exported_class_t::*GetDirections_function_type)( ::std::vector< boost::shared_ptr<VesselNode<2> > > const & ) ;
            typedef ::std::vector< DimensionalChastePoint<2> > ( CellPopulationMigrationRule_less__2__greater__wrapper::*default_GetDirections_function_type)( ::std::vector< boost::shared_ptr<VesselNode<2> > > const & ) ;
            
            CellPopulationMigrationRule2_exposer.def( 
                "GetDirections"
                , GetDirections_function_type(&::AbstractMigrationRule< 2 >::GetDirections)
                , default_GetDirections_function_type(&CellPopulationMigrationRule_less__2__greater__wrapper::default_GetDirections)
                , ( bp::arg("rNodes") ) );
        
        }
        { //::LatticeBasedMigrationRule< 2 >::GetNeighbourMovementIndex
        
            typedef CellPopulationMigrationRule< 2 > exported_class_t;
            typedef int ( CellPopulationMigrationRule_less__2__greater__wrapper::*GetNeighbourMovementIndex_function_type)( ::std::vector< double >,::std::vector< unsigned int > ) ;
            
            CellPopulationMigrationRule2_exposer.def( 
                "GetNeighbourMovementIndex"
                , GetNeighbourMovementIndex_function_type( &CellPopulationMigrationRule_less__2__greater__wrapper::default_GetNeighbourMovementIndex )
                , ( bp::arg("movementProbabilities"), bp::arg("neighbourIndices") ) );
        
        }
        CellPopulationMigrationRule2_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< CellPopulationMigrationRule<2> > >();
        bp::implicitly_convertible< boost::shared_ptr< CellPopulationMigrationRule< 2 > >, boost::shared_ptr< AbstractMigrationRule< 2 > > >();
    }

}