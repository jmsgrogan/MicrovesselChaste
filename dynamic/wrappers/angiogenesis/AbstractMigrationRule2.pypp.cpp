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
#include "AbstractMigrationRule2.pypp.hpp"

namespace bp = boost::python;

struct AbstractMigrationRule_less__2__greater__wrapper : AbstractMigrationRule< 2 >, bp::wrapper< AbstractMigrationRule< 2 > > {

    AbstractMigrationRule_less__2__greater__wrapper(AbstractMigrationRule<2> const & arg )
    : AbstractMigrationRule<2>( arg )
      , bp::wrapper< AbstractMigrationRule< 2 > >(){
        // copy constructor
        
    }

    AbstractMigrationRule_less__2__greater__wrapper( )
    : AbstractMigrationRule<2>( )
      , bp::wrapper< AbstractMigrationRule< 2 > >(){
        // null constructor
    
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

    virtual ::std::vector< int > GetIndices( ::std::vector< boost::shared_ptr<VesselNode<2> > > const & rNodes ) {
        if( bp::override func_GetIndices = this->get_override( "GetIndices" ) )
            return func_GetIndices( boost::ref(rNodes) );
        else{
            return this->AbstractMigrationRule< 2 >::GetIndices( boost::ref(rNodes) );
        }
    }
    
    ::std::vector< int > default_GetIndices( ::std::vector< boost::shared_ptr<VesselNode<2> > > const & rNodes ) {
        return AbstractMigrationRule< 2 >::GetIndices( boost::ref(rNodes) );
    }

};

void register_AbstractMigrationRule2_class(){

    { //::AbstractMigrationRule< 2 >
        typedef bp::class_< AbstractMigrationRule_less__2__greater__wrapper > AbstractMigrationRule2_exposer_t;
        AbstractMigrationRule2_exposer_t AbstractMigrationRule2_exposer = AbstractMigrationRule2_exposer_t( "AbstractMigrationRule2", bp::init< >() );
        bp::scope AbstractMigrationRule2_scope( AbstractMigrationRule2_exposer );
        { //::AbstractMigrationRule< 2 >::Create
        
            typedef AbstractMigrationRule< 2 > exported_class_t;
            typedef ::boost::shared_ptr< AbstractMigrationRule< 2 > > ( *Create_function_type )(  );
            
            AbstractMigrationRule2_exposer.def( 
                "Create"
                , Create_function_type( &::AbstractMigrationRule< 2 >::Create ) );
        
        }
        { //::AbstractMigrationRule< 2 >::GetDirections
        
            typedef AbstractMigrationRule< 2 > exported_class_t;
            typedef ::std::vector< DimensionalChastePoint<2> > ( exported_class_t::*GetDirections_function_type)( ::std::vector< boost::shared_ptr<VesselNode<2> > > const & ) ;
            typedef ::std::vector< DimensionalChastePoint<2> > ( AbstractMigrationRule_less__2__greater__wrapper::*default_GetDirections_function_type)( ::std::vector< boost::shared_ptr<VesselNode<2> > > const & ) ;
            
            AbstractMigrationRule2_exposer.def( 
                "GetDirections"
                , GetDirections_function_type(&::AbstractMigrationRule< 2 >::GetDirections)
                , default_GetDirections_function_type(&AbstractMigrationRule_less__2__greater__wrapper::default_GetDirections)
                , ( bp::arg("rNodes") ) );
        
        }
        { //::AbstractMigrationRule< 2 >::GetIndices
        
            typedef AbstractMigrationRule< 2 > exported_class_t;
            typedef ::std::vector< int > ( exported_class_t::*GetIndices_function_type)( ::std::vector< boost::shared_ptr<VesselNode<2> > > const & ) ;
            typedef ::std::vector< int > ( AbstractMigrationRule_less__2__greater__wrapper::*default_GetIndices_function_type)( ::std::vector< boost::shared_ptr<VesselNode<2> > > const & ) ;
            
            AbstractMigrationRule2_exposer.def( 
                "GetIndices"
                , GetIndices_function_type(&::AbstractMigrationRule< 2 >::GetIndices)
                , default_GetIndices_function_type(&AbstractMigrationRule_less__2__greater__wrapper::default_GetIndices)
                , ( bp::arg("rNodes") ) );
        
        }
        { //::AbstractMigrationRule< 2 >::SetBoundingDomain
        
            typedef AbstractMigrationRule< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetBoundingDomain_function_type)( ::boost::shared_ptr< Part< 2 > > ) ;
            
            AbstractMigrationRule2_exposer.def( 
                "SetBoundingDomain"
                , SetBoundingDomain_function_type( &::AbstractMigrationRule< 2 >::SetBoundingDomain )
                , ( bp::arg("pPart") ) );
        
        }
        { //::AbstractMigrationRule< 2 >::SetCellPopulation
        
            typedef AbstractMigrationRule< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetCellPopulation_function_type)( ::boost::shared_ptr< AbstractCellPopulation< 2, 2 > > ) ;
            
            AbstractMigrationRule2_exposer.def( 
                "SetCellPopulation"
                , SetCellPopulation_function_type( &::AbstractMigrationRule< 2 >::SetCellPopulation )
                , ( bp::arg("pCellPopulation") ) );
        
        }
        { //::AbstractMigrationRule< 2 >::SetDiscreteContinuumSolver
        
            typedef AbstractMigrationRule< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetDiscreteContinuumSolver_function_type)( ::boost::shared_ptr< AbstractDiscreteContinuumSolver< 2 > > ) ;
            
            AbstractMigrationRule2_exposer.def( 
                "SetDiscreteContinuumSolver"
                , SetDiscreteContinuumSolver_function_type( &::AbstractMigrationRule< 2 >::SetDiscreteContinuumSolver )
                , ( bp::arg("pSolver") ) );
        
        }
        { //::AbstractMigrationRule< 2 >::SetGridCalculator
        
            typedef AbstractMigrationRule< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetGridCalculator_function_type)( ::boost::shared_ptr< GridCalculator< 2 > > ) ;
            
            AbstractMigrationRule2_exposer.def( 
                "SetGridCalculator"
                , SetGridCalculator_function_type( &::AbstractMigrationRule< 2 >::SetGridCalculator )
                , ( bp::arg("pGrid") ) );
        
        }
        { //::AbstractMigrationRule< 2 >::SetIsSprouting
        
            typedef AbstractMigrationRule< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetIsSprouting_function_type)( bool ) ;
            
            AbstractMigrationRule2_exposer.def( 
                "SetIsSprouting"
                , SetIsSprouting_function_type( &::AbstractMigrationRule< 2 >::SetIsSprouting )
                , ( bp::arg("isSprouting")=(bool)(true) ) );
        
        }
        { //::AbstractMigrationRule< 2 >::SetNetwork
        
            typedef AbstractMigrationRule< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetNetwork_function_type)( ::boost::shared_ptr< VesselNetwork< 2 > > ) ;
            
            AbstractMigrationRule2_exposer.def( 
                "SetNetwork"
                , SetNetwork_function_type( &::AbstractMigrationRule< 2 >::SetNetwork )
                , ( bp::arg("pNetwork") ) );
        
        }
        { //::AbstractMigrationRule< 2 >::SetUseMooreNeighbourhood
        
            typedef AbstractMigrationRule< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetUseMooreNeighbourhood_function_type)( bool ) ;
            
            AbstractMigrationRule2_exposer.def( 
                "SetUseMooreNeighbourhood"
                , SetUseMooreNeighbourhood_function_type( &::AbstractMigrationRule< 2 >::SetUseMooreNeighbourhood )
                , ( bp::arg("useMooreNeighbourhood") ) );
        
        }
        AbstractMigrationRule2_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< AbstractMigrationRule<2> > >();
    }

}