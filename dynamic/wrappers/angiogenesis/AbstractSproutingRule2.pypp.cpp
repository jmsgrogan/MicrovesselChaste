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
#include "AbstractSproutingRule2.pypp.hpp"

namespace bp = boost::python;

struct AbstractSproutingRule_less__2__greater__wrapper : AbstractSproutingRule< 2 >, bp::wrapper< AbstractSproutingRule< 2 > > {

    AbstractSproutingRule_less__2__greater__wrapper(AbstractSproutingRule<2> const & arg )
    : AbstractSproutingRule<2>( arg )
      , bp::wrapper< AbstractSproutingRule< 2 > >(){
        // copy constructor
        
    }

    AbstractSproutingRule_less__2__greater__wrapper( )
    : AbstractSproutingRule<2>( )
      , bp::wrapper< AbstractSproutingRule< 2 > >(){
        // null constructor
    
    }

    virtual ::std::vector< boost::shared_ptr<VesselNode<2> > > GetSprouts( ::std::vector< boost::shared_ptr<VesselNode<2> > > const & rNodes ) {
        if( bp::override func_GetSprouts = this->get_override( "GetSprouts" ) )
            return func_GetSprouts( boost::ref(rNodes) );
        else{
            return this->AbstractSproutingRule< 2 >::GetSprouts( boost::ref(rNodes) );
        }
    }
    
    ::std::vector< boost::shared_ptr<VesselNode<2> > > default_GetSprouts( ::std::vector< boost::shared_ptr<VesselNode<2> > > const & rNodes ) {
        return AbstractSproutingRule< 2 >::GetSprouts( boost::ref(rNodes) );
    }

    virtual void SetGridCalculator( ::boost::shared_ptr< GridCalculator< 2 > > pGrid ) {
        if( bp::override func_SetGridCalculator = this->get_override( "SetGridCalculator" ) )
            func_SetGridCalculator( pGrid );
        else{
            this->AbstractSproutingRule< 2 >::SetGridCalculator( pGrid );
        }
    }
    
    void default_SetGridCalculator( ::boost::shared_ptr< GridCalculator< 2 > > pGrid ) {
        AbstractSproutingRule< 2 >::SetGridCalculator( pGrid );
    }

};

void register_AbstractSproutingRule2_class(){

    { //::AbstractSproutingRule< 2 >
        typedef bp::class_< AbstractSproutingRule_less__2__greater__wrapper > AbstractSproutingRule2_exposer_t;
        AbstractSproutingRule2_exposer_t AbstractSproutingRule2_exposer = AbstractSproutingRule2_exposer_t( "AbstractSproutingRule2", bp::init< >() );
        bp::scope AbstractSproutingRule2_scope( AbstractSproutingRule2_exposer );
        { //::AbstractSproutingRule< 2 >::GetSproutingProbability
        
            typedef AbstractSproutingRule< 2 > exported_class_t;
            typedef ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ( exported_class_t::*GetSproutingProbability_function_type)(  ) ;
            
            AbstractSproutingRule2_exposer.def( 
                "GetSproutingProbability"
                , GetSproutingProbability_function_type( &::AbstractSproutingRule< 2 >::GetSproutingProbability ) );
        
        }
        { //::AbstractSproutingRule< 2 >::GetSprouts
        
            typedef AbstractSproutingRule< 2 > exported_class_t;
            typedef ::std::vector< boost::shared_ptr<VesselNode<2> > > ( exported_class_t::*GetSprouts_function_type)( ::std::vector< boost::shared_ptr<VesselNode<2> > > const & ) ;
            typedef ::std::vector< boost::shared_ptr<VesselNode<2> > > ( AbstractSproutingRule_less__2__greater__wrapper::*default_GetSprouts_function_type)( ::std::vector< boost::shared_ptr<VesselNode<2> > > const & ) ;
            
            AbstractSproutingRule2_exposer.def( 
                "GetSprouts"
                , GetSprouts_function_type(&::AbstractSproutingRule< 2 >::GetSprouts)
                , default_GetSprouts_function_type(&AbstractSproutingRule_less__2__greater__wrapper::default_GetSprouts)
                , ( bp::arg("rNodes") ) );
        
        }
        { //::AbstractSproutingRule< 2 >::SetDiscreteContinuumSolver
        
            typedef AbstractSproutingRule< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetDiscreteContinuumSolver_function_type)( ::boost::shared_ptr< AbstractDiscreteContinuumSolver< 2 > > ) ;
            
            AbstractSproutingRule2_exposer.def( 
                "SetDiscreteContinuumSolver"
                , SetDiscreteContinuumSolver_function_type( &::AbstractSproutingRule< 2 >::SetDiscreteContinuumSolver )
                , ( bp::arg("pSolver") ) );
        
        }
        { //::AbstractSproutingRule< 2 >::SetGridCalculator
        
            typedef AbstractSproutingRule< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetGridCalculator_function_type)( ::boost::shared_ptr< GridCalculator< 2 > > ) ;
            typedef void ( AbstractSproutingRule_less__2__greater__wrapper::*default_SetGridCalculator_function_type)( ::boost::shared_ptr< GridCalculator< 2 > > ) ;
            
            AbstractSproutingRule2_exposer.def( 
                "SetGridCalculator"
                , SetGridCalculator_function_type(&::AbstractSproutingRule< 2 >::SetGridCalculator)
                , default_SetGridCalculator_function_type(&AbstractSproutingRule_less__2__greater__wrapper::default_SetGridCalculator)
                , ( bp::arg("pGrid") ) );
        
        }
        { //::AbstractSproutingRule< 2 >::SetOnlySproutIfPerfused
        
            typedef AbstractSproutingRule< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetOnlySproutIfPerfused_function_type)( bool ) ;
            
            AbstractSproutingRule2_exposer.def( 
                "SetOnlySproutIfPerfused"
                , SetOnlySproutIfPerfused_function_type( &::AbstractSproutingRule< 2 >::SetOnlySproutIfPerfused )
                , ( bp::arg("onlySproutIfPerfused") ) );
        
        }
        { //::AbstractSproutingRule< 2 >::SetSproutingProbability
        
            typedef AbstractSproutingRule< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetSproutingProbability_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            AbstractSproutingRule2_exposer.def( 
                "SetSproutingProbability"
                , SetSproutingProbability_function_type( &::AbstractSproutingRule< 2 >::SetSproutingProbability )
                , ( bp::arg("probability") ) );
        
        }
        { //::AbstractSproutingRule< 2 >::SetVesselEndCutoff
        
            typedef AbstractSproutingRule< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetVesselEndCutoff_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            AbstractSproutingRule2_exposer.def( 
                "SetVesselEndCutoff"
                , SetVesselEndCutoff_function_type( &::AbstractSproutingRule< 2 >::SetVesselEndCutoff )
                , ( bp::arg("cutoff") ) );
        
        }
        { //::AbstractSproutingRule< 2 >::SetVesselNetwork
        
            typedef AbstractSproutingRule< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetVesselNetwork_function_type)( ::boost::shared_ptr< VesselNetwork< 2 > > ) ;
            
            AbstractSproutingRule2_exposer.def( 
                "SetVesselNetwork"
                , SetVesselNetwork_function_type( &::AbstractSproutingRule< 2 >::SetVesselNetwork )
                , ( bp::arg("pVesselNetwork") ) );
        
        }
        bp::register_ptr_to_python< boost::shared_ptr< AbstractSproutingRule<2> > >();
    }

}
