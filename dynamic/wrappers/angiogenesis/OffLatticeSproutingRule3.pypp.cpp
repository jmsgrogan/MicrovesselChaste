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
#include "OffLatticeSproutingRule3.pypp.hpp"

namespace bp = boost::python;

struct OffLatticeSproutingRule_less__3__greater__wrapper : OffLatticeSproutingRule< 3 >, bp::wrapper< OffLatticeSproutingRule< 3 > > {

    OffLatticeSproutingRule_less__3__greater__wrapper(OffLatticeSproutingRule<3> const & arg )
    : OffLatticeSproutingRule<3>( arg )
      , bp::wrapper< OffLatticeSproutingRule< 3 > >(){
        // copy constructor
        
    }

    OffLatticeSproutingRule_less__3__greater__wrapper( )
    : OffLatticeSproutingRule<3>( )
      , bp::wrapper< OffLatticeSproutingRule< 3 > >(){
        // null constructor
    
    }

    virtual ::std::vector< boost::shared_ptr<VesselNode<3> > > GetSprouts( ::std::vector< boost::shared_ptr<VesselNode<3> > > const & rNodes ) {
        if( bp::override func_GetSprouts = this->get_override( "GetSprouts" ) )
            return func_GetSprouts( boost::ref(rNodes) );
        else{
            return this->OffLatticeSproutingRule< 3 >::GetSprouts( boost::ref(rNodes) );
        }
    }
    
    ::std::vector< boost::shared_ptr<VesselNode<3> > > default_GetSprouts( ::std::vector< boost::shared_ptr<VesselNode<3> > > const & rNodes ) {
        return OffLatticeSproutingRule< 3 >::GetSprouts( boost::ref(rNodes) );
    }

    virtual void SetGridCalculator( ::boost::shared_ptr< GridCalculator< 3 > > pGrid ) {
        if( bp::override func_SetGridCalculator = this->get_override( "SetGridCalculator" ) )
            func_SetGridCalculator( pGrid );
        else{
            this->AbstractSproutingRule< 3 >::SetGridCalculator( pGrid );
        }
    }
    
    void default_SetGridCalculator( ::boost::shared_ptr< GridCalculator< 3 > > pGrid ) {
        AbstractSproutingRule< 3 >::SetGridCalculator( pGrid );
    }

};

void register_OffLatticeSproutingRule3_class(){

    { //::OffLatticeSproutingRule< 3 >
        typedef bp::class_< OffLatticeSproutingRule_less__3__greater__wrapper, bp::bases< AbstractSproutingRule< 3 > > > OffLatticeSproutingRule3_exposer_t;
        OffLatticeSproutingRule3_exposer_t OffLatticeSproutingRule3_exposer = OffLatticeSproutingRule3_exposer_t( "OffLatticeSproutingRule3", bp::init< >() );
        bp::scope OffLatticeSproutingRule3_scope( OffLatticeSproutingRule3_exposer );
        { //::OffLatticeSproutingRule< 3 >::Create
        
            typedef OffLatticeSproutingRule< 3 > exported_class_t;
            typedef ::boost::shared_ptr< OffLatticeSproutingRule< 3 > > ( *Create_function_type )(  );
            
            OffLatticeSproutingRule3_exposer.def( 
                "Create"
                , Create_function_type( &::OffLatticeSproutingRule< 3 >::Create ) );
        
        }
        { //::OffLatticeSproutingRule< 3 >::GetSprouts
        
            typedef OffLatticeSproutingRule< 3 > exported_class_t;
            typedef ::std::vector< boost::shared_ptr<VesselNode<3> > > ( exported_class_t::*GetSprouts_function_type)( ::std::vector< boost::shared_ptr<VesselNode<3> > > const & ) ;
            typedef ::std::vector< boost::shared_ptr<VesselNode<3> > > ( OffLatticeSproutingRule_less__3__greater__wrapper::*default_GetSprouts_function_type)( ::std::vector< boost::shared_ptr<VesselNode<3> > > const & ) ;
            
            OffLatticeSproutingRule3_exposer.def( 
                "GetSprouts"
                , GetSprouts_function_type(&::OffLatticeSproutingRule< 3 >::GetSprouts)
                , default_GetSprouts_function_type(&OffLatticeSproutingRule_less__3__greater__wrapper::default_GetSprouts)
                , ( bp::arg("rNodes") ) );
        
        }
        { //::OffLatticeSproutingRule< 3 >::SetTipExclusionRadius
        
            typedef OffLatticeSproutingRule< 3 > exported_class_t;
            typedef void ( exported_class_t::*SetTipExclusionRadius_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            OffLatticeSproutingRule3_exposer.def( 
                "SetTipExclusionRadius"
                , SetTipExclusionRadius_function_type( &::OffLatticeSproutingRule< 3 >::SetTipExclusionRadius )
                , ( bp::arg("exclusionRadius") ) );
        
        }
        { //::AbstractSproutingRule< 3 >::SetGridCalculator
        
            typedef OffLatticeSproutingRule< 3 > exported_class_t;
            typedef void ( exported_class_t::*SetGridCalculator_function_type)( ::boost::shared_ptr< GridCalculator< 3 > > ) ;
            typedef void ( OffLatticeSproutingRule_less__3__greater__wrapper::*default_SetGridCalculator_function_type)( ::boost::shared_ptr< GridCalculator< 3 > > ) ;
            
            OffLatticeSproutingRule3_exposer.def( 
                "SetGridCalculator"
                , SetGridCalculator_function_type(&::AbstractSproutingRule< 3 >::SetGridCalculator)
                , default_SetGridCalculator_function_type(&OffLatticeSproutingRule_less__3__greater__wrapper::default_SetGridCalculator)
                , ( bp::arg("pGrid") ) );
        
        }
        OffLatticeSproutingRule3_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< OffLatticeSproutingRule<3> > >();
        bp::implicitly_convertible< boost::shared_ptr< OffLatticeSproutingRule< 3 > >, boost::shared_ptr< AbstractSproutingRule< 3 > > >();
    }

}