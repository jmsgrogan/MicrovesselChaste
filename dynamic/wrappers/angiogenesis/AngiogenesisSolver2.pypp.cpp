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
#include "AngiogenesisSolver2.pypp.hpp"

namespace bp = boost::python;

struct AngiogenesisSolver_less__2__greater__wrapper : AngiogenesisSolver< 2 >, bp::wrapper< AngiogenesisSolver< 2 > > {

    AngiogenesisSolver_less__2__greater__wrapper(AngiogenesisSolver<2> const & arg )
    : AngiogenesisSolver<2>( arg )
      , bp::wrapper< AngiogenesisSolver< 2 > >(){
        // copy constructor
        
    }

    AngiogenesisSolver_less__2__greater__wrapper( )
    : AngiogenesisSolver<2>( )
      , bp::wrapper< AngiogenesisSolver< 2 > >(){
        // null constructor
    
    }

    virtual void DoAnastamosis(  ){
        if( bp::override func_DoAnastamosis = this->get_override( "DoAnastamosis" ) )
            func_DoAnastamosis(  );
        else{
            this->AngiogenesisSolver< 2 >::DoAnastamosis(  );
        }
    }
    
    virtual void default_DoAnastamosis(  ){
        AngiogenesisSolver< 2 >::DoAnastamosis( );
    }

    virtual void DoSprouting(  ){
        if( bp::override func_DoSprouting = this->get_override( "DoSprouting" ) )
            func_DoSprouting(  );
        else{
            this->AngiogenesisSolver< 2 >::DoSprouting(  );
        }
    }
    
    virtual void default_DoSprouting(  ){
        AngiogenesisSolver< 2 >::DoSprouting( );
    }

    virtual void Increment(  ) {
        if( bp::override func_Increment = this->get_override( "Increment" ) )
            func_Increment(  );
        else{
            this->AngiogenesisSolver< 2 >::Increment(  );
        }
    }
    
    void default_Increment(  ) {
        AngiogenesisSolver< 2 >::Increment( );
    }

    virtual void UpdateNodalPositions( bool sprouting=false ){
        if( bp::override func_UpdateNodalPositions = this->get_override( "UpdateNodalPositions" ) )
            func_UpdateNodalPositions( sprouting );
        else{
            this->AngiogenesisSolver< 2 >::UpdateNodalPositions( sprouting );
        }
    }
    
    virtual void default_UpdateNodalPositions( bool sprouting=false ){
        AngiogenesisSolver< 2 >::UpdateNodalPositions( sprouting );
    }

};

void register_AngiogenesisSolver2_class(){

    { //::AngiogenesisSolver< 2 >
        typedef bp::class_< AngiogenesisSolver_less__2__greater__wrapper > AngiogenesisSolver2_exposer_t;
        AngiogenesisSolver2_exposer_t AngiogenesisSolver2_exposer = AngiogenesisSolver2_exposer_t( "AngiogenesisSolver2", bp::init< >() );
        bp::scope AngiogenesisSolver2_scope( AngiogenesisSolver2_exposer );
        { //::AngiogenesisSolver< 2 >::Create
        
            typedef AngiogenesisSolver< 2 > exported_class_t;
            typedef ::boost::shared_ptr< AngiogenesisSolver< 2 > > ( *Create_function_type )(  );
            
            AngiogenesisSolver2_exposer.def( 
                "Create"
                , Create_function_type( &::AngiogenesisSolver< 2 >::Create ) );
        
        }
        { //::AngiogenesisSolver< 2 >::DoAnastamosis
        
            typedef AngiogenesisSolver< 2 > exported_class_t;
            typedef void ( AngiogenesisSolver_less__2__greater__wrapper::*DoAnastamosis_function_type)(  ) ;
            
            AngiogenesisSolver2_exposer.def( 
                "DoAnastamosis"
                , DoAnastamosis_function_type( &AngiogenesisSolver_less__2__greater__wrapper::default_DoAnastamosis ) );
        
        }
        { //::AngiogenesisSolver< 2 >::DoSprouting
        
            typedef AngiogenesisSolver< 2 > exported_class_t;
            typedef void ( AngiogenesisSolver_less__2__greater__wrapper::*DoSprouting_function_type)(  ) ;
            
            AngiogenesisSolver2_exposer.def( 
                "DoSprouting"
                , DoSprouting_function_type( &AngiogenesisSolver_less__2__greater__wrapper::default_DoSprouting ) );
        
        }
        { //::AngiogenesisSolver< 2 >::Increment
        
            typedef AngiogenesisSolver< 2 > exported_class_t;
            typedef void ( exported_class_t::*Increment_function_type)(  ) ;
            typedef void ( AngiogenesisSolver_less__2__greater__wrapper::*default_Increment_function_type)(  ) ;
            
            AngiogenesisSolver2_exposer.def( 
                "Increment"
                , Increment_function_type(&::AngiogenesisSolver< 2 >::Increment)
                , default_Increment_function_type(&AngiogenesisSolver_less__2__greater__wrapper::default_Increment) );
        
        }
        { //::AngiogenesisSolver< 2 >::IsSproutingRuleSet
        
            typedef AngiogenesisSolver< 2 > exported_class_t;
            typedef bool ( exported_class_t::*IsSproutingRuleSet_function_type)(  ) ;
            
            AngiogenesisSolver2_exposer.def( 
                "IsSproutingRuleSet"
                , IsSproutingRuleSet_function_type( &::AngiogenesisSolver< 2 >::IsSproutingRuleSet ) );
        
        }
        { //::AngiogenesisSolver< 2 >::Run
        
            typedef AngiogenesisSolver< 2 > exported_class_t;
            typedef void ( exported_class_t::*Run_function_type)( bool ) ;
            
            AngiogenesisSolver2_exposer.def( 
                "Run"
                , Run_function_type( &::AngiogenesisSolver< 2 >::Run )
                , ( bp::arg("writeOutput")=(bool)(false) ) );
        
        }
        { //::AngiogenesisSolver< 2 >::SetAnastamosisRadius
        
            typedef AngiogenesisSolver< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetAnastamosisRadius_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            AngiogenesisSolver2_exposer.def( 
                "SetAnastamosisRadius"
                , SetAnastamosisRadius_function_type( &::AngiogenesisSolver< 2 >::SetAnastamosisRadius )
                , ( bp::arg("radius") ) );
        
        }
        { //::AngiogenesisSolver< 2 >::SetBoundingDomain
        
            typedef AngiogenesisSolver< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetBoundingDomain_function_type)( ::boost::shared_ptr< Part< 2 > > ) ;
            
            AngiogenesisSolver2_exposer.def( 
                "SetBoundingDomain"
                , SetBoundingDomain_function_type( &::AngiogenesisSolver< 2 >::SetBoundingDomain )
                , ( bp::arg("pDomain") ) );
        
        }
        { //::AngiogenesisSolver< 2 >::SetCellPopulation
        
            typedef AngiogenesisSolver< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetCellPopulation_function_type)( ::boost::shared_ptr< AbstractCellPopulation< 2, 2 > >,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            AngiogenesisSolver2_exposer.def( 
                "SetCellPopulation"
                , SetCellPopulation_function_type( &::AngiogenesisSolver< 2 >::SetCellPopulation )
                , ( bp::arg("pCellPopulation"), bp::arg("cellPopulationReferenceLength") ) );
        
        }
        { //::AngiogenesisSolver< 2 >::SetDoAnastomosis
        
            typedef AngiogenesisSolver< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetDoAnastomosis_function_type)( bool ) ;
            
            AngiogenesisSolver2_exposer.def( 
                "SetDoAnastomosis"
                , SetDoAnastomosis_function_type( &::AngiogenesisSolver< 2 >::SetDoAnastomosis )
                , ( bp::arg("doAnastomosis") ) );
        
        }
        { //::AngiogenesisSolver< 2 >::SetMigrationRule
        
            typedef AngiogenesisSolver< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetMigrationRule_function_type)( ::boost::shared_ptr< AbstractMigrationRule< 2 > > ) ;
            
            AngiogenesisSolver2_exposer.def( 
                "SetMigrationRule"
                , SetMigrationRule_function_type( &::AngiogenesisSolver< 2 >::SetMigrationRule )
                , ( bp::arg("pMigrationRule") ) );
        
        }
        { //::AngiogenesisSolver< 2 >::SetOutputFileHandler
        
            typedef AngiogenesisSolver< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetOutputFileHandler_function_type)( ::boost::shared_ptr< OutputFileHandler > ) ;
            
            AngiogenesisSolver2_exposer.def( 
                "SetOutputFileHandler"
                , SetOutputFileHandler_function_type( &::AngiogenesisSolver< 2 >::SetOutputFileHandler )
                , ( bp::arg("pHandler") ) );
        
        }
        { //::AngiogenesisSolver< 2 >::SetSproutingRule
        
            typedef AngiogenesisSolver< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetSproutingRule_function_type)( ::boost::shared_ptr< AbstractSproutingRule< 2 > > ) ;
            
            AngiogenesisSolver2_exposer.def( 
                "SetSproutingRule"
                , SetSproutingRule_function_type( &::AngiogenesisSolver< 2 >::SetSproutingRule )
                , ( bp::arg("pSproutingRule") ) );
        
        }
        { //::AngiogenesisSolver< 2 >::SetVesselGridCalculator
        
            typedef AngiogenesisSolver< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetVesselGridCalculator_function_type)( ::boost::shared_ptr< GridCalculator< 2 > > ) ;
            
            AngiogenesisSolver2_exposer.def( 
                "SetVesselGridCalculator"
                , SetVesselGridCalculator_function_type( &::AngiogenesisSolver< 2 >::SetVesselGridCalculator )
                , ( bp::arg("pVesselGrid") ) );
        
        }
        { //::AngiogenesisSolver< 2 >::SetVesselNetwork
        
            typedef AngiogenesisSolver< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetVesselNetwork_function_type)( ::boost::shared_ptr< VesselNetwork< 2 > > ) ;
            
            AngiogenesisSolver2_exposer.def( 
                "SetVesselNetwork"
                , SetVesselNetwork_function_type( &::AngiogenesisSolver< 2 >::SetVesselNetwork )
                , ( bp::arg("pNetwork") ) );
        
        }
        { //::AngiogenesisSolver< 2 >::UpdateNodalPositions
        
            typedef AngiogenesisSolver< 2 > exported_class_t;
            typedef void ( AngiogenesisSolver_less__2__greater__wrapper::*UpdateNodalPositions_function_type)( bool ) ;
            
            AngiogenesisSolver2_exposer.def( 
                "UpdateNodalPositions"
                , UpdateNodalPositions_function_type( &AngiogenesisSolver_less__2__greater__wrapper::default_UpdateNodalPositions )
                , ( bp::arg("sprouting")=(bool)(false) ) );
        
        }
        AngiogenesisSolver2_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< AngiogenesisSolver<2> > >();
    }

}
