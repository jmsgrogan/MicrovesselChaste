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
#include "VesselSegment2.pypp.hpp"

namespace bp = boost::python;

struct VesselSegment_less__2__greater__wrapper : VesselSegment< 2 >, bp::wrapper< VesselSegment< 2 > > {

    VesselSegment_less__2__greater__wrapper(::VesselSegment< 2 > const & rSegment )
    : VesselSegment<2>( boost::ref(rSegment) )
      , bp::wrapper< VesselSegment< 2 > >(){
        // copy constructor
    
    }

    virtual ::std::map< std::string, double > GetOutputData(  ) {
        if( bp::override func_GetOutputData = this->get_override( "GetOutputData" ) )
            return func_GetOutputData(  );
        else{
            return this->VesselSegment< 2 >::GetOutputData(  );
        }
    }
    
    ::std::map< std::string, double > default_GetOutputData(  ) {
        return VesselSegment< 2 >::GetOutputData( );
    }

    virtual unsigned int GetId(  ) const  {
        if( bp::override func_GetId = this->get_override( "GetId" ) )
            return func_GetId(  );
        else{
            return this->AbstractVesselNetworkComponent< 2 >::GetId(  );
        }
    }
    
    unsigned int default_GetId(  ) const  {
        return AbstractVesselNetworkComponent< 2 >::GetId( );
    }

    virtual ::std::vector< std::string > GetOutputDataKeys(  ) {
        if( bp::override func_GetOutputDataKeys = this->get_override( "GetOutputDataKeys" ) )
            return func_GetOutputDataKeys(  );
        else{
            return this->AbstractVesselNetworkComponent< 2 >::GetOutputDataKeys(  );
        }
    }
    
    ::std::vector< std::string > default_GetOutputDataKeys(  ) {
        return AbstractVesselNetworkComponent< 2 >::GetOutputDataKeys( );
    }

    virtual double GetOutputDataValue( ::std::string const & rKey ) {
        if( bp::override func_GetOutputDataValue = this->get_override( "GetOutputDataValue" ) )
            return func_GetOutputDataValue( rKey );
        else{
            return this->AbstractVesselNetworkComponent< 2 >::GetOutputDataValue( rKey );
        }
    }
    
    double default_GetOutputDataValue( ::std::string const & rKey ) {
        return AbstractVesselNetworkComponent< 2 >::GetOutputDataValue( rKey );
    }

    virtual ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > GetRadius(  ) const  {
        if( bp::override func_GetRadius = this->get_override( "GetRadius" ) )
            return func_GetRadius(  );
        else{
            return this->AbstractVesselNetworkComponent< 2 >::GetRadius(  );
        }
    }
    
    ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > default_GetRadius(  ) const  {
        return AbstractVesselNetworkComponent< 2 >::GetRadius( );
    }

    virtual void SetId( unsigned int id ) {
        if( bp::override func_SetId = this->get_override( "SetId" ) )
            func_SetId( id );
        else{
            this->AbstractVesselNetworkComponent< 2 >::SetId( id );
        }
    }
    
    void default_SetId( unsigned int id ) {
        AbstractVesselNetworkComponent< 2 >::SetId( id );
    }

    virtual void SetOutputData( ::std::string const & rKey, double value ) {
        if( bp::override func_SetOutputData = this->get_override( "SetOutputData" ) )
            func_SetOutputData( rKey, value );
        else{
            this->AbstractVesselNetworkComponent< 2 >::SetOutputData( rKey, value );
        }
    }
    
    void default_SetOutputData( ::std::string const & rKey, double value ) {
        AbstractVesselNetworkComponent< 2 >::SetOutputData( rKey, value );
    }

    virtual void SetRadius( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > radius ) {
        if( bp::override func_SetRadius = this->get_override( "SetRadius" ) )
            func_SetRadius( radius );
        else{
            this->AbstractVesselNetworkComponent< 2 >::SetRadius( radius );
        }
    }
    
    void default_SetRadius( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > radius ) {
        AbstractVesselNetworkComponent< 2 >::SetRadius( radius );
    }

};

void register_VesselSegment2_class(){

    { //::VesselSegment< 2 >
        typedef bp::class_< VesselSegment_less__2__greater__wrapper, bp::bases< AbstractVesselNetworkComponent< 2 > >, boost::shared_ptr< VesselSegment<2> > > VesselSegment2_exposer_t;
        VesselSegment2_exposer_t VesselSegment2_exposer = VesselSegment2_exposer_t( "VesselSegment2", bp::no_init );
        bp::scope VesselSegment2_scope( VesselSegment2_exposer );
        VesselSegment2_exposer.def( bp::init< VesselSegment< 2 > const & >(( bp::arg("rSegment") )) );
        { //::VesselSegment< 2 >::CopyDataFromExistingSegment
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef void ( exported_class_t::*CopyDataFromExistingSegment_function_type)( ::boost::shared_ptr< VesselSegment< 2 > > const ) ;
            
            VesselSegment2_exposer.def( 
                "CopyDataFromExistingSegment"
                , CopyDataFromExistingSegment_function_type( &::VesselSegment< 2 >::CopyDataFromExistingSegment )
                , ( bp::arg("pTargetSegment") ) );
        
        }
        { //::VesselSegment< 2 >::Create
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef ::boost::shared_ptr< VesselSegment< 2 > > ( *Create_function_type )( ::boost::shared_ptr< VesselNode< 2 > >,::boost::shared_ptr< VesselNode< 2 > > );
            
            VesselSegment2_exposer.def( 
                "Create"
                , Create_function_type( &::VesselSegment< 2 >::Create )
                , ( bp::arg("pNode1"), bp::arg("pNode2") ) );
        
        }
        { //::VesselSegment< 2 >::Create
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef ::boost::shared_ptr< VesselSegment< 2 > > ( *Create_function_type )( ::boost::shared_ptr< VesselSegment< 2 > > );
            
            VesselSegment2_exposer.def( 
                "Create"
                , Create_function_type( &::VesselSegment< 2 >::Create )
                , ( bp::arg("pSegment") ) );
        
        }
        { //::VesselSegment< 2 >::GetDistance
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ( exported_class_t::*GetDistance_function_type)( ::DimensionalChastePoint< 2 > const & ) const;
            
            VesselSegment2_exposer.def( 
                "GetDistance"
                , GetDistance_function_type( &::VesselSegment< 2 >::GetDistance )
                , ( bp::arg("location") ) );
        
        }
        { //::VesselSegment< 2 >::GetFlowProperties
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef ::boost::shared_ptr< SegmentFlowProperties< 2 > > ( exported_class_t::*GetFlowProperties_function_type)(  ) const;
            
            VesselSegment2_exposer.def( 
                "GetFlowProperties"
                , GetFlowProperties_function_type( &::VesselSegment< 2 >::GetFlowProperties ) );
        
        }
        { //::VesselSegment< 2 >::GetLength
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ( exported_class_t::*GetLength_function_type)(  ) const;
            
            VesselSegment2_exposer.def( 
                "GetLength"
                , GetLength_function_type( &::VesselSegment< 2 >::GetLength ) );
        
        }
        { //::VesselSegment< 2 >::GetMaturity
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef double ( exported_class_t::*GetMaturity_function_type)(  ) const;
            
            VesselSegment2_exposer.def( 
                "GetMaturity"
                , GetMaturity_function_type( &::VesselSegment< 2 >::GetMaturity ) );
        
        }
        { //::VesselSegment< 2 >::GetMidPoint
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef ::DimensionalChastePoint< 2 > ( exported_class_t::*GetMidPoint_function_type)(  ) const;
            
            VesselSegment2_exposer.def( 
                "GetMidPoint"
                , GetMidPoint_function_type( &::VesselSegment< 2 >::GetMidPoint ) );
        
        }
        { //::VesselSegment< 2 >::GetNode
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef ::boost::shared_ptr< VesselNode< 2 > > ( exported_class_t::*GetNode_function_type)( unsigned int ) const;
            
            VesselSegment2_exposer.def( 
                "GetNode"
                , GetNode_function_type( &::VesselSegment< 2 >::GetNode )
                , ( bp::arg("index") ) );
        
        }
        { //::VesselSegment< 2 >::GetNodes
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef ::std::pair< boost::shared_ptr< VesselNode< 2 > >, boost::shared_ptr< VesselNode< 2 > > > ( exported_class_t::*GetNodes_function_type)(  ) const;
            
            VesselSegment2_exposer.def( 
                "GetNodes"
                , GetNodes_function_type( &::VesselSegment< 2 >::GetNodes ) );
        
        }
        { //::VesselSegment< 2 >::GetOppositeNode
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef ::boost::shared_ptr< VesselNode< 2 > > ( exported_class_t::*GetOppositeNode_function_type)( ::boost::shared_ptr< VesselNode< 2 > > ) const;
            
            VesselSegment2_exposer.def( 
                "GetOppositeNode"
                , GetOppositeNode_function_type( &::VesselSegment< 2 >::GetOppositeNode )
                , ( bp::arg("pInputNode") ) );
        
        }
        { //::VesselSegment< 2 >::GetOutputData
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef ::std::map< std::string, double > ( exported_class_t::*GetOutputData_function_type)(  ) ;
            typedef ::std::map< std::string, double > ( VesselSegment_less__2__greater__wrapper::*default_GetOutputData_function_type)(  ) ;
            
            VesselSegment2_exposer.def( 
                "GetOutputData"
                , GetOutputData_function_type(&::VesselSegment< 2 >::GetOutputData)
                , default_GetOutputData_function_type(&VesselSegment_less__2__greater__wrapper::default_GetOutputData) );
        
        }
        { //::VesselSegment< 2 >::GetPointProjection
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef ::DimensionalChastePoint< 2 > ( exported_class_t::*GetPointProjection_function_type)( ::DimensionalChastePoint< 2 > const &,bool ) const;
            
            VesselSegment2_exposer.def( 
                "GetPointProjection"
                , GetPointProjection_function_type( &::VesselSegment< 2 >::GetPointProjection )
                , ( bp::arg("location"), bp::arg("projectToEnds")=(bool)(false) ) );
        
        }
        { //::VesselSegment< 2 >::GetUnitTangent
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef ::boost::numeric::ublas::c_vector< double, 2 > ( exported_class_t::*GetUnitTangent_function_type)(  ) const;
            
            VesselSegment2_exposer.def( 
                "GetUnitTangent"
                , GetUnitTangent_function_type( &::VesselSegment< 2 >::GetUnitTangent ) );
        
        }
        { //::VesselSegment< 2 >::GetVessel
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef ::boost::shared_ptr< Vessel< 2 > > ( exported_class_t::*GetVessel_function_type)(  ) const;
            
            VesselSegment2_exposer.def( 
                "GetVessel"
                , GetVessel_function_type( &::VesselSegment< 2 >::GetVessel ) );
        
        }
        { //::VesselSegment< 2 >::HasNode
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef bool ( exported_class_t::*HasNode_function_type)( ::boost::shared_ptr< VesselNode< 2 > > ) const;
            
            VesselSegment2_exposer.def( 
                "HasNode"
                , HasNode_function_type( &::VesselSegment< 2 >::HasNode )
                , ( bp::arg("pNode") ) );
        
        }
        { //::VesselSegment< 2 >::IsConnectedTo
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef bool ( exported_class_t::*IsConnectedTo_function_type)( ::boost::shared_ptr< VesselSegment< 2 > > ) const;
            
            VesselSegment2_exposer.def( 
                "IsConnectedTo"
                , IsConnectedTo_function_type( &::VesselSegment< 2 >::IsConnectedTo )
                , ( bp::arg("pOtherSegment") ) );
        
        }
        { //::VesselSegment< 2 >::Remove
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef void ( exported_class_t::*Remove_function_type)(  ) ;
            
            VesselSegment2_exposer.def( 
                "Remove"
                , Remove_function_type( &::VesselSegment< 2 >::Remove ) );
        
        }
        { //::VesselSegment< 2 >::ReplaceNode
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef void ( exported_class_t::*ReplaceNode_function_type)( unsigned int,::boost::shared_ptr< VesselNode< 2 > > ) ;
            
            VesselSegment2_exposer.def( 
                "ReplaceNode"
                , ReplaceNode_function_type( &::VesselSegment< 2 >::ReplaceNode )
                , ( bp::arg("oldNodeIndex"), bp::arg("pNewNode") ) );
        
        }
        { //::VesselSegment< 2 >::SetFlowProperties
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetFlowProperties_function_type)( ::SegmentFlowProperties< 2 > const & ) ;
            
            VesselSegment2_exposer.def( 
                "SetFlowProperties"
                , SetFlowProperties_function_type( &::VesselSegment< 2 >::SetFlowProperties )
                , ( bp::arg("rFlowProperties") ) );
        
        }
        { //::VesselSegment< 2 >::SetMaturity
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetMaturity_function_type)( double ) ;
            
            VesselSegment2_exposer.def( 
                "SetMaturity"
                , SetMaturity_function_type( &::VesselSegment< 2 >::SetMaturity )
                , ( bp::arg("maturity") ) );
        
        }
        { //::AbstractVesselNetworkComponent< 2 >::GetId
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef unsigned int ( exported_class_t::*GetId_function_type)(  ) const;
            typedef unsigned int ( VesselSegment_less__2__greater__wrapper::*default_GetId_function_type)(  ) const;
            
            VesselSegment2_exposer.def( 
                "GetId"
                , GetId_function_type(&::AbstractVesselNetworkComponent< 2 >::GetId)
                , default_GetId_function_type(&VesselSegment_less__2__greater__wrapper::default_GetId) );
        
        }
        { //::AbstractVesselNetworkComponent< 2 >::GetOutputDataKeys
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef ::std::vector< std::string > ( exported_class_t::*GetOutputDataKeys_function_type)(  ) ;
            typedef ::std::vector< std::string > ( VesselSegment_less__2__greater__wrapper::*default_GetOutputDataKeys_function_type)(  ) ;
            
            VesselSegment2_exposer.def( 
                "GetOutputDataKeys"
                , GetOutputDataKeys_function_type(&::AbstractVesselNetworkComponent< 2 >::GetOutputDataKeys)
                , default_GetOutputDataKeys_function_type(&VesselSegment_less__2__greater__wrapper::default_GetOutputDataKeys) );
        
        }
        { //::AbstractVesselNetworkComponent< 2 >::GetOutputDataValue
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef double ( exported_class_t::*GetOutputDataValue_function_type)( ::std::string const & ) ;
            typedef double ( VesselSegment_less__2__greater__wrapper::*default_GetOutputDataValue_function_type)( ::std::string const & ) ;
            
            VesselSegment2_exposer.def( 
                "GetOutputDataValue"
                , GetOutputDataValue_function_type(&::AbstractVesselNetworkComponent< 2 >::GetOutputDataValue)
                , default_GetOutputDataValue_function_type(&VesselSegment_less__2__greater__wrapper::default_GetOutputDataValue)
                , ( bp::arg("rKey") ) );
        
        }
        { //::AbstractVesselNetworkComponent< 2 >::GetRadius
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ( exported_class_t::*GetRadius_function_type)(  ) const;
            typedef ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ( VesselSegment_less__2__greater__wrapper::*default_GetRadius_function_type)(  ) const;
            
            VesselSegment2_exposer.def( 
                "GetRadius"
                , GetRadius_function_type(&::AbstractVesselNetworkComponent< 2 >::GetRadius)
                , default_GetRadius_function_type(&VesselSegment_less__2__greater__wrapper::default_GetRadius) );
        
        }
        { //::AbstractVesselNetworkComponent< 2 >::SetId
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetId_function_type)( unsigned int ) ;
            typedef void ( VesselSegment_less__2__greater__wrapper::*default_SetId_function_type)( unsigned int ) ;
            
            VesselSegment2_exposer.def( 
                "SetId"
                , SetId_function_type(&::AbstractVesselNetworkComponent< 2 >::SetId)
                , default_SetId_function_type(&VesselSegment_less__2__greater__wrapper::default_SetId)
                , ( bp::arg("id") ) );
        
        }
        { //::AbstractVesselNetworkComponent< 2 >::SetOutputData
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetOutputData_function_type)( ::std::string const &,double ) ;
            typedef void ( VesselSegment_less__2__greater__wrapper::*default_SetOutputData_function_type)( ::std::string const &,double ) ;
            
            VesselSegment2_exposer.def( 
                "SetOutputData"
                , SetOutputData_function_type(&::AbstractVesselNetworkComponent< 2 >::SetOutputData)
                , default_SetOutputData_function_type(&VesselSegment_less__2__greater__wrapper::default_SetOutputData)
                , ( bp::arg("rKey"), bp::arg("value") ) );
        
        }
        { //::AbstractVesselNetworkComponent< 2 >::SetRadius
        
            typedef VesselSegment< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetRadius_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            typedef void ( VesselSegment_less__2__greater__wrapper::*default_SetRadius_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            VesselSegment2_exposer.def( 
                "SetRadius"
                , SetRadius_function_type(&::AbstractVesselNetworkComponent< 2 >::SetRadius)
                , default_SetRadius_function_type(&VesselSegment_less__2__greater__wrapper::default_SetRadius)
                , ( bp::arg("radius") ) );
        
        }
        VesselSegment2_exposer.staticmethod( "Create" );
        bp::implicitly_convertible< boost::shared_ptr< VesselSegment< 2 > >, boost::shared_ptr< AbstractVesselNetworkComponent< 2 > > >();
        bp::implicitly_convertible< boost::shared_ptr< VesselSegment< 2 > >, boost::shared_ptr< boost::enable_shared_from_this< VesselSegment< 2 > > > >();
    }

}