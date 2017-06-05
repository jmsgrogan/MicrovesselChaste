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
#include "VesselNetworkPropertyManager3.pypp.hpp"

namespace bp = boost::python;

void register_VesselNetworkPropertyManager3_class(){

    { //::VesselNetworkPropertyManager< 3 >
        typedef bp::class_< VesselNetworkPropertyManager< 3 > > VesselNetworkPropertyManager3_exposer_t;
        VesselNetworkPropertyManager3_exposer_t VesselNetworkPropertyManager3_exposer = VesselNetworkPropertyManager3_exposer_t( "VesselNetworkPropertyManager3", bp::init< >() );
        bp::scope VesselNetworkPropertyManager3_scope( VesselNetworkPropertyManager3_exposer );
        { //::VesselNetworkPropertyManager< 3 >::AssignInflows
        
            typedef VesselNetworkPropertyManager< 3 > exported_class_t;
            typedef void ( *AssignInflows_function_type )( ::boost::shared_ptr< VesselNetwork< 3 > >,::DimensionalChastePoint< 3 >,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > );
            
            VesselNetworkPropertyManager3_exposer.def( 
                "AssignInflows"
                , AssignInflows_function_type( &::VesselNetworkPropertyManager< 3 >::AssignInflows )
                , ( bp::arg("pNetwork"), bp::arg("location"), bp::arg("searchRadius") ) );
        
        }
        { //::VesselNetworkPropertyManager< 3 >::AssignOutflows
        
            typedef VesselNetworkPropertyManager< 3 > exported_class_t;
            typedef void ( *AssignOutflows_function_type )( ::boost::shared_ptr< VesselNetwork< 3 > >,::DimensionalChastePoint< 3 >,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > );
            
            VesselNetworkPropertyManager3_exposer.def( 
                "AssignOutflows"
                , AssignOutflows_function_type( &::VesselNetworkPropertyManager< 3 >::AssignOutflows )
                , ( bp::arg("pNetwork"), bp::arg("location"), bp::arg("searchRadius") ) );
        
        }
        { //::VesselNetworkPropertyManager< 3 >::CopySegmentFlowProperties
        
            typedef VesselNetworkPropertyManager< 3 > exported_class_t;
            typedef void ( *CopySegmentFlowProperties_function_type )( ::boost::shared_ptr< VesselNetwork< 3 > >,unsigned int );
            
            VesselNetworkPropertyManager3_exposer.def( 
                "CopySegmentFlowProperties"
                , CopySegmentFlowProperties_function_type( &::VesselNetworkPropertyManager< 3 >::CopySegmentFlowProperties )
                , ( bp::arg("pNetwork"), bp::arg("index")=(unsigned int)(0) ) );
        
        }
        { //::VesselNetworkPropertyManager< 3 >::Create
        
            typedef VesselNetworkPropertyManager< 3 > exported_class_t;
            typedef ::boost::shared_ptr< VesselNetworkPropertyManager< 3 > > ( *Create_function_type )(  );
            
            VesselNetworkPropertyManager3_exposer.def( 
                "Create"
                , Create_function_type( &::VesselNetworkPropertyManager< 3 >::Create ) );
        
        }
        { //::VesselNetworkPropertyManager< 3 >::GetInflowNodes
        
            typedef VesselNetworkPropertyManager< 3 > exported_class_t;
            typedef ::std::vector< boost::shared_ptr<VesselNode<3> > > ( *GetInflowNodes_function_type )( ::boost::shared_ptr< VesselNetwork< 3 > > );
            
            VesselNetworkPropertyManager3_exposer.def( 
                "GetInflowNodes"
                , GetInflowNodes_function_type( &::VesselNetworkPropertyManager< 3 >::GetInflowNodes )
                , ( bp::arg("pNetwork") ) );
        
        }
        { //::VesselNetworkPropertyManager< 3 >::GetOutflowNodes
        
            typedef VesselNetworkPropertyManager< 3 > exported_class_t;
            typedef ::std::vector< boost::shared_ptr<VesselNode<3> > > ( *GetOutflowNodes_function_type )( ::boost::shared_ptr< VesselNetwork< 3 > > );
            
            VesselNetworkPropertyManager3_exposer.def( 
                "GetOutflowNodes"
                , GetOutflowNodes_function_type( &::VesselNetworkPropertyManager< 3 >::GetOutflowNodes )
                , ( bp::arg("pNetwork") ) );
        
        }
        { //::VesselNetworkPropertyManager< 3 >::SetInflowPressures
        
            typedef VesselNetworkPropertyManager< 3 > exported_class_t;
            typedef void ( *SetInflowPressures_function_type )( ::boost::shared_ptr< VesselNetwork< 3 > >,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::list< boost::units::dim< boost::units::mass_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -2, 1 > >, boost::units::dimensionless_type > > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > );
            
            VesselNetworkPropertyManager3_exposer.def( 
                "SetInflowPressures"
                , SetInflowPressures_function_type( &::VesselNetworkPropertyManager< 3 >::SetInflowPressures )
                , ( bp::arg("pNetwork"), bp::arg("pressure") ) );
        
        }
        { //::VesselNetworkPropertyManager< 3 >::SetNodeRadii
        
            typedef VesselNetworkPropertyManager< 3 > exported_class_t;
            typedef void ( *SetNodeRadii_function_type )( ::boost::shared_ptr< VesselNetwork< 3 > >,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > );
            
            VesselNetworkPropertyManager3_exposer.def( 
                "SetNodeRadii"
                , SetNodeRadii_function_type( &::VesselNetworkPropertyManager< 3 >::SetNodeRadii )
                , ( bp::arg("pNetwork"), bp::arg("radius") ) );
        
        }
        { //::VesselNetworkPropertyManager< 3 >::SetNodeRadiiFromSegments
        
            typedef VesselNetworkPropertyManager< 3 > exported_class_t;
            typedef void ( *SetNodeRadiiFromSegments_function_type )( ::boost::shared_ptr< VesselNetwork< 3 > > );
            
            VesselNetworkPropertyManager3_exposer.def( 
                "SetNodeRadiiFromSegments"
                , SetNodeRadiiFromSegments_function_type( &::VesselNetworkPropertyManager< 3 >::SetNodeRadiiFromSegments )
                , ( bp::arg("pNetwork") ) );
        
        }
        { //::VesselNetworkPropertyManager< 3 >::SetOutflowPressures
        
            typedef VesselNetworkPropertyManager< 3 > exported_class_t;
            typedef void ( *SetOutflowPressures_function_type )( ::boost::shared_ptr< VesselNetwork< 3 > >,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::list< boost::units::dim< boost::units::mass_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -2, 1 > >, boost::units::dimensionless_type > > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > );
            
            VesselNetworkPropertyManager3_exposer.def( 
                "SetOutflowPressures"
                , SetOutflowPressures_function_type( &::VesselNetworkPropertyManager< 3 >::SetOutflowPressures )
                , ( bp::arg("pNetwork"), bp::arg("pressure") ) );
        
        }
        { //::VesselNetworkPropertyManager< 3 >::SetSegmentProperties
        
            typedef VesselNetworkPropertyManager< 3 > exported_class_t;
            typedef void ( *SetSegmentProperties_function_type )( ::boost::shared_ptr< VesselNetwork< 3 > >,::boost::shared_ptr< VesselSegment< 3 > > );
            
            VesselNetworkPropertyManager3_exposer.def( 
                "SetSegmentProperties"
                , SetSegmentProperties_function_type( &::VesselNetworkPropertyManager< 3 >::SetSegmentProperties )
                , ( bp::arg("pNetwork"), bp::arg("prototype") ) );
        
        }
        { //::VesselNetworkPropertyManager< 3 >::SetSegmentRadii
        
            typedef VesselNetworkPropertyManager< 3 > exported_class_t;
            typedef void ( *SetSegmentRadii_function_type )( ::boost::shared_ptr< VesselNetwork< 3 > >,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > );
            
            VesselNetworkPropertyManager3_exposer.def( 
                "SetSegmentRadii"
                , SetSegmentRadii_function_type( &::VesselNetworkPropertyManager< 3 >::SetSegmentRadii )
                , ( bp::arg("pNetwork"), bp::arg("radius") ) );
        
        }
        { //::VesselNetworkPropertyManager< 3 >::SetSegmentViscosity
        
            typedef VesselNetworkPropertyManager< 3 > exported_class_t;
            typedef void ( *SetSegmentViscosity_function_type )( ::boost::shared_ptr< VesselNetwork< 3 > >,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::list< boost::units::dim< boost::units::mass_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< -1, 1 > >, boost::units::dimensionless_type > > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > );
            
            VesselNetworkPropertyManager3_exposer.def( 
                "SetSegmentViscosity"
                , SetSegmentViscosity_function_type( &::VesselNetworkPropertyManager< 3 >::SetSegmentViscosity )
                , ( bp::arg("pNetwork"), bp::arg("viscosity") ) );
        
        }
        VesselNetworkPropertyManager3_exposer.staticmethod( "AssignInflows" );
        VesselNetworkPropertyManager3_exposer.staticmethod( "AssignOutflows" );
        VesselNetworkPropertyManager3_exposer.staticmethod( "CopySegmentFlowProperties" );
        VesselNetworkPropertyManager3_exposer.staticmethod( "Create" );
        VesselNetworkPropertyManager3_exposer.staticmethod( "GetInflowNodes" );
        VesselNetworkPropertyManager3_exposer.staticmethod( "GetOutflowNodes" );
        VesselNetworkPropertyManager3_exposer.staticmethod( "SetInflowPressures" );
        VesselNetworkPropertyManager3_exposer.staticmethod( "SetNodeRadii" );
        VesselNetworkPropertyManager3_exposer.staticmethod( "SetNodeRadiiFromSegments" );
        VesselNetworkPropertyManager3_exposer.staticmethod( "SetOutflowPressures" );
        VesselNetworkPropertyManager3_exposer.staticmethod( "SetSegmentProperties" );
        VesselNetworkPropertyManager3_exposer.staticmethod( "SetSegmentRadii" );
        VesselNetworkPropertyManager3_exposer.staticmethod( "SetSegmentViscosity" );
        bp::register_ptr_to_python< boost::shared_ptr< VesselNetworkPropertyManager<3> > >();
    }

}
