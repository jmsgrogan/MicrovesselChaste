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
#include "VesselNetworkGraphCalculator2.pypp.hpp"

namespace bp = boost::python;

void register_VesselNetworkGraphCalculator2_class(){

    { //::VesselNetworkGraphCalculator< 2 >
        typedef bp::class_< VesselNetworkGraphCalculator< 2 > > VesselNetworkGraphCalculator2_exposer_t;
        VesselNetworkGraphCalculator2_exposer_t VesselNetworkGraphCalculator2_exposer = VesselNetworkGraphCalculator2_exposer_t( "VesselNetworkGraphCalculator2", bp::init< >() );
        bp::scope VesselNetworkGraphCalculator2_scope( VesselNetworkGraphCalculator2_exposer );
        { //::VesselNetworkGraphCalculator< 2 >::Create
        
            typedef VesselNetworkGraphCalculator< 2 > exported_class_t;
            typedef ::boost::shared_ptr< VesselNetworkGraphCalculator< 2 > > ( *Create_function_type )(  );
            
            VesselNetworkGraphCalculator2_exposer.def( 
                "Create"
                , Create_function_type( &::VesselNetworkGraphCalculator< 2 >::Create ) );
        
        }
        { //::VesselNetworkGraphCalculator< 2 >::GetNodeNodeConnectivity
        
            typedef VesselNetworkGraphCalculator< 2 > exported_class_t;
            typedef ::std::vector< std::vector< unsigned int > > ( exported_class_t::*GetNodeNodeConnectivity_function_type)(  ) ;
            
            VesselNetworkGraphCalculator2_exposer.def( 
                "GetNodeNodeConnectivity"
                , GetNodeNodeConnectivity_function_type( &::VesselNetworkGraphCalculator< 2 >::GetNodeNodeConnectivity ) );
        
        }
        { //::VesselNetworkGraphCalculator< 2 >::GetNodeVesselConnectivity
        
            typedef VesselNetworkGraphCalculator< 2 > exported_class_t;
            typedef ::std::vector< std::vector< unsigned int > > ( exported_class_t::*GetNodeVesselConnectivity_function_type)(  ) ;
            
            VesselNetworkGraphCalculator2_exposer.def( 
                "GetNodeVesselConnectivity"
                , GetNodeVesselConnectivity_function_type( &::VesselNetworkGraphCalculator< 2 >::GetNodeVesselConnectivity ) );
        
        }
        { //::VesselNetworkGraphCalculator< 2 >::IsConnected
        
            typedef VesselNetworkGraphCalculator< 2 > exported_class_t;
            typedef bool ( exported_class_t::*IsConnected_function_type)( ::boost::shared_ptr< VesselNode< 2 > >,::boost::shared_ptr< VesselNode< 2 > > ) ;
            
            VesselNetworkGraphCalculator2_exposer.def( 
                "IsConnected"
                , IsConnected_function_type( &::VesselNetworkGraphCalculator< 2 >::IsConnected )
                , ( bp::arg("pSourceNode"), bp::arg("pQueryNode") ) );
        
        }
        { //::VesselNetworkGraphCalculator< 2 >::IsConnected
        
            typedef VesselNetworkGraphCalculator< 2 > exported_class_t;
            typedef ::std::vector< bool > ( exported_class_t::*IsConnected_function_type)( ::std::vector< boost::shared_ptr<VesselNode<2> > >,::std::vector< boost::shared_ptr<VesselNode<2> > > ) ;
            
            VesselNetworkGraphCalculator2_exposer.def( 
                "IsConnected"
                , IsConnected_function_type( &::VesselNetworkGraphCalculator< 2 >::IsConnected )
                , ( bp::arg("sourceNodes"), bp::arg("queryNodes") ) );
        
        }
        { //::VesselNetworkGraphCalculator< 2 >::SetVesselNetwork
        
            typedef VesselNetworkGraphCalculator< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetVesselNetwork_function_type)( ::boost::shared_ptr< VesselNetwork< 2 > > ) ;
            
            VesselNetworkGraphCalculator2_exposer.def( 
                "SetVesselNetwork"
                , SetVesselNetwork_function_type( &::VesselNetworkGraphCalculator< 2 >::SetVesselNetwork )
                , ( bp::arg("pVesselNetwork") ) );
        
        }
        { //::VesselNetworkGraphCalculator< 2 >::WriteConnectivity
        
            typedef VesselNetworkGraphCalculator< 2 > exported_class_t;
            typedef void ( exported_class_t::*WriteConnectivity_function_type)( ::std::string const & ) ;
            
            VesselNetworkGraphCalculator2_exposer.def( 
                "WriteConnectivity"
                , WriteConnectivity_function_type( &::VesselNetworkGraphCalculator< 2 >::WriteConnectivity )
                , ( bp::arg("rFilename") ) );
        
        }
        VesselNetworkGraphCalculator2_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< VesselNetworkGraphCalculator<2> > >();
    }

}