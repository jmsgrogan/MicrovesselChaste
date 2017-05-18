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
#include "MicrovesselVtkScene3.pypp.hpp"

namespace bp = boost::python;

void register_MicrovesselVtkScene3_class(){

    bp::class_< MicrovesselVtkScene< 3 > >( "MicrovesselVtkScene3", bp::init< >() )    
        .def( 
            "End"
            , (void ( ::MicrovesselVtkScene<3>::* )(  ))( &::MicrovesselVtkScene< 3 >::End ) )    
        .def( 
            "GetCellPopulationActorGenerator"
            , (::boost::shared_ptr< CellPopulationActorGenerator< 3 > > ( ::MicrovesselVtkScene<3>::* )(  ))( &::MicrovesselVtkScene< 3 >::GetCellPopulationActorGenerator ) )    
        .def( 
            "GetDiscreteContinuumMeshActorGenerator"
            , (::boost::shared_ptr< DiscreteContinuumMeshActorGenerator< 3 > > ( ::MicrovesselVtkScene<3>::* )(  ))( &::MicrovesselVtkScene< 3 >::GetDiscreteContinuumMeshActorGenerator ) )    
        .def( 
            "GetPartActorGenerator"
            , (::boost::shared_ptr< PartActorGenerator< 3 > > ( ::MicrovesselVtkScene<3>::* )(  ))( &::MicrovesselVtkScene< 3 >::GetPartActorGenerator ) )    
        .def( 
            "GetRegularGridActorGenerator"
            , (::boost::shared_ptr< RegularGridActorGenerator< 3 > > ( ::MicrovesselVtkScene<3>::* )(  ))( &::MicrovesselVtkScene< 3 >::GetRegularGridActorGenerator ) )    
        .def( 
            "GetRenderer"
            , (::vtkSmartPointer< vtkRenderer > ( ::MicrovesselVtkScene<3>::* )(  ))( &::MicrovesselVtkScene< 3 >::GetRenderer ) )    
        .def( 
            "GetSceneAsCharBuffer"
            , (::vtkSmartPointer< vtkUnsignedCharArray > ( ::MicrovesselVtkScene<3>::* )(  ))( &::MicrovesselVtkScene< 3 >::GetSceneAsCharBuffer ) )    
        .def( 
            "GetVesselNetworkActorGenerator"
            , (::boost::shared_ptr< VesselNetworkActorGenerator< 3 > > ( ::MicrovesselVtkScene<3>::* )(  ))( &::MicrovesselVtkScene< 3 >::GetVesselNetworkActorGenerator ) )    
        .def( 
            "ResetRenderer"
            , (void ( ::MicrovesselVtkScene<3>::* )( unsigned int ))( &::MicrovesselVtkScene< 3 >::ResetRenderer )
            , ( bp::arg("timeStep")=(unsigned int)(0) ) )    
        .def( 
            "SetCellPopulation"
            , (void ( ::MicrovesselVtkScene<3>::* )( ::boost::shared_ptr< AbstractCellPopulation< 3, 3 > > ))( &::MicrovesselVtkScene< 3 >::SetCellPopulation )
            , ( bp::arg("pCellPopulation") ) )    
        .def( 
            "SetIsInteractive"
            , (void ( ::MicrovesselVtkScene<3>::* )( bool ))( &::MicrovesselVtkScene< 3 >::SetIsInteractive )
            , ( bp::arg("isInteractive") ) )    
        .def( 
            "SetMesh"
            , (void ( ::MicrovesselVtkScene<3>::* )( ::boost::shared_ptr< DiscreteContinuumMesh< 3, 3 > > ))( &::MicrovesselVtkScene< 3 >::SetMesh )
            , ( bp::arg("pMesh") ) )    
        .def( 
            "SetOutputFilePath"
            , (void ( ::MicrovesselVtkScene<3>::* )( ::std::string const & ))( &::MicrovesselVtkScene< 3 >::SetOutputFilePath )
            , ( bp::arg("rPath") ) )    
        .def( 
            "SetPart"
            , (void ( ::MicrovesselVtkScene<3>::* )( ::boost::shared_ptr< Part< 3 > > ))( &::MicrovesselVtkScene< 3 >::SetPart )
            , ( bp::arg("pPart") ) )    
        .def( 
            "SetRegularGrid"
            , (void ( ::MicrovesselVtkScene<3>::* )( ::boost::shared_ptr< RegularGrid< 3 > > ))( &::MicrovesselVtkScene< 3 >::SetRegularGrid )
            , ( bp::arg("pGrid") ) )    
        .def( 
            "SetSaveAsAnimation"
            , (void ( ::MicrovesselVtkScene<3>::* )( bool ))( &::MicrovesselVtkScene< 3 >::SetSaveAsAnimation )
            , ( bp::arg("saveAsAnimation") ) )    
        .def( 
            "SetSaveAsImages"
            , (void ( ::MicrovesselVtkScene<3>::* )( bool ))( &::MicrovesselVtkScene< 3 >::SetSaveAsImages )
            , ( bp::arg("saveAsImages") ) )    
        .def( 
            "SetVesselNetwork"
            , (void ( ::MicrovesselVtkScene<3>::* )( ::boost::shared_ptr< VesselNetwork< 3 > > ))( &::MicrovesselVtkScene< 3 >::SetVesselNetwork )
            , ( bp::arg("pNetwork") ) )    
        .def( 
            "Start"
            , (void ( ::MicrovesselVtkScene<3>::* )(  ))( &::MicrovesselVtkScene< 3 >::Start ) )    
        .def( 
            "StartInteractiveEventHandler"
            , (void ( ::MicrovesselVtkScene<3>::* )(  ))( &::MicrovesselVtkScene< 3 >::StartInteractiveEventHandler ) );

}