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
#include "ImageToSurface.pypp.hpp"

namespace bp = boost::python;

void register_ImageToSurface_class(){

    { //::ImageToSurface
        typedef bp::class_< ImageToSurface > ImageToSurface_exposer_t;
        ImageToSurface_exposer_t ImageToSurface_exposer = ImageToSurface_exposer_t( "ImageToSurface", bp::init< >() );
        bp::scope ImageToSurface_scope( ImageToSurface_exposer );
        { //::ImageToSurface::Create
        
            typedef ::boost::shared_ptr< ImageToSurface > ( *Create_function_type )(  );
            
            ImageToSurface_exposer.def( 
                "Create"
                , Create_function_type( &::ImageToSurface::Create ) );
        
        }
        { //::ImageToSurface::GetOutput
        
            typedef ::vtkSmartPointer< vtkPolyData > ( ::ImageToSurface::*GetOutput_function_type)(  ) ;
            
            ImageToSurface_exposer.def( 
                "GetOutput"
                , GetOutput_function_type( &::ImageToSurface::GetOutput ) );
        
        }
        { //::ImageToSurface::SetInput
        
            typedef void ( ::ImageToSurface::*SetInput_function_type)( ::vtkSmartPointer< vtkImageData > ) ;
            
            ImageToSurface_exposer.def( 
                "SetInput"
                , SetInput_function_type( &::ImageToSurface::SetInput )
                , ( bp::arg("pImage") ) );
        
        }
        { //::ImageToSurface::SetInputRaw
        
            typedef void ( ::ImageToSurface::*SetInputRaw_function_type)( ::vtkImageData * ) ;
            
            ImageToSurface_exposer.def( 
                "SetInputRaw"
                , SetInputRaw_function_type( &::ImageToSurface::SetInputRaw )
                , ( bp::arg("pImage") ) );
        
        }
        { //::ImageToSurface::SetRemoveDisconnected
        
            typedef void ( ::ImageToSurface::*SetRemoveDisconnected_function_type)( bool ) ;
            
            ImageToSurface_exposer.def( 
                "SetRemoveDisconnected"
                , SetRemoveDisconnected_function_type( &::ImageToSurface::SetRemoveDisconnected )
                , ( bp::arg("removeDisconnected") ) );
        
        }
        { //::ImageToSurface::SetThreshold
        
            typedef void ( ::ImageToSurface::*SetThreshold_function_type)( double,bool ) ;
            
            ImageToSurface_exposer.def( 
                "SetThreshold"
                , SetThreshold_function_type( &::ImageToSurface::SetThreshold )
                , ( bp::arg("threshold"), bp::arg("segmentAboveThreshold") ) );
        
        }
        { //::ImageToSurface::SetUseMarchingCubes
        
            typedef void ( ::ImageToSurface::*SetUseMarchingCubes_function_type)( bool ) ;
            
            ImageToSurface_exposer.def( 
                "SetUseMarchingCubes"
                , SetUseMarchingCubes_function_type( &::ImageToSurface::SetUseMarchingCubes )
                , ( bp::arg("useMarchingCubes") ) );
        
        }
        { //::ImageToSurface::Update
        
            typedef void ( ::ImageToSurface::*Update_function_type)(  ) ;
            
            ImageToSurface_exposer.def( 
                "Update"
                , Update_function_type( &::ImageToSurface::Update ) );
        
        }
        ImageToSurface_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< ImageToSurface > >();
    }

}
