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
#include "BoundaryExtractor.pypp.hpp"

namespace bp = boost::python;

void register_BoundaryExtractor_class(){

    { //::BoundaryExtractor
        typedef bp::class_< BoundaryExtractor > BoundaryExtractor_exposer_t;
        BoundaryExtractor_exposer_t BoundaryExtractor_exposer = BoundaryExtractor_exposer_t( "BoundaryExtractor", bp::init< >() );
        bp::scope BoundaryExtractor_scope( BoundaryExtractor_exposer );
        { //::BoundaryExtractor::Create
        
            typedef ::boost::shared_ptr< BoundaryExtractor > ( *Create_function_type )(  );
            
            BoundaryExtractor_exposer.def( 
                "Create"
                , Create_function_type( &::BoundaryExtractor::Create ) );
        
        }
        { //::BoundaryExtractor::GetOutput
        
            typedef ::vtkSmartPointer< vtkPolyData > ( ::BoundaryExtractor::*GetOutput_function_type)(  ) ;
            
            BoundaryExtractor_exposer.def( 
                "GetOutput"
                , GetOutput_function_type( &::BoundaryExtractor::GetOutput ) );
        
        }
        { //::BoundaryExtractor::SetDoSmoothing
        
            typedef void ( ::BoundaryExtractor::*SetDoSmoothing_function_type)( bool ) ;
            
            BoundaryExtractor_exposer.def( 
                "SetDoSmoothing"
                , SetDoSmoothing_function_type( &::BoundaryExtractor::SetDoSmoothing )
                , ( bp::arg("doSmoothing") ) );
        
        }
        { //::BoundaryExtractor::SetInput
        
            typedef void ( ::BoundaryExtractor::*SetInput_function_type)( ::vtkSmartPointer< vtkPolyData > ) ;
            
            BoundaryExtractor_exposer.def( 
                "SetInput"
                , SetInput_function_type( &::BoundaryExtractor::SetInput )
                , ( bp::arg("pInputSurface") ) );
        
        }
        { //::BoundaryExtractor::SetInputRaw
        
            typedef void ( ::BoundaryExtractor::*SetInputRaw_function_type)( ::vtkPolyData * ) ;
            
            BoundaryExtractor_exposer.def( 
                "SetInputRaw"
                , SetInputRaw_function_type( &::BoundaryExtractor::SetInputRaw )
                , ( bp::arg("pInputSurface") ) );
        
        }
        { //::BoundaryExtractor::SetRemoveDisconnected
        
            typedef void ( ::BoundaryExtractor::*SetRemoveDisconnected_function_type)( bool ) ;
            
            BoundaryExtractor_exposer.def( 
                "SetRemoveDisconnected"
                , SetRemoveDisconnected_function_type( &::BoundaryExtractor::SetRemoveDisconnected )
                , ( bp::arg("removeDisconnected") ) );
        
        }
        { //::BoundaryExtractor::SetSmoothingLength
        
            typedef void ( ::BoundaryExtractor::*SetSmoothingLength_function_type)( double ) ;
            
            BoundaryExtractor_exposer.def( 
                "SetSmoothingLength"
                , SetSmoothingLength_function_type( &::BoundaryExtractor::SetSmoothingLength )
                , ( bp::arg("value") ) );
        
        }
        { //::BoundaryExtractor::Update
        
            typedef void ( ::BoundaryExtractor::*Update_function_type)(  ) ;
            
            BoundaryExtractor_exposer.def( 
                "Update"
                , Update_function_type( &::BoundaryExtractor::Update ) );
        
        }
        BoundaryExtractor_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< BoundaryExtractor > >();
    }

}