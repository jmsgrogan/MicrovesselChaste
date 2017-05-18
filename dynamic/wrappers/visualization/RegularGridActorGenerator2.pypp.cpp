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
#include "RegularGridActorGenerator2.pypp.hpp"

namespace bp = boost::python;

struct RegularGridActorGenerator_less__2__greater__wrapper : RegularGridActorGenerator< 2 >, bp::wrapper< RegularGridActorGenerator< 2 > > {

    RegularGridActorGenerator_less__2__greater__wrapper(RegularGridActorGenerator<2> const & arg )
    : RegularGridActorGenerator<2>( arg )
      , bp::wrapper< RegularGridActorGenerator< 2 > >(){
        // copy constructor
        
    }

    RegularGridActorGenerator_less__2__greater__wrapper( )
    : RegularGridActorGenerator<2>( )
      , bp::wrapper< RegularGridActorGenerator< 2 > >(){
        // null constructor
    
    }

    virtual void AddActor( ::vtkSmartPointer< vtkRenderer > pRenderer ) {
        if( bp::override func_AddActor = this->get_override( "AddActor" ) )
            func_AddActor( pRenderer );
        else{
            this->RegularGridActorGenerator< 2 >::AddActor( pRenderer );
        }
    }
    
    void default_AddActor( ::vtkSmartPointer< vtkRenderer > pRenderer ) {
        RegularGridActorGenerator< 2 >::AddActor( pRenderer );
    }

};

void register_RegularGridActorGenerator2_class(){

    { //::RegularGridActorGenerator< 2 >
        typedef bp::class_< RegularGridActorGenerator_less__2__greater__wrapper, bp::bases< AbstractActorGenerator< 2 > > > RegularGridActorGenerator2_exposer_t;
        RegularGridActorGenerator2_exposer_t RegularGridActorGenerator2_exposer = RegularGridActorGenerator2_exposer_t( "RegularGridActorGenerator2", bp::init< >() );
        bp::scope RegularGridActorGenerator2_scope( RegularGridActorGenerator2_exposer );
        { //::RegularGridActorGenerator< 2 >::AddActor
        
            typedef RegularGridActorGenerator< 2 > exported_class_t;
            typedef void ( exported_class_t::*AddActor_function_type)( ::vtkSmartPointer< vtkRenderer > ) ;
            typedef void ( RegularGridActorGenerator_less__2__greater__wrapper::*default_AddActor_function_type)( ::vtkSmartPointer< vtkRenderer > ) ;
            
            RegularGridActorGenerator2_exposer.def( 
                "AddActor"
                , AddActor_function_type(&::RegularGridActorGenerator< 2 >::AddActor)
                , default_AddActor_function_type(&RegularGridActorGenerator_less__2__greater__wrapper::default_AddActor)
                , ( bp::arg("pRenderer") ) );
        
        }
        { //::RegularGridActorGenerator< 2 >::SetEdgeOpacity
        
            typedef RegularGridActorGenerator< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetEdgeOpacity_function_type)( double ) ;
            
            RegularGridActorGenerator2_exposer.def( 
                "SetEdgeOpacity"
                , SetEdgeOpacity_function_type( &::RegularGridActorGenerator< 2 >::SetEdgeOpacity )
                , ( bp::arg("opacity") ) );
        
        }
        { //::RegularGridActorGenerator< 2 >::SetRegularGrid
        
            typedef RegularGridActorGenerator< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetRegularGrid_function_type)( ::boost::shared_ptr< RegularGrid< 2 > > ) ;
            
            RegularGridActorGenerator2_exposer.def( 
                "SetRegularGrid"
                , SetRegularGrid_function_type( &::RegularGridActorGenerator< 2 >::SetRegularGrid )
                , ( bp::arg("pRegularGrid") ) );
        
        }
        bp::register_ptr_to_python< boost::shared_ptr< RegularGridActorGenerator<2> > >();
        bp::implicitly_convertible< boost::shared_ptr< RegularGridActorGenerator< 2 > >, boost::shared_ptr< AbstractActorGenerator< 2 > > >();
    }

}