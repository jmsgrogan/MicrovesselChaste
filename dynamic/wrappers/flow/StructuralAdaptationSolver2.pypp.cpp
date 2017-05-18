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
#include "StructuralAdaptationSolver2.pypp.hpp"

namespace bp = boost::python;

struct StructuralAdaptationSolver_less__2__greater__wrapper : StructuralAdaptationSolver< 2 >, bp::wrapper< StructuralAdaptationSolver< 2 > > {

    StructuralAdaptationSolver_less__2__greater__wrapper(StructuralAdaptationSolver<2> const & arg )
    : StructuralAdaptationSolver<2>( arg )
      , bp::wrapper< StructuralAdaptationSolver< 2 > >(){
        // copy constructor
        
    }

    StructuralAdaptationSolver_less__2__greater__wrapper( )
    : StructuralAdaptationSolver<2>( )
      , bp::wrapper< StructuralAdaptationSolver< 2 > >(){
        // null constructor
    
    }

    virtual void Iterate(  ) {
        if( bp::override func_Iterate = this->get_override( "Iterate" ) )
            func_Iterate(  );
        else{
            this->StructuralAdaptationSolver< 2 >::Iterate(  );
        }
    }
    
    void default_Iterate(  ) {
        StructuralAdaptationSolver< 2 >::Iterate( );
    }

    virtual void Write(  ) {
        if( bp::override func_Write = this->get_override( "Write" ) )
            func_Write(  );
        else{
            this->AbstractStructuralAdaptationSolver< 2 >::Write(  );
        }
    }
    
    void default_Write(  ) {
        AbstractStructuralAdaptationSolver< 2 >::Write( );
    }

};

void register_StructuralAdaptationSolver2_class(){

    { //::StructuralAdaptationSolver< 2 >
        typedef bp::class_< StructuralAdaptationSolver_less__2__greater__wrapper, bp::bases< AbstractStructuralAdaptationSolver< 2 > > > StructuralAdaptationSolver2_exposer_t;
        StructuralAdaptationSolver2_exposer_t StructuralAdaptationSolver2_exposer = StructuralAdaptationSolver2_exposer_t( "StructuralAdaptationSolver2", bp::init< >() );
        bp::scope StructuralAdaptationSolver2_scope( StructuralAdaptationSolver2_exposer );
        { //::StructuralAdaptationSolver< 2 >::AddPostFlowSolveCalculator
        
            typedef StructuralAdaptationSolver< 2 > exported_class_t;
            typedef void ( exported_class_t::*AddPostFlowSolveCalculator_function_type)( ::boost::shared_ptr< AbstractVesselNetworkCalculator< 2 > > ) ;
            
            StructuralAdaptationSolver2_exposer.def( 
                "AddPostFlowSolveCalculator"
                , AddPostFlowSolveCalculator_function_type( &::StructuralAdaptationSolver< 2 >::AddPostFlowSolveCalculator )
                , ( bp::arg("pCalculator") ) );
        
        }
        { //::StructuralAdaptationSolver< 2 >::AddPreFlowSolveCalculator
        
            typedef StructuralAdaptationSolver< 2 > exported_class_t;
            typedef void ( exported_class_t::*AddPreFlowSolveCalculator_function_type)( ::boost::shared_ptr< AbstractVesselNetworkCalculator< 2 > > ) ;
            
            StructuralAdaptationSolver2_exposer.def( 
                "AddPreFlowSolveCalculator"
                , AddPreFlowSolveCalculator_function_type( &::StructuralAdaptationSolver< 2 >::AddPreFlowSolveCalculator )
                , ( bp::arg("pCalculator") ) );
        
        }
        { //::StructuralAdaptationSolver< 2 >::Create
        
            typedef StructuralAdaptationSolver< 2 > exported_class_t;
            typedef ::boost::shared_ptr< StructuralAdaptationSolver< 2 > > ( *Create_function_type )(  );
            
            StructuralAdaptationSolver2_exposer.def( 
                "Create"
                , Create_function_type( &::StructuralAdaptationSolver< 2 >::Create ) );
        
        }
        { //::StructuralAdaptationSolver< 2 >::GetFlowSolver
        
            typedef StructuralAdaptationSolver< 2 > exported_class_t;
            typedef ::boost::shared_ptr< FlowSolver< 2 > > ( exported_class_t::*GetFlowSolver_function_type)(  ) ;
            
            StructuralAdaptationSolver2_exposer.def( 
                "GetFlowSolver"
                , GetFlowSolver_function_type( &::StructuralAdaptationSolver< 2 >::GetFlowSolver ) );
        
        }
        { //::StructuralAdaptationSolver< 2 >::Iterate
        
            typedef StructuralAdaptationSolver< 2 > exported_class_t;
            typedef void ( exported_class_t::*Iterate_function_type)(  ) ;
            typedef void ( StructuralAdaptationSolver_less__2__greater__wrapper::*default_Iterate_function_type)(  ) ;
            
            StructuralAdaptationSolver2_exposer.def( 
                "Iterate"
                , Iterate_function_type(&::StructuralAdaptationSolver< 2 >::Iterate)
                , default_Iterate_function_type(&StructuralAdaptationSolver_less__2__greater__wrapper::default_Iterate) );
        
        }
        { //::StructuralAdaptationSolver< 2 >::SetFlowSolver
        
            typedef StructuralAdaptationSolver< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetFlowSolver_function_type)( ::boost::shared_ptr< FlowSolver< 2 > > ) ;
            
            StructuralAdaptationSolver2_exposer.def( 
                "SetFlowSolver"
                , SetFlowSolver_function_type( &::StructuralAdaptationSolver< 2 >::SetFlowSolver )
                , ( bp::arg("pSolver") ) );
        
        }
        { //::StructuralAdaptationSolver< 2 >::SetRadiusCalculator
        
            typedef StructuralAdaptationSolver< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetRadiusCalculator_function_type)( ::boost::shared_ptr< RadiusCalculator< 2 > > ) ;
            
            StructuralAdaptationSolver2_exposer.def( 
                "SetRadiusCalculator"
                , SetRadiusCalculator_function_type( &::StructuralAdaptationSolver< 2 >::SetRadiusCalculator )
                , ( bp::arg("pCalculator") ) );
        
        }
        { //::StructuralAdaptationSolver< 2 >::UpdateFlowSolver
        
            typedef StructuralAdaptationSolver< 2 > exported_class_t;
            typedef void ( exported_class_t::*UpdateFlowSolver_function_type)( bool ) ;
            
            StructuralAdaptationSolver2_exposer.def( 
                "UpdateFlowSolver"
                , UpdateFlowSolver_function_type( &::StructuralAdaptationSolver< 2 >::UpdateFlowSolver )
                , ( bp::arg("doFullReset")=(bool)(false) ) );
        
        }
        { //::AbstractStructuralAdaptationSolver< 2 >::Write
        
            typedef StructuralAdaptationSolver< 2 > exported_class_t;
            typedef void ( exported_class_t::*Write_function_type)(  ) ;
            typedef void ( StructuralAdaptationSolver_less__2__greater__wrapper::*default_Write_function_type)(  ) ;
            
            StructuralAdaptationSolver2_exposer.def( 
                "Write"
                , Write_function_type(&::AbstractStructuralAdaptationSolver< 2 >::Write)
                , default_Write_function_type(&StructuralAdaptationSolver_less__2__greater__wrapper::default_Write) );
        
        }
        StructuralAdaptationSolver2_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< StructuralAdaptationSolver<2> > >();
        bp::implicitly_convertible< boost::shared_ptr< StructuralAdaptationSolver< 2 > >, boost::shared_ptr< AbstractStructuralAdaptationSolver< 2 > > >();
    }

}