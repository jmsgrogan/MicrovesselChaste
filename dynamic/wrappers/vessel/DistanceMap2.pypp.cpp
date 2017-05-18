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
#include "DistanceMap2.pypp.hpp"

namespace bp = boost::python;

struct DistanceMap_less__2__greater__wrapper : DistanceMap< 2 >, bp::wrapper< DistanceMap< 2 > > {

    DistanceMap_less__2__greater__wrapper(DistanceMap<2> const & arg )
    : DistanceMap<2>( arg )
      , bp::wrapper< DistanceMap< 2 > >(){
        // copy constructor
        
    }

    DistanceMap_less__2__greater__wrapper( )
    : DistanceMap<2>( )
      , bp::wrapper< DistanceMap< 2 > >(){
        // null constructor
    
    }

    virtual void Solve(  ) {
        if( bp::override func_Solve = this->get_override( "Solve" ) )
            func_Solve(  );
        else{
            this->DistanceMap< 2 >::Solve(  );
        }
    }
    
    void default_Solve(  ) {
        DistanceMap< 2 >::Solve( );
    }

};

void register_DistanceMap2_class(){

    { //::DistanceMap< 2 >
        typedef bp::class_< DistanceMap_less__2__greater__wrapper > DistanceMap2_exposer_t;
        DistanceMap2_exposer_t DistanceMap2_exposer = DistanceMap2_exposer_t( "DistanceMap2", bp::init< >() );
        bp::scope DistanceMap2_scope( DistanceMap2_exposer );
        { //::DistanceMap< 2 >::Create
        
            typedef DistanceMap< 2 > exported_class_t;
            typedef ::boost::shared_ptr< DistanceMap< 2 > > ( *Create_function_type )(  );
            
            DistanceMap2_exposer.def( 
                "Create"
                , Create_function_type( &::DistanceMap< 2 >::Create ) );
        
        }
        { //::DistanceMap< 2 >::SetUseSegmentRadii
        
            typedef DistanceMap< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetUseSegmentRadii_function_type)( bool ) ;
            
            DistanceMap2_exposer.def( 
                "SetUseSegmentRadii"
                , SetUseSegmentRadii_function_type( &::DistanceMap< 2 >::SetUseSegmentRadii )
                , ( bp::arg("useRadii") ) );
        
        }
        { //::DistanceMap< 2 >::Solve
        
            typedef DistanceMap< 2 > exported_class_t;
            typedef void ( exported_class_t::*Solve_function_type)(  ) ;
            typedef void ( DistanceMap_less__2__greater__wrapper::*default_Solve_function_type)(  ) ;
            
            DistanceMap2_exposer.def( 
                "Solve"
                , Solve_function_type(&::DistanceMap< 2 >::Solve)
                , default_Solve_function_type(&DistanceMap_less__2__greater__wrapper::default_Solve) );
        
        }
        DistanceMap2_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< DistanceMap<2> > >();
        bp::implicitly_convertible< boost::shared_ptr< DistanceMap< 2 > >, boost::shared_ptr< AbstractRegularGridDiscreteContinuumSolver< 2 > > >();
        bp::implicitly_convertible< boost::shared_ptr< DistanceMap< 2 > >, boost::shared_ptr< AbstractDiscreteContinuumSolver< 2 > > >();
    }

}