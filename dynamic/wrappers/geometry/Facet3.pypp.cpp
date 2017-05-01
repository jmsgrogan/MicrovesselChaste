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
#include "Facet3.pypp.hpp"

namespace bp = boost::python;

void register_Facet3_class(){

    { //::Facet< 3 >
        typedef bp::class_< Facet< 3 > > Facet3_exposer_t;
        Facet3_exposer_t Facet3_exposer = Facet3_exposer_t( "Facet3", bp::init< std::vector< boost::shared_ptr<Polygon<3> > > >(( bp::arg("polygons") )) );
        bp::scope Facet3_scope( Facet3_exposer );
        bp::implicitly_convertible< std::vector< boost::shared_ptr<Polygon<3> > >, Facet< 3 > >();
        Facet3_exposer.def( bp::init< boost::shared_ptr< Polygon< 3 > > >(( bp::arg("pPolygon") )) );
        bp::implicitly_convertible< boost::shared_ptr< Polygon< 3 > >, Facet< 3 > >();
        { //::Facet< 3 >::AddPolygon
        
            typedef Facet< 3 > exported_class_t;
            typedef void ( exported_class_t::*AddPolygon_function_type)( ::boost::shared_ptr< Polygon< 3 > > ) ;
            
            Facet3_exposer.def( 
                "AddPolygon"
                , AddPolygon_function_type( &::Facet< 3 >::AddPolygon )
                , ( bp::arg("pPolygon") ) );
        
        }
        { //::Facet< 3 >::AddPolygons
        
            typedef Facet< 3 > exported_class_t;
            typedef void ( exported_class_t::*AddPolygons_function_type)( ::std::vector< boost::shared_ptr<Polygon<3> > > ) ;
            
            Facet3_exposer.def( 
                "AddPolygons"
                , AddPolygons_function_type( &::Facet< 3 >::AddPolygons )
                , ( bp::arg("polygons") ) );
        
        }
        { //::Facet< 3 >::ContainsPoint
        
            typedef Facet< 3 > exported_class_t;
            typedef bool ( exported_class_t::*ContainsPoint_function_type)( ::DimensionalChastePoint< 3 > const & ) ;
            
            Facet3_exposer.def( 
                "ContainsPoint"
                , ContainsPoint_function_type( &::Facet< 3 >::ContainsPoint )
                , ( bp::arg("location") ) );
        
        }
        { //::Facet< 3 >::Create
        
            typedef Facet< 3 > exported_class_t;
            typedef ::boost::shared_ptr< Facet< 3 > > ( *Create_function_type )( ::std::vector< boost::shared_ptr<Polygon<3> > > );
            
            Facet3_exposer.def( 
                "Create"
                , Create_function_type( &::Facet< 3 >::Create )
                , ( bp::arg("polygons") ) );
        
        }
        { //::Facet< 3 >::Create
        
            typedef Facet< 3 > exported_class_t;
            typedef ::boost::shared_ptr< Facet< 3 > > ( *Create_function_type )( ::boost::shared_ptr< Polygon< 3 > > );
            
            Facet3_exposer.def( 
                "Create"
                , Create_function_type( &::Facet< 3 >::Create )
                , ( bp::arg("pPolygon") ) );
        
        }
        { //::Facet< 3 >::GetBoundingBox
        
            typedef Facet< 3 > exported_class_t;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( exported_class_t::*GetBoundingBox_function_type)(  ) ;
            
            Facet3_exposer.def( 
                "GetBoundingBox"
                , GetBoundingBox_function_type( &::Facet< 3 >::GetBoundingBox ) );
        
        }
        { //::Facet< 3 >::GetCentroid
        
            typedef Facet< 3 > exported_class_t;
            typedef ::DimensionalChastePoint< 3 > ( exported_class_t::*GetCentroid_function_type)(  ) ;
            
            Facet3_exposer.def( 
                "GetCentroid"
                , GetCentroid_function_type( &::Facet< 3 >::GetCentroid ) );
        
        }
        { //::Facet< 3 >::GetDistance
        
            typedef Facet< 3 > exported_class_t;
            typedef ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ( exported_class_t::*GetDistance_function_type)( ::DimensionalChastePoint< 3 > const & ) ;
            
            Facet3_exposer.def( 
                "GetDistance"
                , GetDistance_function_type( &::Facet< 3 >::GetDistance )
                , ( bp::arg("rLocation") ) );
        
        }
        { //::Facet< 3 >::GetLabel
        
            typedef Facet< 3 > exported_class_t;
            typedef ::std::string ( exported_class_t::*GetLabel_function_type)(  ) ;
            
            Facet3_exposer.def( 
                "GetLabel"
                , GetLabel_function_type( &::Facet< 3 >::GetLabel ) );
        
        }
        { //::Facet< 3 >::GetNormal
        
            typedef Facet< 3 > exported_class_t;
            typedef ::boost::numeric::ublas::c_vector< double, 3 > ( exported_class_t::*GetNormal_function_type)(  ) ;
            
            Facet3_exposer.def( 
                "GetNormal"
                , GetNormal_function_type( &::Facet< 3 >::GetNormal ) );
        
        }
        { //::Facet< 3 >::GetPlane
        
            typedef Facet< 3 > exported_class_t;
            typedef ::vtkSmartPointer< vtkPlane > ( exported_class_t::*GetPlane_function_type)(  ) ;
            
            Facet3_exposer.def( 
                "GetPlane"
                , GetPlane_function_type( &::Facet< 3 >::GetPlane ) );
        
        }
        { //::Facet< 3 >::GetPolygons
        
            typedef Facet< 3 > exported_class_t;
            typedef ::std::vector< boost::shared_ptr<Polygon<3> > > ( exported_class_t::*GetPolygons_function_type)(  ) ;
            
            Facet3_exposer.def( 
                "GetPolygons"
                , GetPolygons_function_type( &::Facet< 3 >::GetPolygons ) );
        
        }
        { //::Facet< 3 >::GetVertices
        
            typedef Facet< 3 > exported_class_t;
            typedef ::std::vector< boost::shared_ptr<DimensionalChastePoint<3> > > ( exported_class_t::*GetVertices_function_type)(  ) ;
            
            Facet3_exposer.def( 
                "GetVertices"
                , GetVertices_function_type( &::Facet< 3 >::GetVertices ) );
        
        }
        { //::Facet< 3 >::GetVtkVertices
        
            typedef Facet< 3 > exported_class_t;
            typedef ::std::pair< vtkSmartPointer< vtkPoints >, vtkSmartPointer< vtkIdTypeArray > > ( exported_class_t::*GetVtkVertices_function_type)(  ) ;
            
            Facet3_exposer.def( 
                "GetVtkVertices"
                , GetVtkVertices_function_type( &::Facet< 3 >::GetVtkVertices ) );
        
        }
        { //::Facet< 3 >::RotateAboutAxis
        
            typedef Facet< 3 > exported_class_t;
            typedef void ( exported_class_t::*RotateAboutAxis_function_type)( ::boost::numeric::ublas::c_vector< double, 3 >,double ) ;
            
            Facet3_exposer.def( 
                "RotateAboutAxis"
                , RotateAboutAxis_function_type( &::Facet< 3 >::RotateAboutAxis )
                , ( bp::arg("axis"), bp::arg("angle") ) );
        
        }
        { //::Facet< 3 >::SetLabel
        
            typedef Facet< 3 > exported_class_t;
            typedef void ( exported_class_t::*SetLabel_function_type)( ::std::string const & ) ;
            
            Facet3_exposer.def( 
                "SetLabel"
                , SetLabel_function_type( &::Facet< 3 >::SetLabel )
                , ( bp::arg("label") ) );
        
        }
        { //::Facet< 3 >::Translate
        
            typedef Facet< 3 > exported_class_t;
            typedef void ( exported_class_t::*Translate_function_type)( ::DimensionalChastePoint< 3 > ) ;
            
            Facet3_exposer.def( 
                "Translate"
                , Translate_function_type( &::Facet< 3 >::Translate )
                , ( bp::arg("translationVector") ) );
        
        }
        { //::Facet< 3 >::UpdateVertices
        
            typedef Facet< 3 > exported_class_t;
            typedef void ( exported_class_t::*UpdateVertices_function_type)(  ) ;
            
            Facet3_exposer.def( 
                "UpdateVertices"
                , UpdateVertices_function_type( &::Facet< 3 >::UpdateVertices ) );
        
        }
        Facet3_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< Facet<3> > >();
    }

}
