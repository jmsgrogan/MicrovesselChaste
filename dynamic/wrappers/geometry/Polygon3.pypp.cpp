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
#include "Polygon3.pypp.hpp"

namespace bp = boost::python;

void register_Polygon3_class(){

    { //::Polygon< 3 >
        typedef bp::class_< Polygon< 3 > > Polygon3_exposer_t;
        Polygon3_exposer_t Polygon3_exposer = Polygon3_exposer_t( "Polygon3", bp::init< std::vector< boost::shared_ptr<DimensionalChastePoint<3> > > >(( bp::arg("vertices") )) );
        bp::scope Polygon3_scope( Polygon3_exposer );
        bp::implicitly_convertible< std::vector< boost::shared_ptr<DimensionalChastePoint<3> > >, Polygon< 3 > >();
        Polygon3_exposer.def( bp::init< boost::shared_ptr< DimensionalChastePoint< 3 > > >(( bp::arg("pVertex") )) );
        bp::implicitly_convertible< boost::shared_ptr< DimensionalChastePoint< 3 > >, Polygon< 3 > >();
        { //::Polygon< 3 >::AddAttribute
        
            typedef Polygon< 3 > exported_class_t;
            typedef void ( exported_class_t::*AddAttribute_function_type)( ::std::string const &,double ) ;
            
            Polygon3_exposer.def( 
                "AddAttribute"
                , AddAttribute_function_type( &::Polygon< 3 >::AddAttribute )
                , ( bp::arg("rLabel"), bp::arg("value") ) );
        
        }
        { //::Polygon< 3 >::AddAttributeToAllEdges
        
            typedef Polygon< 3 > exported_class_t;
            typedef void ( exported_class_t::*AddAttributeToAllEdges_function_type)( ::std::string const &,double ) ;
            
            Polygon3_exposer.def( 
                "AddAttributeToAllEdges"
                , AddAttributeToAllEdges_function_type( &::Polygon< 3 >::AddAttributeToAllEdges )
                , ( bp::arg("rLabel"), bp::arg("value") ) );
        
        }
        { //::Polygon< 3 >::AddAttributeToEdgeIfFound
        
            typedef Polygon< 3 > exported_class_t;
            typedef bool ( exported_class_t::*AddAttributeToEdgeIfFound_function_type)( ::DimensionalChastePoint< 3 >,::std::string const &,double ) ;
            
            Polygon3_exposer.def( 
                "AddAttributeToEdgeIfFound"
                , AddAttributeToEdgeIfFound_function_type( &::Polygon< 3 >::AddAttributeToEdgeIfFound )
                , ( bp::arg("loc"), bp::arg("rLabel"), bp::arg("value") ) );
        
        }
        { //::Polygon< 3 >::AddVertex
        
            typedef Polygon< 3 > exported_class_t;
            typedef void ( exported_class_t::*AddVertex_function_type)( ::boost::shared_ptr< DimensionalChastePoint< 3 > > ) ;
            
            Polygon3_exposer.def( 
                "AddVertex"
                , AddVertex_function_type( &::Polygon< 3 >::AddVertex )
                , ( bp::arg("pVertex") ) );
        
        }
        { //::Polygon< 3 >::AddVertices
        
            typedef Polygon< 3 > exported_class_t;
            typedef void ( exported_class_t::*AddVertices_function_type)( ::std::vector< boost::shared_ptr<DimensionalChastePoint<3> > > ) ;
            
            Polygon3_exposer.def( 
                "AddVertices"
                , AddVertices_function_type( &::Polygon< 3 >::AddVertices )
                , ( bp::arg("vertices") ) );
        
        }
        { //::Polygon< 3 >::ContainsPoint
        
            typedef Polygon< 3 > exported_class_t;
            typedef bool ( exported_class_t::*ContainsPoint_function_type)( ::DimensionalChastePoint< 3 > const &,double ) ;
            
            Polygon3_exposer.def( 
                "ContainsPoint"
                , ContainsPoint_function_type( &::Polygon< 3 >::ContainsPoint )
                , ( bp::arg("rLocation"), bp::arg("tolerance")=0. ) );
        
        }
        { //::Polygon< 3 >::Create
        
            typedef Polygon< 3 > exported_class_t;
            typedef ::boost::shared_ptr< Polygon< 3 > > ( *Create_function_type )( ::std::vector< boost::shared_ptr<DimensionalChastePoint<3> > > );
            
            Polygon3_exposer.def( 
                "Create"
                , Create_function_type( &::Polygon< 3 >::Create )
                , ( bp::arg("vertices") ) );
        
        }
        { //::Polygon< 3 >::Create
        
            typedef Polygon< 3 > exported_class_t;
            typedef ::boost::shared_ptr< Polygon< 3 > > ( *Create_function_type )( ::boost::shared_ptr< DimensionalChastePoint< 3 > > );
            
            Polygon3_exposer.def( 
                "Create"
                , Create_function_type( &::Polygon< 3 >::Create )
                , ( bp::arg("pVertex") ) );
        
        }
        { //::Polygon< 3 >::EdgeHasAttribute
        
            typedef Polygon< 3 > exported_class_t;
            typedef bool ( exported_class_t::*EdgeHasAttribute_function_type)( ::DimensionalChastePoint< 3 >,::std::string const & ) ;
            
            Polygon3_exposer.def( 
                "EdgeHasAttribute"
                , EdgeHasAttribute_function_type( &::Polygon< 3 >::EdgeHasAttribute )
                , ( bp::arg("loc"), bp::arg("rLabel") ) );
        
        }
        { //::Polygon< 3 >::GetAttributes
        
            typedef Polygon< 3 > exported_class_t;
            typedef ::std::map< std::string, double > ( exported_class_t::*GetAttributes_function_type)(  ) ;
            
            Polygon3_exposer.def( 
                "GetAttributes"
                , GetAttributes_function_type( &::Polygon< 3 >::GetAttributes ) );
        
        }
        { //::Polygon< 3 >::GetBoundingBox
        
            typedef Polygon< 3 > exported_class_t;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( exported_class_t::*GetBoundingBox_function_type)(  ) ;
            
            Polygon3_exposer.def( 
                "GetBoundingBox"
                , GetBoundingBox_function_type( &::Polygon< 3 >::GetBoundingBox ) );
        
        }
        { //::Polygon< 3 >::GetCentroid
        
            typedef Polygon< 3 > exported_class_t;
            typedef ::DimensionalChastePoint< 3 > ( exported_class_t::*GetCentroid_function_type)(  ) ;
            
            Polygon3_exposer.def( 
                "GetCentroid"
                , GetCentroid_function_type( &::Polygon< 3 >::GetCentroid ) );
        
        }
        { //::Polygon< 3 >::GetDistance
        
            typedef Polygon< 3 > exported_class_t;
            typedef ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ( exported_class_t::*GetDistance_function_type)( ::DimensionalChastePoint< 3 > const & ) ;
            
            Polygon3_exposer.def( 
                "GetDistance"
                , GetDistance_function_type( &::Polygon< 3 >::GetDistance )
                , ( bp::arg("rLocation") ) );
        
        }
        { //::Polygon< 3 >::GetDistanceToEdges
        
            typedef Polygon< 3 > exported_class_t;
            typedef ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ( exported_class_t::*GetDistanceToEdges_function_type)( ::DimensionalChastePoint< 3 > const & ) ;
            
            Polygon3_exposer.def( 
                "GetDistanceToEdges"
                , GetDistanceToEdges_function_type( &::Polygon< 3 >::GetDistanceToEdges )
                , ( bp::arg("rLocation") ) );
        
        }
        { //::Polygon< 3 >::GetEdgeAttributes
        
            typedef Polygon< 3 > exported_class_t;
            typedef ::std::vector< std::map< std::string, double > > ( exported_class_t::*GetEdgeAttributes_function_type)(  ) ;
            
            Polygon3_exposer.def( 
                "GetEdgeAttributes"
                , GetEdgeAttributes_function_type( &::Polygon< 3 >::GetEdgeAttributes ) );
        
        }
        { //::Polygon< 3 >::GetNormal
        
            typedef Polygon< 3 > exported_class_t;
            typedef ::boost::numeric::ublas::c_vector< double, 3 > ( exported_class_t::*GetNormal_function_type)(  ) ;
            
            Polygon3_exposer.def( 
                "GetNormal"
                , GetNormal_function_type( &::Polygon< 3 >::GetNormal ) );
        
        }
        { //::Polygon< 3 >::GetPlane
        
            typedef Polygon< 3 > exported_class_t;
            typedef ::vtkSmartPointer< vtkPlane > ( exported_class_t::*GetPlane_function_type)(  ) ;
            
            Polygon3_exposer.def( 
                "GetPlane"
                , GetPlane_function_type( &::Polygon< 3 >::GetPlane ) );
        
        }
        { //::Polygon< 3 >::GetVertex
        
            typedef Polygon< 3 > exported_class_t;
            typedef ::boost::shared_ptr< DimensionalChastePoint< 3 > > ( exported_class_t::*GetVertex_function_type)( unsigned int ) ;
            
            Polygon3_exposer.def( 
                "GetVertex"
                , GetVertex_function_type( &::Polygon< 3 >::GetVertex )
                , ( bp::arg("idx") ) );
        
        }
        { //::Polygon< 3 >::GetVertices
        
            typedef Polygon< 3 > exported_class_t;
            typedef ::std::vector< boost::shared_ptr<DimensionalChastePoint<3> > > ( exported_class_t::*GetVertices_function_type)(  ) ;
            
            Polygon3_exposer.def( 
                "GetVertices"
                , GetVertices_function_type( &::Polygon< 3 >::GetVertices ) );
        
        }
        { //::Polygon< 3 >::GetVtkPolygon
        
            typedef Polygon< 3 > exported_class_t;
            typedef ::vtkSmartPointer< vtkPolygon > ( exported_class_t::*GetVtkPolygon_function_type)(  ) ;
            
            Polygon3_exposer.def( 
                "GetVtkPolygon"
                , GetVtkPolygon_function_type( &::Polygon< 3 >::GetVtkPolygon ) );
        
        }
        { //::Polygon< 3 >::GetVtkVertices
        
            typedef Polygon< 3 > exported_class_t;
            typedef ::std::pair< vtkSmartPointer< vtkPoints >, vtkSmartPointer< vtkIdTypeArray > > ( exported_class_t::*GetVtkVertices_function_type)(  ) ;
            
            Polygon3_exposer.def( 
                "GetVtkVertices"
                , GetVtkVertices_function_type( &::Polygon< 3 >::GetVtkVertices ) );
        
        }
        { //::Polygon< 3 >::HasAttribute
        
            typedef Polygon< 3 > exported_class_t;
            typedef bool ( exported_class_t::*HasAttribute_function_type)( ::std::string const & ) ;
            
            Polygon3_exposer.def( 
                "HasAttribute"
                , HasAttribute_function_type( &::Polygon< 3 >::HasAttribute )
                , ( bp::arg("rLabel") ) );
        
        }
        { //::Polygon< 3 >::ReplaceVertex
        
            typedef Polygon< 3 > exported_class_t;
            typedef void ( exported_class_t::*ReplaceVertex_function_type)( unsigned int,::boost::shared_ptr< DimensionalChastePoint< 3 > > ) ;
            
            Polygon3_exposer.def( 
                "ReplaceVertex"
                , ReplaceVertex_function_type( &::Polygon< 3 >::ReplaceVertex )
                , ( bp::arg("idx"), bp::arg("pVertex") ) );
        
        }
        { //::Polygon< 3 >::RotateAboutAxis
        
            typedef Polygon< 3 > exported_class_t;
            typedef void ( exported_class_t::*RotateAboutAxis_function_type)( ::boost::numeric::ublas::c_vector< double, 3 >,double ) ;
            
            Polygon3_exposer.def( 
                "RotateAboutAxis"
                , RotateAboutAxis_function_type( &::Polygon< 3 >::RotateAboutAxis )
                , ( bp::arg("axis"), bp::arg("angle") ) );
        
        }
        { //::Polygon< 3 >::Translate
        
            typedef Polygon< 3 > exported_class_t;
            typedef void ( exported_class_t::*Translate_function_type)( ::DimensionalChastePoint< 3 > ) ;
            
            Polygon3_exposer.def( 
                "Translate"
                , Translate_function_type( &::Polygon< 3 >::Translate )
                , ( bp::arg("translationVector") ) );
        
        }
        Polygon3_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< Polygon<3> > >();
    }

}
