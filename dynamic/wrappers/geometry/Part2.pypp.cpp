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
#include "Part2.pypp.hpp"

namespace bp = boost::python;

void register_Part2_class(){

    { //::Part< 2 >
        typedef bp::class_< Part< 2 > > Part2_exposer_t;
        Part2_exposer_t Part2_exposer = Part2_exposer_t( "Part2", bp::init< >() );
        bp::scope Part2_scope( Part2_exposer );
        { //::Part< 2 >::AddCircle
        
            typedef Part< 2 > exported_class_t;
            typedef ::boost::shared_ptr< Polygon< 2 > > ( exported_class_t::*AddCircle_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double >,::DimensionalChastePoint< 2 >,unsigned int ) ;
            
            Part2_exposer.def( 
                "AddCircle"
                , AddCircle_function_type( &::Part< 2 >::AddCircle )
                , ( bp::arg("radius"), bp::arg("centre"), bp::arg("numSegments")=(unsigned int)(24) ) );
        
        }
        { //::Part< 2 >::AddCuboid
        
            typedef Part< 2 > exported_class_t;
            typedef void ( exported_class_t::*AddCuboid_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double >,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double >,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double >,::DimensionalChastePoint< 2 > ) ;
            
            Part2_exposer.def( 
                "AddCuboid"
                , AddCuboid_function_type( &::Part< 2 >::AddCuboid )
                , ( bp::arg("sizeX"), bp::arg("sizeY"), bp::arg("sizeZ"), bp::arg("origin") ) );
        
        }
        { //::Part< 2 >::AddCylinder
        
            typedef Part< 2 > exported_class_t;
            typedef void ( exported_class_t::*AddCylinder_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double >,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double >,::DimensionalChastePoint< 2 >,unsigned int ) ;
            
            Part2_exposer.def( 
                "AddCylinder"
                , AddCylinder_function_type( &::Part< 2 >::AddCylinder )
                , ( bp::arg("radius"), bp::arg("depth"), bp::arg("centre"), bp::arg("numSegments")=(unsigned int)(24) ) );
        
        }
        { //::Part< 2 >::AddHoleMarker
        
            typedef Part< 2 > exported_class_t;
            typedef void ( exported_class_t::*AddHoleMarker_function_type)( ::DimensionalChastePoint< 2 > ) ;
            
            Part2_exposer.def( 
                "AddHoleMarker"
                , AddHoleMarker_function_type( &::Part< 2 >::AddHoleMarker )
                , ( bp::arg("location") ) );
        
        }
        { //::Part< 2 >::AddPolygon
        
            typedef Part< 2 > exported_class_t;
            typedef ::boost::shared_ptr< Polygon< 2 > > ( exported_class_t::*AddPolygon_function_type)( ::std::vector< boost::shared_ptr<DimensionalChastePoint<2> > >,bool,::boost::shared_ptr< Facet< 2 > > ) ;
            
            Part2_exposer.def( 
                "AddPolygon"
                , AddPolygon_function_type( &::Part< 2 >::AddPolygon )
                , ( bp::arg("vertices"), bp::arg("newFacet")=(bool)(false), bp::arg("pFacet")=boost::shared_ptr<Facet<2> >() ) );
        
        }
        { //::Part< 2 >::AddPolygon
        
            typedef Part< 2 > exported_class_t;
            typedef ::boost::shared_ptr< Polygon< 2 > > ( exported_class_t::*AddPolygon_function_type)( ::boost::shared_ptr< Polygon< 2 > >,bool,::boost::shared_ptr< Facet< 2 > > ) ;
            
            Part2_exposer.def( 
                "AddPolygon"
                , AddPolygon_function_type( &::Part< 2 >::AddPolygon )
                , ( bp::arg("pPolygon"), bp::arg("newFacet")=(bool)(false), bp::arg("pFacet")=boost::shared_ptr<Facet<2> >() ) );
        
        }
        { //::Part< 2 >::AddRectangle
        
            typedef Part< 2 > exported_class_t;
            typedef ::boost::shared_ptr< Polygon< 2 > > ( exported_class_t::*AddRectangle_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double >,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double >,::DimensionalChastePoint< 2 > ) ;
            
            Part2_exposer.def( 
                "AddRectangle"
                , AddRectangle_function_type( &::Part< 2 >::AddRectangle )
                , ( bp::arg("sizeX"), bp::arg("sizeY"), bp::arg("origin") ) );
        
        }
        { //::Part< 2 >::AddVesselNetwork
        
            typedef Part< 2 > exported_class_t;
            typedef void ( exported_class_t::*AddVesselNetwork_function_type)( ::boost::shared_ptr< VesselNetwork< 2 > >,bool ) ;
            
            Part2_exposer.def( 
                "AddVesselNetwork"
                , AddVesselNetwork_function_type( &::Part< 2 >::AddVesselNetwork )
                , ( bp::arg("pVesselNetwork"), bp::arg("surface")=(bool)(false) ) );
        
        }
        { //::Part< 2 >::AppendPart
        
            typedef Part< 2 > exported_class_t;
            typedef void ( exported_class_t::*AppendPart_function_type)( ::boost::shared_ptr< Part< 2 > > ) ;
            
            Part2_exposer.def( 
                "AppendPart"
                , AppendPart_function_type( &::Part< 2 >::AppendPart )
                , ( bp::arg("pPart") ) );
        
        }
        { //::Part< 2 >::BooleanWithNetwork
        
            typedef Part< 2 > exported_class_t;
            typedef void ( exported_class_t::*BooleanWithNetwork_function_type)( ::boost::shared_ptr< VesselNetwork< 2 > > ) ;
            
            Part2_exposer.def( 
                "BooleanWithNetwork"
                , BooleanWithNetwork_function_type( &::Part< 2 >::BooleanWithNetwork )
                , ( bp::arg("pVesselNetwork") ) );
        
        }
        { //::Part< 2 >::Create
        
            typedef Part< 2 > exported_class_t;
            typedef ::boost::shared_ptr< Part< 2 > > ( *Create_function_type )(  );
            
            Part2_exposer.def( 
                "Create"
                , Create_function_type( &::Part< 2 >::Create ) );
        
        }
        { //::Part< 2 >::EdgeHasLabel
        
            typedef Part< 2 > exported_class_t;
            typedef bool ( exported_class_t::*EdgeHasLabel_function_type)( ::DimensionalChastePoint< 2 >,::std::string const & ) ;
            
            Part2_exposer.def( 
                "EdgeHasLabel"
                , EdgeHasLabel_function_type( &::Part< 2 >::EdgeHasLabel )
                , ( bp::arg("loc"), bp::arg("rLabel") ) );
        
        }
        { //::Part< 2 >::Extrude
        
            typedef Part< 2 > exported_class_t;
            typedef void ( exported_class_t::*Extrude_function_type)( ::boost::shared_ptr< Polygon< 2 > >,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            Part2_exposer.def( 
                "Extrude"
                , Extrude_function_type( &::Part< 2 >::Extrude )
                , ( bp::arg("pPolygon"), bp::arg("distance") ) );
        
        }
        { //::Part< 2 >::GetBoundingBox
        
            typedef Part< 2 > exported_class_t;
            typedef ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( exported_class_t::*GetBoundingBox_function_type)(  ) ;
            
            Part2_exposer.def( 
                "GetBoundingBox"
                , GetBoundingBox_function_type( &::Part< 2 >::GetBoundingBox ) );
        
        }
        { //::Part< 2 >::GetContainingGridIndices
        
            typedef Part< 2 > exported_class_t;
            typedef ::std::vector< unsigned int > ( exported_class_t::*GetContainingGridIndices_function_type)( unsigned int,unsigned int,unsigned int,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            Part2_exposer.def( 
                "GetContainingGridIndices"
                , GetContainingGridIndices_function_type( &::Part< 2 >::GetContainingGridIndices )
                , ( bp::arg("num_x"), bp::arg("num_y"), bp::arg("num_z"), bp::arg("spacing") ) );
        
        }
        { //::Part< 2 >::GetFacet
        
            typedef Part< 2 > exported_class_t;
            typedef ::boost::shared_ptr< Facet< 2 > > ( exported_class_t::*GetFacet_function_type)( ::DimensionalChastePoint< 2 > const & ) ;
            
            Part2_exposer.def( 
                "GetFacet"
                , GetFacet_function_type( &::Part< 2 >::GetFacet )
                , ( bp::arg("rLocation") ) );
        
        }
        { //::Part< 2 >::GetFacets
        
            typedef Part< 2 > exported_class_t;
            typedef ::std::vector< boost::shared_ptr<Facet<2> > > ( exported_class_t::*GetFacets_function_type)(  ) ;
            
            Part2_exposer.def( 
                "GetFacets"
                , GetFacets_function_type( &::Part< 2 >::GetFacets ) );
        
        }
        { //::Part< 2 >::GetHoleMarkers
        
            typedef Part< 2 > exported_class_t;
            typedef ::std::vector< DimensionalChastePoint<2> > ( exported_class_t::*GetHoleMarkers_function_type)(  ) ;
            
            Part2_exposer.def( 
                "GetHoleMarkers"
                , GetHoleMarkers_function_type( &::Part< 2 >::GetHoleMarkers ) );
        
        }
        { //::Part< 2 >::GetPolygons
        
            typedef Part< 2 > exported_class_t;
            typedef ::std::vector< boost::shared_ptr<Polygon<2> > > ( exported_class_t::*GetPolygons_function_type)(  ) ;
            
            Part2_exposer.def( 
                "GetPolygons"
                , GetPolygons_function_type( &::Part< 2 >::GetPolygons ) );
        
        }
        { //::Part< 2 >::GetReferenceLengthScale
        
            typedef Part< 2 > exported_class_t;
            typedef ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ( exported_class_t::*GetReferenceLengthScale_function_type)(  ) ;
            
            Part2_exposer.def( 
                "GetReferenceLengthScale"
                , GetReferenceLengthScale_function_type( &::Part< 2 >::GetReferenceLengthScale ) );
        
        }
        { //::Part< 2 >::GetSegmentIndices
        
            typedef Part< 2 > exported_class_t;
            typedef ::std::vector< std::pair<unsigned int, unsigned int> > ( exported_class_t::*GetSegmentIndices_function_type)(  ) ;
            
            Part2_exposer.def( 
                "GetSegmentIndices"
                , GetSegmentIndices_function_type( &::Part< 2 >::GetSegmentIndices ) );
        
        }
        { //::Part< 2 >::GetVertexLocations
        
            typedef Part< 2 > exported_class_t;
            typedef ::std::vector< DimensionalChastePoint<2> > ( exported_class_t::*GetVertexLocations_function_type)(  ) ;
            
            Part2_exposer.def( 
                "GetVertexLocations"
                , GetVertexLocations_function_type( &::Part< 2 >::GetVertexLocations ) );
        
        }
        { //::Part< 2 >::GetVertices
        
            typedef Part< 2 > exported_class_t;
            typedef ::std::vector< boost::shared_ptr<DimensionalChastePoint<2> > > ( exported_class_t::*GetVertices_function_type)(  ) ;
            
            Part2_exposer.def( 
                "GetVertices"
                , GetVertices_function_type( &::Part< 2 >::GetVertices ) );
        
        }
        { //::Part< 2 >::GetVtk
        
            typedef Part< 2 > exported_class_t;
            typedef ::vtkSmartPointer< vtkPolyData > ( exported_class_t::*GetVtk_function_type)( bool ) ;
            
            Part2_exposer.def( 
                "GetVtk"
                , GetVtk_function_type( &::Part< 2 >::GetVtk )
                , ( bp::arg("includeEdges")=(bool)(false) ) );
        
        }
        { //::Part< 2 >::IsPointInPart
        
            typedef Part< 2 > exported_class_t;
            typedef bool ( exported_class_t::*IsPointInPart_function_type)( ::DimensionalChastePoint< 2 > ) ;
            
            Part2_exposer.def( 
                "IsPointInPart"
                , IsPointInPart_function_type( &::Part< 2 >::IsPointInPart )
                , ( bp::arg("location") ) );
        
        }
        { //::Part< 2 >::IsPointInPart
        
            typedef Part< 2 > exported_class_t;
            typedef ::std::vector< bool > ( exported_class_t::*IsPointInPart_function_type)( ::vtkSmartPointer< vtkPoints > ) ;
            
            Part2_exposer.def( 
                "IsPointInPart"
                , IsPointInPart_function_type( &::Part< 2 >::IsPointInPart )
                , ( bp::arg("pPoints") ) );
        
        }
        { //::Part< 2 >::LabelEdges
        
            typedef Part< 2 > exported_class_t;
            typedef void ( exported_class_t::*LabelEdges_function_type)( ::DimensionalChastePoint< 2 >,::std::string const & ) ;
            
            Part2_exposer.def( 
                "LabelEdges"
                , LabelEdges_function_type( &::Part< 2 >::LabelEdges )
                , ( bp::arg("loc"), bp::arg("rLabel") ) );
        
        }
        { //::Part< 2 >::MergeCoincidentVertices
        
            typedef Part< 2 > exported_class_t;
            typedef void ( exported_class_t::*MergeCoincidentVertices_function_type)(  ) ;
            
            Part2_exposer.def( 
                "MergeCoincidentVertices"
                , MergeCoincidentVertices_function_type( &::Part< 2 >::MergeCoincidentVertices ) );
        
        }
        { //::Part< 2 >::RotateAboutAxis
        
            typedef Part< 2 > exported_class_t;
            typedef void ( exported_class_t::*RotateAboutAxis_function_type)( ::boost::numeric::ublas::c_vector< double, 3 >,double ) ;
            
            Part2_exposer.def( 
                "RotateAboutAxis"
                , RotateAboutAxis_function_type( &::Part< 2 >::RotateAboutAxis )
                , ( bp::arg("axis"), bp::arg("angle") ) );
        
        }
        { //::Part< 2 >::SetReferenceLengthScale
        
            typedef Part< 2 > exported_class_t;
            typedef void ( exported_class_t::*SetReferenceLengthScale_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            Part2_exposer.def( 
                "SetReferenceLengthScale"
                , SetReferenceLengthScale_function_type( &::Part< 2 >::SetReferenceLengthScale )
                , ( bp::arg("referenceLength") ) );
        
        }
        { //::Part< 2 >::Translate
        
            typedef Part< 2 > exported_class_t;
            typedef void ( exported_class_t::*Translate_function_type)( ::DimensionalChastePoint< 2 > ) ;
            
            Part2_exposer.def( 
                "Translate"
                , Translate_function_type( &::Part< 2 >::Translate )
                , ( bp::arg("vector") ) );
        
        }
        { //::Part< 2 >::Write
        
            typedef Part< 2 > exported_class_t;
            typedef void ( exported_class_t::*Write_function_type)( ::std::string const &,::GeometryFormat::Value,bool ) ;
            
            Part2_exposer.def( 
                "Write"
                , Write_function_type( &::Part< 2 >::Write )
                , ( bp::arg("rFilename"), bp::arg("format"), bp::arg("includeEdges") ) );
        
        }
        Part2_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< Part<2> > >();
    }

}
