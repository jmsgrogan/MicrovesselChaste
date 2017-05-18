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
#include "GridCalculator3.pypp.hpp"

namespace bp = boost::python;

void register_GridCalculator3_class(){

    { //::GridCalculator< 3 >
        typedef bp::class_< GridCalculator< 3 > > GridCalculator3_exposer_t;
        GridCalculator3_exposer_t GridCalculator3_exposer = GridCalculator3_exposer_t( "GridCalculator3", bp::init< >() );
        bp::scope GridCalculator3_scope( GridCalculator3_exposer );
        { //::GridCalculator< 3 >::CellPopulationIsSet
        
            typedef GridCalculator< 3 > exported_class_t;
            typedef bool ( exported_class_t::*CellPopulationIsSet_function_type)(  ) ;
            
            GridCalculator3_exposer.def( 
                "CellPopulationIsSet"
                , CellPopulationIsSet_function_type( &::GridCalculator< 3 >::CellPopulationIsSet ) );
        
        }
        { //::GridCalculator< 3 >::Create
        
            typedef GridCalculator< 3 > exported_class_t;
            typedef ::boost::shared_ptr< GridCalculator< 3 > > ( *Create_function_type )(  );
            
            GridCalculator3_exposer.def( 
                "Create"
                , Create_function_type( &::GridCalculator< 3 >::Create ) );
        
        }
        { //::GridCalculator< 3 >::GetGrid
        
            typedef GridCalculator< 3 > exported_class_t;
            typedef ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 3, 3 > > ( exported_class_t::*GetGrid_function_type)(  ) ;
            
            GridCalculator3_exposer.def( 
                "GetGrid"
                , GetGrid_function_type( &::GridCalculator< 3 >::GetGrid ) );
        
        }
        { //::GridCalculator< 3 >::GetPointMap
        
            typedef GridCalculator< 3 > exported_class_t;
            typedef ::std::vector< std::vector< unsigned int > > ( exported_class_t::*GetPointMap_function_type)( ::std::vector< DimensionalChastePoint<3> > const & ) ;
            
            GridCalculator3_exposer.def( 
                "GetPointMap"
                , GetPointMap_function_type( &::GridCalculator< 3 >::GetPointMap )
                , ( bp::arg("rInputPoints") ) );
        
        }
        { //::GridCalculator< 3 >::GetPointMap
        
            typedef GridCalculator< 3 > exported_class_t;
            typedef ::std::vector< std::vector< unsigned int > > ( exported_class_t::*GetPointMap_function_type)( ::vtkSmartPointer< vtkPoints > ) ;
            
            GridCalculator3_exposer.def( 
                "GetPointMap"
                , GetPointMap_function_type( &::GridCalculator< 3 >::GetPointMap )
                , ( bp::arg("pInputPoints") ) );
        
        }
        { //::GridCalculator< 3 >::GetVesselNetwork
        
            typedef GridCalculator< 3 > exported_class_t;
            typedef ::boost::shared_ptr< VesselNetwork< 3 > > ( exported_class_t::*GetVesselNetwork_function_type)(  ) ;
            
            GridCalculator3_exposer.def( 
                "GetVesselNetwork"
                , GetVesselNetwork_function_type( &::GridCalculator< 3 >::GetVesselNetwork ) );
        
        }
        { //::GridCalculator< 3 >::HasStructuredGrid
        
            typedef GridCalculator< 3 > exported_class_t;
            typedef bool ( exported_class_t::*HasStructuredGrid_function_type)(  ) ;
            
            GridCalculator3_exposer.def( 
                "HasStructuredGrid"
                , HasStructuredGrid_function_type( &::GridCalculator< 3 >::HasStructuredGrid ) );
        
        }
        { //::GridCalculator< 3 >::HasUnstructuredGrid
        
            typedef GridCalculator< 3 > exported_class_t;
            typedef bool ( exported_class_t::*HasUnstructuredGrid_function_type)(  ) ;
            
            GridCalculator3_exposer.def( 
                "HasUnstructuredGrid"
                , HasUnstructuredGrid_function_type( &::GridCalculator< 3 >::HasUnstructuredGrid ) );
        
        }
        { //::GridCalculator< 3 >::IsSegmentAtLocation
        
            typedef GridCalculator< 3 > exported_class_t;
            typedef bool ( exported_class_t::*IsSegmentAtLocation_function_type)( unsigned int,bool ) ;
            
            GridCalculator3_exposer.def( 
                "IsSegmentAtLocation"
                , IsSegmentAtLocation_function_type( &::GridCalculator< 3 >::IsSegmentAtLocation )
                , ( bp::arg("index"), bp::arg("update") ) );
        
        }
        { //::GridCalculator< 3 >::SetCellPopulation
        
            typedef GridCalculator< 3 > exported_class_t;
            typedef void ( exported_class_t::*SetCellPopulation_function_type)( ::AbstractCellPopulation< 3, 3 > &,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double >,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            GridCalculator3_exposer.def( 
                "SetCellPopulation"
                , SetCellPopulation_function_type( &::GridCalculator< 3 >::SetCellPopulation )
                , ( bp::arg("rCellPopulation"), bp::arg("cellPopulationReferenceLength"), bp::arg("cellPopulationReferenceConcentration") ) );
        
        }
        { //::GridCalculator< 3 >::SetGrid
        
            typedef GridCalculator< 3 > exported_class_t;
            typedef void ( exported_class_t::*SetGrid_function_type)( ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 3, 3 > > ) ;
            
            GridCalculator3_exposer.def( 
                "SetGrid"
                , SetGrid_function_type( &::GridCalculator< 3 >::SetGrid )
                , ( bp::arg("pGrid") ) );
        
        }
        { //::GridCalculator< 3 >::SetVesselNetwork
        
            typedef GridCalculator< 3 > exported_class_t;
            typedef void ( exported_class_t::*SetVesselNetwork_function_type)( ::boost::shared_ptr< VesselNetwork< 3 > > ) ;
            
            GridCalculator3_exposer.def( 
                "SetVesselNetwork"
                , SetVesselNetwork_function_type( &::GridCalculator< 3 >::SetVesselNetwork )
                , ( bp::arg("pNetwork") ) );
        
        }
        { //::GridCalculator< 3 >::rGetCellMap
        
            typedef GridCalculator< 3 > exported_class_t;
            typedef ::std::vector< std::vector< boost::shared_ptr<Cell> > > const & ( exported_class_t::*rGetCellMap_function_type)( bool ) ;
            
            GridCalculator3_exposer.def( 
                "rGetCellMap"
                , rGetCellMap_function_type( &::GridCalculator< 3 >::rGetCellMap )
                , ( bp::arg("update")=(bool)(true) )
                , bp::return_internal_reference< >() );
        
        }
        { //::GridCalculator< 3 >::rGetSegmentMap
        
            typedef GridCalculator< 3 > exported_class_t;
            typedef ::std::vector< std::vector< boost::shared_ptr<VesselSegment<3> > > > const & ( exported_class_t::*rGetSegmentMap_function_type)( bool,bool ) ;
            
            GridCalculator3_exposer.def( 
                "rGetSegmentMap"
                , rGetSegmentMap_function_type( &::GridCalculator< 3 >::rGetSegmentMap )
                , ( bp::arg("update")=(bool)(true), bp::arg("useVesselSurface")=(bool)(false) )
                , bp::return_internal_reference< >() );
        
        }
        { //::GridCalculator< 3 >::rGetVesselNodeMap
        
            typedef GridCalculator< 3 > exported_class_t;
            typedef ::std::vector< std::vector< boost::shared_ptr<VesselNode<3> > > > const & ( exported_class_t::*rGetVesselNodeMap_function_type)( bool ) ;
            
            GridCalculator3_exposer.def( 
                "rGetVesselNodeMap"
                , rGetVesselNodeMap_function_type( &::GridCalculator< 3 >::rGetVesselNodeMap )
                , ( bp::arg("update")=(bool)(true) )
                , bp::return_internal_reference< >() );
        
        }
        GridCalculator3_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< GridCalculator<3> > >();
    }

}