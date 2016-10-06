// This file has been generated by Py++.

#include "boost/python.hpp"

#include "geometry_headers.hpp"

namespace bp = boost::python;

BOOST_PYTHON_MODULE(_chaste_project_Microvessel_geometry){
    { //::Vertex
        typedef bp::class_< Vertex > Vertex_exposer_t;
        Vertex_exposer_t Vertex_exposer = Vertex_exposer_t( "Vertex", bp::init< double, double, double, boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > >(( bp::arg("x"), bp::arg("y"), bp::arg("z"), bp::arg("referenceLength") )) );
        bp::scope Vertex_scope( Vertex_exposer );
        Vertex_exposer.def( bp::init< boost::numeric::ublas::c_vector< double, 3 >, boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > >(( bp::arg("coords"), bp::arg("referenceLength") )) );
        Vertex_exposer.def( bp::init< bp::optional< double, double, double > >(( bp::arg("x")=0., bp::arg("y")=0., bp::arg("z")=0. )) );
        bp::implicitly_convertible< double, Vertex >();
        Vertex_exposer.def( bp::init< boost::numeric::ublas::c_vector< double, 3 > >(( bp::arg("coords") )) );
        bp::implicitly_convertible< boost::numeric::ublas::c_vector< double, 3 >, Vertex >();
        { //::Vertex::Create
        
            typedef ::boost::shared_ptr< Vertex > ( *Create_function_type )( double,double,double,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > );
            
            Vertex_exposer.def( 
                "Create"
                , Create_function_type( &::Vertex::Create )
                , ( bp::arg("x"), bp::arg("y"), bp::arg("z"), bp::arg("referenceLength") ) );
        
        }
        { //::Vertex::Create
        
            typedef ::boost::shared_ptr< Vertex > ( *Create_function_type )( ::boost::numeric::ublas::c_vector< double, 3 >,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > );
            
            Vertex_exposer.def( 
                "Create"
                , Create_function_type( &::Vertex::Create )
                , ( bp::arg("coords"), bp::arg("referenceLength") ) );
        
        }
        { //::Vertex::Create
        
            typedef ::boost::shared_ptr< Vertex > ( *Create_function_type )( double,double,double );
            
            Vertex_exposer.def( 
                "Create"
                , Create_function_type( &::Vertex::Create )
                , ( bp::arg("x")=0., bp::arg("y")=0., bp::arg("z")=0. ) );
        
        }
        { //::Vertex::Create
        
            typedef ::boost::shared_ptr< Vertex > ( *Create_function_type )( ::boost::numeric::ublas::c_vector< double, 3 > );
            
            Vertex_exposer.def( 
                "Create"
                , Create_function_type( &::Vertex::Create )
                , ( bp::arg("coords") ) );
        
        }
        { //::Vertex::GetIndex
        
            typedef unsigned int ( ::Vertex::*GetIndex_function_type)(  ) ;
            
            Vertex_exposer.def( 
                "GetIndex"
                , GetIndex_function_type( &::Vertex::GetIndex ) );
        
        }
        { //::Vertex::RotateAboutAxis
        
            typedef void ( ::Vertex::*RotateAboutAxis_function_type)( ::boost::numeric::ublas::c_vector< double, 3 >,double ) ;
            
            Vertex_exposer.def( 
                "RotateAboutAxis"
                , RotateAboutAxis_function_type( &::Vertex::RotateAboutAxis )
                , ( bp::arg("axis"), bp::arg("angle") ) );
        
        }
        { //::Vertex::SetIndex
        
            typedef void ( ::Vertex::*SetIndex_function_type)( unsigned int ) ;
            
            Vertex_exposer.def( 
                "SetIndex"
                , SetIndex_function_type( &::Vertex::SetIndex )
                , ( bp::arg("index") ) );
        
        }
        { //::Vertex::Translate
        
            typedef void ( ::Vertex::*Translate_function_type)( ::boost::numeric::ublas::c_vector< double, 3 > ) ;
            
            Vertex_exposer.def( 
                "Translate"
                , Translate_function_type( &::Vertex::Translate )
                , ( bp::arg("translationVector") ) );
        
        }
        Vertex_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< Vertex > >();
        bp::implicitly_convertible< boost::shared_ptr< Vertex >, boost::shared_ptr< boost::enable_shared_from_this< Vertex > > >();
        bp::implicitly_convertible< boost::shared_ptr< Vertex >, boost::shared_ptr< DimensionalChastePoint< 3 > > >();
        bp::implicitly_convertible< boost::shared_ptr< Vertex >, boost::shared_ptr< boost::enable_shared_from_this< DimensionalChastePoint< 3 > > > >();
        bp::implicitly_convertible< boost::shared_ptr< Vertex >, boost::shared_ptr< ChastePoint< 3 > > >();
    }
}
