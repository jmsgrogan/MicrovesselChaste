// This file has been generated by Py++.

#include "boost/python.hpp"

#include "utility_headers.hpp"

namespace bp = boost::python;

struct BaseParameterInstance_wrapper : BaseParameterInstance, bp::wrapper< BaseParameterInstance > {

    BaseParameterInstance_wrapper(BaseParameterInstance const & arg )
    : BaseParameterInstance( arg )
      , bp::wrapper< BaseParameterInstance >(){
        // copy constructor
        
    }

    BaseParameterInstance_wrapper( )
    : BaseParameterInstance( )
      , bp::wrapper< BaseParameterInstance >(){
        // null constructor
    
    }

    BaseParameterInstance_wrapper(::std::string const & rName, ::std::string const & rShortDescription, ::std::string const & rSymbol, ::std::string const & rBibliographicInfromation )
    : BaseParameterInstance( rName, rShortDescription, rSymbol, rBibliographicInfromation )
      , bp::wrapper< BaseParameterInstance >(){
        // constructor
    
    }

    virtual ::std::string GetValueAsString(  ) {
        if( bp::override func_GetValueAsString = this->get_override( "GetValueAsString" ) )
            return func_GetValueAsString(  );
        else{
            return this->BaseParameterInstance::GetValueAsString(  );
        }
    }
    
    ::std::string default_GetValueAsString(  ) {
        return BaseParameterInstance::GetValueAsString( );
    }

};

struct BaseUnits_wrapper : BaseUnits, bp::wrapper< BaseUnits > {

    BaseUnits_wrapper( )
    : BaseUnits( )
      , bp::wrapper< BaseUnits >(){
        // null constructor
    
    }

};

struct ParameterCollection_wrapper : ParameterCollection, bp::wrapper< ParameterCollection > {

    ParameterCollection_wrapper( )
    : ParameterCollection( )
      , bp::wrapper< ParameterCollection >(){
        // null constructor
    
    }

};

BOOST_PYTHON_MODULE(_chaste_project_Microvessel_utility){
    { //::BaseParameterInstance
        typedef bp::class_< BaseParameterInstance_wrapper > BaseParameterInstance_exposer_t;
        BaseParameterInstance_exposer_t BaseParameterInstance_exposer = BaseParameterInstance_exposer_t( "BaseParameterInstance", bp::init< >() );
        bp::scope BaseParameterInstance_scope( BaseParameterInstance_exposer );
        BaseParameterInstance_exposer.def( bp::init< std::string const &, std::string const &, std::string const &, std::string const & >(( bp::arg("rName"), bp::arg("rShortDescription"), bp::arg("rSymbol"), bp::arg("rBibliographicInfromation") )) );
        { //::BaseParameterInstance::Create
        
            typedef ::boost::shared_ptr< BaseParameterInstance > ( *Create_function_type )(  );
            
            BaseParameterInstance_exposer.def( 
                "Create"
                , Create_function_type( &::BaseParameterInstance::Create ) );
        
        }
        { //::BaseParameterInstance::GetBibliographicInformation
        
            typedef ::std::string ( ::BaseParameterInstance::*GetBibliographicInformation_function_type)(  ) ;
            
            BaseParameterInstance_exposer.def( 
                "GetBibliographicInformation"
                , GetBibliographicInformation_function_type( &::BaseParameterInstance::GetBibliographicInformation ) );
        
        }
        { //::BaseParameterInstance::GetName
        
            typedef ::std::string ( ::BaseParameterInstance::*GetName_function_type)(  ) ;
            
            BaseParameterInstance_exposer.def( 
                "GetName"
                , GetName_function_type( &::BaseParameterInstance::GetName ) );
        
        }
        { //::BaseParameterInstance::GetShortDescription
        
            typedef ::std::string ( ::BaseParameterInstance::*GetShortDescription_function_type)(  ) ;
            
            BaseParameterInstance_exposer.def( 
                "GetShortDescription"
                , GetShortDescription_function_type( &::BaseParameterInstance::GetShortDescription ) );
        
        }
        { //::BaseParameterInstance::GetSymbol
        
            typedef ::std::string ( ::BaseParameterInstance::*GetSymbol_function_type)(  ) ;
            
            BaseParameterInstance_exposer.def( 
                "GetSymbol"
                , GetSymbol_function_type( &::BaseParameterInstance::GetSymbol ) );
        
        }
        { //::BaseParameterInstance::GetValueAsString
        
            typedef ::std::string ( ::BaseParameterInstance::*GetValueAsString_function_type)(  ) ;
            typedef ::std::string ( BaseParameterInstance_wrapper::*default_GetValueAsString_function_type)(  ) ;
            
            BaseParameterInstance_exposer.def( 
                "GetValueAsString"
                , GetValueAsString_function_type(&::BaseParameterInstance::GetValueAsString)
                , default_GetValueAsString_function_type(&BaseParameterInstance_wrapper::default_GetValueAsString) );
        
        }
        { //::BaseParameterInstance::RegisterWithCollection
        
            typedef void ( ::BaseParameterInstance::*RegisterWithCollection_function_type)( ::std::string const & ) ;
            
            BaseParameterInstance_exposer.def( 
                "RegisterWithCollection"
                , RegisterWithCollection_function_type( &::BaseParameterInstance::RegisterWithCollection )
                , ( bp::arg("rCallingClass") ) );
        
        }
        { //::BaseParameterInstance::SetBibliographicInformation
        
            typedef void ( ::BaseParameterInstance::*SetBibliographicInformation_function_type)( ::std::string const & ) ;
            
            BaseParameterInstance_exposer.def( 
                "SetBibliographicInformation"
                , SetBibliographicInformation_function_type( &::BaseParameterInstance::SetBibliographicInformation )
                , ( bp::arg("rSourceInformation") ) );
        
        }
        { //::BaseParameterInstance::SetName
        
            typedef void ( ::BaseParameterInstance::*SetName_function_type)( ::std::string const & ) ;
            
            BaseParameterInstance_exposer.def( 
                "SetName"
                , SetName_function_type( &::BaseParameterInstance::SetName )
                , ( bp::arg("rName") ) );
        
        }
        { //::BaseParameterInstance::SetShortDescription
        
            typedef void ( ::BaseParameterInstance::*SetShortDescription_function_type)( ::std::string const & ) ;
            
            BaseParameterInstance_exposer.def( 
                "SetShortDescription"
                , SetShortDescription_function_type( &::BaseParameterInstance::SetShortDescription )
                , ( bp::arg("rShortDescription") ) );
        
        }
        { //::BaseParameterInstance::SetSymbol
        
            typedef void ( ::BaseParameterInstance::*SetSymbol_function_type)( ::std::string const & ) ;
            
            BaseParameterInstance_exposer.def( 
                "SetSymbol"
                , SetSymbol_function_type( &::BaseParameterInstance::SetSymbol )
                , ( bp::arg("rSymbol") ) );
        
        }
        BaseParameterInstance_exposer.staticmethod( "Create" );
        bp::register_ptr_to_python< boost::shared_ptr< BaseParameterInstance > >();
        bp::implicitly_convertible< boost::shared_ptr< BaseParameterInstance >, boost::shared_ptr< boost::enable_shared_from_this< BaseParameterInstance > > >();
    }

    { //::BaseUnits
        typedef bp::class_< BaseUnits_wrapper, boost::noncopyable > BaseUnits_exposer_t;
        BaseUnits_exposer_t BaseUnits_exposer = BaseUnits_exposer_t( "BaseUnits", bp::no_init );
        bp::scope BaseUnits_scope( BaseUnits_exposer );
        BaseUnits_exposer.def( bp::init< >() );
        { //::BaseUnits::Destroy
        
            typedef void ( *Destroy_function_type )(  );
            
            BaseUnits_exposer.def( 
                "Destroy"
                , Destroy_function_type( &::BaseUnits::Destroy ) );
        
        }
        { //::BaseUnits::GetReferenceConcentrationScale
        
            typedef ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ( ::BaseUnits::*GetReferenceConcentrationScale_function_type)(  ) ;
            
            BaseUnits_exposer.def( 
                "GetReferenceConcentrationScale"
                , GetReferenceConcentrationScale_function_type( &::BaseUnits::GetReferenceConcentrationScale ) );
        
        }
        { //::BaseUnits::GetReferenceLengthScale
        
            typedef ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ( ::BaseUnits::*GetReferenceLengthScale_function_type)(  ) ;
            
            BaseUnits_exposer.def( 
                "GetReferenceLengthScale"
                , GetReferenceLengthScale_function_type( &::BaseUnits::GetReferenceLengthScale ) );
        
        }
        { //::BaseUnits::GetReferenceTimeScale
        
            typedef ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ( ::BaseUnits::*GetReferenceTimeScale_function_type)(  ) ;
            
            BaseUnits_exposer.def( 
                "GetReferenceTimeScale"
                , GetReferenceTimeScale_function_type( &::BaseUnits::GetReferenceTimeScale ) );
        
        }
        { //::BaseUnits::Instance
        
            typedef ::BaseUnits * ( *Instance_function_type )(  );
            
            BaseUnits_exposer.def( 
                "Instance"
                , Instance_function_type( &::BaseUnits::Instance )
                    /* undefined call policies */ );
        
        }
        { //::BaseUnits::SetReferenceConcentrationScale
        
            typedef void ( ::BaseUnits::*SetReferenceConcentrationScale_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            BaseUnits_exposer.def( 
                "SetReferenceConcentrationScale"
                , SetReferenceConcentrationScale_function_type( &::BaseUnits::SetReferenceConcentrationScale )
                , ( bp::arg("referenceConcentrationScale") ) );
        
        }
        { //::BaseUnits::SetReferenceLengthScale
        
            typedef void ( ::BaseUnits::*SetReferenceLengthScale_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            BaseUnits_exposer.def( 
                "SetReferenceLengthScale"
                , SetReferenceLengthScale_function_type( &::BaseUnits::SetReferenceLengthScale )
                , ( bp::arg("referenceLengthScale") ) );
        
        }
        { //::BaseUnits::SetReferenceTimeScale
        
            typedef void ( ::BaseUnits::*SetReferenceTimeScale_function_type)( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::time_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ) ;
            
            BaseUnits_exposer.def( 
                "SetReferenceTimeScale"
                , SetReferenceTimeScale_function_type( &::BaseUnits::SetReferenceTimeScale )
                , ( bp::arg("referenceTimeScale") ) );
        
        }
        { //::BaseUnits::SharedInstance
        
            typedef ::boost::shared_ptr< BaseUnits > ( *SharedInstance_function_type )(  );
            
            BaseUnits_exposer.def( 
                "SharedInstance"
                , SharedInstance_function_type( &::BaseUnits::SharedInstance ) );
        
        }
        BaseUnits_exposer.staticmethod( "Destroy" );
        BaseUnits_exposer.staticmethod( "Instance" );
        BaseUnits_exposer.staticmethod( "SharedInstance" );
        bp::register_ptr_to_python< boost::shared_ptr< BaseUnits > >();
        bp::implicitly_convertible< boost::shared_ptr< BaseUnits >, boost::shared_ptr< SerializableSingleton< BaseUnits > > >();
    }

    { //::ParameterCollection
        typedef bp::class_< ParameterCollection_wrapper, boost::noncopyable > ParameterCollection_exposer_t;
        ParameterCollection_exposer_t ParameterCollection_exposer = ParameterCollection_exposer_t( "ParameterCollection", bp::no_init );
        bp::scope ParameterCollection_scope( ParameterCollection_exposer );
        ParameterCollection_exposer.def( bp::init< >() );
        { //::ParameterCollection::AddParameter
        
            typedef void ( ::ParameterCollection::*AddParameter_function_type)( ::boost::shared_ptr< BaseParameterInstance >,::std::string const & ) ;
            
            ParameterCollection_exposer.def( 
                "AddParameter"
                , AddParameter_function_type( &::ParameterCollection::AddParameter )
                , ( bp::arg("pParameter"), bp::arg("rFirstInstantiated") ) );
        
        }
        { //::ParameterCollection::Destroy
        
            typedef void ( *Destroy_function_type )(  );
            
            ParameterCollection_exposer.def( 
                "Destroy"
                , Destroy_function_type( &::ParameterCollection::Destroy ) );
        
        }
        { //::ParameterCollection::DumpToFile
        
            typedef void ( ::ParameterCollection::*DumpToFile_function_type)( ::std::string const & ) ;
            
            ParameterCollection_exposer.def( 
                "DumpToFile"
                , DumpToFile_function_type( &::ParameterCollection::DumpToFile )
                , ( bp::arg("rFilename") ) );
        
        }
        { //::ParameterCollection::GetParameter
        
            typedef ::boost::shared_ptr< BaseParameterInstance > ( ::ParameterCollection::*GetParameter_function_type)( ::std::string const & ) ;
            
            ParameterCollection_exposer.def( 
                "GetParameter"
                , GetParameter_function_type( &::ParameterCollection::GetParameter )
                , ( bp::arg("rName") ) );
        
        }
        { //::ParameterCollection::Instance
        
            typedef ::ParameterCollection * ( *Instance_function_type )(  );
            
            ParameterCollection_exposer.def( 
                "Instance"
                , Instance_function_type( &::ParameterCollection::Instance )
                    /* undefined call policies */ );
        
        }
        { //::ParameterCollection::SharedInstance
        
            typedef ::boost::shared_ptr< ParameterCollection > ( *SharedInstance_function_type )(  );
            
            ParameterCollection_exposer.def( 
                "SharedInstance"
                , SharedInstance_function_type( &::ParameterCollection::SharedInstance ) );
        
        }
        ParameterCollection_exposer.staticmethod( "Destroy" );
        ParameterCollection_exposer.staticmethod( "Instance" );
        ParameterCollection_exposer.staticmethod( "SharedInstance" );
        bp::register_ptr_to_python< boost::shared_ptr< ParameterCollection > >();
        bp::implicitly_convertible< boost::shared_ptr< ParameterCollection >, boost::shared_ptr< SerializableSingleton< ParameterCollection > > >();
    }
}
