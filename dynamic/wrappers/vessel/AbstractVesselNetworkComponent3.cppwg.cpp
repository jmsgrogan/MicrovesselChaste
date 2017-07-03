#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AbstractVesselNetworkComponent.hpp"

#include "AbstractVesselNetworkComponent3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractVesselNetworkComponent<3 > AbstractVesselNetworkComponent3;
;
typedef unsigned int unsignedint;
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double;
typedef ::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > > _std_vectorstd_basic_stringchar_std_allocatorstd_basic_stringchar;
typedef ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> _boost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double;

class AbstractVesselNetworkComponent3_Overloads : public AbstractVesselNetworkComponent3{
    public:
    using AbstractVesselNetworkComponent3::AbstractVesselNetworkComponent;
    unsigned int GetId() const  override {
        PYBIND11_OVERLOAD(
            unsignedint,
            AbstractVesselNetworkComponent3,
            GetId,
            );
    }
    double GetOutputDataValue(::std::string const & rKey) override {
        PYBIND11_OVERLOAD(
            double,
            AbstractVesselNetworkComponent3,
            GetOutputDataValue,
            rKey);
    }
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() override {
        PYBIND11_OVERLOAD_PURE(
            _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double,
            AbstractVesselNetworkComponent3,
            GetOutputData,
            );
    }
    ::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > > GetOutputDataKeys() override {
        PYBIND11_OVERLOAD(
            _std_vectorstd_basic_stringchar_std_allocatorstd_basic_stringchar,
            AbstractVesselNetworkComponent3,
            GetOutputDataKeys,
            );
    }
    ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> GetRadius() const  override {
        PYBIND11_OVERLOAD(
            _boost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double,
            AbstractVesselNetworkComponent3,
            GetRadius,
            );
    }
    void SetId(unsigned int id) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponent3,
            SetId,
            id);
    }
    void SetOutputData(::std::string const & rKey, double value) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponent3,
            SetOutputData,
            rKey, 
value);
    }
    void SetRadius(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> radius) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponent3,
            SetRadius,
            radius);
    }

};
void register_AbstractVesselNetworkComponent3_class(py::module &m){
py::class_<AbstractVesselNetworkComponent3 , AbstractVesselNetworkComponent3_Overloads   >(m, "AbstractVesselNetworkComponent3")
        .def(
            "GetId", 
            (unsigned int(AbstractVesselNetworkComponent3::*)() const ) &AbstractVesselNetworkComponent3::GetId, 
            " " )
        .def(
            "GetOutputDataValue", 
            (double(AbstractVesselNetworkComponent3::*)(::std::string const &)) &AbstractVesselNetworkComponent3::GetOutputDataValue, 
            " " , py::arg("rKey"))
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(AbstractVesselNetworkComponent3::*)()) &AbstractVesselNetworkComponent3::GetOutputData, 
            " " )
        .def(
            "GetOutputDataKeys", 
            (::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > >(AbstractVesselNetworkComponent3::*)()) &AbstractVesselNetworkComponent3::GetOutputDataKeys, 
            " " )
        .def(
            "GetRadius", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(AbstractVesselNetworkComponent3::*)() const ) &AbstractVesselNetworkComponent3::GetRadius, 
            " " )
        .def(
            "SetId", 
            (void(AbstractVesselNetworkComponent3::*)(unsigned int)) &AbstractVesselNetworkComponent3::SetId, 
            " " , py::arg("id"))
        .def(
            "SetOutputData", 
            (void(AbstractVesselNetworkComponent3::*)(::std::string const &, double)) &AbstractVesselNetworkComponent3::SetOutputData, 
            " " , py::arg("rKey"), py::arg("value"))
        .def(
            "SetRadius", 
            (void(AbstractVesselNetworkComponent3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AbstractVesselNetworkComponent3::SetRadius, 
            " " , py::arg("radius"))
    ;
}
