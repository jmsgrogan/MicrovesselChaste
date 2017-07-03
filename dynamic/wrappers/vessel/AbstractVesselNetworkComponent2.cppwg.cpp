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

#include "AbstractVesselNetworkComponent2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractVesselNetworkComponent<2 > AbstractVesselNetworkComponent2;
;
typedef unsigned int unsignedint;
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double;
typedef ::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > > _std_vectorstd_basic_stringchar_std_allocatorstd_basic_stringchar;
typedef ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> _boost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double;

class AbstractVesselNetworkComponent2_Overloads : public AbstractVesselNetworkComponent2{
    public:
    using AbstractVesselNetworkComponent2::AbstractVesselNetworkComponent;
    unsigned int GetId() const  override {
        PYBIND11_OVERLOAD(
            unsignedint,
            AbstractVesselNetworkComponent2,
            GetId,
            );
    }
    double GetOutputDataValue(::std::string const & rKey) override {
        PYBIND11_OVERLOAD(
            double,
            AbstractVesselNetworkComponent2,
            GetOutputDataValue,
            rKey);
    }
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() override {
        PYBIND11_OVERLOAD_PURE(
            _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double,
            AbstractVesselNetworkComponent2,
            GetOutputData,
            );
    }
    ::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > > GetOutputDataKeys() override {
        PYBIND11_OVERLOAD(
            _std_vectorstd_basic_stringchar_std_allocatorstd_basic_stringchar,
            AbstractVesselNetworkComponent2,
            GetOutputDataKeys,
            );
    }
    ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> GetRadius() const  override {
        PYBIND11_OVERLOAD(
            _boost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double,
            AbstractVesselNetworkComponent2,
            GetRadius,
            );
    }
    void SetId(unsigned int id) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponent2,
            SetId,
            id);
    }
    void SetOutputData(::std::string const & rKey, double value) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponent2,
            SetOutputData,
            rKey, 
value);
    }
    void SetRadius(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> radius) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponent2,
            SetRadius,
            radius);
    }

};
void register_AbstractVesselNetworkComponent2_class(py::module &m){
py::class_<AbstractVesselNetworkComponent2 , AbstractVesselNetworkComponent2_Overloads   >(m, "AbstractVesselNetworkComponent2")
        .def(
            "GetId", 
            (unsigned int(AbstractVesselNetworkComponent2::*)() const ) &AbstractVesselNetworkComponent2::GetId, 
            " " )
        .def(
            "GetOutputDataValue", 
            (double(AbstractVesselNetworkComponent2::*)(::std::string const &)) &AbstractVesselNetworkComponent2::GetOutputDataValue, 
            " " , py::arg("rKey"))
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(AbstractVesselNetworkComponent2::*)()) &AbstractVesselNetworkComponent2::GetOutputData, 
            " " )
        .def(
            "GetOutputDataKeys", 
            (::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > >(AbstractVesselNetworkComponent2::*)()) &AbstractVesselNetworkComponent2::GetOutputDataKeys, 
            " " )
        .def(
            "GetRadius", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(AbstractVesselNetworkComponent2::*)() const ) &AbstractVesselNetworkComponent2::GetRadius, 
            " " )
        .def(
            "SetId", 
            (void(AbstractVesselNetworkComponent2::*)(unsigned int)) &AbstractVesselNetworkComponent2::SetId, 
            " " , py::arg("id"))
        .def(
            "SetOutputData", 
            (void(AbstractVesselNetworkComponent2::*)(::std::string const &, double)) &AbstractVesselNetworkComponent2::SetOutputData, 
            " " , py::arg("rKey"), py::arg("value"))
        .def(
            "SetRadius", 
            (void(AbstractVesselNetworkComponent2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AbstractVesselNetworkComponent2::SetRadius, 
            " " , py::arg("radius"))
    ;
}
