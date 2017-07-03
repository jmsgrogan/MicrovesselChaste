#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "DimensionalChastePoint.hpp"

#include "DimensionalChastePoint3.cppwg.hpp"

namespace py = pybind11;
typedef DimensionalChastePoint<3 > DimensionalChastePoint3;
;

void register_DimensionalChastePoint3_class(py::module &m){
py::class_<DimensionalChastePoint3    >(m, "DimensionalChastePoint3")
        .def(py::init<double, double, double, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> >(), py::arg("x") = 0., py::arg("y") = 0., py::arg("z") = 0., py::arg("referenceLength") = 9.9999999999999995E-7 * unit::metres)
        .def(py::init<::boost::numeric::ublas::c_vector<double, 3>, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> >(), py::arg("coords"), py::arg("referenceLength"))
        .def_static(
            "Create", 
            (::std::shared_ptr<DimensionalChastePoint<3> >(*)(double, double, double, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &DimensionalChastePoint3::Create, 
            " " , py::arg("x"), py::arg("y"), py::arg("z"), py::arg("referenceLength"))
        .def_static(
            "Create", 
            (::std::shared_ptr<DimensionalChastePoint<3> >(*)(::boost::numeric::ublas::c_vector<double, 3>, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &DimensionalChastePoint3::Create, 
            " " , py::arg("coords"), py::arg("referenceLength"))
        .def(
            "AddAttribute", 
            (void(DimensionalChastePoint3::*)(::std::string const &, double)) &DimensionalChastePoint3::AddAttribute, 
            " " , py::arg("rAttribute"), py::arg("value"))
        .def(
            "GetDistance", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(DimensionalChastePoint3::*)(::DimensionalChastePoint<3> const &) const ) &DimensionalChastePoint3::GetDistance, 
            " " , py::arg("rLocation"))
        .def(
            "GetIndex", 
            (unsigned int(DimensionalChastePoint3::*)()) &DimensionalChastePoint3::GetIndex, 
            " " )
        .def(
            "GetAttributes", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(DimensionalChastePoint3::*)()) &DimensionalChastePoint3::GetAttributes, 
            " " )
        .def(
            "GetMidPoint", 
            (::DimensionalChastePoint<3>(DimensionalChastePoint3::*)(::DimensionalChastePoint<3> const &) const ) &DimensionalChastePoint3::GetMidPoint, 
            " " , py::arg("rLocation"))
        .def(
            "GetNorm2", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(DimensionalChastePoint3::*)()) &DimensionalChastePoint3::GetNorm2, 
            " " )
        .def(
            "GetReferenceLengthScale", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(DimensionalChastePoint3::*)() const ) &DimensionalChastePoint3::GetReferenceLengthScale, 
            " " )
        .def(
            "GetLocation", 
            (::boost::numeric::ublas::c_vector<double, 3>(DimensionalChastePoint3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &DimensionalChastePoint3::GetLocation, 
            " " , py::arg("scale"))
        .def(
            "GetLocation", 
            (::boost::numeric::ublas::c_vector<double, 3> const(DimensionalChastePoint3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>) const ) &DimensionalChastePoint3::GetLocation, 
            " " , py::arg("scale"))
        .def(
            "GetUnitVector", 
            (::boost::numeric::ublas::c_vector<double, 3>(DimensionalChastePoint3::*)() const ) &DimensionalChastePoint3::GetUnitVector, 
            " " )
        .def(
            "GetUnitTangent", 
            (::boost::numeric::ublas::c_vector<double, 3>(DimensionalChastePoint3::*)(::DimensionalChastePoint<3> const &) const ) &DimensionalChastePoint3::GetUnitTangent, 
            " " , py::arg("rLocation"))
        .def(
            "IsCoincident", 
            (bool(DimensionalChastePoint3::*)(::DimensionalChastePoint<3> const &) const ) &DimensionalChastePoint3::IsCoincident, 
            " " , py::arg("rLocation"))
        .def(
            "RotateAboutAxis", 
            (void(DimensionalChastePoint3::*)(::boost::numeric::ublas::c_vector<double, 3>, double)) &DimensionalChastePoint3::RotateAboutAxis, 
            " " , py::arg("axis"), py::arg("angle"))
        .def(
            "SetReferenceLengthScale", 
            (void(DimensionalChastePoint3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &DimensionalChastePoint3::SetReferenceLengthScale, 
            " " , py::arg("lenthScale"))
        .def(
            "SetIndex", 
            (void(DimensionalChastePoint3::*)(unsigned int)) &DimensionalChastePoint3::SetIndex, 
            " " , py::arg("index"))
        .def(
            "Translate", 
            (void(DimensionalChastePoint3::*)(::DimensionalChastePoint<3>)) &DimensionalChastePoint3::Translate, 
            " " , py::arg("rVector"))
        .def(
            "TranslateTo", 
            (void(DimensionalChastePoint3::*)(::DimensionalChastePoint<3>)) &DimensionalChastePoint3::TranslateTo, 
            " " , py::arg("rPoint"))
    ;
}
