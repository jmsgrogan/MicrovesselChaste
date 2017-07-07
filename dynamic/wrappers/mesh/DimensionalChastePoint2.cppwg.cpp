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

#include "DimensionalChastePoint2.cppwg.hpp"

namespace py = pybind11;
typedef DimensionalChastePoint<2 > DimensionalChastePoint2;
;

void register_DimensionalChastePoint2_class(py::module &m){
py::class_<DimensionalChastePoint2    >(m, "DimensionalChastePoint2")
        .def(py::init<double, double, double, ::QLength >(), py::arg("x") = 0., py::arg("y") = 0., py::arg("z") = 0., py::arg("referenceLength") = 9.9999999999999995E-7 * unit::metres)
        .def(py::init<::boost::numeric::ublas::c_vector<double, 2>, ::QLength >(), py::arg("coords"), py::arg("referenceLength"))
        .def_static(
            "Create", 
            (::std::shared_ptr<DimensionalChastePoint<2> >(*)(double, double, double, ::QLength)) &DimensionalChastePoint2::Create, 
            " " , py::arg("x"), py::arg("y"), py::arg("z"), py::arg("referenceLength") )
        .def_static(
            "Create", 
            (::std::shared_ptr<DimensionalChastePoint<2> >(*)(::boost::numeric::ublas::c_vector<double, 2>, ::QLength)) &DimensionalChastePoint2::Create, 
            " " , py::arg("coords"), py::arg("referenceLength") )
        .def(
            "AddAttribute", 
            (void(DimensionalChastePoint2::*)(::std::string const &, double)) &DimensionalChastePoint2::AddAttribute, 
            " " , py::arg("rAttribute"), py::arg("value") )
        .def(
            "GetDistance", 
            (::QLength(DimensionalChastePoint2::*)(::DimensionalChastePoint<2> const &) const ) &DimensionalChastePoint2::GetDistance, 
            " " , py::arg("rLocation") )
        .def(
            "GetIndex", 
            (unsigned int(DimensionalChastePoint2::*)()) &DimensionalChastePoint2::GetIndex, 
            " "  )
        .def(
            "GetAttributes", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(DimensionalChastePoint2::*)()) &DimensionalChastePoint2::GetAttributes, 
            " "  )
        .def(
            "GetMidPoint", 
            (::DimensionalChastePoint<2>(DimensionalChastePoint2::*)(::DimensionalChastePoint<2> const &) const ) &DimensionalChastePoint2::GetMidPoint, 
            " " , py::arg("rLocation") )
        .def(
            "GetNorm2", 
            (::QLength(DimensionalChastePoint2::*)()) &DimensionalChastePoint2::GetNorm2, 
            " "  )
        .def(
            "GetReferenceLengthScale", 
            (::QLength(DimensionalChastePoint2::*)() const ) &DimensionalChastePoint2::GetReferenceLengthScale, 
            " "  )
        .def(
            "GetLocation", 
            (::boost::numeric::ublas::c_vector<double, 2>(DimensionalChastePoint2::*)(::QLength)) &DimensionalChastePoint2::GetLocation, 
            " " , py::arg("scale") )
        .def(
            "GetLocation", 
            (::boost::numeric::ublas::c_vector<double, 2> const(DimensionalChastePoint2::*)(::QLength) const ) &DimensionalChastePoint2::GetLocation, 
            " " , py::arg("scale") )
        .def(
            "GetUnitVector", 
            (::boost::numeric::ublas::c_vector<double, 2>(DimensionalChastePoint2::*)() const ) &DimensionalChastePoint2::GetUnitVector, 
            " "  )
        .def(
            "GetUnitTangent", 
            (::boost::numeric::ublas::c_vector<double, 2>(DimensionalChastePoint2::*)(::DimensionalChastePoint<2> const &) const ) &DimensionalChastePoint2::GetUnitTangent, 
            " " , py::arg("rLocation") )
        .def(
            "IsCoincident", 
            (bool(DimensionalChastePoint2::*)(::DimensionalChastePoint<2> const &) const ) &DimensionalChastePoint2::IsCoincident, 
            " " , py::arg("rLocation") )
        .def(
            "RotateAboutAxis", 
            (void(DimensionalChastePoint2::*)(::boost::numeric::ublas::c_vector<double, 3>, double)) &DimensionalChastePoint2::RotateAboutAxis, 
            " " , py::arg("axis"), py::arg("angle") )
        .def(
            "SetReferenceLengthScale", 
            (void(DimensionalChastePoint2::*)(::QLength)) &DimensionalChastePoint2::SetReferenceLengthScale, 
            " " , py::arg("lenthScale") )
        .def(
            "SetIndex", 
            (void(DimensionalChastePoint2::*)(unsigned int)) &DimensionalChastePoint2::SetIndex, 
            " " , py::arg("index") )
        .def(
            "Translate", 
            (void(DimensionalChastePoint2::*)(::DimensionalChastePoint<2>)) &DimensionalChastePoint2::Translate, 
            " " , py::arg("rVector") )
        .def(
            "TranslateTo", 
            (void(DimensionalChastePoint2::*)(::DimensionalChastePoint<2>)) &DimensionalChastePoint2::TranslateTo, 
            " " , py::arg("rPoint") )
    ;
}
