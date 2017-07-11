#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "Vertex.hpp"

#include "Vertex3.cppwg.hpp"

namespace py = pybind11;
typedef Vertex<3 > Vertex3;
;

void register_Vertex3_class(py::module &m){
py::class_<Vertex3    >(m, "Vertex3")
        .def(py::init<double, double, double, ::QLength >(), py::arg("x") = 0., py::arg("y") = 0., py::arg("z") = 0., py::arg("referenceLength") = 9.9999999999999995E-7 * unit::metres)
        .def(py::init<::boost::numeric::ublas::c_vector<double, 3>, ::QLength >(), py::arg("coords"), py::arg("referenceLength"))
        .def_static(
            "Create", 
            (::std::shared_ptr<Vertex<3> >(*)(double, double, double, ::QLength)) &Vertex3::Create, 
            " " , py::arg("x"), py::arg("y"), py::arg("z"), py::arg("referenceLength") )
        .def_static(
            "Create", 
            (::std::shared_ptr<Vertex<3> >(*)(::boost::numeric::ublas::c_vector<double, 3>, ::QLength)) &Vertex3::Create, 
            " " , py::arg("coords"), py::arg("referenceLength") )
        .def(
            "AddAttribute", 
            (void(Vertex3::*)(::std::string const &, double)) &Vertex3::AddAttribute, 
            " " , py::arg("rAttribute"), py::arg("value") )
        .def(
            "GetDistance", 
            (::QLength(Vertex3::*)(::Vertex<3> const &) const ) &Vertex3::GetDistance, 
            " " , py::arg("rLocation") )
        .def(
            "GetIndex", 
            (unsigned int(Vertex3::*)()) &Vertex3::GetIndex, 
            " "  )
        .def(
            "GetAttributes", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(Vertex3::*)()) &Vertex3::GetAttributes, 
            " "  )
        .def(
            "GetMidPoint", 
            (::Vertex<3>(Vertex3::*)(::Vertex<3> const &) const ) &Vertex3::GetMidPoint, 
            " " , py::arg("rLocation") )
        .def(
            "GetNorm2", 
            (::QLength(Vertex3::*)()) &Vertex3::GetNorm2, 
            " "  )
        .def(
            "GetReferenceLengthScale", 
            (::QLength(Vertex3::*)() const ) &Vertex3::GetReferenceLengthScale, 
            " "  )
        .def(
            "GetLocation", 
            (::boost::numeric::ublas::c_vector<double, 3>(Vertex3::*)(::QLength)) &Vertex3::GetLocation, 
            " " , py::arg("scale") )
        .def(
            "GetLocation", 
            (::boost::numeric::ublas::c_vector<double, 3> const(Vertex3::*)(::QLength) const ) &Vertex3::GetLocation, 
            " " , py::arg("scale") )
        .def(
            "GetUnitVector", 
            (::boost::numeric::ublas::c_vector<double, 3>(Vertex3::*)() const ) &Vertex3::GetUnitVector, 
            " "  )
        .def(
            "GetUnitTangent", 
            (::boost::numeric::ublas::c_vector<double, 3>(Vertex3::*)(::Vertex<3> const &) const ) &Vertex3::GetUnitTangent, 
            " " , py::arg("rLocation") )
        .def(
            "IsCoincident", 
            (bool(Vertex3::*)(::Vertex<3> const &) const ) &Vertex3::IsCoincident, 
            " " , py::arg("rLocation") )
        .def(
            "RotateAboutAxis", 
            (void(Vertex3::*)(::boost::numeric::ublas::c_vector<double, 3>, double)) &Vertex3::RotateAboutAxis, 
            " " , py::arg("axis"), py::arg("angle") )
        .def(
            "SetReferenceLengthScale", 
            (void(Vertex3::*)(::QLength)) &Vertex3::SetReferenceLengthScale, 
            " " , py::arg("lenthScale") )
        .def(
            "SetIndex", 
            (void(Vertex3::*)(unsigned int)) &Vertex3::SetIndex, 
            " " , py::arg("index") )
        .def(
            "Translate", 
            (void(Vertex3::*)(::Vertex<3>)) &Vertex3::Translate, 
            " " , py::arg("rVector") )
        .def(
            "TranslateTo", 
            (void(Vertex3::*)(::Vertex<3>)) &Vertex3::TranslateTo, 
            " " , py::arg("rPoint") )
    ;
}
