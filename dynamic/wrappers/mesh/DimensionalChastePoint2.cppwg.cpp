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

#include "Vertex2.cppwg.hpp"

namespace py = pybind11;
typedef Vertex<2 > Vertex2;
;

void register_Vertex2_class(py::module &m){
py::class_<Vertex2    >(m, "Vertex2")
        .def(py::init<double, double, double, ::QLength >(), py::arg("x") = 0., py::arg("y") = 0., py::arg("z") = 0., py::arg("referenceLength") = 9.9999999999999995E-7 * unit::metres)
        .def(py::init<::boost::numeric::ublas::c_vector<double, 2>, ::QLength >(), py::arg("coords"), py::arg("referenceLength"))
        .def_static(
            "Create", 
            (::std::shared_ptr<Vertex<2> >(*)(double, double, double, ::QLength)) &Vertex2::Create, 
            " " , py::arg("x"), py::arg("y"), py::arg("z"), py::arg("referenceLength") )
        .def_static(
            "Create", 
            (::std::shared_ptr<Vertex<2> >(*)(::boost::numeric::ublas::c_vector<double, 2>, ::QLength)) &Vertex2::Create, 
            " " , py::arg("coords"), py::arg("referenceLength") )
        .def(
            "AddAttribute", 
            (void(Vertex2::*)(::std::string const &, double)) &Vertex2::AddAttribute, 
            " " , py::arg("rAttribute"), py::arg("value") )
        .def(
            "GetDistance", 
            (::QLength(Vertex2::*)(::Vertex<2> const &) const ) &Vertex2::GetDistance, 
            " " , py::arg("rLocation") )
        .def(
            "GetIndex", 
            (unsigned int(Vertex2::*)()) &Vertex2::GetIndex, 
            " "  )
        .def(
            "GetAttributes", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(Vertex2::*)()) &Vertex2::GetAttributes, 
            " "  )
        .def(
            "GetMidPoint", 
            (::Vertex<2>(Vertex2::*)(::Vertex<2> const &) const ) &Vertex2::GetMidPoint, 
            " " , py::arg("rLocation") )
        .def(
            "GetNorm2", 
            (::QLength(Vertex2::*)()) &Vertex2::GetNorm2, 
            " "  )
        .def(
            "GetReferenceLengthScale", 
            (::QLength(Vertex2::*)() const ) &Vertex2::GetReferenceLengthScale, 
            " "  )
        .def(
            "GetLocation", 
            (::boost::numeric::ublas::c_vector<double, 2>(Vertex2::*)(::QLength)) &Vertex2::GetLocation, 
            " " , py::arg("scale") )
        .def(
            "GetLocation", 
            (::boost::numeric::ublas::c_vector<double, 2> const(Vertex2::*)(::QLength) const ) &Vertex2::GetLocation, 
            " " , py::arg("scale") )
        .def(
            "GetUnitVector", 
            (::boost::numeric::ublas::c_vector<double, 2>(Vertex2::*)() const ) &Vertex2::GetUnitVector, 
            " "  )
        .def(
            "GetUnitTangent", 
            (::boost::numeric::ublas::c_vector<double, 2>(Vertex2::*)(::Vertex<2> const &) const ) &Vertex2::GetUnitTangent, 
            " " , py::arg("rLocation") )
        .def(
            "IsCoincident", 
            (bool(Vertex2::*)(::Vertex<2> const &) const ) &Vertex2::IsCoincident, 
            " " , py::arg("rLocation") )
        .def(
            "RotateAboutAxis", 
            (void(Vertex2::*)(::boost::numeric::ublas::c_vector<double, 3>, double)) &Vertex2::RotateAboutAxis, 
            " " , py::arg("axis"), py::arg("angle") )
        .def(
            "SetReferenceLengthScale", 
            (void(Vertex2::*)(::QLength)) &Vertex2::SetReferenceLengthScale, 
            " " , py::arg("lenthScale") )
        .def(
            "SetIndex", 
            (void(Vertex2::*)(unsigned int)) &Vertex2::SetIndex, 
            " " , py::arg("index") )
        .def(
            "Translate", 
            (void(Vertex2::*)(::Vertex<2>)) &Vertex2::Translate, 
            " " , py::arg("rVector") )
        .def(
            "TranslateTo", 
            (void(Vertex2::*)(::Vertex<2>)) &Vertex2::TranslateTo, 
            " " , py::arg("rPoint") )
    ;
}
