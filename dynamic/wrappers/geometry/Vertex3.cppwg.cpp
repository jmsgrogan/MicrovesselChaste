#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "vtkPolyData.h"
#include "Vertex.hpp"

#include "Vertex3.cppwg.hpp"

namespace py = pybind11;
typedef Vertex<3 > Vertex3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_Vertex3_class(py::module &m){
py::class_<Vertex3  , std::shared_ptr<Vertex3 >   >(m, "Vertex3")
        .def(py::init<::QLength, ::QLength, ::QLength >(), py::arg("x") = 0_m, py::arg("y") = 0_m, py::arg("z") = 0_m)
        .def(py::init<::boost::numeric::ublas::c_vector<double, 3> const &, ::QLength >(), py::arg("rCoords"), py::arg("referenceLength"))
        .def(py::init<::RVectorQuantity<RQuantity<std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, 3> >(), py::arg("loc"))
        .def_static(
            "Create", 
            (::std::shared_ptr<Vertex<3> >(*)(::QLength, ::QLength, ::QLength)) &Vertex3::Create, 
            " " , py::arg("x") = 0_m, py::arg("y") = 0_m, py::arg("z") = 0_m )
        .def_static(
            "Create", 
            (::std::shared_ptr<Vertex<3> >(*)(::boost::numeric::ublas::c_vector<double, 3> const &, ::QLength)) &Vertex3::Create, 
            " " , py::arg("rCoords"), py::arg("referenceLength") )
        .def_static(
            "Create", 
            (::std::shared_ptr<Vertex<3> >(*)(::Vertex<3> const &)) &Vertex3::Create, 
            " " , py::arg("loc") )
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
            "Convert3", 
            (::boost::numeric::ublas::c_vector<double, 3>(Vertex3::*)(::QLength) const ) &Vertex3::Convert3, 
            " " , py::arg("referenceLength") )
        .def(
            "Convert", 
            (::boost::numeric::ublas::c_vector<double, 3>(Vertex3::*)(::QLength) const ) &Vertex3::Convert, 
            " " , py::arg("referenceLength") )
        .def(
            "rGetLocation", 
            (::RVectorQuantity<RQuantity<std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, 3> &(Vertex3::*)()) &Vertex3::rGetLocation, 
            " "  , py::return_value_policy::reference_internal)
        .def(
            "rGetLocation", 
            (::RVectorQuantity<RQuantity<std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, 3> const &(Vertex3::*)() const ) &Vertex3::rGetLocation, 
            " "  , py::return_value_policy::reference_internal)
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
            "SetIndex", 
            (void(Vertex3::*)(unsigned int)) &Vertex3::SetIndex, 
            " " , py::arg("index") )
        .def(
            "Translate", 
            (void(Vertex3::*)(::Vertex<3> const &)) &Vertex3::Translate, 
            " " , py::arg("rVector") )
        .def(
            "TranslateTo", 
            (void(Vertex3::*)(::Vertex<3> const &)) &Vertex3::TranslateTo, 
            " " , py::arg("rPoint") )
    ;
}
