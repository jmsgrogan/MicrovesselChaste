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

#include "PythonObjectConverters.hpp"
#include "Vertex2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef Vertex<2 > Vertex2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_Vertex2_class(py::module &m){
py::class_<Vertex2  , std::shared_ptr<Vertex2 >   >(m, "Vertex2")
        .def(py::init<::QLength, ::QLength, ::QLength >(), py::arg("x") = 0_m, py::arg("y") = 0_m, py::arg("z") = 0_m)
        .def(py::init<::boost::numeric::ublas::c_vector<double, 2> const &, ::QLength >(), py::arg("rCoords"), py::arg("referenceLength"))
        .def(py::init<::RVectorQuantity<RQuantity<std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, 2> >(), py::arg("loc"))
        .def_static(
            "Create", 
            (::std::shared_ptr<Vertex<2> >(*)(::QLength, ::QLength, ::QLength)) &Vertex2::Create, 
            " " , py::arg("x") = 0_m, py::arg("y") = 0_m, py::arg("z") = 0_m )
        .def_static(
            "Create", 
            (::std::shared_ptr<Vertex<2> >(*)(::boost::numeric::ublas::c_vector<double, 2> const &, ::QLength)) &Vertex2::Create, 
            " " , py::arg("rCoords"), py::arg("referenceLength") )
        .def_static(
            "Create", 
            (::std::shared_ptr<Vertex<2> >(*)(::Vertex<2> const &)) &Vertex2::Create, 
            " " , py::arg("loc") )
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
            "Convert3", 
            (::boost::numeric::ublas::c_vector<double, 3>(Vertex2::*)(::QLength) const ) &Vertex2::Convert3, 
            " " , py::arg("referenceLength") )
        .def(
            "Convert", 
            (::boost::numeric::ublas::c_vector<double, 2>(Vertex2::*)(::QLength) const ) &Vertex2::Convert, 
            " " , py::arg("referenceLength") )
        .def(
            "rGetLocation", 
            (::RVectorQuantity<RQuantity<std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, 2> &(Vertex2::*)()) &Vertex2::rGetLocation, 
            " "  , py::return_value_policy::reference_internal)
        .def(
            "rGetLocation", 
            (::RVectorQuantity<RQuantity<std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, 2> const &(Vertex2::*)() const ) &Vertex2::rGetLocation, 
            " "  , py::return_value_policy::reference_internal)
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
            "SetIndex", 
            (void(Vertex2::*)(unsigned int)) &Vertex2::SetIndex, 
            " " , py::arg("index") )
        .def(
            "Translate", 
            (void(Vertex2::*)(::Vertex<2> const &)) &Vertex2::Translate, 
            " " , py::arg("rVector") )
        .def(
            "TranslateTo", 
            (void(Vertex2::*)(::Vertex<2> const &)) &Vertex2::TranslateTo, 
            " " , py::arg("rPoint") )
    ;
}
