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
#include "MappableGridGenerator.hpp"

#include "PythonObjectConverters.hpp"
#include "MappableGridGenerator3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef MappableGridGenerator<3 > MappableGridGenerator3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_MappableGridGenerator3_class(py::module &m){
py::class_<MappableGridGenerator3  , std::shared_ptr<MappableGridGenerator3 >   >(m, "MappableGridGenerator3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<MappableGridGenerator<3> >(*)()) &MappableGridGenerator3::Create, 
            " "  )
        .def(
            "GeneratePlane", 
            (::std::shared_ptr<Part<3> >(MappableGridGenerator3::*)(unsigned int, unsigned int, bool, bool)) &MappableGridGenerator3::GeneratePlane, 
            " " , py::arg("numX"), py::arg("numY"), py::arg("isShell") = false, py::arg("withEndCaps") = true )
        .def(
            "GenerateCylinder", 
            (::std::shared_ptr<Part<3> >(MappableGridGenerator3::*)(::QLength, ::QLength, ::QLength, unsigned int, unsigned int, double)) &MappableGridGenerator3::GenerateCylinder, 
            " " , py::arg("cylinderRadius"), py::arg("cylinderThickness"), py::arg("cylinderHeight"), py::arg("numX"), py::arg("numY"), py::arg("cylinderAngle") = 2. * 3.1415926535897931 )
        .def(
            "GenerateHemisphere", 
            (::std::shared_ptr<Part<3> >(MappableGridGenerator3::*)(::QLength, ::QLength, unsigned int, unsigned int, double, double)) &MappableGridGenerator3::GenerateHemisphere, 
            " " , py::arg("sphereRadius"), py::arg("sphereThickness"), py::arg("numX"), py::arg("numY"), py::arg("sphereAzimuthAngle") = 2. * 3.1415926535897931, py::arg("spherePolarAngle") = 0.5 * 3.1415926535897931 )
        .def(
            "GetReferenceLengthScale", 
            (::QLength(MappableGridGenerator3::*)()) &MappableGridGenerator3::GetReferenceLengthScale, 
            " "  )
    ;
}
