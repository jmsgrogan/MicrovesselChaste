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

#include "MappableGridGenerator2.cppwg.hpp"

namespace py = pybind11;
typedef MappableGridGenerator<2 > MappableGridGenerator2;
;

void register_MappableGridGenerator2_class(py::module &m){
py::class_<MappableGridGenerator2    >(m, "MappableGridGenerator2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<MappableGridGenerator<2> >(*)()) &MappableGridGenerator2::Create, 
            " "  )
        .def(
            "GeneratePlane", 
            (::std::shared_ptr<Part<2> >(MappableGridGenerator2::*)(unsigned int, unsigned int, bool, bool)) &MappableGridGenerator2::GeneratePlane, 
            " " , py::arg("numX"), py::arg("numY"), py::arg("isShell") = false, py::arg("withEndCaps") = true )
        .def(
            "GenerateCylinder", 
            (::std::shared_ptr<Part<2> >(MappableGridGenerator2::*)(::QLength, ::QLength, ::QLength, unsigned int, unsigned int, double)) &MappableGridGenerator2::GenerateCylinder, 
            " " , py::arg("cylinderRadius"), py::arg("cylinderThickness"), py::arg("cylinderHeight"), py::arg("numX"), py::arg("numY"), py::arg("cylinderAngle") = 2. * 3.1415926535897931 )
        .def(
            "GenerateHemisphere", 
            (::std::shared_ptr<Part<2> >(MappableGridGenerator2::*)(::QLength, ::QLength, unsigned int, unsigned int, double, double)) &MappableGridGenerator2::GenerateHemisphere, 
            " " , py::arg("sphereRadius"), py::arg("sphereThickness"), py::arg("numX"), py::arg("numY"), py::arg("sphereAzimuthAngle") = 2. * 3.1415926535897931, py::arg("spherePolarAngle") = 0.5 * 3.1415926535897931 )
        .def(
            "GetReferenceLengthScale", 
            (::QLength(MappableGridGenerator2::*)()) &MappableGridGenerator2::GetReferenceLengthScale, 
            " "  )
    ;
}
