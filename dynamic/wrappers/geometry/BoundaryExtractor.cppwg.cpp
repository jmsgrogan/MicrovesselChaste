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
#include "BoundaryExtractor.hpp"

#include "PythonObjectConverters.hpp"
#include "BoundaryExtractor.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef BoundaryExtractor BoundaryExtractor;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_BoundaryExtractor_class(py::module &m){
py::class_<BoundaryExtractor  , std::shared_ptr<BoundaryExtractor >   >(m, "BoundaryExtractor")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<BoundaryExtractor>(*)()) &BoundaryExtractor::Create, 
            " "  )
        .def(
            "GetOutput", 
            (::vtkSmartPointer<vtkPolyData>(BoundaryExtractor::*)()) &BoundaryExtractor::GetOutput, 
            " "  )
        .def(
            "SetInput", 
            (void(BoundaryExtractor::*)(::vtkSmartPointer<vtkPolyData>)) &BoundaryExtractor::SetInput, 
            " " , py::arg("pInputSurface") )
        .def(
            "SetInputRaw", 
            (void(BoundaryExtractor::*)(::vtkPolyData *)) &BoundaryExtractor::SetInputRaw, 
            " " , py::arg("pInputSurface") )
        .def(
            "SetSmoothingLength", 
            (void(BoundaryExtractor::*)(double)) &BoundaryExtractor::SetSmoothingLength, 
            " " , py::arg("value") )
        .def(
            "SetDoSmoothing", 
            (void(BoundaryExtractor::*)(bool)) &BoundaryExtractor::SetDoSmoothing, 
            " " , py::arg("doSmoothing") )
        .def(
            "SetRemoveDisconnected", 
            (void(BoundaryExtractor::*)(bool)) &BoundaryExtractor::SetRemoveDisconnected, 
            " " , py::arg("removeDisconnected") )
        .def(
            "Update", 
            (void(BoundaryExtractor::*)()) &BoundaryExtractor::Update, 
            " "  )
    ;
}
