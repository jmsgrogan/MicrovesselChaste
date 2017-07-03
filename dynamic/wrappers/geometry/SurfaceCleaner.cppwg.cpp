#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "SurfaceCleaner.hpp"

#include "SurfaceCleaner.cppwg.hpp"

namespace py = pybind11;
typedef SurfaceCleaner SurfaceCleaner;
;

void register_SurfaceCleaner_class(py::module &m){
py::class_<SurfaceCleaner    >(m, "SurfaceCleaner")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<SurfaceCleaner>(*)()) &SurfaceCleaner::Create, 
            " " )
        .def(
            "GetOutput", 
            (::vtkSmartPointer<vtkPolyData>(SurfaceCleaner::*)()) &SurfaceCleaner::GetOutput, 
            " " )
        .def(
            "SetInput", 
            (void(SurfaceCleaner::*)(::vtkSmartPointer<vtkPolyData>)) &SurfaceCleaner::SetInput, 
            " " , py::arg("pInputSurface"))
        .def(
            "SetDecimateTargetReduction", 
            (void(SurfaceCleaner::*)(double)) &SurfaceCleaner::SetDecimateTargetReduction, 
            " " , py::arg("value"))
        .def(
            "SetDecimateFeatureAngle", 
            (void(SurfaceCleaner::*)(double)) &SurfaceCleaner::SetDecimateFeatureAngle, 
            " " , py::arg("value"))
        .def(
            "SetLinearSubdivisionNumber", 
            (void(SurfaceCleaner::*)(double)) &SurfaceCleaner::SetLinearSubdivisionNumber, 
            " " , py::arg("value"))
        .def(
            "Update", 
            (void(SurfaceCleaner::*)()) &SurfaceCleaner::Update, 
            " " )
    ;
}
