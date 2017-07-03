#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "ImageToSurface.hpp"

#include "ImageToSurface.cppwg.hpp"

namespace py = pybind11;
typedef ImageToSurface ImageToSurface;
;

void register_ImageToSurface_class(py::module &m){
py::class_<ImageToSurface    >(m, "ImageToSurface")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<ImageToSurface>(*)()) &ImageToSurface::Create, 
            " " )
        .def(
            "GetOutput", 
            (::vtkSmartPointer<vtkPolyData>(ImageToSurface::*)()) &ImageToSurface::GetOutput, 
            " " )
        .def(
            "SetInput", 
            (void(ImageToSurface::*)(::vtkSmartPointer<vtkImageData>)) &ImageToSurface::SetInput, 
            " " , py::arg("pImage"))
        .def(
            "SetInputRaw", 
            (void(ImageToSurface::*)(::vtkImageData *)) &ImageToSurface::SetInputRaw, 
            " " , py::arg("pImage"))
        .def(
            "SetThreshold", 
            (void(ImageToSurface::*)(double, bool)) &ImageToSurface::SetThreshold, 
            " " , py::arg("threshold"), py::arg("segmentAboveThreshold"))
        .def(
            "SetUseMarchingCubes", 
            (void(ImageToSurface::*)(bool)) &ImageToSurface::SetUseMarchingCubes, 
            " " , py::arg("useMarchingCubes"))
        .def(
            "SetRemoveDisconnected", 
            (void(ImageToSurface::*)(bool)) &ImageToSurface::SetRemoveDisconnected, 
            " " , py::arg("removeDisconnected"))
        .def(
            "Update", 
            (void(ImageToSurface::*)()) &ImageToSurface::Update, 
            " " )
    ;
}
