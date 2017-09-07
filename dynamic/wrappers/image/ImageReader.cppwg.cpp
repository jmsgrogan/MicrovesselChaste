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
#include "ImageReader.hpp"

#include "PythonObjectConverters.hpp"
#include "ImageReader.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef ImageReader ImageReader;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_ImageReader_class(py::module &m){
py::class_<ImageReader  , std::shared_ptr<ImageReader >   >(m, "ImageReader")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<ImageReader>(*)()) &ImageReader::Create, 
            " "  )
        .def(
            "GetImage", 
            (::vtkSmartPointer<vtkImageData>(ImageReader::*)()) &ImageReader::GetImage, 
            " "  )
        .def(
            "SetFilename", 
            (void(ImageReader::*)(::std::string const &)) &ImageReader::SetFilename, 
            " " , py::arg("rFilename") )
        .def(
            "SetImageResizeFactors", 
            (void(ImageReader::*)(double, double, double)) &ImageReader::SetImageResizeFactors, 
            " " , py::arg("factorX"), py::arg("factorY") = 1., py::arg("factorZ") = 1. )
        .def(
            "Read", 
            (void(ImageReader::*)()) &ImageReader::Read, 
            " "  )
    ;
}
