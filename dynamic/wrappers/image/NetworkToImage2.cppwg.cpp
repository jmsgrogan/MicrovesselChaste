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
#include "NetworkToImage.hpp"

#include "PythonObjectConverters.hpp"
#include "NetworkToImage2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef NetworkToImage<2 > NetworkToImage2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_NetworkToImage2_class(py::module &m){
py::class_<NetworkToImage2  , std::shared_ptr<NetworkToImage2 >   >(m, "NetworkToImage2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<NetworkToImage<2> >(*)()) &NetworkToImage2::Create, 
            " "  )
        .def(
            "GetOutput", 
            (::vtkSmartPointer<vtkImageData>(NetworkToImage2::*)()) &NetworkToImage2::GetOutput, 
            " "  )
        .def(
            "SetNetwork", 
            (void(NetworkToImage2::*)(::std::shared_ptr<VesselNetwork<2> >)) &NetworkToImage2::SetNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetGridSpacing", 
            (void(NetworkToImage2::*)(::QLength)) &NetworkToImage2::SetGridSpacing, 
            " " , py::arg("spacing") )
        .def(
            "SetPaddingFactors", 
            (void(NetworkToImage2::*)(double, double, double)) &NetworkToImage2::SetPaddingFactors, 
            " " , py::arg("paddingX"), py::arg("paddingY"), py::arg("paddingZ") )
        .def(
            "SetImageDimension", 
            (void(NetworkToImage2::*)(unsigned int)) &NetworkToImage2::SetImageDimension, 
            " " , py::arg("dimension") )
        .def(
            "Update", 
            (void(NetworkToImage2::*)()) &NetworkToImage2::Update, 
            " "  )
    ;
}
