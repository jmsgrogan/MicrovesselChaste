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

#include "NetworkToImage3.cppwg.hpp"

namespace py = pybind11;
typedef NetworkToImage<3 > NetworkToImage3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_NetworkToImage3_class(py::module &m){
py::class_<NetworkToImage3  , std::shared_ptr<NetworkToImage3 >   >(m, "NetworkToImage3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<NetworkToImage<3> >(*)()) &NetworkToImage3::Create, 
            " "  )
        .def(
            "GetOutput", 
            (::vtkSmartPointer<vtkImageData>(NetworkToImage3::*)()) &NetworkToImage3::GetOutput, 
            " "  )
        .def(
            "SetNetwork", 
            (void(NetworkToImage3::*)(::std::shared_ptr<VesselNetwork<3> >)) &NetworkToImage3::SetNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetGridSpacing", 
            (void(NetworkToImage3::*)(::QLength)) &NetworkToImage3::SetGridSpacing, 
            " " , py::arg("spacing") )
        .def(
            "SetPaddingFactors", 
            (void(NetworkToImage3::*)(double, double, double)) &NetworkToImage3::SetPaddingFactors, 
            " " , py::arg("paddingX"), py::arg("paddingY"), py::arg("paddingZ") )
        .def(
            "SetImageDimension", 
            (void(NetworkToImage3::*)(unsigned int)) &NetworkToImage3::SetImageDimension, 
            " " , py::arg("dimension") )
        .def(
            "Update", 
            (void(NetworkToImage3::*)()) &NetworkToImage3::Update, 
            " "  )
    ;
}
