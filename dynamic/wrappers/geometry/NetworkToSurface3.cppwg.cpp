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
#include "NetworkToSurface.hpp"

#include "PythonObjectConverters.hpp"
#include "NetworkToSurface3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef NetworkToSurface<3 > NetworkToSurface3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_NetworkToSurface3_class(py::module &m){
py::class_<NetworkToSurface3  , std::shared_ptr<NetworkToSurface3 >   >(m, "NetworkToSurface3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<NetworkToSurface<3> >(*)()) &NetworkToSurface3::Create, 
            " "  )
        .def(
            "GetNetworkToImageTool", 
            (::std::shared_ptr<NetworkToImage<3> >(NetworkToSurface3::*)()) &NetworkToSurface3::GetNetworkToImageTool, 
            " "  )
        .def(
            "GetSurface", 
            (::vtkSmartPointer<vtkPolyData>(NetworkToSurface3::*)()) &NetworkToSurface3::GetSurface, 
            " "  )
        .def(
            "SetDoSmoothing", 
            (void(NetworkToSurface3::*)(bool)) &NetworkToSurface3::SetDoSmoothing, 
            " " , py::arg("doSmoothing") )
        .def(
            "SetNumSmoothingIterations", 
            (void(NetworkToSurface3::*)(unsigned int)) &NetworkToSurface3::SetNumSmoothingIterations, 
            " " , py::arg("numIterations") )
        .def(
            "SetSmoothingFeatureAngle", 
            (void(NetworkToSurface3::*)(double)) &NetworkToSurface3::SetSmoothingFeatureAngle, 
            " " , py::arg("featureAngle") )
        .def(
            "SetBandPassFrequency", 
            (void(NetworkToSurface3::*)(double)) &NetworkToSurface3::SetBandPassFrequency, 
            " " , py::arg("bandPassFrequency") )
        .def(
            "SetVesselNetwork", 
            (void(NetworkToSurface3::*)(::std::shared_ptr<VesselNetwork<3> >)) &NetworkToSurface3::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetResamplingSplineSize", 
            (void(NetworkToSurface3::*)(::QLength)) &NetworkToSurface3::SetResamplingSplineSize, 
            " " , py::arg("splineResampleSize") )
        .def(
            "SetDoSurfaceRemeshing", 
            (void(NetworkToSurface3::*)(bool)) &NetworkToSurface3::SetDoSurfaceRemeshing, 
            " " , py::arg("doRemeshing") )
        .def(
            "SetRemeshingTargetEdgeLength", 
            (void(NetworkToSurface3::*)(double)) &NetworkToSurface3::SetRemeshingTargetEdgeLength, 
            " " , py::arg("length") )
        .def(
            "Update", 
            (void(NetworkToSurface3::*)()) &NetworkToSurface3::Update, 
            " "  )
    ;
}
