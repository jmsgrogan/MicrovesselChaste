#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "VesselNetworkVtkConverter.hpp"

#include "VesselNetworkVtkConverter3.cppwg.hpp"

namespace py = pybind11;
typedef VesselNetworkVtkConverter<3 > VesselNetworkVtkConverter3;
;

void register_VesselNetworkVtkConverter3_class(py::module &m){
py::class_<VesselNetworkVtkConverter3    >(m, "VesselNetworkVtkConverter3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNetworkVtkConverter<3> >(*)()) &VesselNetworkVtkConverter3::Create, 
            " " )
        .def_static(
            "GetVtkRepresentation", 
            (::vtkSmartPointer<vtkPolyData>(*)(::std::shared_ptr<VesselNetwork<3> >)) &VesselNetworkVtkConverter3::GetVtkRepresentation, 
            " " , py::arg("pNetwork"))
        .def_static(
            "GetGlobalVtkRepresentation", 
            (::vtkSmartPointer<vtkPolyData>(*)(::std::shared_ptr<VesselNetwork<3> >)) &VesselNetworkVtkConverter3::GetGlobalVtkRepresentation, 
            " " , py::arg("pNetwork"))
    ;
}
