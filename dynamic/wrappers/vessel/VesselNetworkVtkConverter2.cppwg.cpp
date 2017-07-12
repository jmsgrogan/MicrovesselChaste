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
#include "VesselNetworkVtkConverter.hpp"

#include "VesselNetworkVtkConverter2.cppwg.hpp"

namespace py = pybind11;
typedef VesselNetworkVtkConverter<2 > VesselNetworkVtkConverter2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_VesselNetworkVtkConverter2_class(py::module &m){
py::class_<VesselNetworkVtkConverter2  , std::shared_ptr<VesselNetworkVtkConverter2 >   >(m, "VesselNetworkVtkConverter2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNetworkVtkConverter<2> >(*)()) &VesselNetworkVtkConverter2::Create, 
            " "  )
        .def_static(
            "GetVtkRepresentation", 
            (::vtkSmartPointer<vtkPolyData>(*)(::std::shared_ptr<VesselNetwork<2> >)) &VesselNetworkVtkConverter2::GetVtkRepresentation, 
            " " , py::arg("pNetwork") )
        .def_static(
            "GetGlobalVtkRepresentation", 
            (::vtkSmartPointer<vtkPolyData>(*)(::std::shared_ptr<VesselNetwork<2> >)) &VesselNetworkVtkConverter2::GetGlobalVtkRepresentation, 
            " " , py::arg("pNetwork") )
    ;
}
