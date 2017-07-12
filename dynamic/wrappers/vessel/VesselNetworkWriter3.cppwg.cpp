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
#include "VesselNetworkWriter.hpp"

#include "VesselNetworkWriter3.cppwg.hpp"

namespace py = pybind11;
typedef VesselNetworkWriter<3 > VesselNetworkWriter3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_VesselNetworkWriter3_class(py::module &m){
py::class_<VesselNetworkWriter3  , std::shared_ptr<VesselNetworkWriter3 >   >(m, "VesselNetworkWriter3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNetworkWriter<3> >(*)()) &VesselNetworkWriter3::Create, 
            " "  )
        .def(
            "SetVesselNetwork", 
            (void(VesselNetworkWriter3::*)(::std::shared_ptr<VesselNetwork<3> >)) &VesselNetworkWriter3::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetFileName", 
            (void(VesselNetworkWriter3::*)(::std::string const &)) &VesselNetworkWriter3::SetFileName, 
            " " , py::arg("rFileName") )
        .def(
            "Write", 
            (void(VesselNetworkWriter3::*)(bool)) &VesselNetworkWriter3::Write, 
            " " , py::arg("masterOnly") = true )
        .def(
            "GetOutput", 
            (::vtkSmartPointer<vtkPolyData>(VesselNetworkWriter3::*)()) &VesselNetworkWriter3::GetOutput, 
            " "  )
        .def(
            "SetReferenceLengthScale", 
            (void(VesselNetworkWriter3::*)(::QLength)) &VesselNetworkWriter3::SetReferenceLengthScale, 
            " " , py::arg("rReferenceLength") )
    ;
}
