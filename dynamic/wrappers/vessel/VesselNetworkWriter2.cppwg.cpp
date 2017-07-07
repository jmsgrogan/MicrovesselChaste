#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "VesselNetworkWriter.hpp"

#include "VesselNetworkWriter2.cppwg.hpp"

namespace py = pybind11;
typedef VesselNetworkWriter<2 > VesselNetworkWriter2;
;

void register_VesselNetworkWriter2_class(py::module &m){
py::class_<VesselNetworkWriter2    >(m, "VesselNetworkWriter2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNetworkWriter<2> >(*)()) &VesselNetworkWriter2::Create, 
            " "  )
        .def(
            "SetVesselNetwork", 
            (void(VesselNetworkWriter2::*)(::std::shared_ptr<VesselNetwork<2> >)) &VesselNetworkWriter2::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetFileName", 
            (void(VesselNetworkWriter2::*)(::std::string const &)) &VesselNetworkWriter2::SetFileName, 
            " " , py::arg("rFileName") )
        .def(
            "Write", 
            (void(VesselNetworkWriter2::*)(bool)) &VesselNetworkWriter2::Write, 
            " " , py::arg("masterOnly") = true )
        .def(
            "GetOutput", 
            (::vtkSmartPointer<vtkPolyData>(VesselNetworkWriter2::*)()) &VesselNetworkWriter2::GetOutput, 
            " "  )
        .def(
            "SetReferenceLengthScale", 
            (void(VesselNetworkWriter2::*)(::QLength)) &VesselNetworkWriter2::SetReferenceLengthScale, 
            " " , py::arg("rReferenceLength") )
    ;
}
