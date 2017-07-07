#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "VesselNetworkGraphCalculator.hpp"

#include "VesselNetworkGraphCalculator3.cppwg.hpp"

namespace py = pybind11;
typedef VesselNetworkGraphCalculator<3 > VesselNetworkGraphCalculator3;
;

void register_VesselNetworkGraphCalculator3_class(py::module &m){
py::class_<VesselNetworkGraphCalculator3    >(m, "VesselNetworkGraphCalculator3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNetworkGraphCalculator<3> >(*)()) &VesselNetworkGraphCalculator3::Create, 
            " "  )
        .def(
            "SetVesselNetwork", 
            (void(VesselNetworkGraphCalculator3::*)(::std::shared_ptr<VesselNetwork<3> >)) &VesselNetworkGraphCalculator3::SetVesselNetwork, 
            " " , py::arg("pVesselNetwork") )
        .def(
            "GetNodeNodeConnectivity", 
            (::std::vector<std::vector<unsigned int, std::allocator<unsigned int> >, std::allocator<std::vector<unsigned int, std::allocator<unsigned int> > > >(VesselNetworkGraphCalculator3::*)()) &VesselNetworkGraphCalculator3::GetNodeNodeConnectivity, 
            " "  )
        .def(
            "GetNodeVesselConnectivity", 
            (::std::vector<std::vector<unsigned int, std::allocator<unsigned int> >, std::allocator<std::vector<unsigned int, std::allocator<unsigned int> > > >(VesselNetworkGraphCalculator3::*)()) &VesselNetworkGraphCalculator3::GetNodeVesselConnectivity, 
            " "  )
        .def(
            "IsConnected", 
            (bool(VesselNetworkGraphCalculator3::*)(::std::shared_ptr<VesselNode<3> >, ::std::shared_ptr<VesselNode<3> >)) &VesselNetworkGraphCalculator3::IsConnected, 
            " " , py::arg("pSourceNode"), py::arg("pQueryNode") )
        .def(
            "IsConnected", 
            (::std::vector<bool, std::allocator<bool> >(VesselNetworkGraphCalculator3::*)(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > >, ::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > >)) &VesselNetworkGraphCalculator3::IsConnected, 
            " " , py::arg("sourceNodes"), py::arg("queryNodes") )
        .def(
            "WriteConnectivity", 
            (void(VesselNetworkGraphCalculator3::*)(::std::string const &)) &VesselNetworkGraphCalculator3::WriteConnectivity, 
            " " , py::arg("rFilename") )
    ;
}
