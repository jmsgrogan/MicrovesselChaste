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
#include "VesselNetworkGraphCalculator.hpp"

#include "PythonObjectConverters.hpp"
#include "VesselNetworkGraphCalculator2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef VesselNetworkGraphCalculator<2 > VesselNetworkGraphCalculator2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_VesselNetworkGraphCalculator2_class(py::module &m){
py::class_<VesselNetworkGraphCalculator2  , std::shared_ptr<VesselNetworkGraphCalculator2 >   >(m, "VesselNetworkGraphCalculator2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNetworkGraphCalculator<2> >(*)()) &VesselNetworkGraphCalculator2::Create, 
            " "  )
        .def(
            "SetVesselNetwork", 
            (void(VesselNetworkGraphCalculator2::*)(::std::shared_ptr<VesselNetwork<2> >)) &VesselNetworkGraphCalculator2::SetVesselNetwork, 
            " " , py::arg("pVesselNetwork") )
        .def(
            "GetNodeNodeConnectivity", 
            (::std::vector<std::vector<unsigned int, std::allocator<unsigned int> >, std::allocator<std::vector<unsigned int, std::allocator<unsigned int> > > >(VesselNetworkGraphCalculator2::*)()) &VesselNetworkGraphCalculator2::GetNodeNodeConnectivity, 
            " "  )
        .def(
            "GetNodeVesselConnectivity", 
            (::std::vector<std::vector<unsigned int, std::allocator<unsigned int> >, std::allocator<std::vector<unsigned int, std::allocator<unsigned int> > > >(VesselNetworkGraphCalculator2::*)()) &VesselNetworkGraphCalculator2::GetNodeVesselConnectivity, 
            " "  )
        .def(
            "IsConnected", 
            (bool(VesselNetworkGraphCalculator2::*)(::std::shared_ptr<VesselNode<2> >, ::std::shared_ptr<VesselNode<2> >)) &VesselNetworkGraphCalculator2::IsConnected, 
            " " , py::arg("pSourceNode"), py::arg("pQueryNode") )
        .def(
            "IsConnected", 
            (::std::vector<bool, std::allocator<bool> >(VesselNetworkGraphCalculator2::*)(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > >, ::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > >)) &VesselNetworkGraphCalculator2::IsConnected, 
            " " , py::arg("sourceNodes"), py::arg("queryNodes") )
        .def(
            "WriteConnectivity", 
            (void(VesselNetworkGraphCalculator2::*)(::std::string const &)) &VesselNetworkGraphCalculator2::WriteConnectivity, 
            " " , py::arg("rFilename") )
    ;
}
