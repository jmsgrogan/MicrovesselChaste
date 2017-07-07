#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "VesselNetworkPartitioner.hpp"

#include "VesselNetworkPartitioner3.cppwg.hpp"

namespace py = pybind11;
typedef VesselNetworkPartitioner<3 > VesselNetworkPartitioner3;
;

void register_VesselNetworkPartitioner3_class(py::module &m){
py::class_<VesselNetworkPartitioner3    >(m, "VesselNetworkPartitioner3")
        .def(py::init< >())
        .def(
            "SetUseSimpleGeometricPartitioning", 
            (void(VesselNetworkPartitioner3::*)(bool)) &VesselNetworkPartitioner3::SetUseSimpleGeometricPartitioning, 
            " " , py::arg("useSimple") )
        .def(
            "SetPartitionAxis", 
            (void(VesselNetworkPartitioner3::*)(unsigned int)) &VesselNetworkPartitioner3::SetPartitionAxis, 
            " " , py::arg("partitionAxis") )
        .def(
            "SetVesselNetwork", 
            (void(VesselNetworkPartitioner3::*)(::std::shared_ptr<VesselNetwork<3> >)) &VesselNetworkPartitioner3::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "Update", 
            (void(VesselNetworkPartitioner3::*)()) &VesselNetworkPartitioner3::Update, 
            " "  )
    ;
}
