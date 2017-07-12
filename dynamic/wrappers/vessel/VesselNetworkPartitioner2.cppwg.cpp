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
#include "VesselNetworkPartitioner.hpp"

#include "VesselNetworkPartitioner2.cppwg.hpp"

namespace py = pybind11;
typedef VesselNetworkPartitioner<2 > VesselNetworkPartitioner2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_VesselNetworkPartitioner2_class(py::module &m){
py::class_<VesselNetworkPartitioner2  , std::shared_ptr<VesselNetworkPartitioner2 >   >(m, "VesselNetworkPartitioner2")
        .def(py::init< >())
        .def(
            "SetUseSimpleGeometricPartitioning", 
            (void(VesselNetworkPartitioner2::*)(bool)) &VesselNetworkPartitioner2::SetUseSimpleGeometricPartitioning, 
            " " , py::arg("useSimple") )
        .def(
            "SetPartitionAxis", 
            (void(VesselNetworkPartitioner2::*)(unsigned int)) &VesselNetworkPartitioner2::SetPartitionAxis, 
            " " , py::arg("partitionAxis") )
        .def(
            "SetVesselNetwork", 
            (void(VesselNetworkPartitioner2::*)(::std::shared_ptr<VesselNetwork<2> >)) &VesselNetworkPartitioner2::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "Update", 
            (void(VesselNetworkPartitioner2::*)()) &VesselNetworkPartitioner2::Update, 
            " "  )
    ;
}
