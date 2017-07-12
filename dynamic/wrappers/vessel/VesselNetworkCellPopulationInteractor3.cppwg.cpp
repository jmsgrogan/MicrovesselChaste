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
#include "VesselNetworkCellPopulationInteractor.hpp"

#include "VesselNetworkCellPopulationInteractor3.cppwg.hpp"

namespace py = pybind11;
typedef VesselNetworkCellPopulationInteractor<3 > VesselNetworkCellPopulationInteractor3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_VesselNetworkCellPopulationInteractor3_class(py::module &m){
py::class_<VesselNetworkCellPopulationInteractor3  , std::shared_ptr<VesselNetworkCellPopulationInteractor3 >   >(m, "VesselNetworkCellPopulationInteractor3")
        .def(py::init< >())
        .def(
            "LabelVesselsInCellPopulation", 
            (void(VesselNetworkCellPopulationInteractor3::*)(::AbstractCellPopulation<3, 3> &, ::QLength, ::boost::shared_ptr<AbstractCellMutationState>, ::boost::shared_ptr<AbstractCellMutationState>, double)) &VesselNetworkCellPopulationInteractor3::LabelVesselsInCellPopulation, 
            " " , py::arg("cellPopulation"), py::arg("cellLengthScale"), py::arg("pTipMutationState"), py::arg("pStalkState"), py::arg("threshold") = 1.2500000000000001E-6 )
        .def(
            "PartitionNetworkOverCells", 
            (void(VesselNetworkCellPopulationInteractor3::*)(::AbstractCellPopulation<3, 3> &, ::QLength, double)) &VesselNetworkCellPopulationInteractor3::PartitionNetworkOverCells, 
            " " , py::arg("rCellPopulation"), py::arg("cellLengthScale"), py::arg("threshold") = 1.2500000000000001E-6 )
        .def(
            "KillNonVesselOverlappingCells", 
            (void(VesselNetworkCellPopulationInteractor3::*)(::AbstractCellPopulation<3, 3> &, ::QLength, double)) &VesselNetworkCellPopulationInteractor3::KillNonVesselOverlappingCells, 
            " " , py::arg("rCellPopulation"), py::arg("cellLengthScale"), py::arg("threshold") = 1.2500000000000001E-6 )
        .def(
            "KillOverlappingVesselCells", 
            (void(VesselNetworkCellPopulationInteractor3::*)(::AbstractCellPopulation<3, 3> &, ::QLength, double)) &VesselNetworkCellPopulationInteractor3::KillOverlappingVesselCells, 
            " " , py::arg("rCellPopulation"), py::arg("cellLengthScale"), py::arg("threshold") = 1.2500000000000001E-6 )
        .def(
            "SetVesselNetwork", 
            (void(VesselNetworkCellPopulationInteractor3::*)(::std::shared_ptr<VesselNetwork<3> >)) &VesselNetworkCellPopulationInteractor3::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
    ;
}
