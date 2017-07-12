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

#include "VesselNetworkCellPopulationInteractor2.cppwg.hpp"

namespace py = pybind11;
typedef VesselNetworkCellPopulationInteractor<2 > VesselNetworkCellPopulationInteractor2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_VesselNetworkCellPopulationInteractor2_class(py::module &m){
py::class_<VesselNetworkCellPopulationInteractor2  , std::shared_ptr<VesselNetworkCellPopulationInteractor2 >   >(m, "VesselNetworkCellPopulationInteractor2")
        .def(py::init< >())
        .def(
            "LabelVesselsInCellPopulation", 
            (void(VesselNetworkCellPopulationInteractor2::*)(::AbstractCellPopulation<2, 2> &, ::QLength, ::boost::shared_ptr<AbstractCellMutationState>, ::boost::shared_ptr<AbstractCellMutationState>, double)) &VesselNetworkCellPopulationInteractor2::LabelVesselsInCellPopulation, 
            " " , py::arg("cellPopulation"), py::arg("cellLengthScale"), py::arg("pTipMutationState"), py::arg("pStalkState"), py::arg("threshold") = 1.2500000000000001E-6 )
        .def(
            "PartitionNetworkOverCells", 
            (void(VesselNetworkCellPopulationInteractor2::*)(::AbstractCellPopulation<2, 2> &, ::QLength, double)) &VesselNetworkCellPopulationInteractor2::PartitionNetworkOverCells, 
            " " , py::arg("rCellPopulation"), py::arg("cellLengthScale"), py::arg("threshold") = 1.2500000000000001E-6 )
        .def(
            "KillNonVesselOverlappingCells", 
            (void(VesselNetworkCellPopulationInteractor2::*)(::AbstractCellPopulation<2, 2> &, ::QLength, double)) &VesselNetworkCellPopulationInteractor2::KillNonVesselOverlappingCells, 
            " " , py::arg("rCellPopulation"), py::arg("cellLengthScale"), py::arg("threshold") = 1.2500000000000001E-6 )
        .def(
            "KillOverlappingVesselCells", 
            (void(VesselNetworkCellPopulationInteractor2::*)(::AbstractCellPopulation<2, 2> &, ::QLength, double)) &VesselNetworkCellPopulationInteractor2::KillOverlappingVesselCells, 
            " " , py::arg("rCellPopulation"), py::arg("cellLengthScale"), py::arg("threshold") = 1.2500000000000001E-6 )
        .def(
            "SetVesselNetwork", 
            (void(VesselNetworkCellPopulationInteractor2::*)(::std::shared_ptr<VesselNetwork<2> >)) &VesselNetworkCellPopulationInteractor2::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
    ;
}
