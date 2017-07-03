#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "VesselNetworkCellPopulationInteractor.hpp"

#include "VesselNetworkCellPopulationInteractor3.cppwg.hpp"

namespace py = pybind11;
typedef VesselNetworkCellPopulationInteractor<3 > VesselNetworkCellPopulationInteractor3;
;

void register_VesselNetworkCellPopulationInteractor3_class(py::module &m){
py::class_<VesselNetworkCellPopulationInteractor3    >(m, "VesselNetworkCellPopulationInteractor3")
        .def(py::init< >())
        .def(
            "LabelVesselsInCellPopulation", 
            (void(VesselNetworkCellPopulationInteractor3::*)(::AbstractCellPopulation<3, 3> &, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, ::boost::shared_ptr<AbstractCellMutationState>, ::boost::shared_ptr<AbstractCellMutationState>, double)) &VesselNetworkCellPopulationInteractor3::LabelVesselsInCellPopulation, 
            " " , py::arg("cellPopulation"), py::arg("cellLengthScale"), py::arg("pTipMutationState"), py::arg("pStalkState"), py::arg("threshold") = 1.2500000000000001E-6)
        .def(
            "PartitionNetworkOverCells", 
            (void(VesselNetworkCellPopulationInteractor3::*)(::AbstractCellPopulation<3, 3> &, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, double)) &VesselNetworkCellPopulationInteractor3::PartitionNetworkOverCells, 
            " " , py::arg("rCellPopulation"), py::arg("cellLengthScale"), py::arg("threshold") = 1.2500000000000001E-6)
        .def(
            "KillNonVesselOverlappingCells", 
            (void(VesselNetworkCellPopulationInteractor3::*)(::AbstractCellPopulation<3, 3> &, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, double)) &VesselNetworkCellPopulationInteractor3::KillNonVesselOverlappingCells, 
            " " , py::arg("rCellPopulation"), py::arg("cellLengthScale"), py::arg("threshold") = 1.2500000000000001E-6)
        .def(
            "KillOverlappingVesselCells", 
            (void(VesselNetworkCellPopulationInteractor3::*)(::AbstractCellPopulation<3, 3> &, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, double)) &VesselNetworkCellPopulationInteractor3::KillOverlappingVesselCells, 
            " " , py::arg("rCellPopulation"), py::arg("cellLengthScale"), py::arg("threshold") = 1.2500000000000001E-6)
        .def(
            "SetVesselNetwork", 
            (void(VesselNetworkCellPopulationInteractor3::*)(::std::shared_ptr<VesselNetwork<3> >)) &VesselNetworkCellPopulationInteractor3::SetVesselNetwork, 
            " " , py::arg("pNetwork"))
    ;
}
