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

#include "VesselNetworkCellPopulationInteractor2.cppwg.hpp"

namespace py = pybind11;
typedef VesselNetworkCellPopulationInteractor<2 > VesselNetworkCellPopulationInteractor2;
;

void register_VesselNetworkCellPopulationInteractor2_class(py::module &m){
py::class_<VesselNetworkCellPopulationInteractor2    >(m, "VesselNetworkCellPopulationInteractor2")
        .def(py::init< >())
        .def(
            "LabelVesselsInCellPopulation", 
            (void(VesselNetworkCellPopulationInteractor2::*)(::AbstractCellPopulation<2, 2> &, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, ::boost::shared_ptr<AbstractCellMutationState>, ::boost::shared_ptr<AbstractCellMutationState>, double)) &VesselNetworkCellPopulationInteractor2::LabelVesselsInCellPopulation, 
            " " , py::arg("cellPopulation"), py::arg("cellLengthScale"), py::arg("pTipMutationState"), py::arg("pStalkState"), py::arg("threshold") = 1.2500000000000001E-6)
        .def(
            "PartitionNetworkOverCells", 
            (void(VesselNetworkCellPopulationInteractor2::*)(::AbstractCellPopulation<2, 2> &, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, double)) &VesselNetworkCellPopulationInteractor2::PartitionNetworkOverCells, 
            " " , py::arg("rCellPopulation"), py::arg("cellLengthScale"), py::arg("threshold") = 1.2500000000000001E-6)
        .def(
            "KillNonVesselOverlappingCells", 
            (void(VesselNetworkCellPopulationInteractor2::*)(::AbstractCellPopulation<2, 2> &, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, double)) &VesselNetworkCellPopulationInteractor2::KillNonVesselOverlappingCells, 
            " " , py::arg("rCellPopulation"), py::arg("cellLengthScale"), py::arg("threshold") = 1.2500000000000001E-6)
        .def(
            "KillOverlappingVesselCells", 
            (void(VesselNetworkCellPopulationInteractor2::*)(::AbstractCellPopulation<2, 2> &, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, double)) &VesselNetworkCellPopulationInteractor2::KillOverlappingVesselCells, 
            " " , py::arg("rCellPopulation"), py::arg("cellLengthScale"), py::arg("threshold") = 1.2500000000000001E-6)
        .def(
            "SetVesselNetwork", 
            (void(VesselNetworkCellPopulationInteractor2::*)(::std::shared_ptr<VesselNetwork<2> >)) &VesselNetworkCellPopulationInteractor2::SetVesselNetwork, 
            " " , py::arg("pNetwork"))
    ;
}
