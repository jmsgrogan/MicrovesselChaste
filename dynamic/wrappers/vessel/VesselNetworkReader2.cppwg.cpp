#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "VesselNetworkReader.hpp"

#include "VesselNetworkReader2.cppwg.hpp"

namespace py = pybind11;
typedef VesselNetworkReader<2 > VesselNetworkReader2;
;

void register_VesselNetworkReader2_class(py::module &m){
py::class_<VesselNetworkReader2    >(m, "VesselNetworkReader2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNetworkReader<2> >(*)()) &VesselNetworkReader2::Create, 
            " " )
        .def(
            "Read", 
            (::std::shared_ptr<VesselNetwork<2> >(VesselNetworkReader2::*)()) &VesselNetworkReader2::Read, 
            " " )
        .def(
            "SetRadiusArrayName", 
            (void(VesselNetworkReader2::*)(::std::string const &)) &VesselNetworkReader2::SetRadiusArrayName, 
            " " , py::arg("rRadius"))
        .def(
            "SetMergeCoincidentPoints", 
            (void(VesselNetworkReader2::*)(bool)) &VesselNetworkReader2::SetMergeCoincidentPoints, 
            " " , py::arg("mergePoints"))
        .def(
            "SetTargetSegmentLength", 
            (void(VesselNetworkReader2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &VesselNetworkReader2::SetTargetSegmentLength, 
            " " , py::arg("targetSegmentLength"))
        .def(
            "SetFileName", 
            (void(VesselNetworkReader2::*)(::std::string const &)) &VesselNetworkReader2::SetFileName, 
            " " , py::arg("rFileName"))
        .def(
            "SetReferenceLengthScale", 
            (void(VesselNetworkReader2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &VesselNetworkReader2::SetReferenceLengthScale, 
            " " , py::arg("rReferenceLength"))
    ;
}
