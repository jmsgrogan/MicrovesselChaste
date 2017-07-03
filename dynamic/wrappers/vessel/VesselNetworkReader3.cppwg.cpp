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

#include "VesselNetworkReader3.cppwg.hpp"

namespace py = pybind11;
typedef VesselNetworkReader<3 > VesselNetworkReader3;
;

void register_VesselNetworkReader3_class(py::module &m){
py::class_<VesselNetworkReader3    >(m, "VesselNetworkReader3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNetworkReader<3> >(*)()) &VesselNetworkReader3::Create, 
            " " )
        .def(
            "Read", 
            (::std::shared_ptr<VesselNetwork<3> >(VesselNetworkReader3::*)()) &VesselNetworkReader3::Read, 
            " " )
        .def(
            "SetRadiusArrayName", 
            (void(VesselNetworkReader3::*)(::std::string const &)) &VesselNetworkReader3::SetRadiusArrayName, 
            " " , py::arg("rRadius"))
        .def(
            "SetMergeCoincidentPoints", 
            (void(VesselNetworkReader3::*)(bool)) &VesselNetworkReader3::SetMergeCoincidentPoints, 
            " " , py::arg("mergePoints"))
        .def(
            "SetTargetSegmentLength", 
            (void(VesselNetworkReader3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &VesselNetworkReader3::SetTargetSegmentLength, 
            " " , py::arg("targetSegmentLength"))
        .def(
            "SetFileName", 
            (void(VesselNetworkReader3::*)(::std::string const &)) &VesselNetworkReader3::SetFileName, 
            " " , py::arg("rFileName"))
        .def(
            "SetReferenceLengthScale", 
            (void(VesselNetworkReader3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &VesselNetworkReader3::SetReferenceLengthScale, 
            " " , py::arg("rReferenceLength"))
    ;
}
