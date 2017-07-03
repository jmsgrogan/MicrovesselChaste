#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "VesselNetworkWriter.hpp"

#include "VesselNetworkWriter3.cppwg.hpp"

namespace py = pybind11;
typedef VesselNetworkWriter<3 > VesselNetworkWriter3;
;

void register_VesselNetworkWriter3_class(py::module &m){
py::class_<VesselNetworkWriter3    >(m, "VesselNetworkWriter3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNetworkWriter<3> >(*)()) &VesselNetworkWriter3::Create, 
            " " )
        .def(
            "SetVesselNetwork", 
            (void(VesselNetworkWriter3::*)(::std::shared_ptr<VesselNetwork<3> >)) &VesselNetworkWriter3::SetVesselNetwork, 
            " " , py::arg("pNetwork"))
        .def(
            "SetFileName", 
            (void(VesselNetworkWriter3::*)(::std::string const &)) &VesselNetworkWriter3::SetFileName, 
            " " , py::arg("rFileName"))
        .def(
            "Write", 
            (void(VesselNetworkWriter3::*)(bool)) &VesselNetworkWriter3::Write, 
            " " , py::arg("masterOnly") = true)
        .def(
            "GetOutput", 
            (::vtkSmartPointer<vtkPolyData>(VesselNetworkWriter3::*)()) &VesselNetworkWriter3::GetOutput, 
            " " )
        .def(
            "SetReferenceLengthScale", 
            (void(VesselNetworkWriter3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &VesselNetworkWriter3::SetReferenceLengthScale, 
            " " , py::arg("rReferenceLength"))
    ;
}
