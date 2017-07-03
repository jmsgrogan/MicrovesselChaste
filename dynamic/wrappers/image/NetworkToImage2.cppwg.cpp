#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "NetworkToImage.hpp"

#include "NetworkToImage2.cppwg.hpp"

namespace py = pybind11;
typedef NetworkToImage<2 > NetworkToImage2;
;

void register_NetworkToImage2_class(py::module &m){
py::class_<NetworkToImage2    >(m, "NetworkToImage2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<NetworkToImage<2> >(*)()) &NetworkToImage2::Create, 
            " " )
        .def(
            "GetOutput", 
            (::vtkSmartPointer<vtkImageData>(NetworkToImage2::*)()) &NetworkToImage2::GetOutput, 
            " " )
        .def(
            "SetNetwork", 
            (void(NetworkToImage2::*)(::std::shared_ptr<VesselNetwork<2> >)) &NetworkToImage2::SetNetwork, 
            " " , py::arg("pNetwork"))
        .def(
            "SetGridSpacing", 
            (void(NetworkToImage2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &NetworkToImage2::SetGridSpacing, 
            " " , py::arg("spacing"))
        .def(
            "SetPaddingFactors", 
            (void(NetworkToImage2::*)(double, double, double)) &NetworkToImage2::SetPaddingFactors, 
            " " , py::arg("paddingX"), py::arg("paddingY"), py::arg("paddingZ"))
        .def(
            "SetImageDimension", 
            (void(NetworkToImage2::*)(unsigned int)) &NetworkToImage2::SetImageDimension, 
            " " , py::arg("dimension"))
        .def(
            "Update", 
            (void(NetworkToImage2::*)()) &NetworkToImage2::Update, 
            " " )
    ;
}
