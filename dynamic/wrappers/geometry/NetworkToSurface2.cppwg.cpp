#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "NetworkToSurface.hpp"

#include "NetworkToSurface2.cppwg.hpp"

namespace py = pybind11;
typedef NetworkToSurface<2 > NetworkToSurface2;
;

void register_NetworkToSurface2_class(py::module &m){
py::class_<NetworkToSurface2    >(m, "NetworkToSurface2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<NetworkToSurface<2> >(*)()) &NetworkToSurface2::Create, 
            " " )
        .def(
            "GetNetworkToImageTool", 
            (::std::shared_ptr<NetworkToImage<2> >(NetworkToSurface2::*)()) &NetworkToSurface2::GetNetworkToImageTool, 
            " " )
        .def(
            "GetSurface", 
            (::vtkSmartPointer<vtkPolyData>(NetworkToSurface2::*)()) &NetworkToSurface2::GetSurface, 
            " " )
        .def(
            "SetDoSmoothing", 
            (void(NetworkToSurface2::*)(bool)) &NetworkToSurface2::SetDoSmoothing, 
            " " , py::arg("doSmoothing"))
        .def(
            "SetNumSmoothingIterations", 
            (void(NetworkToSurface2::*)(unsigned int)) &NetworkToSurface2::SetNumSmoothingIterations, 
            " " , py::arg("numIterations"))
        .def(
            "SetSmoothingFeatureAngle", 
            (void(NetworkToSurface2::*)(double)) &NetworkToSurface2::SetSmoothingFeatureAngle, 
            " " , py::arg("featureAngle"))
        .def(
            "SetBandPassFrequency", 
            (void(NetworkToSurface2::*)(double)) &NetworkToSurface2::SetBandPassFrequency, 
            " " , py::arg("bandPassFrequency"))
        .def(
            "SetVesselNetwork", 
            (void(NetworkToSurface2::*)(::std::shared_ptr<VesselNetwork<2> >)) &NetworkToSurface2::SetVesselNetwork, 
            " " , py::arg("pNetwork"))
        .def(
            "SetResamplingSplineSize", 
            (void(NetworkToSurface2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &NetworkToSurface2::SetResamplingSplineSize, 
            " " , py::arg("splineResampleSize"))
        .def(
            "SetDoSurfaceRemeshing", 
            (void(NetworkToSurface2::*)(bool)) &NetworkToSurface2::SetDoSurfaceRemeshing, 
            " " , py::arg("doRemeshing"))
        .def(
            "SetRemeshingTargetEdgeLength", 
            (void(NetworkToSurface2::*)(double)) &NetworkToSurface2::SetRemeshingTargetEdgeLength, 
            " " , py::arg("length"))
        .def(
            "Update", 
            (void(NetworkToSurface2::*)()) &NetworkToSurface2::Update, 
            " " )
    ;
}
