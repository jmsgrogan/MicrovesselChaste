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

#include "NetworkToSurface3.cppwg.hpp"

namespace py = pybind11;
typedef NetworkToSurface<3 > NetworkToSurface3;
;

void register_NetworkToSurface3_class(py::module &m){
py::class_<NetworkToSurface3    >(m, "NetworkToSurface3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<NetworkToSurface<3> >(*)()) &NetworkToSurface3::Create, 
            " " )
        .def(
            "GetNetworkToImageTool", 
            (::std::shared_ptr<NetworkToImage<3> >(NetworkToSurface3::*)()) &NetworkToSurface3::GetNetworkToImageTool, 
            " " )
        .def(
            "GetSurface", 
            (::vtkSmartPointer<vtkPolyData>(NetworkToSurface3::*)()) &NetworkToSurface3::GetSurface, 
            " " )
        .def(
            "SetDoSmoothing", 
            (void(NetworkToSurface3::*)(bool)) &NetworkToSurface3::SetDoSmoothing, 
            " " , py::arg("doSmoothing"))
        .def(
            "SetNumSmoothingIterations", 
            (void(NetworkToSurface3::*)(unsigned int)) &NetworkToSurface3::SetNumSmoothingIterations, 
            " " , py::arg("numIterations"))
        .def(
            "SetSmoothingFeatureAngle", 
            (void(NetworkToSurface3::*)(double)) &NetworkToSurface3::SetSmoothingFeatureAngle, 
            " " , py::arg("featureAngle"))
        .def(
            "SetBandPassFrequency", 
            (void(NetworkToSurface3::*)(double)) &NetworkToSurface3::SetBandPassFrequency, 
            " " , py::arg("bandPassFrequency"))
        .def(
            "SetVesselNetwork", 
            (void(NetworkToSurface3::*)(::std::shared_ptr<VesselNetwork<3> >)) &NetworkToSurface3::SetVesselNetwork, 
            " " , py::arg("pNetwork"))
        .def(
            "SetResamplingSplineSize", 
            (void(NetworkToSurface3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &NetworkToSurface3::SetResamplingSplineSize, 
            " " , py::arg("splineResampleSize"))
        .def(
            "SetDoSurfaceRemeshing", 
            (void(NetworkToSurface3::*)(bool)) &NetworkToSurface3::SetDoSurfaceRemeshing, 
            " " , py::arg("doRemeshing"))
        .def(
            "SetRemeshingTargetEdgeLength", 
            (void(NetworkToSurface3::*)(double)) &NetworkToSurface3::SetRemeshingTargetEdgeLength, 
            " " , py::arg("length"))
        .def(
            "Update", 
            (void(NetworkToSurface3::*)()) &NetworkToSurface3::Update, 
            " " )
    ;
}
