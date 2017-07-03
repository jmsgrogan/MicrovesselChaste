#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"

#include "DiscreteContinuumMeshGenerator3_3.cppwg.hpp"

namespace py = pybind11;
typedef DiscreteContinuumMeshGenerator<3,3 > DiscreteContinuumMeshGenerator3_3;
;

void register_DiscreteContinuumMeshGenerator3_3_class(py::module &m){
py::class_<DiscreteContinuumMeshGenerator3_3    >(m, "DiscreteContinuumMeshGenerator3_3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<DiscreteContinuumMeshGenerator<3, 3> >(*)()) &DiscreteContinuumMeshGenerator3_3::Create, 
            " " )
        .def(
            "GetMesh", 
            (::std::shared_ptr<DiscreteContinuumMesh<3, 3> >(DiscreteContinuumMeshGenerator3_3::*)()) &DiscreteContinuumMeshGenerator3_3::GetMesh, 
            " " )
        .def(
            "SetDomain", 
            (void(DiscreteContinuumMeshGenerator3_3::*)(::std::shared_ptr<Part<3> >)) &DiscreteContinuumMeshGenerator3_3::SetDomain, 
            " " , py::arg("pDomain"))
        .def(
            "SetDomain", 
            (void(DiscreteContinuumMeshGenerator3_3::*)(::vtkSmartPointer<vtkPolyData>)) &DiscreteContinuumMeshGenerator3_3::SetDomain, 
            " " , py::arg("pDomain"))
        .def(
            "SetDomain", 
            (void(DiscreteContinuumMeshGenerator3_3::*)(::std::string const &)) &DiscreteContinuumMeshGenerator3_3::SetDomain, 
            " " , py::arg("rPathToStl"))
        .def(
            "SetMaxElementArea", 
            (void(DiscreteContinuumMeshGenerator3_3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<3, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &DiscreteContinuumMeshGenerator3_3::SetMaxElementArea, 
            " " , py::arg("maxElementArea"))
        .def(
            "SetHoles", 
            (void(DiscreteContinuumMeshGenerator3_3::*)(::std::vector<DimensionalChastePoint<3>, std::allocator<DimensionalChastePoint<3> > >)) &DiscreteContinuumMeshGenerator3_3::SetHoles, 
            " " , py::arg("holes"))
        .def(
            "SetRegionMarkers", 
            (void(DiscreteContinuumMeshGenerator3_3::*)(::std::vector<DimensionalChastePoint<3>, std::allocator<DimensionalChastePoint<3> > >)) &DiscreteContinuumMeshGenerator3_3::SetRegionMarkers, 
            " " , py::arg("regionMarkers"))
        .def(
            "Update", 
            (void(DiscreteContinuumMeshGenerator3_3::*)()) &DiscreteContinuumMeshGenerator3_3::Update, 
            " " )
    ;
}
