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

#include "DiscreteContinuumMeshGenerator2_2.cppwg.hpp"

namespace py = pybind11;
typedef DiscreteContinuumMeshGenerator<2,2 > DiscreteContinuumMeshGenerator2_2;
;

void register_DiscreteContinuumMeshGenerator2_2_class(py::module &m){
py::class_<DiscreteContinuumMeshGenerator2_2    >(m, "DiscreteContinuumMeshGenerator2_2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<DiscreteContinuumMeshGenerator<2, 2> >(*)()) &DiscreteContinuumMeshGenerator2_2::Create, 
            " " )
        .def(
            "GetMesh", 
            (::std::shared_ptr<DiscreteContinuumMesh<2, 2> >(DiscreteContinuumMeshGenerator2_2::*)()) &DiscreteContinuumMeshGenerator2_2::GetMesh, 
            " " )
        .def(
            "SetDomain", 
            (void(DiscreteContinuumMeshGenerator2_2::*)(::std::shared_ptr<Part<2> >)) &DiscreteContinuumMeshGenerator2_2::SetDomain, 
            " " , py::arg("pDomain"))
        .def(
            "SetDomain", 
            (void(DiscreteContinuumMeshGenerator2_2::*)(::vtkSmartPointer<vtkPolyData>)) &DiscreteContinuumMeshGenerator2_2::SetDomain, 
            " " , py::arg("pDomain"))
        .def(
            "SetDomain", 
            (void(DiscreteContinuumMeshGenerator2_2::*)(::std::string const &)) &DiscreteContinuumMeshGenerator2_2::SetDomain, 
            " " , py::arg("rPathToStl"))
        .def(
            "SetMaxElementArea", 
            (void(DiscreteContinuumMeshGenerator2_2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<3, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &DiscreteContinuumMeshGenerator2_2::SetMaxElementArea, 
            " " , py::arg("maxElementArea"))
        .def(
            "SetHoles", 
            (void(DiscreteContinuumMeshGenerator2_2::*)(::std::vector<DimensionalChastePoint<2>, std::allocator<DimensionalChastePoint<2> > >)) &DiscreteContinuumMeshGenerator2_2::SetHoles, 
            " " , py::arg("holes"))
        .def(
            "SetRegionMarkers", 
            (void(DiscreteContinuumMeshGenerator2_2::*)(::std::vector<DimensionalChastePoint<2>, std::allocator<DimensionalChastePoint<2> > >)) &DiscreteContinuumMeshGenerator2_2::SetRegionMarkers, 
            " " , py::arg("regionMarkers"))
        .def(
            "Update", 
            (void(DiscreteContinuumMeshGenerator2_2::*)()) &DiscreteContinuumMeshGenerator2_2::Update, 
            " " )
    ;
}
