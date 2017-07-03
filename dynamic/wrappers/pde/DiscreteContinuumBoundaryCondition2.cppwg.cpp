#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"

#include "DiscreteContinuumBoundaryCondition2.cppwg.hpp"

namespace py = pybind11;
typedef DiscreteContinuumBoundaryCondition<2 > DiscreteContinuumBoundaryCondition2;
;

void register_DiscreteContinuumBoundaryCondition2_class(py::module &m){
py::class_<DiscreteContinuumBoundaryCondition2    >(m, "DiscreteContinuumBoundaryCondition2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<DiscreteContinuumBoundaryCondition<2> >(*)()) &DiscreteContinuumBoundaryCondition2::Create, 
            " " )
        .def(
            "GetType", 
            (::BoundaryConditionType::Value(DiscreteContinuumBoundaryCondition2::*)()) &DiscreteContinuumBoundaryCondition2::GetType, 
            " " )
        .def(
            "GetValue", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(DiscreteContinuumBoundaryCondition2::*)()) &DiscreteContinuumBoundaryCondition2::GetValue, 
            " " )
        .def(
            "GetValue", 
            (::std::pair<bool, boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> >(DiscreteContinuumBoundaryCondition2::*)(::DimensionalChastePoint<2>, double)) &DiscreteContinuumBoundaryCondition2::GetValue, 
            " " , py::arg("location"), py::arg("tolerance"))
        .def(
            "UpdateBoundaryConditions", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::std::shared_ptr<BoundaryConditionsContainer<2, 2, 1> >)) &DiscreteContinuumBoundaryCondition2::UpdateBoundaryConditions, 
            " " , py::arg("pContainer"))
        .def(
            "UpdateBoundaryConditions", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::std::shared_ptr<std::vector<std::pair<bool, boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> >, std::allocator<std::pair<bool, boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > > >)) &DiscreteContinuumBoundaryCondition2::UpdateBoundaryConditions, 
            " " , py::arg("pBoundaryConditions"))
        .def(
            "SetDomain", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::std::shared_ptr<Part<2> >)) &DiscreteContinuumBoundaryCondition2::SetDomain, 
            " " , py::arg("pDomain"))
        .def(
            "SetIsRobin", 
            (void(DiscreteContinuumBoundaryCondition2::*)(bool)) &DiscreteContinuumBoundaryCondition2::SetIsRobin, 
            " " , py::arg("isRobin"))
        .def(
            "SetIsNeumann", 
            (void(DiscreteContinuumBoundaryCondition2::*)(bool)) &DiscreteContinuumBoundaryCondition2::SetIsNeumann, 
            " " , py::arg("isNeumann"))
        .def(
            "SetLabel", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::std::string const &)) &DiscreteContinuumBoundaryCondition2::SetLabel, 
            " " , py::arg("label"))
        .def(
            "SetPoints", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::std::vector<DimensionalChastePoint<2>, std::allocator<DimensionalChastePoint<2> > >)) &DiscreteContinuumBoundaryCondition2::SetPoints, 
            " " , py::arg("points"))
        .def(
            "SetPoints", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::vtkSmartPointer<vtkPoints>)) &DiscreteContinuumBoundaryCondition2::SetPoints, 
            " " , py::arg("pPoints"))
        .def(
            "SetGridCalculator", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::std::shared_ptr<GridCalculator<2> >)) &DiscreteContinuumBoundaryCondition2::SetGridCalculator, 
            " " , py::arg("pGridCalculator"))
        .def(
            "SetSource", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::BoundaryConditionSource::Value)) &DiscreteContinuumBoundaryCondition2::SetSource, 
            " " , py::arg("boundarySource"))
        .def(
            "SetType", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::BoundaryConditionType::Value)) &DiscreteContinuumBoundaryCondition2::SetType, 
            " " , py::arg("boundaryType"))
        .def(
            "SetNetwork", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::std::shared_ptr<VesselNetwork<2> >)) &DiscreteContinuumBoundaryCondition2::SetNetwork, 
            " " , py::arg("pNetwork"))
        .def(
            "SetValue", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &DiscreteContinuumBoundaryCondition2::SetValue, 
            " " , py::arg("value"))
    ;
}
