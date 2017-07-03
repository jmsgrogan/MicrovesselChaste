#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "BetteridgeHaematocritSolver.hpp"

#include "BetteridgeHaematocritSolver3.cppwg.hpp"

namespace py = pybind11;
typedef BetteridgeHaematocritSolver<3 > BetteridgeHaematocritSolver3;
;

class BetteridgeHaematocritSolver3_Overloads : public BetteridgeHaematocritSolver3{
    public:
    using BetteridgeHaematocritSolver3::BetteridgeHaematocritSolver;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            BetteridgeHaematocritSolver3,
            Calculate,
            );
    }

};
void register_BetteridgeHaematocritSolver3_class(py::module &m){
py::class_<BetteridgeHaematocritSolver3 , BetteridgeHaematocritSolver3_Overloads   >(m, "BetteridgeHaematocritSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<BetteridgeHaematocritSolver<3> >(*)()) &BetteridgeHaematocritSolver3::Create, 
            " " )
        .def(
            "Calculate", 
            (void(BetteridgeHaematocritSolver3::*)()) &BetteridgeHaematocritSolver3::Calculate, 
            " " )
        .def(
            "SetExceptionOnFailedConverge", 
            (void(BetteridgeHaematocritSolver3::*)(bool)) &BetteridgeHaematocritSolver3::SetExceptionOnFailedConverge, 
            " " , py::arg("setException"))
        .def(
            "SetTHR", 
            (void(BetteridgeHaematocritSolver3::*)(::boost::units::quantity<boost::units::unit<boost::units::dimensionless_type, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &BetteridgeHaematocritSolver3::SetTHR, 
            " " , py::arg("thr"))
        .def(
            "SetAlpha", 
            (void(BetteridgeHaematocritSolver3::*)(::boost::units::quantity<boost::units::unit<boost::units::dimensionless_type, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &BetteridgeHaematocritSolver3::SetAlpha, 
            " " , py::arg("alpha"))
        .def(
            "SetHaematocrit", 
            (void(BetteridgeHaematocritSolver3::*)(::boost::units::quantity<boost::units::unit<boost::units::dimensionless_type, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &BetteridgeHaematocritSolver3::SetHaematocrit, 
            " " , py::arg("haematocrit"))
        .def(
            "SetUseHigherConnectivityBranches", 
            (void(BetteridgeHaematocritSolver3::*)(bool)) &BetteridgeHaematocritSolver3::SetUseHigherConnectivityBranches, 
            " " , py::arg("useHighConnectivity"))
        .def(
            "SetTurnOffFungModel", 
            (void(BetteridgeHaematocritSolver3::*)(bool)) &BetteridgeHaematocritSolver3::SetTurnOffFungModel, 
            " " , py::arg("turnOffFungModel"))
        .def(
            "SetUseRandomSplittingModel", 
            (void(BetteridgeHaematocritSolver3::*)(bool)) &BetteridgeHaematocritSolver3::SetUseRandomSplittingModel, 
            " " , py::arg("useRandomSplittingModel"))
    ;
}
