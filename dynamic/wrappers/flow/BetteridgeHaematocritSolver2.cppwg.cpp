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

#include "BetteridgeHaematocritSolver2.cppwg.hpp"

namespace py = pybind11;
typedef BetteridgeHaematocritSolver<2 > BetteridgeHaematocritSolver2;
;

class BetteridgeHaematocritSolver2_Overloads : public BetteridgeHaematocritSolver2{
    public:
    using BetteridgeHaematocritSolver2::BetteridgeHaematocritSolver;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            BetteridgeHaematocritSolver2,
            Calculate,
            );
    }

};
void register_BetteridgeHaematocritSolver2_class(py::module &m){
py::class_<BetteridgeHaematocritSolver2 , BetteridgeHaematocritSolver2_Overloads   >(m, "BetteridgeHaematocritSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<BetteridgeHaematocritSolver<2> >(*)()) &BetteridgeHaematocritSolver2::Create, 
            " " )
        .def(
            "Calculate", 
            (void(BetteridgeHaematocritSolver2::*)()) &BetteridgeHaematocritSolver2::Calculate, 
            " " )
        .def(
            "SetExceptionOnFailedConverge", 
            (void(BetteridgeHaematocritSolver2::*)(bool)) &BetteridgeHaematocritSolver2::SetExceptionOnFailedConverge, 
            " " , py::arg("setException"))
        .def(
            "SetTHR", 
            (void(BetteridgeHaematocritSolver2::*)(::boost::units::quantity<boost::units::unit<boost::units::dimensionless_type, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &BetteridgeHaematocritSolver2::SetTHR, 
            " " , py::arg("thr"))
        .def(
            "SetAlpha", 
            (void(BetteridgeHaematocritSolver2::*)(::boost::units::quantity<boost::units::unit<boost::units::dimensionless_type, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &BetteridgeHaematocritSolver2::SetAlpha, 
            " " , py::arg("alpha"))
        .def(
            "SetHaematocrit", 
            (void(BetteridgeHaematocritSolver2::*)(::boost::units::quantity<boost::units::unit<boost::units::dimensionless_type, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &BetteridgeHaematocritSolver2::SetHaematocrit, 
            " " , py::arg("haematocrit"))
        .def(
            "SetUseHigherConnectivityBranches", 
            (void(BetteridgeHaematocritSolver2::*)(bool)) &BetteridgeHaematocritSolver2::SetUseHigherConnectivityBranches, 
            " " , py::arg("useHighConnectivity"))
        .def(
            "SetTurnOffFungModel", 
            (void(BetteridgeHaematocritSolver2::*)(bool)) &BetteridgeHaematocritSolver2::SetTurnOffFungModel, 
            " " , py::arg("turnOffFungModel"))
        .def(
            "SetUseRandomSplittingModel", 
            (void(BetteridgeHaematocritSolver2::*)(bool)) &BetteridgeHaematocritSolver2::SetUseRandomSplittingModel, 
            " " , py::arg("useRandomSplittingModel"))
    ;
}
