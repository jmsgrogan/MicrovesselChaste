#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "LQRadiotherapyCellKiller.hpp"

#include "LQRadiotherapyCellKiller3.cppwg.hpp"

namespace py = pybind11;
typedef LQRadiotherapyCellKiller<3 > LQRadiotherapyCellKiller3;
;

class LQRadiotherapyCellKiller3_Overloads : public LQRadiotherapyCellKiller3{
    public:
    using LQRadiotherapyCellKiller3::LQRadiotherapyCellKiller;
    void CheckAndLabelCellsForApoptosisOrDeath() override {
        PYBIND11_OVERLOAD(
            void,
            LQRadiotherapyCellKiller3,
            CheckAndLabelCellsForApoptosisOrDeath,
            );
    }
    void OutputCellKillerParameters(::out_stream & rParamsFile) override {
        PYBIND11_OVERLOAD(
            void,
            LQRadiotherapyCellKiller3,
            OutputCellKillerParameters,
            rParamsFile);
    }

};
void register_LQRadiotherapyCellKiller3_class(py::module &m){
py::class_<LQRadiotherapyCellKiller3 , LQRadiotherapyCellKiller3_Overloads   >(m, "LQRadiotherapyCellKiller3")
        .def(py::init<::AbstractCellPopulation<3, 3> * >(), py::arg("pCellPopulation"))
        .def(
            "AddTimeOfRadiation", 
            (void(LQRadiotherapyCellKiller3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &LQRadiotherapyCellKiller3::AddTimeOfRadiation, 
            " " , py::arg("time"))
        .def(
            "CheckAndLabelSingleCellForApoptosis", 
            (void(LQRadiotherapyCellKiller3::*)(::CellPtr)) &LQRadiotherapyCellKiller3::CheckAndLabelSingleCellForApoptosis, 
            " " , py::arg("pCell"))
        .def(
            "CheckAndLabelCellsForApoptosisOrDeath", 
            (void(LQRadiotherapyCellKiller3::*)()) &LQRadiotherapyCellKiller3::CheckAndLabelCellsForApoptosisOrDeath, 
            " " )
        .def(
            "OutputCellKillerParameters", 
            (void(LQRadiotherapyCellKiller3::*)(::out_stream &)) &LQRadiotherapyCellKiller3::OutputCellKillerParameters, 
            " " , py::arg("rParamsFile"))
        .def(
            "SetDoseInjected", 
            (void(LQRadiotherapyCellKiller3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<2, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-2, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &LQRadiotherapyCellKiller3::SetDoseInjected, 
            " " , py::arg("d"))
        .def(
            "SetTimeOfRadiation", 
            (void(LQRadiotherapyCellKiller3::*)(::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > >)) &LQRadiotherapyCellKiller3::SetTimeOfRadiation, 
            " " , py::arg("t"))
        .def(
            "SetCancerousRadiosensitivity", 
            (void(LQRadiotherapyCellKiller3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-2, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<2, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-4, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<4, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &LQRadiotherapyCellKiller3::SetCancerousRadiosensitivity, 
            " " , py::arg("alpha"), py::arg("beta"))
        .def(
            "SetNormalRadiosensitivity", 
            (void(LQRadiotherapyCellKiller3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-2, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<2, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-4, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<4, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &LQRadiotherapyCellKiller3::SetNormalRadiosensitivity, 
            " " , py::arg("alpha"), py::arg("beta"))
        .def(
            "SetOerAlphaMax", 
            (void(LQRadiotherapyCellKiller3::*)(double)) &LQRadiotherapyCellKiller3::SetOerAlphaMax, 
            " " , py::arg("value"))
        .def(
            "SetOerAlphaMin", 
            (void(LQRadiotherapyCellKiller3::*)(double)) &LQRadiotherapyCellKiller3::SetOerAlphaMin, 
            " " , py::arg("value"))
        .def(
            "SetOerBetaMax", 
            (void(LQRadiotherapyCellKiller3::*)(double)) &LQRadiotherapyCellKiller3::SetOerBetaMax, 
            " " , py::arg("value"))
        .def(
            "SetOerBetaMin", 
            (void(LQRadiotherapyCellKiller3::*)(double)) &LQRadiotherapyCellKiller3::SetOerBetaMin, 
            " " , py::arg("value"))
        .def(
            "SetOerConstant", 
            (void(LQRadiotherapyCellKiller3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &LQRadiotherapyCellKiller3::SetOerConstant, 
            " " , py::arg("value"))
        .def(
            "SetAlphaMax", 
            (void(LQRadiotherapyCellKiller3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-2, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<2, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &LQRadiotherapyCellKiller3::SetAlphaMax, 
            " " , py::arg("value"))
        .def(
            "SetBetaMax", 
            (void(LQRadiotherapyCellKiller3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-4, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<4, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &LQRadiotherapyCellKiller3::SetBetaMax, 
            " " , py::arg("value"))
        .def(
            "UseOer", 
            (void(LQRadiotherapyCellKiller3::*)(bool)) &LQRadiotherapyCellKiller3::UseOer, 
            " " , py::arg("useOer"))
    ;
}
