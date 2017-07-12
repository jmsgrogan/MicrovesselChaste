#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "vtkPolyData.h"
#include "LQRadiotherapyCellKiller.hpp"

#include "LQRadiotherapyCellKiller2.cppwg.hpp"

namespace py = pybind11;
typedef LQRadiotherapyCellKiller<2 > LQRadiotherapyCellKiller2;
;

class LQRadiotherapyCellKiller2_Overloads : public LQRadiotherapyCellKiller2{
    public:
    using LQRadiotherapyCellKiller2::LQRadiotherapyCellKiller;
    void CheckAndLabelCellsForApoptosisOrDeath() override {
        PYBIND11_OVERLOAD(
            void,
            LQRadiotherapyCellKiller2,
            CheckAndLabelCellsForApoptosisOrDeath,
            );
    }
    void OutputCellKillerParameters(::out_stream & rParamsFile) override {
        PYBIND11_OVERLOAD(
            void,
            LQRadiotherapyCellKiller2,
            OutputCellKillerParameters,
            rParamsFile);
    }

};
void register_LQRadiotherapyCellKiller2_class(py::module &m){
py::class_<LQRadiotherapyCellKiller2 , LQRadiotherapyCellKiller2_Overloads   >(m, "LQRadiotherapyCellKiller2")
        .def(py::init<::AbstractCellPopulation<2, 2> * >(), py::arg("pCellPopulation"))
        .def(
            "AddTimeOfRadiation", 
            (void(LQRadiotherapyCellKiller2::*)(::QTime)) &LQRadiotherapyCellKiller2::AddTimeOfRadiation, 
            " " , py::arg("time") )
        .def(
            "CheckAndLabelSingleCellForApoptosis", 
            (void(LQRadiotherapyCellKiller2::*)(::CellPtr)) &LQRadiotherapyCellKiller2::CheckAndLabelSingleCellForApoptosis, 
            " " , py::arg("pCell") )
        .def(
            "CheckAndLabelCellsForApoptosisOrDeath", 
            (void(LQRadiotherapyCellKiller2::*)()) &LQRadiotherapyCellKiller2::CheckAndLabelCellsForApoptosisOrDeath, 
            " "  )
        .def(
            "OutputCellKillerParameters", 
            (void(LQRadiotherapyCellKiller2::*)(::out_stream &)) &LQRadiotherapyCellKiller2::OutputCellKillerParameters, 
            " " , py::arg("rParamsFile") )
        .def(
            "SetDoseInjected", 
            (void(LQRadiotherapyCellKiller2::*)(::QAbsorbedDose)) &LQRadiotherapyCellKiller2::SetDoseInjected, 
            " " , py::arg("d") )
        .def(
            "SetTimeOfRadiation", 
            (void(LQRadiotherapyCellKiller2::*)(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > >)) &LQRadiotherapyCellKiller2::SetTimeOfRadiation, 
            " " , py::arg("t") )
        .def(
            "SetCancerousRadiosensitivity", 
            (void(LQRadiotherapyCellKiller2::*)(::QPerAbsorbedDose, ::QPerAbsorbedDoseSquared)) &LQRadiotherapyCellKiller2::SetCancerousRadiosensitivity, 
            " " , py::arg("alpha"), py::arg("beta") )
        .def(
            "SetNormalRadiosensitivity", 
            (void(LQRadiotherapyCellKiller2::*)(::QPerAbsorbedDose, ::QPerAbsorbedDoseSquared)) &LQRadiotherapyCellKiller2::SetNormalRadiosensitivity, 
            " " , py::arg("alpha"), py::arg("beta") )
        .def(
            "SetOerAlphaMax", 
            (void(LQRadiotherapyCellKiller2::*)(double)) &LQRadiotherapyCellKiller2::SetOerAlphaMax, 
            " " , py::arg("value") )
        .def(
            "SetOerAlphaMin", 
            (void(LQRadiotherapyCellKiller2::*)(double)) &LQRadiotherapyCellKiller2::SetOerAlphaMin, 
            " " , py::arg("value") )
        .def(
            "SetOerBetaMax", 
            (void(LQRadiotherapyCellKiller2::*)(double)) &LQRadiotherapyCellKiller2::SetOerBetaMax, 
            " " , py::arg("value") )
        .def(
            "SetOerBetaMin", 
            (void(LQRadiotherapyCellKiller2::*)(double)) &LQRadiotherapyCellKiller2::SetOerBetaMin, 
            " " , py::arg("value") )
        .def(
            "SetOerConstant", 
            (void(LQRadiotherapyCellKiller2::*)(::QConcentration)) &LQRadiotherapyCellKiller2::SetOerConstant, 
            " " , py::arg("value") )
        .def(
            "SetAlphaMax", 
            (void(LQRadiotherapyCellKiller2::*)(::QPerAbsorbedDose)) &LQRadiotherapyCellKiller2::SetAlphaMax, 
            " " , py::arg("value") )
        .def(
            "SetBetaMax", 
            (void(LQRadiotherapyCellKiller2::*)(::QPerAbsorbedDoseSquared)) &LQRadiotherapyCellKiller2::SetBetaMax, 
            " " , py::arg("value") )
        .def(
            "UseOer", 
            (void(LQRadiotherapyCellKiller2::*)(bool)) &LQRadiotherapyCellKiller2::UseOer, 
            " " , py::arg("useOer") )
    ;
}
