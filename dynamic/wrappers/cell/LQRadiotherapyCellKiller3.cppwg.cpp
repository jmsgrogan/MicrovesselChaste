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

#include "LQRadiotherapyCellKiller3.cppwg.hpp"

namespace py = pybind11;
typedef LQRadiotherapyCellKiller<3 > LQRadiotherapyCellKiller3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

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
py::class_<LQRadiotherapyCellKiller3 , LQRadiotherapyCellKiller3_Overloads , std::shared_ptr<LQRadiotherapyCellKiller3 >   >(m, "LQRadiotherapyCellKiller3")
        .def(py::init<::AbstractCellPopulation<3, 3> * >(), py::arg("pCellPopulation"))
        .def(
            "AddTimeOfRadiation", 
            (void(LQRadiotherapyCellKiller3::*)(::QTime)) &LQRadiotherapyCellKiller3::AddTimeOfRadiation, 
            " " , py::arg("time") )
        .def(
            "CheckAndLabelSingleCellForApoptosis", 
            (void(LQRadiotherapyCellKiller3::*)(::CellPtr)) &LQRadiotherapyCellKiller3::CheckAndLabelSingleCellForApoptosis, 
            " " , py::arg("pCell") )
        .def(
            "CheckAndLabelCellsForApoptosisOrDeath", 
            (void(LQRadiotherapyCellKiller3::*)()) &LQRadiotherapyCellKiller3::CheckAndLabelCellsForApoptosisOrDeath, 
            " "  )
        .def(
            "OutputCellKillerParameters", 
            (void(LQRadiotherapyCellKiller3::*)(::out_stream &)) &LQRadiotherapyCellKiller3::OutputCellKillerParameters, 
            " " , py::arg("rParamsFile") )
        .def(
            "SetDoseInjected", 
            (void(LQRadiotherapyCellKiller3::*)(::QAbsorbedDose)) &LQRadiotherapyCellKiller3::SetDoseInjected, 
            " " , py::arg("d") )
        .def(
            "SetTimeOfRadiation", 
            (void(LQRadiotherapyCellKiller3::*)(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > >)) &LQRadiotherapyCellKiller3::SetTimeOfRadiation, 
            " " , py::arg("t") )
        .def(
            "SetCancerousRadiosensitivity", 
            (void(LQRadiotherapyCellKiller3::*)(::QPerAbsorbedDose, ::QPerAbsorbedDoseSquared)) &LQRadiotherapyCellKiller3::SetCancerousRadiosensitivity, 
            " " , py::arg("alpha"), py::arg("beta") )
        .def(
            "SetNormalRadiosensitivity", 
            (void(LQRadiotherapyCellKiller3::*)(::QPerAbsorbedDose, ::QPerAbsorbedDoseSquared)) &LQRadiotherapyCellKiller3::SetNormalRadiosensitivity, 
            " " , py::arg("alpha"), py::arg("beta") )
        .def(
            "SetOerAlphaMax", 
            (void(LQRadiotherapyCellKiller3::*)(double)) &LQRadiotherapyCellKiller3::SetOerAlphaMax, 
            " " , py::arg("value") )
        .def(
            "SetOerAlphaMin", 
            (void(LQRadiotherapyCellKiller3::*)(double)) &LQRadiotherapyCellKiller3::SetOerAlphaMin, 
            " " , py::arg("value") )
        .def(
            "SetOerBetaMax", 
            (void(LQRadiotherapyCellKiller3::*)(double)) &LQRadiotherapyCellKiller3::SetOerBetaMax, 
            " " , py::arg("value") )
        .def(
            "SetOerBetaMin", 
            (void(LQRadiotherapyCellKiller3::*)(double)) &LQRadiotherapyCellKiller3::SetOerBetaMin, 
            " " , py::arg("value") )
        .def(
            "SetOerConstant", 
            (void(LQRadiotherapyCellKiller3::*)(::QConcentration)) &LQRadiotherapyCellKiller3::SetOerConstant, 
            " " , py::arg("value") )
        .def(
            "SetAlphaMax", 
            (void(LQRadiotherapyCellKiller3::*)(::QPerAbsorbedDose)) &LQRadiotherapyCellKiller3::SetAlphaMax, 
            " " , py::arg("value") )
        .def(
            "SetBetaMax", 
            (void(LQRadiotherapyCellKiller3::*)(::QPerAbsorbedDoseSquared)) &LQRadiotherapyCellKiller3::SetBetaMax, 
            " " , py::arg("value") )
        .def(
            "UseOer", 
            (void(LQRadiotherapyCellKiller3::*)(bool)) &LQRadiotherapyCellKiller3::UseOer, 
            " " , py::arg("useOer") )
    ;
}
