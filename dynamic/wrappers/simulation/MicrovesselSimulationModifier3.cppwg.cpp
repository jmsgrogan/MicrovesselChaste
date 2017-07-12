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
#include "MicrovesselSimulationModifier.hpp"

#include "MicrovesselSimulationModifier3.cppwg.hpp"

namespace py = pybind11;
typedef MicrovesselSimulationModifier<3 > MicrovesselSimulationModifier3;
;

class MicrovesselSimulationModifier3_Overloads : public MicrovesselSimulationModifier3{
    public:
    using MicrovesselSimulationModifier3::MicrovesselSimulationModifier;
    void OutputSimulationModifierParameters(::out_stream & rParamsFile) override {
        PYBIND11_OVERLOAD(
            void,
            MicrovesselSimulationModifier3,
            OutputSimulationModifierParameters,
            rParamsFile);
    }
    void SetupSolve(::AbstractCellPopulation<3, 3> & rCellPopulation, ::std::string outputDirectory) override {
        PYBIND11_OVERLOAD(
            void,
            MicrovesselSimulationModifier3,
            SetupSolve,
            rCellPopulation, 
outputDirectory);
    }
    void UpdateAtEndOfTimeStep(::AbstractCellPopulation<3, 3> & rCellPopulation) override {
        PYBIND11_OVERLOAD(
            void,
            MicrovesselSimulationModifier3,
            UpdateAtEndOfTimeStep,
            rCellPopulation);
    }

};
void register_MicrovesselSimulationModifier3_class(py::module &m){
py::class_<MicrovesselSimulationModifier3 , MicrovesselSimulationModifier3_Overloads   >(m, "MicrovesselSimulationModifier3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<MicrovesselSimulationModifier<3> >(*)()) &MicrovesselSimulationModifier3::Create, 
            " "  )
        .def(
            "OutputSimulationModifierParameters", 
            (void(MicrovesselSimulationModifier3::*)(::out_stream &)) &MicrovesselSimulationModifier3::OutputSimulationModifierParameters, 
            " " , py::arg("rParamsFile") )
        .def(
            "SetCellDataUpdateLabels", 
            (void(MicrovesselSimulationModifier3::*)(::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > >)) &MicrovesselSimulationModifier3::SetCellDataUpdateLabels, 
            " " , py::arg("labels") )
        .def(
            "SetMicrovesselSolver", 
            (void(MicrovesselSimulationModifier3::*)(::std::shared_ptr<MicrovesselSolver<3> >)) &MicrovesselSimulationModifier3::SetMicrovesselSolver, 
            " " , py::arg("pSolver") )
        .def(
            "SetGridCalculator", 
            (void(MicrovesselSimulationModifier3::*)(::std::shared_ptr<GridCalculator<3> >)) &MicrovesselSimulationModifier3::SetGridCalculator, 
            " " , py::arg("pGridCalculator") )
        .def(
            "SetupSolve", 
            (void(MicrovesselSimulationModifier3::*)(::AbstractCellPopulation<3, 3> &, ::std::string)) &MicrovesselSimulationModifier3::SetupSolve, 
            " " , py::arg("rCellPopulation"), py::arg("outputDirectory") )
        .def(
            "SetCellPopulationLengthScale", 
            (void(MicrovesselSimulationModifier3::*)(::QLength)) &MicrovesselSimulationModifier3::SetCellPopulationLengthScale, 
            " " , py::arg("cellLengthScale") )
        .def(
            "SetCellPopulationConcentrationScale", 
            (void(MicrovesselSimulationModifier3::*)(::QConcentration)) &MicrovesselSimulationModifier3::SetCellPopulationConcentrationScale, 
            " " , py::arg("cellConcentrationScale") )
        .def(
            "UpdateAtEndOfTimeStep", 
            (void(MicrovesselSimulationModifier3::*)(::AbstractCellPopulation<3, 3> &)) &MicrovesselSimulationModifier3::UpdateAtEndOfTimeStep, 
            " " , py::arg("rCellPopulation") )
        .def(
            "UpdateCellData", 
            (void(MicrovesselSimulationModifier3::*)(::AbstractCellPopulation<3, 3> &)) &MicrovesselSimulationModifier3::UpdateCellData, 
            " " , py::arg("rCellPopulation") )
    ;
}
