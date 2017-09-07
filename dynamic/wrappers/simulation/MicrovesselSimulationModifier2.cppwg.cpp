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

#include "PythonObjectConverters.hpp"
#include "MicrovesselSimulationModifier2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef MicrovesselSimulationModifier<2 > MicrovesselSimulationModifier2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class MicrovesselSimulationModifier2_Overloads : public MicrovesselSimulationModifier2{
    public:
    using MicrovesselSimulationModifier2::MicrovesselSimulationModifier;
    void OutputSimulationModifierParameters(::out_stream & rParamsFile) override {
        PYBIND11_OVERLOAD(
            void,
            MicrovesselSimulationModifier2,
            OutputSimulationModifierParameters,
            rParamsFile);
    }
    void SetupSolve(::AbstractCellPopulation<2, 2> & rCellPopulation, ::std::string outputDirectory) override {
        PYBIND11_OVERLOAD(
            void,
            MicrovesselSimulationModifier2,
            SetupSolve,
            rCellPopulation, 
outputDirectory);
    }
    void UpdateAtEndOfTimeStep(::AbstractCellPopulation<2, 2> & rCellPopulation) override {
        PYBIND11_OVERLOAD(
            void,
            MicrovesselSimulationModifier2,
            UpdateAtEndOfTimeStep,
            rCellPopulation);
    }

};
void register_MicrovesselSimulationModifier2_class(py::module &m){
py::class_<MicrovesselSimulationModifier2 , MicrovesselSimulationModifier2_Overloads , std::shared_ptr<MicrovesselSimulationModifier2 >   >(m, "MicrovesselSimulationModifier2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<MicrovesselSimulationModifier<2> >(*)()) &MicrovesselSimulationModifier2::Create, 
            " "  )
        .def(
            "OutputSimulationModifierParameters", 
            (void(MicrovesselSimulationModifier2::*)(::out_stream &)) &MicrovesselSimulationModifier2::OutputSimulationModifierParameters, 
            " " , py::arg("rParamsFile") )
        .def(
            "SetCellDataUpdateLabels", 
            (void(MicrovesselSimulationModifier2::*)(::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > >)) &MicrovesselSimulationModifier2::SetCellDataUpdateLabels, 
            " " , py::arg("labels") )
        .def(
            "SetMicrovesselSolver", 
            (void(MicrovesselSimulationModifier2::*)(::std::shared_ptr<MicrovesselSolver<2> >)) &MicrovesselSimulationModifier2::SetMicrovesselSolver, 
            " " , py::arg("pSolver") )
        .def(
            "SetGridCalculator", 
            (void(MicrovesselSimulationModifier2::*)(::std::shared_ptr<GridCalculator<2> >)) &MicrovesselSimulationModifier2::SetGridCalculator, 
            " " , py::arg("pGridCalculator") )
        .def(
            "SetupSolve", 
            (void(MicrovesselSimulationModifier2::*)(::AbstractCellPopulation<2, 2> &, ::std::string)) &MicrovesselSimulationModifier2::SetupSolve, 
            " " , py::arg("rCellPopulation"), py::arg("outputDirectory") )
        .def(
            "SetCellPopulationLengthScale", 
            (void(MicrovesselSimulationModifier2::*)(::QLength)) &MicrovesselSimulationModifier2::SetCellPopulationLengthScale, 
            " " , py::arg("cellLengthScale") )
        .def(
            "SetCellPopulationConcentrationScale", 
            (void(MicrovesselSimulationModifier2::*)(::QConcentration)) &MicrovesselSimulationModifier2::SetCellPopulationConcentrationScale, 
            " " , py::arg("cellConcentrationScale") )
        .def(
            "UpdateAtEndOfTimeStep", 
            (void(MicrovesselSimulationModifier2::*)(::AbstractCellPopulation<2, 2> &)) &MicrovesselSimulationModifier2::UpdateAtEndOfTimeStep, 
            " " , py::arg("rCellPopulation") )
        .def(
            "UpdateCellData", 
            (void(MicrovesselSimulationModifier2::*)(::AbstractCellPopulation<2, 2> &)) &MicrovesselSimulationModifier2::UpdateCellData, 
            " " , py::arg("rCellPopulation") )
    ;
}
