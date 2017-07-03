#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "Owen2011TrackingModifier.hpp"

#include "Owen2011TrackingModifier3.cppwg.hpp"

namespace py = pybind11;
typedef Owen2011TrackingModifier<3 > Owen2011TrackingModifier3;
;

class Owen2011TrackingModifier3_Overloads : public Owen2011TrackingModifier3{
    public:
    using Owen2011TrackingModifier3::Owen2011TrackingModifier;
    void UpdateAtEndOfTimeStep(::AbstractCellPopulation<3, 3> & rCellPopulation) override {
        PYBIND11_OVERLOAD(
            void,
            Owen2011TrackingModifier3,
            UpdateAtEndOfTimeStep,
            rCellPopulation);
    }
    void SetupSolve(::AbstractCellPopulation<3, 3> & rCellPopulation, ::std::string outputDirectory) override {
        PYBIND11_OVERLOAD(
            void,
            Owen2011TrackingModifier3,
            SetupSolve,
            rCellPopulation, 
outputDirectory);
    }
    void OutputSimulationModifierParameters(::out_stream & rParamsFile) override {
        PYBIND11_OVERLOAD(
            void,
            Owen2011TrackingModifier3,
            OutputSimulationModifierParameters,
            rParamsFile);
    }

};
void register_Owen2011TrackingModifier3_class(py::module &m){
py::class_<Owen2011TrackingModifier3 , Owen2011TrackingModifier3_Overloads   >(m, "Owen2011TrackingModifier3")
        .def(py::init< >())
        .def(
            "UpdateAtEndOfTimeStep", 
            (void(Owen2011TrackingModifier3::*)(::AbstractCellPopulation<3, 3> &)) &Owen2011TrackingModifier3::UpdateAtEndOfTimeStep, 
            " " , py::arg("rCellPopulation"))
        .def(
            "SetupSolve", 
            (void(Owen2011TrackingModifier3::*)(::AbstractCellPopulation<3, 3> &, ::std::string)) &Owen2011TrackingModifier3::SetupSolve, 
            " " , py::arg("rCellPopulation"), py::arg("outputDirectory"))
        .def(
            "UpdateCellData", 
            (void(Owen2011TrackingModifier3::*)(::AbstractCellPopulation<3, 3> &)) &Owen2011TrackingModifier3::UpdateCellData, 
            " " , py::arg("rCellPopulation"))
        .def(
            "OutputSimulationModifierParameters", 
            (void(Owen2011TrackingModifier3::*)(::out_stream &)) &Owen2011TrackingModifier3::OutputSimulationModifierParameters, 
            " " , py::arg("rParamsFile"))
    ;
}
