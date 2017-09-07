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
#include "Owen2011TrackingModifier.hpp"

#include "PythonObjectConverters.hpp"
#include "Owen2011TrackingModifier2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef Owen2011TrackingModifier<2 > Owen2011TrackingModifier2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class Owen2011TrackingModifier2_Overloads : public Owen2011TrackingModifier2{
    public:
    using Owen2011TrackingModifier2::Owen2011TrackingModifier;
    void UpdateAtEndOfTimeStep(::AbstractCellPopulation<2, 2> & rCellPopulation) override {
        PYBIND11_OVERLOAD(
            void,
            Owen2011TrackingModifier2,
            UpdateAtEndOfTimeStep,
            rCellPopulation);
    }
    void SetupSolve(::AbstractCellPopulation<2, 2> & rCellPopulation, ::std::string outputDirectory) override {
        PYBIND11_OVERLOAD(
            void,
            Owen2011TrackingModifier2,
            SetupSolve,
            rCellPopulation, 
outputDirectory);
    }
    void OutputSimulationModifierParameters(::out_stream & rParamsFile) override {
        PYBIND11_OVERLOAD(
            void,
            Owen2011TrackingModifier2,
            OutputSimulationModifierParameters,
            rParamsFile);
    }

};
void register_Owen2011TrackingModifier2_class(py::module &m){
py::class_<Owen2011TrackingModifier2 , Owen2011TrackingModifier2_Overloads , std::shared_ptr<Owen2011TrackingModifier2 >   >(m, "Owen2011TrackingModifier2")
        .def(py::init< >())
        .def(
            "UpdateAtEndOfTimeStep", 
            (void(Owen2011TrackingModifier2::*)(::AbstractCellPopulation<2, 2> &)) &Owen2011TrackingModifier2::UpdateAtEndOfTimeStep, 
            " " , py::arg("rCellPopulation") )
        .def(
            "SetupSolve", 
            (void(Owen2011TrackingModifier2::*)(::AbstractCellPopulation<2, 2> &, ::std::string)) &Owen2011TrackingModifier2::SetupSolve, 
            " " , py::arg("rCellPopulation"), py::arg("outputDirectory") )
        .def(
            "UpdateCellData", 
            (void(Owen2011TrackingModifier2::*)(::AbstractCellPopulation<2, 2> &)) &Owen2011TrackingModifier2::UpdateCellData, 
            " " , py::arg("rCellPopulation") )
        .def(
            "OutputSimulationModifierParameters", 
            (void(Owen2011TrackingModifier2::*)(::out_stream &)) &Owen2011TrackingModifier2::OutputSimulationModifierParameters, 
            " " , py::arg("rParamsFile") )
    ;
}
