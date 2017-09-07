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
#include "Owen11CaUpdateRule.hpp"

#include "PythonObjectConverters.hpp"
#include "Owen11CaUpdateRule2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef Owen11CaUpdateRule<2 > Owen11CaUpdateRule2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class Owen11CaUpdateRule2_Overloads : public Owen11CaUpdateRule2{
    public:
    using Owen11CaUpdateRule2::Owen11CaUpdateRule;
    double EvaluateProbability(unsigned int currentNodeIndex, unsigned int targetNodeIndex, ::CaBasedCellPopulation<2> & rCellPopulation, double dt, double deltaX, ::CellPtr cell) override {
        PYBIND11_OVERLOAD(
            double,
            Owen11CaUpdateRule2,
            EvaluateProbability,
            currentNodeIndex, 
targetNodeIndex, 
rCellPopulation, 
dt, 
deltaX, 
cell);
    }
    void OutputUpdateRuleParameters(::out_stream & rParamsFile) override {
        PYBIND11_OVERLOAD(
            void,
            Owen11CaUpdateRule2,
            OutputUpdateRuleParameters,
            rParamsFile);
    }

};
void register_Owen11CaUpdateRule2_class(py::module &m){
py::class_<Owen11CaUpdateRule2 , Owen11CaUpdateRule2_Overloads , std::shared_ptr<Owen11CaUpdateRule2 >   >(m, "Owen11CaUpdateRule2")
        .def(py::init< >())
        .def(
            "EvaluateProbability", 
            (double(Owen11CaUpdateRule2::*)(unsigned int, unsigned int, ::CaBasedCellPopulation<2> &, double, double, ::CellPtr)) &Owen11CaUpdateRule2::EvaluateProbability, 
            " " , py::arg("currentNodeIndex"), py::arg("targetNodeIndex"), py::arg("rCellPopulation"), py::arg("dt"), py::arg("deltaX"), py::arg("cell") )
        .def(
            "GetDiffusionParameter", 
            (::QDiffusivity(Owen11CaUpdateRule2::*)()) &Owen11CaUpdateRule2::GetDiffusionParameter, 
            " "  )
        .def(
            "SetVesselNetwork", 
            (void(Owen11CaUpdateRule2::*)(::std::shared_ptr<VesselNetwork<2> >)) &Owen11CaUpdateRule2::SetVesselNetwork, 
            " " , py::arg("pVesselNetwork") )
        .def(
            "SetGridCalculator", 
            (void(Owen11CaUpdateRule2::*)(::std::shared_ptr<GridCalculator<2> >)) &Owen11CaUpdateRule2::SetGridCalculator, 
            " " , py::arg("pGridCalculator") )
        .def(
            "SetReferenceLengthScale", 
            (void(Owen11CaUpdateRule2::*)(::QLength)) &Owen11CaUpdateRule2::SetReferenceLengthScale, 
            " " , py::arg("referenceLengthScale") )
        .def(
            "SetDiffusionParameter", 
            (void(Owen11CaUpdateRule2::*)(::QDiffusivity)) &Owen11CaUpdateRule2::SetDiffusionParameter, 
            " " , py::arg("diffusionParameter") )
        .def(
            "OutputUpdateRuleParameters", 
            (void(Owen11CaUpdateRule2::*)(::out_stream &)) &Owen11CaUpdateRule2::OutputUpdateRuleParameters, 
            " " , py::arg("rParamsFile") )
    ;
}
