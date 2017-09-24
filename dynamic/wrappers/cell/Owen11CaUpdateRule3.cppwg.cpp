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
#include "Owen11CaUpdateRule3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef Owen11CaUpdateRule<3 > Owen11CaUpdateRule3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class Owen11CaUpdateRule3_Overloads : public Owen11CaUpdateRule3{
    public:
    using Owen11CaUpdateRule3::Owen11CaUpdateRule;
    double EvaluateProbability(unsigned int currentNodeIndex, unsigned int targetNodeIndex, ::CaBasedCellPopulation<3> & rCellPopulation, double dt, double deltaX, ::CellPtr cell) override {
        PYBIND11_OVERLOAD(
            double,
            Owen11CaUpdateRule3,
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
            Owen11CaUpdateRule3,
            OutputUpdateRuleParameters,
            rParamsFile);
    }

};
void register_Owen11CaUpdateRule3_class(py::module &m){
py::class_<Owen11CaUpdateRule3 , Owen11CaUpdateRule3_Overloads , std::shared_ptr<Owen11CaUpdateRule3 >   >(m, "Owen11CaUpdateRule3")
        .def(py::init< >())
        .def(
            "EvaluateProbability", 
            (double(Owen11CaUpdateRule3::*)(unsigned int, unsigned int, ::CaBasedCellPopulation<3> &, double, double, ::CellPtr)) &Owen11CaUpdateRule3::EvaluateProbability, 
            " " , py::arg("currentNodeIndex"), py::arg("targetNodeIndex"), py::arg("rCellPopulation"), py::arg("dt"), py::arg("deltaX"), py::arg("cell") )
        .def(
            "GetDiffusionParameter", 
            (::QDiffusivity(Owen11CaUpdateRule3::*)()) &Owen11CaUpdateRule3::GetDiffusionParameter, 
            " "  )
        .def(
            "SetVesselNetwork", 
            (void(Owen11CaUpdateRule3::*)(::std::shared_ptr<VesselNetwork<3> >)) &Owen11CaUpdateRule3::SetVesselNetwork, 
            " " , py::arg("pVesselNetwork") )
        .def(
            "SetGridCalculator", 
            (void(Owen11CaUpdateRule3::*)(::std::shared_ptr<GridCalculator<3> >)) &Owen11CaUpdateRule3::SetGridCalculator, 
            " " , py::arg("pGridCalculator") )
        .def(
            "SetReferenceLengthScale", 
            (void(Owen11CaUpdateRule3::*)(::QLength)) &Owen11CaUpdateRule3::SetReferenceLengthScale, 
            " " , py::arg("referenceLengthScale") )
        .def(
            "SetDiffusionParameter", 
            (void(Owen11CaUpdateRule3::*)(::QDiffusivity)) &Owen11CaUpdateRule3::SetDiffusionParameter, 
            " " , py::arg("diffusionParameter") )
        .def(
            "OutputUpdateRuleParameters", 
            (void(Owen11CaUpdateRule3::*)(::out_stream &)) &Owen11CaUpdateRule3::OutputUpdateRuleParameters, 
            " " , py::arg("rParamsFile") )
    ;
}
