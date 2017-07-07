#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "StructuralAdaptationSolver.hpp"

#include "StructuralAdaptationSolver2.cppwg.hpp"

namespace py = pybind11;
typedef StructuralAdaptationSolver<2 > StructuralAdaptationSolver2;
;

class StructuralAdaptationSolver2_Overloads : public StructuralAdaptationSolver2{
    public:
    using StructuralAdaptationSolver2::StructuralAdaptationSolver;
    void Iterate() override {
        PYBIND11_OVERLOAD(
            void,
            StructuralAdaptationSolver2,
            Iterate,
            );
    }

};
void register_StructuralAdaptationSolver2_class(py::module &m){
py::class_<StructuralAdaptationSolver2 , StructuralAdaptationSolver2_Overloads   >(m, "StructuralAdaptationSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<StructuralAdaptationSolver<2> >(*)()) &StructuralAdaptationSolver2::Create, 
            " "  )
        .def(
            "GetFlowSolver", 
            (::std::shared_ptr<FlowSolver<2> >(StructuralAdaptationSolver2::*)()) &StructuralAdaptationSolver2::GetFlowSolver, 
            " "  )
        .def(
            "Iterate", 
            (void(StructuralAdaptationSolver2::*)()) &StructuralAdaptationSolver2::Iterate, 
            " "  )
        .def(
            "AddPreFlowSolveCalculator", 
            (void(StructuralAdaptationSolver2::*)(::std::shared_ptr<AbstractVesselNetworkCalculator<2> >)) &StructuralAdaptationSolver2::AddPreFlowSolveCalculator, 
            " " , py::arg("pCalculator") )
        .def(
            "AddPostFlowSolveCalculator", 
            (void(StructuralAdaptationSolver2::*)(::std::shared_ptr<AbstractVesselNetworkCalculator<2> >)) &StructuralAdaptationSolver2::AddPostFlowSolveCalculator, 
            " " , py::arg("pCalculator") )
        .def(
            "SetFlowSolver", 
            (void(StructuralAdaptationSolver2::*)(::std::shared_ptr<FlowSolver<2> >)) &StructuralAdaptationSolver2::SetFlowSolver, 
            " " , py::arg("pSolver") )
        .def(
            "SetRadiusCalculator", 
            (void(StructuralAdaptationSolver2::*)(::std::shared_ptr<RadiusCalculator<2> >)) &StructuralAdaptationSolver2::SetRadiusCalculator, 
            " " , py::arg("pCalculator") )
        .def(
            "UpdateFlowSolver", 
            (void(StructuralAdaptationSolver2::*)(bool)) &StructuralAdaptationSolver2::UpdateFlowSolver, 
            " " , py::arg("doFullReset") = false )
    ;
}
