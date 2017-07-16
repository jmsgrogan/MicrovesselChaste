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
#include "StructuralAdaptationSolver.hpp"

#include "StructuralAdaptationSolver3.cppwg.hpp"

namespace py = pybind11;
typedef StructuralAdaptationSolver<3 > StructuralAdaptationSolver3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class StructuralAdaptationSolver3_Overloads : public StructuralAdaptationSolver3{
    public:
    using StructuralAdaptationSolver3::StructuralAdaptationSolver;
    void Iterate() override {
        PYBIND11_OVERLOAD(
            void,
            StructuralAdaptationSolver3,
            Iterate,
            );
    }

};
void register_StructuralAdaptationSolver3_class(py::module &m){
py::class_<StructuralAdaptationSolver3 , StructuralAdaptationSolver3_Overloads , std::shared_ptr<StructuralAdaptationSolver3 >  , AbstractStructuralAdaptationSolver<3>  >(m, "StructuralAdaptationSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<StructuralAdaptationSolver<3> >(*)()) &StructuralAdaptationSolver3::Create, 
            " "  )
        .def(
            "GetFlowSolver", 
            (::std::shared_ptr<FlowSolver<3> >(StructuralAdaptationSolver3::*)()) &StructuralAdaptationSolver3::GetFlowSolver, 
            " "  )
        .def(
            "Iterate", 
            (void(StructuralAdaptationSolver3::*)()) &StructuralAdaptationSolver3::Iterate, 
            " "  )
        .def(
            "AddPreFlowSolveCalculator", 
            (void(StructuralAdaptationSolver3::*)(::std::shared_ptr<AbstractVesselNetworkCalculator<3> >)) &StructuralAdaptationSolver3::AddPreFlowSolveCalculator, 
            " " , py::arg("pCalculator") )
        .def(
            "AddPostFlowSolveCalculator", 
            (void(StructuralAdaptationSolver3::*)(::std::shared_ptr<AbstractVesselNetworkCalculator<3> >)) &StructuralAdaptationSolver3::AddPostFlowSolveCalculator, 
            " " , py::arg("pCalculator") )
        .def(
            "SetFlowSolver", 
            (void(StructuralAdaptationSolver3::*)(::std::shared_ptr<FlowSolver<3> >)) &StructuralAdaptationSolver3::SetFlowSolver, 
            " " , py::arg("pSolver") )
        .def(
            "SetRadiusCalculator", 
            (void(StructuralAdaptationSolver3::*)(::std::shared_ptr<RadiusCalculator<3> >)) &StructuralAdaptationSolver3::SetRadiusCalculator, 
            " " , py::arg("pCalculator") )
        .def(
            "UpdateFlowSolver", 
            (void(StructuralAdaptationSolver3::*)(bool)) &StructuralAdaptationSolver3::UpdateFlowSolver, 
            " " , py::arg("doFullReset") = false )
    ;
}
