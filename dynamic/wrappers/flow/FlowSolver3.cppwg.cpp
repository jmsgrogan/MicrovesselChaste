#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "FlowSolver.hpp"

#include "FlowSolver3.cppwg.hpp"

namespace py = pybind11;
typedef FlowSolver<3 > FlowSolver3;
;

void register_FlowSolver3_class(py::module &m){
py::class_<FlowSolver3    >(m, "FlowSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<FlowSolver<3> >(*)()) &FlowSolver3::Create, 
            " "  )
        .def(
            "SetUseDirectSolver", 
            (void(FlowSolver3::*)(bool)) &FlowSolver3::SetUseDirectSolver, 
            " " , py::arg("useDirectSolver") )
        .def(
            "SetUp", 
            (void(FlowSolver3::*)()) &FlowSolver3::SetUp, 
            " "  )
        .def(
            "SetVesselNetwork", 
            (void(FlowSolver3::*)(::std::shared_ptr<VesselNetwork<3> >)) &FlowSolver3::SetVesselNetwork, 
            " " , py::arg("pVesselNetwork") )
        .def(
            "Solve", 
            (void(FlowSolver3::*)()) &FlowSolver3::Solve, 
            " "  )
        .def(
            "Update", 
            (void(FlowSolver3::*)(bool)) &FlowSolver3::Update, 
            " " , py::arg("runSetup") = false )
    ;
}
