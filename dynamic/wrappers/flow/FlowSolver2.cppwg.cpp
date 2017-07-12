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
#include "FlowSolver.hpp"

#include "FlowSolver2.cppwg.hpp"

namespace py = pybind11;
typedef FlowSolver<2 > FlowSolver2;
;

void register_FlowSolver2_class(py::module &m){
py::class_<FlowSolver2    >(m, "FlowSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<FlowSolver<2> >(*)()) &FlowSolver2::Create, 
            " "  )
        .def(
            "SetUseDirectSolver", 
            (void(FlowSolver2::*)(bool)) &FlowSolver2::SetUseDirectSolver, 
            " " , py::arg("useDirectSolver") )
        .def(
            "SetUp", 
            (void(FlowSolver2::*)()) &FlowSolver2::SetUp, 
            " "  )
        .def(
            "SetVesselNetwork", 
            (void(FlowSolver2::*)(::std::shared_ptr<VesselNetwork<2> >)) &FlowSolver2::SetVesselNetwork, 
            " " , py::arg("pVesselNetwork") )
        .def(
            "Solve", 
            (void(FlowSolver2::*)()) &FlowSolver2::Solve, 
            " "  )
        .def(
            "Update", 
            (void(FlowSolver2::*)(bool)) &FlowSolver2::Update, 
            " " , py::arg("runSetup") = false )
    ;
}
