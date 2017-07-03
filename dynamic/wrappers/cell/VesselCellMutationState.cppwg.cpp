#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "VesselCellMutationState.hpp"

#include "VesselCellMutationState.cppwg.hpp"

namespace py = pybind11;
typedef VesselCellMutationState VesselCellMutationState;
;

void register_VesselCellMutationState_class(py::module &m){
py::class_<VesselCellMutationState   , AbstractCellMutationState  >(m, "VesselCellMutationState")
        .def(py::init< >())
    ;
}
