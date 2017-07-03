#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "StalkCellMutationState.hpp"

#include "StalkCellMutationState.cppwg.hpp"

namespace py = pybind11;
typedef StalkCellMutationState StalkCellMutationState;
;

void register_StalkCellMutationState_class(py::module &m){
py::class_<StalkCellMutationState   , AbstractCellMutationState  >(m, "StalkCellMutationState")
        .def(py::init< >())
    ;
}
