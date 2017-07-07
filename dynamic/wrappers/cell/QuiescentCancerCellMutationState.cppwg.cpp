#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "QuiescentCancerCellMutationState.hpp"

#include "QuiescentCancerCellMutationState.cppwg.hpp"

namespace py = pybind11;
typedef QuiescentCancerCellMutationState QuiescentCancerCellMutationState;
;

void register_QuiescentCancerCellMutationState_class(py::module &m){
py::class_<QuiescentCancerCellMutationState   , AbstractCellMutationState  >(m, "QuiescentCancerCellMutationState")
        .def(py::init< >())
    ;
}
