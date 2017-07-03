#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "MacrophageMutationState.hpp"

#include "MacrophageMutationState.cppwg.hpp"

namespace py = pybind11;
typedef MacrophageMutationState MacrophageMutationState;
;

void register_MacrophageMutationState_class(py::module &m){
py::class_<MacrophageMutationState   , AbstractCellMutationState  >(m, "MacrophageMutationState")
        .def(py::init< >())
    ;
}
