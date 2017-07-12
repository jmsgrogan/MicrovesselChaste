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
#include "TipCellMutationState.hpp"

#include "TipCellMutationState.cppwg.hpp"

namespace py = pybind11;
typedef TipCellMutationState TipCellMutationState;
;

void register_TipCellMutationState_class(py::module &m){
py::class_<TipCellMutationState   , AbstractCellMutationState  >(m, "TipCellMutationState")
        .def(py::init< >())
    ;
}
