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
#include "CancerCellMutationState.hpp"

#include "CancerCellMutationState.cppwg.hpp"

namespace py = pybind11;
typedef CancerCellMutationState CancerCellMutationState;
;

void register_CancerCellMutationState_class(py::module &m){
py::class_<CancerCellMutationState   , AbstractCellMutationState  >(m, "CancerCellMutationState")
        .def(py::init< >())
    ;
}
