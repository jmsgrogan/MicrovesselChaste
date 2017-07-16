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
#include "QuiescentCancerCellMutationState.hpp"

#include "QuiescentCancerCellMutationState.cppwg.hpp"

namespace py = pybind11;
typedef QuiescentCancerCellMutationState QuiescentCancerCellMutationState;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_QuiescentCancerCellMutationState_class(py::module &m){
py::class_<QuiescentCancerCellMutationState  , std::shared_ptr<QuiescentCancerCellMutationState >   >(m, "QuiescentCancerCellMutationState")
        .def(py::init< >())
    ;
}
