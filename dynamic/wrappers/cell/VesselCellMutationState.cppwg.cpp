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
#include "VesselCellMutationState.hpp"

#include "VesselCellMutationState.cppwg.hpp"

namespace py = pybind11;
typedef VesselCellMutationState VesselCellMutationState;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_VesselCellMutationState_class(py::module &m){
py::class_<VesselCellMutationState  , std::shared_ptr<VesselCellMutationState >   >(m, "VesselCellMutationState")
        .def(py::init< >())
    ;
}
