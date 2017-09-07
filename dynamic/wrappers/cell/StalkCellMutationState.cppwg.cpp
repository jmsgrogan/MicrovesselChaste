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
#include "StalkCellMutationState.hpp"

#include "PythonObjectConverters.hpp"
#include "StalkCellMutationState.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef StalkCellMutationState StalkCellMutationState;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_StalkCellMutationState_class(py::module &m){
py::class_<StalkCellMutationState  , std::shared_ptr<StalkCellMutationState >   >(m, "StalkCellMutationState")
        .def(py::init< >())
    ;
}
