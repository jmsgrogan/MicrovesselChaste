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
#include "MacrophageMutationState.hpp"

#include "PythonObjectConverters.hpp"
#include "MacrophageMutationState.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef MacrophageMutationState MacrophageMutationState;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_MacrophageMutationState_class(py::module &m){
py::class_<MacrophageMutationState  , std::shared_ptr<MacrophageMutationState >   >(m, "MacrophageMutationState")
        .def(py::init< >())
    ;
}
