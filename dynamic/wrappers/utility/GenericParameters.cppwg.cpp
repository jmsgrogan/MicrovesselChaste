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
#include "GenericParameters.hpp"

#include "GenericParameters.cppwg.hpp"

namespace py = pybind11;
typedef GenericParameters GenericParameters;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_GenericParameters_class(py::module &m){
py::class_<GenericParameters  , std::shared_ptr<GenericParameters >   >(m, "GenericParameters")
    ;
}
