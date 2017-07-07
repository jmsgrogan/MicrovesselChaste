#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "GenericParameters.hpp"

#include "GenericParameters.cppwg.hpp"

namespace py = pybind11;
typedef GenericParameters GenericParameters;
;

void register_GenericParameters_class(py::module &m){
py::class_<GenericParameters    >(m, "GenericParameters")
    ;
}
