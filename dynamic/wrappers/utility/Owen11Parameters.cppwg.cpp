#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "Owen11Parameters.hpp"

#include "Owen11Parameters.cppwg.hpp"

namespace py = pybind11;
typedef Owen11Parameters Owen11Parameters;
;

void register_Owen11Parameters_class(py::module &m){
py::class_<Owen11Parameters    >(m, "Owen11Parameters")
    ;
}
