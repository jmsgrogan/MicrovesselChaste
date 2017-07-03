#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "Connor17Parameters.hpp"

#include "Connor17Parameters.cppwg.hpp"

namespace py = pybind11;
typedef Connor17Parameters Connor17Parameters;
;

void register_Connor17Parameters_class(py::module &m){
py::class_<Connor17Parameters    >(m, "Connor17Parameters")
    ;
}
