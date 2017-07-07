#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"

#include "BoundaryConditionSource.cppwg.hpp"

namespace py = pybind11;
typedef BoundaryConditionSource BoundaryConditionSource;
;

void register_BoundaryConditionSource_class(py::module &m){
py::class_<BoundaryConditionSource    >(m, "BoundaryConditionSource")
    ;
}
