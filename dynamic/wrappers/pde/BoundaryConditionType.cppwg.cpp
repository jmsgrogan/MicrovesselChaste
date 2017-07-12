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
#include "DiscreteContinuumBoundaryCondition.hpp"

#include "BoundaryConditionType.cppwg.hpp"

namespace py = pybind11;
typedef BoundaryConditionType BoundaryConditionType;
;

void register_BoundaryConditionType_class(py::module &m){
py::class_<BoundaryConditionType    >(m, "BoundaryConditionType")
    ;
}
