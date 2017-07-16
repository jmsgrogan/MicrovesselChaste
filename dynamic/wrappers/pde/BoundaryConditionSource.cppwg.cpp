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

#include "BoundaryConditionSource.cppwg.hpp"

namespace py = pybind11;
typedef BoundaryConditionSource BoundaryConditionSource;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
void register_BoundaryConditionSource_class(py::module &m){
    py::class_<BoundaryConditionSource> myclass(m, "BoundaryConditionSource");
    py::enum_<BoundaryConditionSource::Value>(myclass, "Value")
        .value("LABEL_BASED", BoundaryConditionSource::Value::LABEL_BASED)
        .value("PRESCRIBED", BoundaryConditionSource::Value::PRESCRIBED)
    .export_values();
}
