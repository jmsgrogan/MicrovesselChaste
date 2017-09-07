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

#include "PythonObjectConverters.hpp"
#include "BoundaryConditionType.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef BoundaryConditionType BoundaryConditionType;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
void register_BoundaryConditionType_class(py::module &m){
    py::class_<BoundaryConditionType> myclass(m, "BoundaryConditionType");
    py::enum_<BoundaryConditionType::Value>(myclass, "Value")
        .value("POINT", BoundaryConditionType::Value::POINT)
        .value("POLYGON", BoundaryConditionType::Value::POLYGON)
        .value("EDGE", BoundaryConditionType::Value::EDGE)
        .value("OUTER", BoundaryConditionType::Value::OUTER)
        .value("VESSEL_LINE", BoundaryConditionType::Value::VESSEL_LINE)
        .value("VESSEL_VOLUME", BoundaryConditionType::Value::VESSEL_VOLUME)
        .value("CELL", BoundaryConditionType::Value::CELL)
        .value("IN_PART", BoundaryConditionType::Value::IN_PART)
    .export_values();
}
