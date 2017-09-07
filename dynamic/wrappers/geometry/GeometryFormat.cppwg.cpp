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
#include "GeometryWriter.hpp"

#include "PythonObjectConverters.hpp"
#include "GeometryFormat.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef GeometryFormat GeometryFormat;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
void register_GeometryFormat_class(py::module &m){
    py::class_<GeometryFormat> myclass(m, "GeometryFormat");
    py::enum_<GeometryFormat::Value>(myclass, "Value")
        .value("VTP", GeometryFormat::Value::VTP)
        .value("STL", GeometryFormat::Value::STL)
    .export_values();
}
