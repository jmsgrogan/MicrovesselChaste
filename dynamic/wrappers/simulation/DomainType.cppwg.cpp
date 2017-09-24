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
#include "CornealMicropocketSimulation.hpp"

#include "PythonObjectConverters.hpp"
#include "DomainType.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef DomainType DomainType;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
void register_DomainType_class(py::module &m){
    py::class_<DomainType> myclass(m, "DomainType");
    py::enum_<DomainType::Value>(myclass, "Value")
        .value("PLANAR_2D", DomainType::Value::PLANAR_2D)
        .value("PLANAR_3D", DomainType::Value::PLANAR_3D)
        .value("PLANAR_2D_FINITE", DomainType::Value::PLANAR_2D_FINITE)
        .value("PLANAR_3D_FINITE", DomainType::Value::PLANAR_3D_FINITE)
        .value("CIRCLE_2D", DomainType::Value::CIRCLE_2D)
        .value("CIRCLE_3D", DomainType::Value::CIRCLE_3D)
        .value("HEMISPHERE", DomainType::Value::HEMISPHERE)
    .export_values();
}
