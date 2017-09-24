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
#include "MultiFormatMeshWriter.hpp"

#include "PythonObjectConverters.hpp"
#include "MeshFormat.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef MeshFormat MeshFormat;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
void register_MeshFormat_class(py::module &m){
    py::class_<MeshFormat> myclass(m, "MeshFormat");
    py::enum_<MeshFormat::Value>(myclass, "Value")
        .value("VTU", MeshFormat::Value::VTU)
        .value("DOLFIN", MeshFormat::Value::DOLFIN)
        .value("STL", MeshFormat::Value::STL)
    .export_values();
}
