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

#include "GeometryFormat.cppwg.hpp"

namespace py = pybind11;
typedef GeometryFormat GeometryFormat;
;

void register_GeometryFormat_class(py::module &m){
py::class_<GeometryFormat    >(m, "GeometryFormat")
    ;
}
