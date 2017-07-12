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

#include "MeshFormat.cppwg.hpp"

namespace py = pybind11;
typedef MeshFormat MeshFormat;
;

void register_MeshFormat_class(py::module &m){
py::class_<MeshFormat    >(m, "MeshFormat")
    ;
}
