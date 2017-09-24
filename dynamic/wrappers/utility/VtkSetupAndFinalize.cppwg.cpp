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
#include "VtkSetupAndFinalize.hpp"

#include "PythonObjectConverters.hpp"
#include "VtkSetupAndFinalize.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef VtkSetupAndFinalize VtkSetupAndFinalize;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_VtkSetupAndFinalize_class(py::module &m){
py::class_<VtkSetupAndFinalize  , std::shared_ptr<VtkSetupAndFinalize >   >(m, "VtkSetupAndFinalize")
        .def(py::init< >())
        .def(
            "setUpVtk", 
            (bool(VtkSetupAndFinalize::*)()) &VtkSetupAndFinalize::setUpVtk, 
            " "  )
        .def(
            "tearDownVtk", 
            (bool(VtkSetupAndFinalize::*)()) &VtkSetupAndFinalize::tearDownVtk, 
            " "  )
    ;
}
