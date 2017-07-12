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
#include "Secomb04Parameters.hpp"

#include "Secomb04Parameters.cppwg.hpp"

namespace py = pybind11;
typedef Secomb04Parameters Secomb04Parameters;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_Secomb04Parameters_class(py::module &m){
py::class_<Secomb04Parameters  , std::shared_ptr<Secomb04Parameters >   >(m, "Secomb04Parameters")
    ;
}
