#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "CornealMicropocketSimulation.hpp"

#include "DomainType.cppwg.hpp"

namespace py = pybind11;
typedef DomainType DomainType;
;

void register_DomainType_class(py::module &m){
py::class_<DomainType    >(m, "DomainType")
    ;
}
