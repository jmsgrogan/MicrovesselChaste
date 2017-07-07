#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "ParameterCollection.hpp"

#include "ParameterCollection.cppwg.hpp"

namespace py = pybind11;
typedef ParameterCollection ParameterCollection;
;

void register_ParameterCollection_class(py::module &m){
py::class_<ParameterCollection    >(m, "ParameterCollection")
        .def_static(
            "Instance", 
            (::ParameterCollection *(*)()) &ParameterCollection::Instance, 
            " "  )
        .def_static(
            "SharedInstance", 
            (::std::shared_ptr<ParameterCollection>(*)()) &ParameterCollection::SharedInstance, 
            " "  )
        .def(
            "AddParameter", 
            (void(ParameterCollection::*)(::std::shared_ptr<BaseParameterInstance>, ::std::string const &)) &ParameterCollection::AddParameter, 
            " " , py::arg("pParameter"), py::arg("rFirstInstantiated") )
        .def_static(
            "Destroy", 
            (void(*)()) &ParameterCollection::Destroy, 
            " "  )
        .def(
            "DumpToFile", 
            (void(ParameterCollection::*)(::std::string const &)) &ParameterCollection::DumpToFile, 
            " " , py::arg("rFilename") )
        .def(
            "GetParameter", 
            (::std::shared_ptr<BaseParameterInstance>(ParameterCollection::*)(::std::string const &)) &ParameterCollection::GetParameter, 
            " " , py::arg("rName") )
    ;
}
