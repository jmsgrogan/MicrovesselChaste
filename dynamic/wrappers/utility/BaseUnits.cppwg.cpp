#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "BaseUnits.hpp"

#include "BaseUnits.cppwg.hpp"

namespace py = pybind11;
typedef BaseUnits BaseUnits;
;

void register_BaseUnits_class(py::module &m){
py::class_<BaseUnits    >(m, "BaseUnits")
        .def_static(
            "Instance", 
            (::BaseUnits *(*)()) &BaseUnits::Instance, 
            " "  )
        .def_static(
            "SharedInstance", 
            (::std::shared_ptr<BaseUnits>(*)()) &BaseUnits::SharedInstance, 
            " "  )
        .def(
            "GetReferenceTimeScale", 
            (::QTime(BaseUnits::*)()) &BaseUnits::GetReferenceTimeScale, 
            " "  )
        .def(
            "GetReferenceLengthScale", 
            (::QLength(BaseUnits::*)()) &BaseUnits::GetReferenceLengthScale, 
            " "  )
        .def(
            "GetReferenceConcentrationScale", 
            (::QConcentration(BaseUnits::*)()) &BaseUnits::GetReferenceConcentrationScale, 
            " "  )
        .def(
            "SetReferenceTimeScale", 
            (void(BaseUnits::*)(::QTime)) &BaseUnits::SetReferenceTimeScale, 
            " " , py::arg("referenceTimeScale") )
        .def(
            "SetReferenceLengthScale", 
            (void(BaseUnits::*)(::QLength)) &BaseUnits::SetReferenceLengthScale, 
            " " , py::arg("referenceLengthScale") )
        .def(
            "SetReferenceConcentrationScale", 
            (void(BaseUnits::*)(::QConcentration)) &BaseUnits::SetReferenceConcentrationScale, 
            " " , py::arg("referenceConcentrationScale") )
        .def_static(
            "Destroy", 
            (void(*)()) &BaseUnits::Destroy, 
            " "  )
    ;
}
