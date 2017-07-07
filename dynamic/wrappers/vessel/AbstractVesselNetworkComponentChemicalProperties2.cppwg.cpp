#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AbstractVesselNetworkComponentChemicalProperties.hpp"

#include "AbstractVesselNetworkComponentChemicalProperties2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractVesselNetworkComponentChemicalProperties<2 > AbstractVesselNetworkComponentChemicalProperties2;
;

class AbstractVesselNetworkComponentChemicalProperties2_Overloads : public AbstractVesselNetworkComponentChemicalProperties2{
    public:
    using AbstractVesselNetworkComponentChemicalProperties2::AbstractVesselNetworkComponentChemicalProperties;
    void SetPermeability(::QMembranePermeability permeability) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponentChemicalProperties2,
            SetPermeability,
            permeability);
    }

};
void register_AbstractVesselNetworkComponentChemicalProperties2_class(py::module &m){
py::class_<AbstractVesselNetworkComponentChemicalProperties2 , AbstractVesselNetworkComponentChemicalProperties2_Overloads   >(m, "AbstractVesselNetworkComponentChemicalProperties2")
        .def(
            "GetPermeability", 
            (::QMembranePermeability(AbstractVesselNetworkComponentChemicalProperties2::*)() const ) &AbstractVesselNetworkComponentChemicalProperties2::GetPermeability, 
            " "  )
        .def(
            "SetPermeability", 
            (void(AbstractVesselNetworkComponentChemicalProperties2::*)(::QMembranePermeability)) &AbstractVesselNetworkComponentChemicalProperties2::SetPermeability, 
            " " , py::arg("permeability") )
    ;
}
