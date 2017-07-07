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

#include "AbstractVesselNetworkComponentChemicalProperties3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractVesselNetworkComponentChemicalProperties<3 > AbstractVesselNetworkComponentChemicalProperties3;
;

class AbstractVesselNetworkComponentChemicalProperties3_Overloads : public AbstractVesselNetworkComponentChemicalProperties3{
    public:
    using AbstractVesselNetworkComponentChemicalProperties3::AbstractVesselNetworkComponentChemicalProperties;
    void SetPermeability(::QMembranePermeability permeability) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponentChemicalProperties3,
            SetPermeability,
            permeability);
    }

};
void register_AbstractVesselNetworkComponentChemicalProperties3_class(py::module &m){
py::class_<AbstractVesselNetworkComponentChemicalProperties3 , AbstractVesselNetworkComponentChemicalProperties3_Overloads   >(m, "AbstractVesselNetworkComponentChemicalProperties3")
        .def(
            "GetPermeability", 
            (::QMembranePermeability(AbstractVesselNetworkComponentChemicalProperties3::*)() const ) &AbstractVesselNetworkComponentChemicalProperties3::GetPermeability, 
            " "  )
        .def(
            "SetPermeability", 
            (void(AbstractVesselNetworkComponentChemicalProperties3::*)(::QMembranePermeability)) &AbstractVesselNetworkComponentChemicalProperties3::SetPermeability, 
            " " , py::arg("permeability") )
    ;
}
