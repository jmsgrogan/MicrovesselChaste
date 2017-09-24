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
#include "AbstractVesselNetworkComponentChemicalProperties.hpp"

#include "PythonObjectConverters.hpp"
#include "AbstractVesselNetworkComponentChemicalProperties3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef AbstractVesselNetworkComponentChemicalProperties<3 > AbstractVesselNetworkComponentChemicalProperties3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AbstractVesselNetworkComponentChemicalProperties3_Overloads : public AbstractVesselNetworkComponentChemicalProperties3{
    public:
    using AbstractVesselNetworkComponentChemicalProperties3::AbstractVesselNetworkComponentChemicalProperties;
    void SetVegfConcentration(::QConcentration concentration) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponentChemicalProperties3,
            SetVegfConcentration,
            concentration);
    }
    void SetPermeability(::QMembranePermeability permeability) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponentChemicalProperties3,
            SetPermeability,
            permeability);
    }

};
void register_AbstractVesselNetworkComponentChemicalProperties3_class(py::module &m){
py::class_<AbstractVesselNetworkComponentChemicalProperties3 , AbstractVesselNetworkComponentChemicalProperties3_Overloads , std::shared_ptr<AbstractVesselNetworkComponentChemicalProperties3 >  , AbstractVesselNetworkComponentProperties<3>  >(m, "AbstractVesselNetworkComponentChemicalProperties3")
        .def(
            "GetVegfConcentration", 
            (::QConcentration(AbstractVesselNetworkComponentChemicalProperties3::*)() const ) &AbstractVesselNetworkComponentChemicalProperties3::GetVegfConcentration, 
            " "  )
        .def(
            "SetVegfConcentration", 
            (void(AbstractVesselNetworkComponentChemicalProperties3::*)(::QConcentration)) &AbstractVesselNetworkComponentChemicalProperties3::SetVegfConcentration, 
            " " , py::arg("concentration") )
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
