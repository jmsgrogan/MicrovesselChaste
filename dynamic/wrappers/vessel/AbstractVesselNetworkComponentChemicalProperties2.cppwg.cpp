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
    void SetPermeability(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> permeability) override {
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
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(AbstractVesselNetworkComponentChemicalProperties2::*)() const ) &AbstractVesselNetworkComponentChemicalProperties2::GetPermeability, 
            " " )
        .def(
            "SetPermeability", 
            (void(AbstractVesselNetworkComponentChemicalProperties2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AbstractVesselNetworkComponentChemicalProperties2::SetPermeability, 
            " " , py::arg("permeability"))
    ;
}
