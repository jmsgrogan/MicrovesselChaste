#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AbstractVesselNetworkComponentFlowProperties.hpp"

#include "AbstractVesselNetworkComponentFlowProperties2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractVesselNetworkComponentFlowProperties<2 > AbstractVesselNetworkComponentFlowProperties2;
;

class AbstractVesselNetworkComponentFlowProperties2_Overloads : public AbstractVesselNetworkComponentFlowProperties2{
    public:
    using AbstractVesselNetworkComponentFlowProperties2::AbstractVesselNetworkComponentFlowProperties;
    void SetPressure(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::mass_base_dimension, boost::units::static_rational<1, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-2, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> pressure) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponentFlowProperties2,
            SetPressure,
            pressure);
    }

};
void register_AbstractVesselNetworkComponentFlowProperties2_class(py::module &m){
py::class_<AbstractVesselNetworkComponentFlowProperties2 , AbstractVesselNetworkComponentFlowProperties2_Overloads   >(m, "AbstractVesselNetworkComponentFlowProperties2")
        .def(
            "GetPressure", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::mass_base_dimension, boost::units::static_rational<1, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-2, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(AbstractVesselNetworkComponentFlowProperties2::*)() const ) &AbstractVesselNetworkComponentFlowProperties2::GetPressure, 
            " " )
        .def(
            "SetPressure", 
            (void(AbstractVesselNetworkComponentFlowProperties2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::mass_base_dimension, boost::units::static_rational<1, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-2, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AbstractVesselNetworkComponentFlowProperties2::SetPressure, 
            " " , py::arg("pressure"))
    ;
}
