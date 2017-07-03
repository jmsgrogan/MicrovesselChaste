#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "RadiusCalculator.hpp"

#include "RadiusCalculator2.cppwg.hpp"

namespace py = pybind11;
typedef RadiusCalculator<2 > RadiusCalculator2;
;

class RadiusCalculator2_Overloads : public RadiusCalculator2{
    public:
    using RadiusCalculator2::RadiusCalculator;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            RadiusCalculator2,
            Calculate,
            );
    }

};
void register_RadiusCalculator2_class(py::module &m){
py::class_<RadiusCalculator2 , RadiusCalculator2_Overloads   >(m, "RadiusCalculator2")
        .def(py::init< >())
        .def(
            "SetMinRadius", 
            (void(RadiusCalculator2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &RadiusCalculator2::SetMinRadius, 
            " " , py::arg("minRadius"))
        .def(
            "SetMaxRadius", 
            (void(RadiusCalculator2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &RadiusCalculator2::SetMaxRadius, 
            " " , py::arg("maxRadius"))
        .def(
            "SetTimestep", 
            (void(RadiusCalculator2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &RadiusCalculator2::SetTimestep, 
            " " , py::arg("dt"))
        .def(
            "Calculate", 
            (void(RadiusCalculator2::*)()) &RadiusCalculator2::Calculate, 
            " " )
    ;
}
