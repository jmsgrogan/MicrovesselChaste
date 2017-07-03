#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "ViscosityCalculator.hpp"

#include "ViscosityCalculator3.cppwg.hpp"

namespace py = pybind11;
typedef ViscosityCalculator<3 > ViscosityCalculator3;
;

class ViscosityCalculator3_Overloads : public ViscosityCalculator3{
    public:
    using ViscosityCalculator3::ViscosityCalculator;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            ViscosityCalculator3,
            Calculate,
            );
    }

};
void register_ViscosityCalculator3_class(py::module &m){
py::class_<ViscosityCalculator3 , ViscosityCalculator3_Overloads   >(m, "ViscosityCalculator3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<ViscosityCalculator<3> >(*)()) &ViscosityCalculator3::Create, 
            " " )
        .def(
            "Calculate", 
            (void(ViscosityCalculator3::*)()) &ViscosityCalculator3::Calculate, 
            " " )
        .def(
            "SetPlasmaViscosity", 
            (void(ViscosityCalculator3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::mass_base_dimension, boost::units::static_rational<1, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &ViscosityCalculator3::SetPlasmaViscosity, 
            " " , py::arg("viscosity"))
    ;
}
