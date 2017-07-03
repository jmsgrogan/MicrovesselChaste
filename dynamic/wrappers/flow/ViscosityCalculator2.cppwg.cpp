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

#include "ViscosityCalculator2.cppwg.hpp"

namespace py = pybind11;
typedef ViscosityCalculator<2 > ViscosityCalculator2;
;

class ViscosityCalculator2_Overloads : public ViscosityCalculator2{
    public:
    using ViscosityCalculator2::ViscosityCalculator;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            ViscosityCalculator2,
            Calculate,
            );
    }

};
void register_ViscosityCalculator2_class(py::module &m){
py::class_<ViscosityCalculator2 , ViscosityCalculator2_Overloads   >(m, "ViscosityCalculator2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<ViscosityCalculator<2> >(*)()) &ViscosityCalculator2::Create, 
            " " )
        .def(
            "Calculate", 
            (void(ViscosityCalculator2::*)()) &ViscosityCalculator2::Calculate, 
            " " )
        .def(
            "SetPlasmaViscosity", 
            (void(ViscosityCalculator2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::mass_base_dimension, boost::units::static_rational<1, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &ViscosityCalculator2::SetPlasmaViscosity, 
            " " , py::arg("viscosity"))
    ;
}
