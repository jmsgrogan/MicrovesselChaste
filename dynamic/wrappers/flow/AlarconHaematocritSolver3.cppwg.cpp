#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AlarconHaematocritSolver.hpp"

#include "AlarconHaematocritSolver3.cppwg.hpp"

namespace py = pybind11;
typedef AlarconHaematocritSolver<3 > AlarconHaematocritSolver3;
;

class AlarconHaematocritSolver3_Overloads : public AlarconHaematocritSolver3{
    public:
    using AlarconHaematocritSolver3::AlarconHaematocritSolver;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            AlarconHaematocritSolver3,
            Calculate,
            );
    }

};
void register_AlarconHaematocritSolver3_class(py::module &m){
py::class_<AlarconHaematocritSolver3 , AlarconHaematocritSolver3_Overloads   >(m, "AlarconHaematocritSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<AlarconHaematocritSolver<3> >(*)()) &AlarconHaematocritSolver3::Create, 
            " " )
        .def(
            "Calculate", 
            (void(AlarconHaematocritSolver3::*)()) &AlarconHaematocritSolver3::Calculate, 
            " " )
        .def(
            "SetTHR", 
            (void(AlarconHaematocritSolver3::*)(::boost::units::quantity<boost::units::unit<boost::units::dimensionless_type, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AlarconHaematocritSolver3::SetTHR, 
            " " , py::arg("thr"))
        .def(
            "SetAlpha", 
            (void(AlarconHaematocritSolver3::*)(::boost::units::quantity<boost::units::unit<boost::units::dimensionless_type, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AlarconHaematocritSolver3::SetAlpha, 
            " " , py::arg("alpha"))
        .def(
            "SetHaematocrit", 
            (void(AlarconHaematocritSolver3::*)(::boost::units::quantity<boost::units::unit<boost::units::dimensionless_type, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AlarconHaematocritSolver3::SetHaematocrit, 
            " " , py::arg("haematocrit"))
    ;
}
